#!/usr/bin/env python3
"""One in-process subscriber for the topic captures of test_mujoco_minco_mpc_chain.sh.

The runner used to start one ``ros2 topic echo`` per capture.  During a goal that was up to nine
processes (four leg Path captures, two MPC debug Path captures, two Twist streams and the
ExecutionCommand stream), 45-47 % of a core each on the 8-core machine (report 5.1.7 B/I), which
slowed the simulation and stole CPU from the nodes being measured.  This process holds all of them:

* One DDS participant.  Each capture is opened and closed through a control FIFO, so the runner keeps
  its start/stop points.  A capture is "ready" once its subscription exists; the runner waits for
  the ``ok <id>`` line in the ack file before it publishes the goal, as it waited for the echo process
  to survive one second before.
* The files keep the ``ros2 topic echo`` YAML layout (``message_to_yaml`` + ``---``, arrays truncated
  at 128 like ros2cli's default), so every awk gate in the runner reads them unchanged.
* ``mode all`` writes every message (Twist command streams, ExecutionCommand).
* ``mode path`` is for nav_msgs/Path.  The runner only asks whether a non-empty Path with the expected
  frame_id arrived.  The subscription is raw: frame_id and the pose count are read from the CDR bytes,
  and a message is deserialised and written only when (frame_id, empty/non-empty) differs from the last
  written one.  Every message is still counted; ``<file>.count`` gets ``received written`` on close.

Control lines (one per line, space separated, no spaces inside fields):

    open <id> <topic> <type> <file> <mode> <reliability>   reliability: reliable|best_effort|auto
    close <id>
    quit

Answers in the ack file: ``ok <id>`` (open), ``closed <id>`` (close), ``bye`` (quit) or
``err <id> <reason>``.

    python3 scripts/runner_topic_capture.py --control /tmp/x.fifo --ack /tmp/x.ack

Only the standard library is imported at module level (rclpy inside main()), so the pure parts are
tested with ``python3 -I scripts/test_runner_topic_capture.py``.
"""

import argparse
import os
import select
import signal
import struct
import sys

TRUNCATE_LENGTH = 128  # ros2topic.verb.echo DEFAULT_TRUNCATE_LENGTH
MODES = ("all", "path")
RELIABILITIES = ("reliable", "best_effort", "auto")


def parse_command(line):
    """Return (verb, fields) or raise ValueError with a short reason."""
    parts = line.split()
    if not parts:
        raise ValueError("empty")
    verb = parts[0]
    if verb == "open":
        if len(parts) != 7:
            raise ValueError("open_needs_6_fields")
        _, cid, topic, msg_type, path, mode, reliability = parts
        if mode not in MODES:
            raise ValueError("bad_mode")
        if reliability not in RELIABILITIES:
            raise ValueError("bad_reliability")
        if not topic.startswith("/") or msg_type.count("/") != 2:
            raise ValueError("bad_topic_or_type")
        return verb, {"id": cid, "topic": topic, "type": msg_type, "path": path,
                      "mode": mode, "reliability": reliability}
    if verb == "close":
        if len(parts) != 2:
            raise ValueError("close_needs_id")
        return verb, {"id": parts[1]}
    if verb == "quit":
        return verb, {}
    raise ValueError("unknown_verb")


def path_header_from_cdr(data):
    """(frame_id, pose_count) of a serialised nav_msgs/Path, or None if the bytes are too short.

    CDR layout after the 4-byte encapsulation header: int32 sec, uint32 nanosec, string frame_id
    (uint32 length including the NUL, then bytes), padding to 4, uint32 sequence length of poses.
    Alignment is relative to the start of the payload (after the encapsulation header).
    """
    if len(data) < 16:
        return None
    little = data[1] in (1, 3)  # CDR_LE / PL_CDR_LE
    fmt = "<I" if little else ">I"
    body = memoryview(data)[4:]
    offset = 8
    if len(body) < offset + 4:
        return None
    (str_len,) = struct.unpack_from(fmt, body, offset)
    offset += 4
    if str_len < 1 or len(body) < offset + str_len:
        return None
    frame_id = bytes(body[offset:offset + str_len - 1]).decode("utf-8", "replace")
    offset += str_len
    offset = (offset + 3) & ~3
    if len(body) < offset + 4:
        return None
    (count,) = struct.unpack_from(fmt, body, offset)
    return frame_id, count


class PathSampler:
    """Decides which Path messages are written: the first one and every change of (frame, non-empty)."""

    def __init__(self):
        self.last = None
        self.received = 0
        self.written = 0

    def offer(self, frame_id, pose_count):
        self.received += 1
        key = (frame_id, pose_count > 0)
        if key == self.last:
            return False
        self.last = key
        self.written += 1
        return True


def choose_reliability(requested, publisher_reliabilities, reliable_value, best_effort_value):
    """Same rule as ``ros2 topic echo`` without --qos flags: reliable only if every publisher is."""
    if requested == "reliable":
        return reliable_value
    if requested == "best_effort":
        return best_effort_value
    if publisher_reliabilities and all(r == reliable_value for r in publisher_reliabilities):
        return reliable_value
    return best_effort_value


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    parser.add_argument("--control", required=True, help="control FIFO (created by the runner)")
    parser.add_argument("--ack", required=True, help="append-only acknowledgement file")
    args = parser.parse_args(argv)

    import rclpy
    from rclpy.executors import SingleThreadedExecutor
    from rclpy.qos import QoSDurabilityPolicy, QoSProfile, QoSReliabilityPolicy
    from rclpy.serialization import deserialize_message
    from rosidl_runtime_py import message_to_yaml
    from rosidl_runtime_py.utilities import get_message

    rclpy.init(args=None)
    node = rclpy.create_node("runner_topic_capture", start_parameter_services=False)
    executor = SingleThreadedExecutor()
    executor.add_node(node)
    captures = {}
    ack = open(args.ack, "a", buffering=1)

    def reply(text):
        ack.write(text + "\n")
        ack.flush()

    def write_message(capture, msg):
        capture["file"].write(message_to_yaml(msg, truncate_length=TRUNCATE_LENGTH) + "---\n")
        capture["file"].flush()

    def open_capture(spec):
        if spec["id"] in captures:
            raise ValueError("duplicate_id")
        msg_class = get_message(spec["type"])
        infos = node.get_publishers_info_by_topic(spec["topic"])
        reliability = choose_reliability(
            spec["reliability"], [i.qos_profile.reliability for i in infos],
            QoSReliabilityPolicy.RELIABLE, QoSReliabilityPolicy.BEST_EFFORT)
        qos = QoSProfile(depth=10, reliability=reliability,
                         durability=QoSDurabilityPolicy.VOLATILE)
        out = open(spec["path"], "w", buffering=1)
        capture = dict(spec, file=out, cls=msg_class, sampler=PathSampler(), sub=None)
        if spec["mode"] == "path":
            def on_raw(data, capture=capture):
                header = path_header_from_cdr(data)
                if header is None:
                    return
                if capture["sampler"].offer(*header):
                    write_message(capture, deserialize_message(data, capture["cls"]))
            capture["sub"] = node.create_subscription(msg_class, spec["topic"], on_raw, qos, raw=True)
        else:
            def on_msg(msg, capture=capture):
                capture["sampler"].received += 1
                capture["sampler"].written += 1
                write_message(capture, msg)
            capture["sub"] = node.create_subscription(msg_class, spec["topic"], on_msg, qos)
        captures[spec["id"]] = capture

    def close_capture(cid):
        capture = captures.pop(cid, None)
        if capture is None:
            raise ValueError("unknown_id")
        node.destroy_subscription(capture["sub"])
        capture["file"].close()
        with open(capture["path"] + ".count", "w") as count_file:
            count_file.write(f"{capture['sampler'].received} {capture['sampler'].written}\n")

    stopping = []
    signal.signal(signal.SIGTERM, lambda *_: stopping.append(True))
    signal.signal(signal.SIGINT, lambda *_: stopping.append(True))
    # O_RDWR keeps the FIFO open even between writers, so select() never sees a permanent EOF.
    control_fd = os.open(args.control, os.O_RDWR | os.O_NONBLOCK)
    pending = b""
    reply("ready")
    try:
        while not stopping:
            readable, _, _ = select.select([control_fd], [], [], 0.0)
            if readable:
                pending += os.read(control_fd, 65536)
                while b"\n" in pending:
                    raw_line, pending = pending.split(b"\n", 1)
                    line = raw_line.decode("utf-8", "replace").strip()
                    if not line:
                        continue
                    cid = line.split()[1] if len(line.split()) > 1 else "-"
                    try:
                        verb, fields = parse_command(line)
                        if verb == "open":
                            open_capture(fields)
                            reply(f"ok {cid}")
                        elif verb == "close":
                            close_capture(fields["id"])
                            reply(f"closed {cid}")
                        else:
                            stopping.append(True)
                            reply("bye")
                    except Exception as exc:  # report and keep serving the other captures
                        reply(f"err {cid} {type(exc).__name__}:{str(exc).replace(' ', '_')[:80]}")
            executor.spin_once(timeout_sec=0.05)
    finally:
        for cid in list(captures):
            close_capture(cid)
        os.close(control_fd)
        ack.close()
        node.destroy_node()
        rclpy.try_shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
