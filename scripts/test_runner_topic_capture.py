#!/usr/bin/env python3
"""Tests for scripts/runner_topic_capture.py.

    python3 -I scripts/test_runner_topic_capture.py

The pure parts (command parsing, the CDR header reader, the Path sampler, the QoS rule) run without
ROS.  With a sourced ROS 2 environment the CDR reader is also checked against rclpy's own serializer,
and one end-to-end test starts the capture process on a private ROS_DOMAIN_ID, publishes Path and
Twist messages and reads the files back with the same awk patterns the runner uses.
"""

import importlib.util
import os
import struct
import subprocess
import sys
import tempfile
import time
import unittest
from pathlib import Path

SCRIPTS = Path(__file__).resolve().parent


def load_module(name, filename):
    spec = importlib.util.spec_from_file_location(name, SCRIPTS / filename)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


capture = load_module("runner_topic_capture", "runner_topic_capture.py")


def ros_paths_on_sys_path():
    # -I drops PYTHONPATH; append (never prepend) the ROS entries so rclpy can load.
    for entry in os.environ.get("PYTHONPATH", "").split(os.pathsep):
        if entry and entry not in sys.path:
            sys.path.append(entry)
    try:
        import rclpy  # noqa: F401
        return True
    except ImportError:
        return False


HAVE_ROS = ros_paths_on_sys_path()


def cdr_path(frame_id, count, little=True):
    end = "<" if little else ">"
    body = struct.pack(end + "iI", 12, 34)
    raw = frame_id.encode() + b"\0"
    body += struct.pack(end + "I", len(raw)) + raw
    body += b"\0" * ((-len(body)) % 4)
    body += struct.pack(end + "I", count)
    return bytes([0, 1 if little else 0, 0, 0]) + body


class ParseCommandTest(unittest.TestCase):
    def test_execution_summary_mode_is_explicit_and_type_checked(self):
        _, fields = capture.parse_command(
            "open lease /planner/execution_command ats_navigation_interfaces/msg/ExecutionCommand "
            "/tmp/lease.out execution reliable")
        self.assertEqual(fields["mode"], "execution")
        with self.assertRaises(ValueError):
            capture.parse_command(
                "open lease /path nav_msgs/msg/Path /tmp/lease.out execution reliable")

    def test_open(self):
        verb, fields = capture.parse_command(
            "open c1 /minco/raw_path nav_msgs/msg/Path /tmp/a.out path reliable")
        self.assertEqual(verb, "open")
        self.assertEqual(fields["mode"], "path")
        self.assertEqual(fields["path"], "/tmp/a.out")

    def test_rejects(self):
        for line in ("", "open c1 /t nav_msgs/msg/Path /tmp/a path",
                     "open c1 /t nav_msgs/msg/Path /tmp/a sample reliable",
                     "open c1 /t nav_msgs/msg/Path /tmp/a all keep_last",
                     "open c1 t nav_msgs/msg/Path /tmp/a all reliable",
                     "open c1 /t Path /tmp/a all reliable", "close", "stop c1"):
            with self.assertRaises(ValueError, msg=line):
                capture.parse_command(line)

    def test_close_and_quit(self):
        self.assertEqual(capture.parse_command("close c3"), ("close", {"id": "c3"}))
        self.assertEqual(capture.parse_command("quit"), ("quit", {}))


class ExecutionSummaryTest(unittest.TestCase):
    def test_header_summary_preserves_authority_without_mutating_the_input(self):
        from types import SimpleNamespace as NS
        original = NS(mode=1, command_sequence=31, map_generation=8,
                      validation_map_publication_sequence=12,
                      validation_map_source_generation=10,
                      reference=NS(header=NS(frame_id="odom", stamp=NS(sec=2, nanosec=3)),
                                   poses=[NS(pose=NS(x=1))] * 500))
        summary, count = capture.compact_execution_message(original)
        self.assertEqual(count, 500)
        self.assertEqual(len(original.reference.poses), 500)
        self.assertEqual(summary.reference.poses, [])
        self.assertIsNot(summary.reference, original.reference)
        self.assertIs(summary.reference.header, original.reference.header)
        for key in ("mode", "command_sequence", "map_generation",
                    "validation_map_publication_sequence", "validation_map_source_generation"):
            self.assertEqual(getattr(summary, key), getattr(original, key))


class CdrHeaderTest(unittest.TestCase):
    def test_frame_and_count_for_each_padding(self):
        for frame in ("", "m", "map", "odom", "odom_x"):
            for little in (True, False):
                self.assertEqual(capture.path_header_from_cdr(cdr_path(frame, 7, little)), (frame, 7))

    def test_truncated(self):
        data = cdr_path("odom", 3)
        self.assertIsNone(capture.path_header_from_cdr(data[:-1]))
        self.assertIsNone(capture.path_header_from_cdr(b"\0\1\0\0"))

    @unittest.skipUnless(HAVE_ROS, "rclpy not importable")
    def test_matches_rclpy_serializer(self):
        from geometry_msgs.msg import PoseStamped
        from nav_msgs.msg import Path as PathMsg
        from rclpy.serialization import serialize_message
        for frame, count in (("map", 0), ("odom", 1), ("gimbal_yaw_odom", 130)):
            msg = PathMsg()
            msg.header.frame_id = frame
            msg.poses = [PoseStamped() for _ in range(count)]
            self.assertEqual(capture.path_header_from_cdr(serialize_message(msg)), (frame, count))


class PathSamplerTest(unittest.TestCase):
    def test_writes_first_and_changes_only(self):
        sampler = capture.PathSampler()
        offers = [("odom", 0), ("odom", 0), ("odom", 5), ("odom", 9), ("map", 9), ("map", 0), ("map", 0)]
        written = [sampler.offer(*o) for o in offers]
        self.assertEqual(written, [True, False, True, False, True, True, False])
        self.assertEqual((sampler.received, sampler.written), (7, 4))


class QosRuleTest(unittest.TestCase):
    def test_echo_rule(self):
        rule = capture.choose_reliability
        self.assertEqual(rule("auto", ["R", "R"], "R", "B"), "R")
        self.assertEqual(rule("auto", ["R", "B"], "R", "B"), "B")
        self.assertEqual(rule("auto", [], "R", "B"), "B")
        self.assertEqual(rule("reliable", ["B"], "R", "B"), "R")
        self.assertEqual(rule("best_effort", ["R"], "R", "B"), "B")


def read_text(path):
    with open(path) as handle:
        return handle.read()


def awk_has_nonempty_path(path):
    return subprocess.run(
        ["awk", r"/^[[:space:]]*-[[:space:]]+header:$/ {found = 1} END {exit found ? 0 : 1}", path]
    ).returncode == 0


def awk_frame(path, expected):
    prog = r'''
    /^header:$/ {in_path_header = 1; frame = ""; next}
    /^poses:$/ { if (in_path_header && frame == expected) found = 1; in_path_header = 0; next }
    in_path_header && $1 == "frame_id:" {frame = $2}
    END {exit found ? 0 : 1}'''
    return subprocess.run(["awk", "-v", f"expected={expected}", prog, path]).returncode == 0


@unittest.skipUnless(HAVE_ROS, "rclpy not importable")
class EndToEndTest(unittest.TestCase):
    def test_path_and_twist_capture(self):
        import rclpy
        from ats_navigation_interfaces.msg import ExecutionCommand
        from geometry_msgs.msg import PoseStamped, Twist
        from nav_msgs.msg import Path as PathMsg

        tmp = tempfile.mkdtemp(prefix="runner_capture_test_")
        fifo, ack = os.path.join(tmp, "c.fifo"), os.path.join(tmp, "c.ack")
        os.mkfifo(fifo)
        # Allocation/cleanup belong to nav_domain_guard, never a PID-derived ID.
        # The pure tests remain runnable without starting a ROS participant.
        domain = os.environ.get("ROS_DOMAIN_ID")
        if domain is None:
            self.skipTest("run the ROS integration case under nav_domain_guard with ROS_DOMAIN_ID")
        if not domain.isdecimal() or not 0 <= int(domain) <= 232 or int(domain) in (5, 77, 91, 92):
            self.fail("invalid explicitly allocated ROS_DOMAIN_ID")
        env = dict(os.environ)
        proc = subprocess.Popen([sys.executable, str(SCRIPTS / "runner_topic_capture.py"),
                                 "--control", fifo, "--ack", ack], env=env)

        def wait_ack(line, timeout=20.0):
            deadline = time.time() + timeout
            while time.time() < deadline:
                if os.path.exists(ack) and line in read_text(ack).split("\n"):
                    return True
                time.sleep(0.05)
            return False

        def send(line):
            with open(fifo, "w") as handle:
                handle.write(line + "\n")

        os.environ["ROS_DOMAIN_ID"] = env["ROS_DOMAIN_ID"]
        rclpy.init()
        node = rclpy.create_node("capture_test_publisher")
        try:
            self.assertTrue(wait_ack("ready"))
            path_pub = node.create_publisher(PathMsg, "/test/path", 10)
            twist_pub = node.create_publisher(Twist, "/test/cmd", 10)
            execution_pub = node.create_publisher(ExecutionCommand, "/test/execution", 10)
            path_out, twist_out = os.path.join(tmp, "path.out"), os.path.join(tmp, "cmd.out")
            execution_out = os.path.join(tmp, "execution.out")
            send(f"open c1 /test/path nav_msgs/msg/Path {path_out} path reliable")
            send(f"open c2 /test/cmd geometry_msgs/msg/Twist {twist_out} all auto")
            send(f"open c4 /test/execution ats_navigation_interfaces/msg/ExecutionCommand "
                 f"{execution_out} execution reliable")
            send("open c3 /test/x nav_msgs/msg/Path /nonexistent_dir/x.out path reliable")
            self.assertTrue(wait_ack("ok c1") and wait_ack("ok c2") and wait_ack("ok c4"))
            self.assertTrue(any(l.startswith("err c3 ") for l in read_text(ack).split("\n")))
            deadline = time.time() + 10
            while time.time() < deadline and (path_pub.get_subscription_count() < 1 or
                                               twist_pub.get_subscription_count() < 1 or
                                               execution_pub.get_subscription_count() < 1):
                rclpy.spin_once(node, timeout_sec=0.05)
            empty = PathMsg()
            empty.header.frame_id = "odom"
            full = PathMsg()
            full.header.frame_id = "odom"
            full.poses = [PoseStamped() for _ in range(3)]
            twist = Twist()
            twist.linear.y = 0.5
            execution = ExecutionCommand()
            execution.mode = ExecutionCommand.MODE_EXECUTE
            execution.reference.header.frame_id = "odom"
            execution.reference.poses = [PoseStamped() for _ in range(500)]
            execution.validation_map_publication_sequence = 17
            for msg in (empty, empty, full, full, full):
                path_pub.publish(msg)
                twist_pub.publish(twist)
                execution.command_sequence += 1
                execution_pub.publish(execution)
                time.sleep(0.05)
            time.sleep(0.5)
            send("close c1")
            send("close c2")
            send("close c4")
            self.assertTrue(wait_ack("closed c1") and wait_ack("closed c2") and
                            wait_ack("closed c4"))
            self.assertEqual(read_text(path_out + ".count").split(), ["5", "2"])
            self.assertEqual(read_text(twist_out + ".count").split(), ["5", "5"])
            self.assertTrue(awk_has_nonempty_path(path_out))
            self.assertTrue(awk_frame(path_out, "odom"))
            self.assertFalse(awk_frame(path_out, "map"))
            self.assertEqual(read_text(twist_out).count("---\n"), 5)
            self.assertIn("y: 0.5", read_text(twist_out))
            self.assertEqual(read_text(execution_out + ".count").split(), ["5", "5"])
            execution_text = read_text(execution_out)
            self.assertEqual(execution_text.count("reference_pose_count: 500"), 5)
            self.assertEqual(execution_text.count("poses: []"), 5)
            self.assertIn("validation_map_publication_sequence: 17", execution_text)
            for sequence in range(1, 6):
                self.assertIn(f"command_sequence: {sequence}\n", execution_text)
            self.assertLess(len(execution_text), 15000)
            send("quit")
            self.assertTrue(wait_ack("bye"))
            self.assertEqual(proc.wait(timeout=10), 0)
        finally:
            if proc.poll() is None:
                proc.kill()
            node.destroy_node()
            rclpy.try_shutdown()


if __name__ == "__main__":
    unittest.main(verbosity=2)
