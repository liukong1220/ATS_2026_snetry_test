#!/usr/bin/env python3
"""Read-only receive timeline for lease/stall/stale fault recovery.

Phase markers and durations use the host monotonic clock, including when ROS
time freezes. ROS time is retained separately as identity, never as duration.
This observes DDS arrival order; it does not assert cross-topic atomicity.
"""
import argparse
import json
import math
import os
from pathlib import Path
import signal
import time


STREAMS = ("autonomy_raw", "selected", "motion_control")


def analyze(events, zero_window_sec=5.0, stop_deadline_sec=8.0):
    phases = {e["phase"]: e["mono"] for e in events if e["kind"] == "phase"}
    failures = []
    required = ("fault", "stopped", "recovery_requested", "recovered", "complete")
    if any(p not in phases for p in required):
        return {"passed": False, "failures": ["missing phases"], "phases": phases}
    if not all(phases[a] <= phases[b] for a, b in zip(required, required[1:])):
        failures.append("phase order")
    start, end = phases["recovered"], phases["complete"]
    if end - start < zero_window_sec:
        failures.append("recovery observation too short")
    before = [e for e in events if e["mono"] < phases["fault"]]
    references = {e["stamp_ns"] for e in before if e["kind"] == "reference"}
    if not references:
        failures.append("no pre-fault committed reference")
    if not any(e["kind"] == "execution" and e["mode"] == 1 for e in before):
        failures.append("no pre-fault execution authorization")
    if not any(e["kind"] == "selected" and e["magnitude"] > 0.001 for e in before):
        failures.append("no pre-fault motion")
    stops = [e for e in events if e["kind"] == "emergency_stop" and
             e["value"] and e["mono"] >= phases["fault"]]
    stop_latency = stops[0]["mono"] - phases["fault"] if stops else None
    if stop_latency is None or stop_latency > stop_deadline_sec:
        failures.append("missing or late emergency stop")
    windows = {}
    for label, lo, hi in (("stopped", phases["stopped"], phases["recovery_requested"]),
                          ("recovering", phases["recovery_requested"], end),
                          ("recovered", start, end)):
        windows[label] = {}
        for kind in STREAMS:
            samples = [e for e in events if e["kind"] == kind and lo <= e["mono"] <= hi]
            times = [lo] + [e["mono"] for e in samples] + [hi]
            gap = max(b - a for a, b in zip(times, times[1:]))
            values = [e["magnitude"] for e in samples]
            valid = bool(samples) and gap <= 1.0 and all(
                math.isfinite(v) and v <= 0.001 for v in values)
            windows[label][kind] = {"samples": len(samples), "max_gap_sec": gap,
                                    "max_magnitude": max(values, default=None), "passed": valid}
            if not valid:
                failures.append(f"{label} {kind} zero stream missing/nonzero/gap")
    # The goal is already terminal before recovery is requested. A brief revival
    # while waiting for ready/generation must not disappear from the verdict.
    recovered = [e for e in events if phases["recovery_requested"] <= e["mono"] <= end]
    reissued = [e for e in recovered if e["kind"] == "reference"]
    executed = [e for e in recovered if e["kind"] == "execution" and e["mode"] == 1]
    # A canceled goal must produce neither an old nor a newly retimed reference.
    if reissued or executed:
        failures.append("reference/EXECUTE revived without a new goal")
    ready = any(e["kind"] == "ready" and e["value"] and e["mono"] >= start for e in recovered)
    if not ready:
        failures.append("no recovered ready heartbeat")
    return {"passed": not failures, "failures": failures, "phases": phases,
            "stop_latency_sec_monotonic": stop_latency,
            "recovery_duration_sec_monotonic": end - start, "windows": windows,
            "pre_fault_reference_stamps_ns": sorted(references),
            "recovered_reference_count": len(reissued), "recovered_execute_count": len(executed),
            "recovered_old_reference_count": sum(e["stamp_ns"] in references for e in reissued),
            "timing_scope": "DDS receive order, not cross-topic atomic publish order"}


def main():
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import QoSProfile, ReliabilityPolicy
    from ats_navigation_interfaces.msg import ExecutionCommand, PlanningMapStatus
    from geometry_msgs.msg import Twist
    from manda_can_control.msg import MotionCtrl
    from nav_msgs.msg import Path as PathMessage
    from std_msgs.msg import Bool

    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--phases", type=Path, required=True)
    args = parser.parse_args()
    args.output.mkdir(parents=True, exist_ok=False)
    rclpy.init()
    node = Node("p2_transition_observer")
    events = []
    stream = (args.output / "events.jsonl").open("w")
    qos = QoSProfile(depth=100, reliability=ReliabilityPolicy.BEST_EFFORT)
    reliable = QoSProfile(depth=100)
    stopping = False
    latest_ros_ns = None

    def record(kind, payload):
        event = {"mono": time.monotonic(), "ros_ns": latest_ros_ns,
                 "kind": kind, **payload}
        events.append(event)
        stream.write(json.dumps(event, allow_nan=False) + "\n")
        stream.flush()

    def magnitude(values):
        return max(abs(float(v)) for v in values)

    def stamp(value):
        return int(value.sec) * 1000000000 + int(value.nanosec)

    for name, topic in (("autonomy_raw", "/cmd_vel/autonomy_raw"),
                        ("selected", "/cmd_vel/selected")):
        node.create_subscription(Twist, topic, lambda m, name=name: record(name, {
            "magnitude": magnitude((m.linear.x, m.linear.y, m.angular.z))}), qos)
    node.create_subscription(MotionCtrl, "/motion_control", lambda m: record("motion_control", {
        "magnitude": magnitude((m.linear_x, m.linear_y, m.angular_z))}), qos)
    for name, topic in (("ready", "/rog_map_adapter/ready"),
                        ("emergency_stop", "/planner/emergency_stop")):
        node.create_subscription(Bool, topic, lambda m, name=name:
                                 record(name, {"value": bool(m.data)}), reliable)
    node.create_subscription(PathMessage, "/minco/reference_path", lambda m: record("reference", {
        "stamp_ns": stamp(m.header.stamp), "poses": len(m.poses)}), reliable)
    node.create_subscription(ExecutionCommand, "/planner/execution_command", lambda m: record(
        "execution", {"mode": int(m.mode), "sequence": int(m.command_sequence),
                      "goal_id": int(m.goal_id), "stamp_ns": stamp(m.header.stamp),
                      "reference_stamp_ns": stamp(m.reference.header.stamp),
                      "localization_epoch": int(m.localization_epoch)}), reliable)
    node.create_subscription(PlanningMapStatus, "/rog_map_adapter/status", lambda m: record(
        "status", {"ready": bool(m.ready), "source_generation": int(m.rog_generation),
                   "publication_sequence": int(m.publication_sequence),
                   "localization_epoch": int(m.localization_epoch), "message": m.message}), reliable)
    # The node does not use sim time for scheduling, but records /clock explicitly.
    from rosgraph_msgs.msg import Clock
    def on_clock(message):
        nonlocal latest_ros_ns
        latest_ros_ns = stamp(message.clock)
        record("clock", {"stamp_ns": latest_ros_ns})
    node.create_subscription(Clock, "/clock", on_clock, qos)

    def request_stop(*_):
        nonlocal stopping
        stopping = True

    signal.signal(signal.SIGINT, request_stop)
    signal.signal(signal.SIGTERM, request_stop)
    phase_count = 0
    (args.output / "process.json").write_text(json.dumps({"pid": os.getpid(),
        "pgid": os.getpgrp(), "domain": os.environ.get("ROS_DOMAIN_ID")}))
    print("OBSERVER_READY", flush=True)
    try:
        while not stopping and rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.02)
            lines = args.phases.read_text().splitlines() if args.phases.exists() else []
            for line in lines[phase_count:]:
                marker = json.loads(line)
                record("phase", marker)
                phase_count += 1
                if marker["phase"] == "complete":
                    stopping = True
    finally:
        result = analyze(events)
        (args.output / "summary.json").write_text(json.dumps(result, indent=2) + "\n")
        stream.close()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    print(json.dumps(result), flush=True)
    return 0 if result["passed"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
