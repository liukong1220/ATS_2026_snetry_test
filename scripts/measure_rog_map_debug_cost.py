#!/usr/bin/env python3
"""CPU cost of ROG-Map's native debug clouds, measured on a running stack.

`ats_rog_map_node` builds /rog_map/occ, inf_occ, unk, esdf and viz only while something subscribes
to them (publishDebug, 2 Hz); core.visualization.* is not involved.  This script adds one
Best-Effort subscriber after another (the way RViz displays do) and, for every phase, reports

  * CPU of the ats_rog_map_node process over the phase (clock ticks from /proc),
  * the node's own `P2 debug build ... total_ms / map_lock_hold_ms` log lines (from --launch-log),
  * the message rate each subscriber sees.

map_lock_hold_ms matters beyond CPU: the debug build holds the map mutex that the projection service
(the planner's map source) also needs.  The numbers belong to the machine they were taken on.

    ROS_DOMAIN_ID=88 python3 scripts/measure_rog_map_debug_cost.py --launch-log /tmp/ats_minco_mpc_test_launch_88.log
"""

import argparse
import os
import re
import signal
import statistics
import subprocess
import sys
import time

PHASES = (
    ("baseline (no debug subscriber added)", []),
    ("+ /rog_map/occ", ["/rog_map/occ"]),
    ("+ /rog_map/inf_occ", ["/rog_map/occ", "/rog_map/inf_occ"]),
    ("+ /rog_map/unk", ["/rog_map/occ", "/rog_map/inf_occ", "/rog_map/unk"]),
    ("+ /rog_map/esdf (all four RViz clouds)", ["/rog_map/occ", "/rog_map/inf_occ", "/rog_map/unk", "/rog_map/esdf"]),
    ("+ /rog_map/viz (debug voxel state)", ["/rog_map/occ", "/rog_map/inf_occ", "/rog_map/unk", "/rog_map/esdf", "/rog_map/viz"]),
)
SUBSCRIBER = """
import sys, time, rclpy
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import PointCloud2
rclpy.init()
node = rclpy.create_node("debug_cost_sub_" + sys.argv[1].strip("/").replace("/", "_"))
state = {"n": 0, "bytes": 0}
def on_message(message):
    state["n"] += 1
    state["bytes"] = len(message.data)
node.create_subscription(PointCloud2, sys.argv[1], on_message, QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT))
start = time.time()
while rclpy.ok():
    rclpy.spin_once(node, timeout_sec=1.0)
    print(state["n"], round(time.time() - start, 1), state["bytes"], flush=True)
"""
DEBUG_LINE = re.compile(r"P2 debug build .*?\btotal_ms=([0-9.]+)")      # not the projection service lines, which also carry total_ms
LOCK_LINE = re.compile(r"P2 debug build .*?\bmap_lock_hold_ms=([0-9.]+)")


def find_rog_map_pid(domain):
    for entry in os.listdir("/proc"):
        if not entry.isdigit():
            continue
        try:
            with open(f"/proc/{entry}/cmdline", "rb") as handle:
                cmdline = handle.read().split(b"\0")[0].decode(errors="ignore")
            if not cmdline.endswith("ats_rog_map_node"):
                continue
            with open(f"/proc/{entry}/environ", "rb") as handle:
                environment = dict(item.split(b"=", 1) for item in handle.read().split(b"\0") if b"=" in item)
        except OSError:
            continue
        if environment.get(b"ROS_DOMAIN_ID", b"0").decode() == str(domain):
            return int(entry)
    raise SystemExit(f"no ats_rog_map_node found in ROS_DOMAIN_ID={domain}")


def cpu_ticks(pid):
    with open(f"/proc/{pid}/stat") as handle:
        fields = handle.read().rsplit(")", 1)[1].split()
    return int(fields[11]) + int(fields[12])      # utime + stime


def log_tail(path, offset):
    if not path:
        return "", offset
    with open(path, "rb") as handle:
        handle.seek(offset)
        data = handle.read()
    return data.decode(errors="ignore"), offset + len(data)


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--seconds", type=float, default=20.0, help="length of every phase")
    parser.add_argument("--launch-log", help="stdout log of the stack (for the P2 debug build lines)")
    parser.add_argument("--domain", default=os.environ.get("ROS_DOMAIN_ID", "0"))
    args = parser.parse_args()

    pid = find_rog_map_pid(args.domain)
    ticks_per_second = os.sysconf("SC_CLK_TCK")
    offset = os.path.getsize(args.launch_log) if args.launch_log else 0
    print(f"ats_rog_map_node pid {pid} (ROS_DOMAIN_ID={args.domain}), {args.seconds:.0f} s per phase\n")
    print(f"{'phase':44s} {'CPU % of one core':>18s} {'debug builds':>13s} {'debug total_ms mean/max':>24s} {'lock hold ms mean/max':>22s}")
    subscribers = {}
    try:
        for name, topics in PHASES:
            for topic in topics:
                if topic not in subscribers:
                    subscribers[topic] = subprocess.Popen(
                        [sys.executable, "-u", "-c", SUBSCRIBER, topic],
                        stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True, preexec_fn=os.setsid)
            time.sleep(3.0)                               # let discovery and the first builds settle
            _, offset = log_tail(args.launch_log, offset)
            start_ticks, start_time = cpu_ticks(pid), time.monotonic()
            time.sleep(args.seconds)
            cpu = 100.0 * (cpu_ticks(pid) - start_ticks) / ticks_per_second / (time.monotonic() - start_time)
            text, offset = log_tail(args.launch_log, offset)
            totals = [float(v) for v in DEBUG_LINE.findall(text)]
            locks = [float(v) for v in LOCK_LINE.findall(text)]
            total_text = f"{statistics.mean(totals):.1f} / {max(totals):.1f}" if totals else "no debug build"
            lock_text = f"{statistics.mean(locks):.1f} / {max(locks):.1f}" if locks else "-"
            print(f"{name:44s} {cpu:18.1f} {len(totals):13d} {total_text:>24s} {lock_text:>22s}")
        print("\nmessage rates seen by the subscribers (last window):")
        for topic, process in subscribers.items():
            os.killpg(process.pid, signal.SIGINT)
            try:
                output = process.communicate(timeout=5)[0]
            except subprocess.TimeoutExpired:
                os.killpg(process.pid, signal.SIGKILL)
                output = ""
            numeric = [line.split() for line in output.splitlines() if re.fullmatch(r"\d+ [0-9.]+ \d+", line.strip())]
            last = numeric[-1] if numeric else []
            if len(last) == 3 and float(last[1]) > 0 and int(last[0]) > 0:
                text = f"{int(last[0]) / float(last[1]):.2f} Hz, {int(last[2]) / 1024:.0f} KiB per message"
            else:
                text = "no messages: " + output.strip()[-160:].replace(chr(10), " | ")
            print(f"  {topic:18s} {text}")
        subscribers.clear()
    finally:
        for process in subscribers.values():
            try:
                os.killpg(process.pid, signal.SIGKILL)
            except ProcessLookupError:
                pass


if __name__ == "__main__":
    sys.exit(main())
