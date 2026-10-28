#!/usr/bin/env python3
"""Repeatable start-up localization measurement for MuJoCo ``localization_source:=lio``.

Why this exists (report 5.1.3 F / 5.1.4): after the spawn-height fix the runner's 30 s tracking gate
passed about 1 run in 4.  This script launches the same stack as ``test_mujoco_minco_mpc_chain.sh``
(same launch arguments, no goal), watches ``/localization/status`` until the first TRACKING state,
and records for every run

  * the first TRACKING time in simulation seconds, and the same time counted from the first
    ``/localization/status`` message (that is where the runner's 30 s gate starts),
  * the simulation/wall ratio,
  * ``map->odom`` (TF) and ``/localization`` against the ground truth: translation and yaw error,
  * the GICP node log lines that explain accept / reject / discard decisions.

Two sub-commands:

  run     N sequential runs (own ROS_DOMAIN_ID per run, ``ros2 launch`` in its own session so only
          our processes are signalled)
  record  the rclpy recorder (started by ``run``; can also be attached to a stack you launched)
  report  tabulate the result.json files under a directory

Example (4 runs of the tunnel start-up with RViz, domain 91..94, evidence in ~/ats_evidence):
  python3 scripts/measure_lio_startup_localization.py run --profile tunnel_2027 --runs 4 \
      --domain-base 91 --rviz --tag before_fix
"""

import argparse
import csv
import json
import math
import os
import re
import signal
import subprocess
import sys
import time
from pathlib import Path

WORKSPACE = Path(__file__).resolve().parent.parent
DEFAULT_EVIDENCE = Path.home() / "ats_evidence" / "gicp_startup"
STATE_NAMES = {
    0: "UNINITIALIZED", 1: "TRACKING", 2: "RELOCALIZING", 3: "DEGRADED", 4: "LOST",
    5: "BOOTSTRAP", 6: "CONFIRMED",
}

# Same defaults as test_mujoco_minco_mpc_chain.sh (profile tunnel_2027 / red_box_2027).
PROFILES = {
    "tunnel_2027": {"scene": "rmuc_2027", "start": None, "z": 0.550, "tunnel": "tunnel_B"},
    "red_box_2027": {"scene": "rmuc_2027", "start": (4.5, 6.25, 0.0), "z": 0.465},
    "red_box": {"scene": "rmuc_2025", "start": (-0.18, 0.06, 0.0), "z": 0.381},
}


# ----------------------------------------------------------------------------- SE(2) helpers
def wrap(angle):
    return (angle + math.pi) % (2.0 * math.pi) - math.pi


def compose(a, b):
    ca, sa = math.cos(a[2]), math.sin(a[2])
    return (a[0] + ca * b[0] - sa * b[1], a[1] + sa * b[0] + ca * b[1], wrap(a[2] + b[2]))


def inverse(a):
    ca, sa = math.cos(a[2]), math.sin(a[2])
    return (-(ca * a[0] + sa * a[1]), -(-sa * a[0] + ca * a[1]), wrap(-a[2]))


def yaw_of(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def pose_error(est, ref):
    """(translation error m, signed yaw error rad) of ``est`` against ``ref``."""
    return math.hypot(est[0] - ref[0], est[1] - ref[1]), wrap(est[2] - ref[2])


# ----------------------------------------------------------------------------- recorder
def run_recorder(args):
    import rclpy
    from nav_msgs.msg import Odometry
    from rclpy.node import Node
    from rclpy.parameter import Parameter
    from rclpy.qos import QoSProfile, ReliabilityPolicy
    from rosgraph_msgs.msg import Clock
    import tf2_ros
    from ats_navigation_interfaces.msg import LocalizationStatus

    out = Path(args.out)
    out.mkdir(parents=True, exist_ok=True)
    best_effort = QoSProfile(depth=200, reliability=ReliabilityPolicy.BEST_EFFORT)

    class Recorder(Node):
        def __init__(self):
            super().__init__("lio_startup_recorder", parameter_overrides=[
                Parameter("use_sim_time", Parameter.Type.BOOL, True)])
            self.sim_now = None
            self.first_clock_wall = None
            self.first_clock_sim = None
            self.last_clock_change_wall = time.monotonic()
            self.status_first_sim = None
            self.status_first_wall = None
            self.tracking_sim = None
            self.tracking_wall = None
            self.tracking_snapshot = None
            self.last_status_key = None
            self.truth = []   # (stamp, x, y, z, yaw)
            self.odom = []    # (stamp, x, y, yaw)
            self.loc = None   # (stamp, x, y, z, yaw)
            self.map_odom_last = None
            self.map_odom_updates = 0
            self.samples = []
            self.events = []
            self.done_reason = None
            self.start_wall = time.monotonic()
            self.tf_buffer = tf2_ros.Buffer()
            if not args.minimal:
                self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
            self.create_subscription(Clock, "/clock", self.on_clock, 10)
            self.create_subscription(
                LocalizationStatus, "/localization/status", self.on_status, 10)
            if not args.minimal:
                self.create_subscription(Odometry, "/localization", self.on_loc, best_effort)
                self.create_subscription(Odometry, "/odometry", self.on_odom, best_effort)
                self.create_subscription(
                    Odometry, "/simulation/ground_truth/odometry", self.on_truth, best_effort)
            # Steady clock on purpose: with use_sim_time the default timer never fires until
            # /clock exists, which is exactly the failure (no /clock) the end conditions must catch.
            self.create_timer(
                0.25, self.on_timer,
                clock=rclpy.clock.Clock(clock_type=rclpy.clock.ClockType.STEADY_TIME))

        # -- inputs
        def on_clock(self, msg):
            now = msg.clock.sec + msg.clock.nanosec * 1e-9
            if self.sim_now is None or now > self.sim_now:
                self.last_clock_change_wall = time.monotonic()
            if self.first_clock_wall is None:
                self.first_clock_wall = time.monotonic()
                self.first_clock_sim = now
            self.sim_now = now

        def on_truth(self, msg):
            p = msg.pose.pose
            self.truth.append((msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9,
                               p.position.x, p.position.y, p.position.z, yaw_of(p.orientation)))
            del self.truth[:-300]

        def on_odom(self, msg):
            p = msg.pose.pose
            self.odom.append((msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9,
                              p.position.x, p.position.y, yaw_of(p.orientation)))
            del self.odom[:-300]

        def on_loc(self, msg):
            p = msg.pose.pose
            self.loc = (msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9,
                        p.position.x, p.position.y, p.position.z, yaw_of(p.orientation))

        def on_status(self, msg):
            sim = self.sim_now if self.sim_now is not None else float("nan")
            wall = time.monotonic() - self.start_wall
            if self.status_first_sim is None:
                self.status_first_sim, self.status_first_wall = sim, wall
            key = (msg.state, msg.epoch, msg.observation_sequence, msg.consecutive_rejections)
            if key != self.last_status_key:
                self.last_status_key = key
                self.events.append({
                    "sim_s": round(sim, 3), "wall_s": round(wall, 2),
                    "state": STATE_NAMES.get(msg.state, str(msg.state)), "epoch": msg.epoch,
                    "observation_sequence": msg.observation_sequence,
                    "last_correction_translation": round(msg.last_correction_translation, 4),
                    "last_correction_yaw": round(msg.last_correction_yaw, 4),
                    "consecutive_rejections": msg.consecutive_rejections,
                    "message": msg.message,
                })
            if msg.state == LocalizationStatus.STATE_TRACKING and self.tracking_sim is None:
                self.tracking_sim, self.tracking_wall = sim, wall
                self.tracking_snapshot = self.snapshot()

        # -- evaluation
        @staticmethod
        def nearest(rows, stamp):
            best = None
            for row in rows:
                if best is None or abs(row[0] - stamp) < abs(best[0] - stamp):
                    best = row
            return best

        def lookup_map_odom(self):
            try:
                tf = self.tf_buffer.lookup_transform("map", "odom", rclpy.time.Time())
            except Exception:
                return None
            t = tf.transform
            return (t.translation.x, t.translation.y, yaw_of(t.rotation))

        def snapshot(self):
            """Errors of /localization and of map->odom against the ground truth right now."""
            row = {"sim_s": self.sim_now, "wall_s": time.monotonic() - self.start_wall}
            if self.loc is None:
                return row
            truth = self.nearest(self.truth, self.loc[0])
            odom = self.nearest(self.odom, self.loc[0])
            if truth is None or abs(truth[0] - self.loc[0]) > 0.1:
                return row
            ref = (truth[1], truth[2], truth[4])
            map_odom = self.lookup_map_odom()
            # /localization is the odometry pass-through (odom frame, report 5.1.3 D), so the map
            # frame base pose is map->odom (TF) composed with it.
            if map_odom is not None:
                est = compose(map_odom, (self.loc[1], self.loc[2], self.loc[4]))
                row["base_err_m"], row["base_err_yaw_rad"] = pose_error(est, ref)
                row["base_xy_yaw"] = [round(v, 4) for v in est]
            row["truth_xy_yaw"] = [round(v, 4) for v in ref]
            if map_odom is not None and odom is not None and abs(odom[0] - self.loc[0]) < 0.2:
                truth_map_odom = compose(ref, inverse((odom[1], odom[2], odom[3])))
                row["map_odom_err_m"], row["map_odom_err_yaw_rad"] = pose_error(
                    map_odom, truth_map_odom)
                row["map_odom_xy_yaw"] = [round(v, 4) for v in map_odom]
            return row

        def on_timer(self):
            wall = time.monotonic() - self.start_wall
            map_odom = self.lookup_map_odom()
            if map_odom is not None:
                if (self.map_odom_last is None
                        or math.hypot(map_odom[0] - self.map_odom_last[0],
                                      map_odom[1] - self.map_odom_last[1]) > 1e-4
                        or abs(wrap(map_odom[2] - self.map_odom_last[2])) > 1e-4):
                    self.map_odom_updates += 1
                self.map_odom_last = map_odom
            snap = self.snapshot()
            if self.sim_now is not None:
                snap["state"] = (STATE_NAMES.get(self.last_status_key[0], "?")
                                 if self.last_status_key else "none")
                self.samples.append(snap)
            # end conditions
            if self.tracking_sim is not None and self.sim_now is not None and (
                    self.sim_now - self.tracking_sim >= args.hold_after_tracking_s):
                self.done_reason = "tracking"
            elif self.sim_now is not None and self.sim_now >= args.max_sim_s:
                self.done_reason = "sim_timeout"
            elif wall >= args.max_wall_s:
                self.done_reason = "wall_timeout"
            elif (self.sim_now is not None
                  and time.monotonic() - self.last_clock_change_wall > args.stall_wall_s):
                self.done_reason = "clock_stalled"
            elif self.sim_now is None and wall > args.no_clock_wall_s:
                self.done_reason = "no_clock"

    rclpy.init()
    node = Recorder()
    try:
        while rclpy.ok() and node.done_reason is None:
            rclpy.spin_once(node, timeout_sec=0.2)
    except KeyboardInterrupt:
        node.done_reason = node.done_reason or "interrupted"
    final = node.snapshot()
    wall_total = time.monotonic() - node.start_wall
    gate_from_status = None
    if node.tracking_sim is not None and node.status_first_sim is not None:
        gate_from_status = node.tracking_sim - node.status_first_sim
    sim_elapsed = (node.sim_now - node.first_clock_sim) if node.first_clock_sim is not None else None
    result = {
        "outcome": node.done_reason,
        "tracking_reached": node.tracking_sim is not None,
        "first_tracking_sim_s": node.tracking_sim,
        "first_tracking_wall_s": node.tracking_wall,
        "status_first_seen_sim_s": node.status_first_sim,
        "tracking_after_status_sim_s": gate_from_status,
        "runner_gate_30s_would_pass": (gate_from_status is not None and gate_from_status <= 30.0),
        "sim_per_wall": (sim_elapsed / (time.monotonic() - node.first_clock_wall)
                         if node.first_clock_wall and sim_elapsed else None),
        "sim_end_s": node.sim_now,
        "wall_total_s": wall_total,
        "map_odom_updates": node.map_odom_updates,
        "at_first_tracking": node.tracking_snapshot,
        "at_end": final,
        "states_seen": [e["state"] for e in node.events],
    }
    (out / "result.json").write_text(json.dumps(result, indent=2, ensure_ascii=False) + "\n")
    with (out / "events.csv").open("w", newline="") as handle:
        if node.events:
            writer = csv.DictWriter(handle, fieldnames=list(node.events[0].keys()))
            writer.writeheader()
            writer.writerows(node.events)
    with (out / "samples.csv").open("w", newline="") as handle:
        keys = ["sim_s", "wall_s", "state", "base_err_m", "base_err_yaw_rad",
                "map_odom_err_m", "map_odom_err_yaw_rad"]
        writer = csv.DictWriter(handle, fieldnames=keys, extrasaction="ignore")
        writer.writeheader()
        writer.writerows(node.samples)
    print(json.dumps({k: result[k] for k in (
        "outcome", "first_tracking_sim_s", "tracking_after_status_sim_s",
        "runner_gate_30s_would_pass", "sim_per_wall", "map_odom_updates")}, ensure_ascii=False))
    try:
        node.destroy_node()
        rclpy.shutdown()
    except Exception:          # SIGINT already shut rclpy down
        pass
    return 0


# ----------------------------------------------------------------------------- driver
GICP_LINE = re.compile(
    r"重定位|GICP|multi_guess|多猜测|精配准|筛选完成|冷启动|丢弃")



def process_group_cpu_s(pgid):
    """Total CPU seconds (user+system) of the live processes in one process group (our launch)."""
    total = 0.0
    ticks = os.sysconf("SC_CLK_TCK")
    for entry in Path("/proc").iterdir():
        if not entry.name.isdigit():
            continue
        try:
            stat = (entry / "stat").read_text()
            fields = stat[stat.rindex(")") + 2:].split()
            if int(fields[2]) != pgid:           # field 5 of /proc/pid/stat (after "pid (comm) state ppid")
                continue
            total += (int(fields[11]) + int(fields[12])) / ticks      # utime + stime
        except (OSError, ValueError, IndexError):
            continue
    return total


def gicp_stats(lines):
    """Counts that explain the start-up: how often a finished fine result was thrown away."""
    def num(pattern, line):
        m = re.search(pattern, line)
        return float(m.group(1)) if m else None
    discards = [l for l in lines if "精配准丢弃" in l]
    verdicts = [l for l in lines if "【重定位精配准】" in l]
    screening = [num(r"筛选用时=([0-9.]+)s", l) for l in lines if "筛选完成" in l]
    would = [l for l in discards if "质量门判决=通过" in l]
    seed = [l for l in discards if re.search(r"候选=0 ", l)]
    return {
        "multi_guess_runs": len(screening),
        "screening_s_mean": (sum(screening) / len(screening)) if screening else None,
        "fine_inside_budget": len(verdicts),
        "fine_discarded_late": len(discards),
        "discarded_but_gate_pass": len(would),
        "seed_candidate_discarded": len(seed),
        "max_overrun_s": max((num(r"超出=([-0-9.]+)s", l) or 0.0) for l in discards) if discards else None,
        "stale_generation_drops": sum(1 for l in lines if "丢弃过期" in l),
    }


def tunnel_start(name):
    import yaml
    config = yaml.safe_load(
        (WORKSPACE / "src/sim/ats_mujoco_sim/config/tunnels_2027.yaml").read_text(encoding="utf-8"))
    for tunnel in config["tunnels"]:
        if tunnel["name"] == name:
            return tuple(tunnel["entry"])
    raise SystemExit(f"unknown tunnel {name}")


def write_params_file(run_dir, overrides, candidate_log):
    """Copy of the installed node_params.yaml with small_gicp_relocalization overrides."""
    import yaml
    source = WORKSPACE / "install/ats_sentry_bringup/share/ats_sentry_bringup/params/node_params.yaml"
    data = yaml.safe_load(source.read_text(encoding="utf-8"))
    section = data["small_gicp_relocalization"]["ros__parameters"]
    for item in overrides:
        key, _, value = item.partition("=")
        section[key] = yaml.safe_load(value)
    if candidate_log:
        section["multi_guess.candidate_log_path"] = str(run_dir / "candidates.csv")
        section["multi_guess.log_candidates"] = True
    path = run_dir / "node_params.yaml"
    path.write_text(yaml.safe_dump(data, allow_unicode=True, sort_keys=False), encoding="utf-8")
    return path


def terminate_session(proc, grace_s=45.0):
    """SIGINT the launch session, escalate to SIGTERM/SIGKILL only on that session."""
    if proc.poll() is not None:
        return "already_exited"
    try:
        pgid = os.getpgid(proc.pid)
    except ProcessLookupError:
        return "gone"
    for sig, wait in ((signal.SIGINT, grace_s), (signal.SIGTERM, 15.0), (signal.SIGKILL, 5.0)):
        try:
            os.killpg(pgid, sig)
        except ProcessLookupError:
            return "gone"
        deadline = time.monotonic() + wait
        while time.monotonic() < deadline:
            if proc.poll() is not None:
                return signal.Signals(sig).name
            time.sleep(0.5)
    return "unkillable"


def run_driver(args):
    profile = PROFILES[args.profile]
    if args.profile == "tunnel_2027":
        sx, sy, syaw = tunnel_start(profile["tunnel"])
    else:
        sx, sy, syaw = profile["start"]
    sz = args.start_z if args.start_z is not None else profile["z"]
    tag = args.tag or time.strftime("%Y%m%d_%H%M%S")
    root = Path(args.evidence) / f"{args.profile}_{'rviz' if args.rviz else 'norviz'}_{tag}"
    root.mkdir(parents=True, exist_ok=True)
    rows = []
    for index in range(args.runs):
        domain = args.domain_base + index
        run_dir = root / f"run_{index + 1:02d}"
        run_dir.mkdir(exist_ok=True)
        launch_args = [
            "ats_mujoco_sim", f"{profile['scene']}_mujoco.launch.py",
            f"use_viewer:={'true' if args.rviz else 'false'}",
            f"show_viewer:={'true' if args.rviz else 'false'}",
            f"use_rviz:={'true' if args.rviz else 'false'}",
            "launch_mujoco_rviz:=false", "force_body_yaw_follow:=false",
            "enable_lidar:=true", "lidar_backend:=cpu", "lidar_downsample:=1",
            "enable_tof:=false", "freeze_motion:=false",
            f"start_x:={sx}", f"start_y:={sy}", f"start_z:={sz}", f"start_yaw:={syaw}",
            "nav_start_delay_sec:=9.0", "rog_map_start_delay_sec:=15.0",
            "map_start_delay_sec:=2.0", f"rviz_delay_sec:={args.rviz_delay_s if args.rviz else 1000.0}",
            "planning_grid_owner:=rog_map", "solver_mode:=ilqr", f"log_level:={args.log_level}",
            "localization_source:=lio",
        ]
        launch_args += args.launch_arg
        if args.gicp_param or args.candidate_log:
            params = write_params_file(run_dir, args.gicp_param, args.candidate_log)
            launch_args.append(f"params_file:={params}")
        env = dict(os.environ, ROS_DOMAIN_ID=str(domain), ROS_LOCALHOST_ONLY="0")
        env.setdefault("DISPLAY", ":0")
        launch_log = run_dir / "launch.log"
        (run_dir / "command.txt").write_text(
            f"ROS_DOMAIN_ID={domain} ros2 launch {' '.join(launch_args)}\n")
        print(f"[run {index + 1}/{args.runs}] domain={domain} -> {run_dir}", flush=True)
        t0 = time.monotonic()
        with launch_log.open("wb") as log_handle:
            launch = subprocess.Popen(
                ["ros2", "launch", *launch_args], stdout=log_handle, stderr=subprocess.STDOUT,
                env=env, start_new_session=True)
            recorder = subprocess.Popen(
                [sys.executable, str(Path(__file__).resolve()), "record", "--out", str(run_dir),
                 "--max-sim-s", str(args.max_sim_s), "--max-wall-s", str(args.max_wall_s),
                 "--hold-after-tracking-s", str(args.hold_after_tracking_s)],
                env=env, stdout=(run_dir / "recorder.log").open("wb"), stderr=subprocess.STDOUT)
            pgid = os.getpgid(launch.pid)
            cpu_samples = []           # (wall_s, cumulative cpu_s of the live processes of the launch)
            deadline = time.monotonic() + args.max_wall_s + 120
            while recorder.poll() is None and time.monotonic() < deadline:
                cpu_samples.append((time.monotonic(), process_group_cpu_s(pgid)))
                time.sleep(2.0)
            if recorder.poll() is None:
                recorder.kill()
            teardown = terminate_session(launch)
        wall = time.monotonic() - t0
        result_path = run_dir / "result.json"
        result = json.loads(result_path.read_text()) if result_path.exists() else {"outcome": "no_result"}
        lines = [ln for ln in launch_log.read_text(errors="replace").splitlines()
                 if GICP_LINE.search(ln)]
        (run_dir / "gicp.log").write_text("\n".join(lines) + "\n", encoding="utf-8")
        trace_ms = [float(x) for x in re.findall(r"平均追踪 ([0-9.]+) ms/帧",
                                                  launch_log.read_text(errors="replace"))]
        if trace_ms:
            ordered = sorted(trace_ms)
            result["lidar_trace_ms_per_frame"] = {
                "n": len(trace_ms), "median": ordered[len(ordered) // 2], "max": ordered[-1]}
        result["gicp"] = gicp_stats(lines)
        if len(cpu_samples) > 5:
            # Whole-stack CPU in cores, from the cumulative CPU of the processes alive at each
            # sample (processes that exit drop out, so this is a lower bound at the end).  The
            # window starts 20 s in (after the nodes are up) and ends at the last sample.
            win = [c for c in cpu_samples if c[0] - cpu_samples[0][0] >= 20.0]
            if len(win) > 3:
                result["stack_cpu_cores"] = (win[-1][1] - win[0][1]) / (win[-1][0] - win[0][0])
        result["teardown"] = teardown
        result["driver_wall_s"] = round(wall, 1)
        result["domain"] = domain
        result_path.write_text(json.dumps(result, indent=2, ensure_ascii=False) + "\n")
        rows.append(result)
        print(f"  -> {result.get('outcome')} first_tracking_sim={result.get('first_tracking_sim_s')} "
              f"after_status={result.get('tracking_after_status_sim_s')} teardown={teardown}",
              flush=True)
        time.sleep(3.0)
    print(report_table(root))
    return 0


# ----------------------------------------------------------------------------- report
def fmt(value, spec="{:.2f}"):
    return "-" if value is None else spec.format(value)


def report_table(root):
    root = Path(root)
    lines = ["| run | outcome | first TRACKING (sim s) | after first status (sim s) | 30 s gate | "
             "sim/wall | map->odom updates | map->odom err at TRACKING (m / deg) | "
             "map-frame base err at TRACKING (m / deg) | multi_guess runs | fine kept / discarded late (gate-pass) |",
             "| --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- |"]
    for path in sorted(root.glob("run_*/result.json")):
        r = json.loads(path.read_text())
        snap = r.get("at_first_tracking") or {}
        gicp_log = path.parent / "gicp.log"
        g = r.get("gicp") or (gicp_stats(gicp_log.read_text(errors="replace").splitlines())
                              if gicp_log.exists() else {})
        mo = (f"{fmt(snap.get('map_odom_err_m'), '{:.3f}')} / "
              f"{fmt(None if snap.get('map_odom_err_yaw_rad') is None else math.degrees(snap['map_odom_err_yaw_rad']), '{:.2f}')}")
        lc = (f"{fmt(snap.get('base_err_m'), '{:.3f}')} / "
              f"{fmt(None if snap.get('base_err_yaw_rad') is None else math.degrees(snap['base_err_yaw_rad']), '{:.2f}')}")
        lines.append(
            f"| {path.parent.name} | {r.get('outcome')} | {fmt(r.get('first_tracking_sim_s'))} | "
            f"{fmt(r.get('tracking_after_status_sim_s'))} | "
            f"{'pass' if r.get('runner_gate_30s_would_pass') else 'FAIL'} | "
            f"{fmt(r.get('sim_per_wall'), '{:.2f}')} | {r.get('map_odom_updates')} | {mo} | {lc} | "
            f"{g.get('multi_guess_runs', '-')} | {g.get('fine_inside_budget', '-')} / "
            f"{g.get('fine_discarded_late', '-')} ({g.get('discarded_but_gate_pass', '-')}) |")
    return "\n".join(lines)


def main():
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    sub = parser.add_subparsers(dest="command", required=True)

    run = sub.add_parser("run", help="N sequential launches + recorder")
    run.add_argument("--profile", choices=sorted(PROFILES), required=True)
    run.add_argument("--runs", type=int, default=3)
    run.add_argument("--domain-base", type=int, default=91)
    run.add_argument("--rviz", action="store_true", help="USE_RVIZ=true (also opens the MuJoCo window)")
    run.add_argument("--rviz-delay-s", type=float, default=18.0)
    run.add_argument("--tag", default="")
    run.add_argument("--evidence", default=str(DEFAULT_EVIDENCE))
    run.add_argument("--start-z", type=float, default=None)
    run.add_argument("--log-level", default="warn", help="runner default is warn")
    run.add_argument("--launch-arg", action="append", default=[], help="extra name:=value")
    run.add_argument("--gicp-param", action="append", default=[],
                     help="small_gicp_relocalization override key=value (yaml value)")
    run.add_argument("--candidate-log", action="store_true",
                     help="write multi_guess candidate CSV + log_candidates=true")
    run.add_argument("--max-sim-s", type=float, default=60.0)
    run.add_argument("--max-wall-s", type=float, default=900.0)
    run.add_argument("--hold-after-tracking-s", type=float, default=5.0)

    rec = sub.add_parser("record", help="rclpy recorder")
    rec.add_argument("--out", required=True)
    rec.add_argument("--max-sim-s", type=float, default=60.0)
    rec.add_argument("--max-wall-s", type=float, default=900.0)
    rec.add_argument("--stall-wall-s", type=float, default=90.0)
    rec.add_argument("--no-clock-wall-s", type=float, default=240.0)
    rec.add_argument("--hold-after-tracking-s", type=float, default=5.0)
    rec.add_argument("--minimal", action="store_true",
                     help="only /clock and /localization/status (safe next to the runner's "
                          "subscriber-count assertions); no error evaluation")

    rep = sub.add_parser("report", help="tabulate result.json files")
    rep.add_argument("root")

    args = parser.parse_args()
    if args.command == "record":
        return run_recorder(args)
    if args.command == "run":
        return run_driver(args)
    print(report_table(args.root))
    return 0


if __name__ == "__main__":
    sys.exit(main())
