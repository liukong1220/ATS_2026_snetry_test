#!/usr/bin/env python3
"""Scenario driver and trajectory evaluator for MuJoCo ``localization_source:=lio``.

Records the estimated pose (/odometry from sensor_scan_generation, Point-LIO's
start frame) and the evaluation-only ground truth, drives the chassis through a
scripted scenario, and reports drift, ATE, end-point error, sensor rates and
tracing cost.  Everything is judged in simulation time (/clock); wall time is
only used for the sim/wall ratio.

Scenarios (world-frame velocities, converted to the body frame with the true
yaw so a spinning chassis still translates along a straight line):
  static      hold still for --duration seconds
  straight    out-and-back along +x at --speed (default 1.5 m/s), --leg metres
  spin_walk   out-and-back along +x at --speed (default 1.0 m/s) while spinning
              at --spin rad/s (default 3.0)
  observe     send nothing (another driver, e.g. the planner, moves the robot)

Run with the stack already launched, e.g.
  ros2 launch ats_mujoco_sim lio_localization_test.launch.py
  python3 scripts/lio_eval.py --scenario static --duration 60 --out /tmp/lio_static
"""

import argparse
import json
import math
import time
from pathlib import Path

import numpy as np
import rclpy
from livox_ros_driver2.msg import CustomMsg
from manda_can_control.msg import MotionCtrl
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy

DEEP_SENSOR_QOS = QoSProfile(depth=2000, reliability=ReliabilityPolicy.BEST_EFFORT)
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import Imu
from std_msgs.msg import Float64MultiArray


def stamp_s(header):
    return header.stamp.sec + header.stamp.nanosec * 1e-9


def yaw_of(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def wrap(angle):
    return (angle + math.pi) % (2.0 * math.pi) - math.pi


def rate_of(times):
    return (len(times) - 1) / (times[-1] - times[0]) if len(times) > 2 else None


def pose_metrics(est_rows, truth_rows, args):
    """Drift, ATE and end-point error of /odometry against the ground truth."""
    result = {}
    if len(est_rows) < 5 or len(truth_rows) < 5:
        result["error"] = "not enough pose samples"
        return result
    est = np.array(est_rows)
    gt = np.array(truth_rows)
    result["estimate_messages"] = len(est)
    result["estimate_hz_sim"] = rate_of(est[:, 0].tolist())

    # Estimated pose in the world frame: odom is anchored at the known start
    # pose (x0, y0, yaw0) on the ground, which is also the map->odom seed.
    x0, y0, yaw0 = args.start_x, args.start_y, args.start_yaw
    c, s = math.cos(yaw0), math.sin(yaw0)
    wx = x0 + c * est[:, 1] - s * est[:, 2]
    wy = y0 + s * est[:, 1] + c * est[:, 2]
    wyaw = np.array([wrap(yaw0 + a) for a in est[:, 4]])
    # odom z=0 is the ground; the truth z is the base height in the world.
    ground_offset = gt[0, 3] - args.base_height
    wz_ = est[:, 3] + ground_offset
    gx = np.interp(est[:, 0], gt[:, 0], gt[:, 1])
    gy = np.interp(est[:, 0], gt[:, 0], gt[:, 2])
    gz = np.interp(est[:, 0], gt[:, 0], gt[:, 3])
    gyaw = np.interp(est[:, 0], gt[:, 0], np.unwrap(gt[:, 4]))
    planar = np.hypot(wx - gx, wy - gy)
    vertical = wz_ - gz
    yaw_err = np.array([abs(wrap(a - b)) for a, b in zip(wyaw, gyaw)])
    result["ate_known_start"] = {
        "planar_rmse_m": float(np.sqrt(np.mean(planar ** 2))),
        "planar_max_m": float(planar.max()),
        "z_rmse_m": float(np.sqrt(np.mean(vertical ** 2))),
        "z_max_abs_m": float(np.abs(vertical).max()),
        "yaw_rmse_rad": float(np.sqrt(np.mean(yaw_err ** 2))),
    }
    result["endpoint_error"] = {
        "planar_m": float(planar[-1]), "z_m": float(vertical[-1]),
        "yaw_rad": float(yaw_err[-1])}

    # Best-fit rigid alignment (Umeyama on x, y, z): isolates drift from the
    # start-pose assumption.
    src = np.stack([wx, wy, wz_], axis=1)
    dst = np.stack([gx, gy, gz], axis=1)
    mu_s, mu_d = src.mean(axis=0), dst.mean(axis=0)
    cov = (dst - mu_d).T @ (src - mu_s) / len(src)
    u, _, vt = np.linalg.svd(cov)
    sign = np.eye(3)
    if np.linalg.det(u) * np.linalg.det(vt) < 0:
        sign[2, 2] = -1
    rot = u @ sign @ vt
    aligned = (rot @ (src - mu_s).T).T + mu_d
    fit = np.linalg.norm(aligned - dst, axis=1)
    result["ate_best_fit"] = {"rmse_m": float(np.sqrt(np.mean(fit ** 2))),
                              "max_m": float(fit.max())}

    drift = np.linalg.norm(est[:, 1:4] - est[0, 1:4], axis=1)
    path = float(np.sum(np.hypot(np.diff(gx), np.diff(gy))))
    result["estimate_displacement_max_m"] = float(drift.max())
    result["estimate_displacement_final_m"] = float(drift[-1])
    result["truth_path_length_m"] = path
    result["drift_percent_of_path"] = float(100.0 * planar[-1] / path) if path > 1.0 else None
    return result


class Evaluator(Node):
    def __init__(self, args):
        super().__init__("lio_eval", parameter_overrides=[
            rclpy.parameter.Parameter("use_sim_time", value=True)])
        self.args = args
        self.est = []      # (t, x, y, z, yaw) in odom
        self.truth = []    # (t, x, y, z, yaw) in map
        self.imu_t = []
        self.lidar_t = []
        self.lidar_points = []
        self.lidar_offset_max = []
        self.trace_stats = []
        self.sim_time = 0.0
        self.wall_start = None
        self.sim_start = None
        self.create_subscription(Odometry, args.estimate_topic, self._est, 10)
        self.create_subscription(Odometry, args.truth_topic, self._truth, 10)
        self.create_subscription(Imu, "/livox/imu", self._imu, DEEP_SENSOR_QOS)
        self.create_subscription(CustomMsg, "/livox/lidar", self._lidar, DEEP_SENSOR_QOS)
        self.create_subscription(Float64MultiArray, "/simulation/lidar_frame_stats", self._stats, 10)
        self.create_subscription(Clock, "/clock", self._on_clock, 10)
        self.cmd_pub = self.create_publisher(MotionCtrl, "/motion_control", 10)
        self.timer = self.create_timer(0.02, self._drive)
        self.scenario_start = None
        self.done = False

    def _on_clock(self, msg):
        self.sim_time = msg.clock.sec + msg.clock.nanosec * 1e-9
        if self.wall_start is None:
            self.wall_start = time.perf_counter()
            self.sim_start = self.sim_time

    def _est(self, msg):
        p, q = msg.pose.pose.position, msg.pose.pose.orientation
        self.est.append((stamp_s(msg.header), p.x, p.y, p.z, yaw_of(q)))

    def _truth(self, msg):
        p, q = msg.pose.pose.position, msg.pose.pose.orientation
        self.truth.append((stamp_s(msg.header), p.x, p.y, p.z, yaw_of(q)))

    def _imu(self, msg):
        self.imu_t.append(stamp_s(msg.header))

    def _lidar(self, msg):
        self.lidar_t.append(stamp_s(msg.header))
        self.lidar_points.append(msg.point_num)
        if msg.points:
            self.lidar_offset_max.append(max(p.offset_time for p in msg.points))

    def _stats(self, msg):
        self.trace_stats.append(list(msg.data))

    # -- scenario driver -------------------------------------------------
    def _drive(self):
        args = self.args
        if self.truth == [] or args.scenario == "observe":
            return
        if self.scenario_start is None:
            self.scenario_start = self.sim_time + args.settle
        t = self.sim_time - self.scenario_start
        if t < 0.0:
            self._send(0.0, 0.0, 0.0)
            return
        yaw = self.truth[-1][4]
        vx_w, vy_w, wz = 0.0, 0.0, 0.0
        if args.scenario == "static":
            pass
        elif args.scenario in ("straight", "spin_walk"):
            speed = args.speed if args.speed is not None else (
                1.5 if args.scenario == "straight" else 1.0)
            # Triangle-wave out-and-back: --leg metres each way.
            leg_time = args.leg / speed
            phase = int(t // leg_time)
            vx_w = speed if phase % 2 == 0 else -speed
            if args.scenario == "spin_walk":
                wz = args.spin
        # world -> body
        c, s = math.cos(yaw), math.sin(yaw)
        self._send(c * vx_w + s * vy_w, -s * vx_w + c * vy_w, wz)
        if t >= args.duration and not self.done:
            self._send(0.0, 0.0, 0.0)
            self.done = True

    def _send(self, vx, vy, wz):
        msg = MotionCtrl()
        msg.linear_x, msg.linear_y, msg.angular_z = float(vx), float(vy), float(wz)
        self.cmd_pub.publish(msg)

    # -- metrics ---------------------------------------------------------
    def report(self):
        args = self.args
        result = {"scenario": args.scenario}
        sim_span = (self.sim_time - self.sim_start) if self.sim_start is not None else 0.0
        wall_span = (time.perf_counter() - self.wall_start) if self.wall_start else 0.0
        result["sim_seconds"] = sim_span
        result["sim_over_wall"] = sim_span / wall_span if wall_span > 0 else None

        rate = rate_of

        result["imu_hz_sim"] = rate(self.imu_t)
        result["imu_messages"] = len(self.imu_t)
        result["lidar_hz_sim"] = rate(self.lidar_t)
        result["lidar_frames"] = len(self.lidar_t)
        if self.lidar_points:
            pts = np.array(self.lidar_points)
            result["lidar_points_per_frame"] = {
                "mean": float(pts.mean()), "min": int(pts.min()), "max": int(pts.max())}
        if self.lidar_offset_max:
            result["lidar_offset_time_max_ms"] = max(self.lidar_offset_max) / 1e6
        if self.trace_stats:
            stats = np.array(self.trace_stats)
            result["trace_ms_per_frame"] = {
                "mean": float(stats[:, 1].mean()),
                "p95": float(np.percentile(stats[:, 1], 95)),
                "max": float(stats[:, 1].max())}
            result["wait_ms_per_frame_mean"] = float(stats[:, 2].mean())

        result.update(pose_metrics(self.est, self.truth, args))
        return result


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--scenario", choices=["static", "straight", "spin_walk", "observe"],
                        default="static")
    parser.add_argument("--duration", type=float, default=60.0, help="scenario seconds (sim time)")
    parser.add_argument("--settle", type=float, default=3.0, help="seconds before motion starts")
    parser.add_argument("--speed", type=float, default=None)
    parser.add_argument("--leg", type=float, default=4.0, help="out-and-back leg length, m")
    parser.add_argument("--spin", type=float, default=3.0)
    parser.add_argument("--start-x", type=float, default=4.0)
    parser.add_argument("--start-y", type=float, default=5.5)
    parser.add_argument("--start-yaw", type=float, default=0.0)
    parser.add_argument("--base-height", type=float, default=0.18,
                        help="base_footprint_height used by loam_interface")
    parser.add_argument("--estimate-topic", default="/odometry")
    parser.add_argument("--truth-topic", default="/simulation/ground_truth/odometry")
    parser.add_argument("--warmup", type=float, default=2.0,
                        help="sim seconds to wait for /clock and the chain before recording starts")
    parser.add_argument("--out", required=True, help="output prefix (writes .json and .csv)")
    parser.add_argument("--reanalyze", action="store_true",
                        help="recompute pose metrics from <out>.est.csv/.gt.csv without ROS")
    args = parser.parse_args()

    if args.reanalyze:
        out = Path(args.out)
        est = np.loadtxt(out.with_suffix(".est.csv"), delimiter=",", skiprows=1, ndmin=2)
        gt = np.loadtxt(out.with_suffix(".gt.csv"), delimiter=",", skiprows=1, ndmin=2)
        result = pose_metrics(est.tolist(), gt.tolist(), args)
        print(json.dumps(result, indent=2, sort_keys=True))
        return

    rclpy.init()
    node = Evaluator(args)
    deadline_wall = time.time() + 3600
    try:
        while rclpy.ok() and time.time() < deadline_wall:
            rclpy.spin_once(node, timeout_sec=0.05)
            if node.scenario_start is not None and node.sim_time - node.scenario_start > args.duration + 1.0:
                break
            if args.scenario == "observe" and node.sim_start is not None and \
                    node.sim_time - node.sim_start > args.duration:
                break
    except KeyboardInterrupt:
        pass
    result = node.report()
    out = Path(args.out)
    out.parent.mkdir(parents=True, exist_ok=True)
    out.with_suffix(".json").write_text(json.dumps(result, indent=2, sort_keys=True), "utf-8")
    np.savetxt(out.with_suffix(".est.csv"), np.array(node.est), delimiter=",",
               header="t,x,y,z,yaw", comments="")
    np.savetxt(out.with_suffix(".gt.csv"), np.array(node.truth), delimiter=",",
               header="t,x,y,z,yaw", comments="")
    print(json.dumps(result, indent=2, sort_keys=True))
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
