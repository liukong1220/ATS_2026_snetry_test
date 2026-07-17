#!/usr/bin/env python3
"""Exercise and evaluate the MuJoCo four-swerve actuator envelope."""

import argparse
import math
import json
from pathlib import Path
import time

from ats_navigation_interfaces.msg import SwerveTelemetry
from manda_can_control.msg import MotionCtrl
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool


WHEEL_NAMES = ("lf", "lr", "rf", "rr")
MAX_DRIVE_RPM = 450.0 / 1.2
MAX_STEER_RATE = 2.0 * 3.141592653589793 * 120.0 / (60.0 * 1.2)


class SwerveDynamicsEvaluator(Node):
    def __init__(self) -> None:
        super().__init__("swerve_dynamics_evaluator")
        self.samples = []
        self.motion_pub = self.create_publisher(MotionCtrl, "/motion_control", 10)
        self.stop_pub = self.create_publisher(Bool, "/planner/emergency_stop", 10)
        self.create_subscription(
            SwerveTelemetry,
            "/swerve/telemetry",
            self._on_telemetry,
            20,
        )

    def _on_telemetry(self, message: SwerveTelemetry) -> None:
        self.samples.append(message)

    def publish_stop(self, stop: bool) -> None:
        message = Bool()
        message.data = stop
        self.stop_pub.publish(message)

    def run_phase(
        self,
        name: str,
        command: tuple[float, float, float],
        duration: float,
        emergency_stop: bool = False,
    ) -> dict:
        start_index = len(self.samples)
        self.publish_stop(emergency_stop)
        deadline = time.monotonic() + duration
        next_publish = 0.0
        while time.monotonic() < deadline:
            now = time.monotonic()
            if now >= next_publish:
                message = MotionCtrl()
                message.linear_x = command[0]
                message.linear_y = command[1]
                message.angular_z = command[2]
                self.motion_pub.publish(message)
                self.publish_stop(emergency_stop)
                next_publish = now + 0.04
            rclpy.spin_once(self, timeout_sec=0.01)

        phase_samples = self.samples[start_index:]
        if not phase_samples:
            raise RuntimeError(f"{name}: no /swerve/telemetry samples")
        return summarize_phase(name, command, phase_samples)


def summarize_phase(name: str, command: tuple[float, float, float], samples) -> dict:
    def maximum(field: str) -> float:
        return max(float(getattr(sample, field)) for sample in samples)

    def maximum_abs_array(field: str) -> float:
        return max(
            abs(float(value)) for sample in samples for value in getattr(sample, field)
        )

    def percentile_abs_array(field: str, selected_samples, percentile: float) -> float:
        values = sorted(
            abs(float(value))
            for sample in selected_samples
            for value in getattr(sample, field)
        )
        rank = max(0, math.ceil(percentile * len(values)) - 1)
        return values[rank]

    first = samples[0]
    last = samples[-1]
    # 舵向切换会产生预期内的短暂滑移；用阶段末段 P95 评价稳定运动，峰值仍保留诊断。
    stable_start = min(len(samples) - 1, math.floor(len(samples) * 0.6))
    stable_samples = samples[stable_start:]
    return {
        "name": name,
        "command": list(command),
        "samples": len(samples),
        "max_drive_rpm": maximum_abs_array("drive_rpm"),
        "max_steer_rate_radps": maximum_abs_array("steer_rate"),
        "max_longitudinal_slip_mps": maximum_abs_array("longitudinal_slip_mps"),
        "max_lateral_slip_mps": maximum_abs_array("lateral_slip_mps"),
        "stable_samples": len(stable_samples),
        "stable_p95_longitudinal_slip_mps": percentile_abs_array(
            "longitudinal_slip_mps", stable_samples, 0.95
        ),
        "stable_p95_lateral_slip_mps": percentile_abs_array(
            "lateral_slip_mps", stable_samples, 0.95
        ),
        "max_measured_vx": maximum("measured_vx"),
        "max_measured_vy": maximum("measured_vy"),
        "max_measured_wz": maximum("measured_wz"),
        "min_measured_vy": min(float(sample.measured_vy) for sample in samples),
        "drive_speed_saturations": int(
            last.drive_speed_saturation_count - first.drive_speed_saturation_count
        ),
        "drive_acceleration_saturations": int(
            last.drive_acceleration_saturation_count
            - first.drive_acceleration_saturation_count
        ),
        "steer_rate_saturations": int(
            last.steer_rate_saturation_count - first.steer_rate_saturation_count
        ),
        "contact_violations": int(
            last.contact_violation_count - first.contact_violation_count
        ),
        "contact_violation_total": int(last.contact_violation_count),
        "max_contact_force": float(last.max_contact_force),
        "final_drive_rpm": [float(value) for value in last.drive_rpm],
        "final_command": [
            float(last.command_vx),
            float(last.command_vy),
            float(last.command_wz),
        ],
    }


def wait_for_telemetry(node: SwerveDynamicsEvaluator, timeout: float = 10.0) -> None:
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline and not node.samples:
        rclpy.spin_once(node, timeout_sec=0.1)
    if not node.samples:
        raise RuntimeError("timed out waiting for /swerve/telemetry")


def evaluate(phases: list[dict]) -> list[str]:
    failures = []
    for phase in phases:
        if phase["max_drive_rpm"] > MAX_DRIVE_RPM + 0.5:
            failures.append(f"{phase['name']}: drive RPM exceeded conservative limit")
        if phase["max_steer_rate_radps"] > MAX_STEER_RATE + 0.05:
            failures.append(f"{phase['name']}: steer rate exceeded conservative limit")
        if phase["contact_violations"] != 0:
            failures.append(f"{phase['name']}: non-ground physical contact detected")

    by_name = {phase["name"]: phase for phase in phases}
    if by_name["forward"]["max_measured_vx"] < 0.15:
        failures.append("forward: insufficient positive vx")
    if by_name["lateral"]["max_measured_vy"] < 0.12:
        failures.append("lateral: insufficient true positive vy")
    if (
        by_name["diagonal"]["max_measured_vx"] < 0.10
        or by_name["diagonal"]["max_measured_vy"] < 0.10
    ):
        failures.append("diagonal: vx/vy did not both respond")
    if by_name["rotation"]["max_measured_wz"] < 0.20:
        failures.append("rotation: insufficient positive wz")
    if (
        by_name["combined"]["max_measured_vx"] < 0.08
        or by_name["combined"]["max_measured_vy"] < 0.05
        or by_name["combined"]["max_measured_wz"] < 0.10
    ):
        failures.append("combined: vx/vy/wz did not all respond")
    if by_name["acceleration"]["drive_acceleration_saturations"] == 0:
        failures.append(
            "acceleration: per-wheel acceleration limiter was not exercised"
        )
    if by_name["steer_reverse"]["steer_rate_saturations"] == 0:
        failures.append("steer_reverse: steering rate limiter was not exercised")
    for name, longitudinal_limit, lateral_limit in (
        ("forward", 0.05, 0.02),
        ("lateral", 0.15, 0.08),
        ("diagonal", 0.10, 0.08),
        ("rotation", 0.15, 0.15),
        ("combined", 0.16, 0.08),
        ("acceleration", 0.10, 0.08),
    ):
        phase = by_name[name]
        if phase["stable_p95_longitudinal_slip_mps"] > longitudinal_limit:
            failures.append(
                f"{name}: stable P95 longitudinal slip exceeded {longitudinal_limit}"
            )
        if phase["stable_p95_lateral_slip_mps"] > lateral_limit:
            failures.append(f"{name}: stable P95 lateral slip exceeded {lateral_limit}")
    emergency = by_name["emergency_stop"]
    if any(abs(value) > 1e-6 for value in emergency["final_command"]):
        failures.append("emergency_stop: effective chassis command did not become zero")
    if any(abs(value) > 2.0 for value in emergency["final_drive_rpm"]):
        failures.append("emergency_stop: wheel RPM did not reach zero")
    return failures


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()

    rclpy.init()
    node = SwerveDynamicsEvaluator()
    try:
        wait_for_telemetry(node)
        phases = []
        node.publish_stop(False)
        phases.append(node.run_phase("settle", (0.0, 0.0, 0.0), 0.5))
        phases.append(node.run_phase("forward", (0.5, 0.0, 0.0), 2.0))
        phases.append(node.run_phase("forward_stop", (0.0, 0.0, 0.0), 0.8))
        phases.append(node.run_phase("lateral", (0.0, 0.5, 0.0), 2.5))
        phases.append(node.run_phase("lateral_stop", (0.0, 0.0, 0.0), 0.8))
        phases.append(node.run_phase("diagonal", (0.35, 0.35, 0.0), 2.2))
        phases.append(node.run_phase("diagonal_stop", (0.0, 0.0, 0.0), 0.8))
        phases.append(node.run_phase("rotation", (0.0, 0.0, 0.8), 2.5))
        phases.append(node.run_phase("rotation_stop", (0.0, 0.0, 0.0), 0.8))
        phases.append(node.run_phase("combined", (0.30, 0.20, 0.50), 2.5))
        phases.append(node.run_phase("combined_stop", (0.0, 0.0, 0.0), 0.8))
        phases.append(node.run_phase("acceleration", (1.5, 0.0, 0.0), 1.5))
        phases.append(node.run_phase("deceleration", (0.0, 0.0, 0.0), 1.2))
        phases.append(node.run_phase("steer_prepare", (0.4, 0.4, 0.0), 1.2))
        phases.append(node.run_phase("steer_reverse", (0.4, -0.4, 0.0), 2.0))
        phases.append(node.run_phase("pre_emergency", (0.8, 0.0, 0.3), 1.0))
        phases.append(node.run_phase("emergency_stop", (0.8, 0.0, 0.3), 0.5, True))
        node.publish_stop(False)
        report = {
            "wheel_order": list(WHEEL_NAMES),
            "max_drive_rpm": MAX_DRIVE_RPM,
            "max_steer_rate_radps": MAX_STEER_RATE,
            "phases": phases,
        }
        failures = evaluate(phases)
        report["failures"] = failures
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(
            json.dumps(report, ensure_ascii=False, indent=2) + "\n",
            encoding="utf-8",
        )
        print(json.dumps(report, ensure_ascii=False, indent=2))
        return 1 if failures else 0
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    raise SystemExit(main())
