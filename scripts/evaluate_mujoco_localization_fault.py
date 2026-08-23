#!/usr/bin/env python3
"""Inject localization faults into the full Nav2-free MuJoCo chain."""

import argparse
from collections import deque
from copy import deepcopy
import json
import math
import os
from pathlib import Path
import signal
import time

from ats_navigation_interfaces.action import NavigateToPose
from ats_navigation_interfaces.msg import LocalizationStatus
from ats_navigation_interfaces.msg import PlannerGoal
from ats_navigation_interfaces.msg import PlanningMapStatus
from ats_navigation_interfaces.msg import RelocalizationObservation
from nav_msgs.msg import Odometry
from nav_msgs.msg import Path as NavPath
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy
from rclpy.qos import QoSProfile
from rclpy.qos import ReliabilityPolicy
from rclpy.qos import qos_profile_sensor_data
from std_msgs.msg import Bool
from geometry_msgs.msg import Twist


FAULTS = (
    "odometry_stale",
    "delayed",
    "gicp_rejected",
    "false_match",
    "epoch_jump",
    "tf_loss",
)


class LocalizationFaultEvaluator(Node):
    def __init__(self, fault: str) -> None:
        super().__init__("mujoco_localization_fault_evaluator")
        self.fault = fault
        self.relay_enabled = True
        self.raw_odometry = deque(maxlen=100)
        self.localization_statuses = []
        self.map_statuses = []
        self.planner_goals = []
        self.references = []
        self.stop_states = []
        self.latest_cmd = None
        self.maintenance_sequence = None
        self.maintenance_correction_x = 0.0
        self.fusion_pid = None
        self.status_count_before_tf_loss = 0

        transient_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.odom_pub = self.create_publisher(
            Odometry, "/odometry", qos_profile_sensor_data
        )
        self.observation_pub = self.create_publisher(
            RelocalizationObservation, "/relocalization_observation", 10
        )
        self.create_subscription(
            Odometry, "/odometry_raw", self._on_raw_odometry, qos_profile_sensor_data
        )
        self.create_subscription(
            LocalizationStatus,
            "/localization/status",
            self.localization_statuses.append,
            transient_qos,
        )
        self.create_subscription(
            PlanningMapStatus,
            "/rog_map_adapter/status",
            self.map_statuses.append,
            transient_qos,
        )
        self.create_subscription(
            PlannerGoal,
            "/ats_goal_manager/planner_goal",
            self.planner_goals.append,
            10,
        )
        self.create_subscription(
            NavPath, "/minco/reference_path", self.references.append, 10
        )
        self.create_subscription(
            Bool,
            "/planner/emergency_stop",
            lambda message: self.stop_states.append(message.data),
            transient_qos,
        )
        self.create_subscription(Twist, "/cmd_vel/selected", self._on_command, 20)
        self.action_client = ActionClient(self, NavigateToPose, "/ats_navigate_to_pose")
        self.create_timer(0.5, self._publish_maintenance_observation)

    def _on_raw_odometry(self, message: Odometry) -> None:
        self.raw_odometry.append(message)
        if self.relay_enabled:
            self.odom_pub.publish(message)

    def _on_command(self, message: Twist) -> None:
        self.latest_cmd = (
            float(message.linear.x),
            float(message.linear.y),
            float(message.angular.z),
        )

    def _publish_maintenance_observation(self) -> None:
        if self.maintenance_sequence is None or not self.raw_odometry:
            return
        self.publish_observation(
            self.raw_odometry[-1],
            self.maintenance_sequence,
            correction_x=self.maintenance_correction_x,
        )
        self.maintenance_sequence += 1

    @staticmethod
    def find_process(name: str):
        for entry in Path("/proc").iterdir():
            if not entry.name.isdigit():
                continue
            try:
                command = (entry / "cmdline").read_bytes().replace(b"\0", b" ")
            except (FileNotFoundError, PermissionError, ProcessLookupError):
                continue
            if name.encode() in command:
                return int(entry.name)
        return None

    def resume_fusion(self) -> None:
        if self.fusion_pid is None:
            return
        try:
            os.kill(self.fusion_pid, signal.SIGCONT)
        except ProcessLookupError:
            pass
        self.fusion_pid = None

    def wait_for(self, predicate, timeout: float, label: str) -> None:
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.05)
            if predicate():
                return
        raise RuntimeError(f"timeout waiting for {label}")

    def latest_status(self):
        return self.localization_statuses[-1] if self.localization_statuses else None

    def latest_map_status(self):
        return self.map_statuses[-1] if self.map_statuses else None

    def wait_for_fresh_raw(self, previous_stamp=None, timeout: float = 3.0):
        def fresh():
            if not self.raw_odometry:
                return False
            if previous_stamp is None:
                return True
            current = self.raw_odometry[-1].header.stamp
            return (current.sec, current.nanosec) > previous_stamp

        self.wait_for(fresh, timeout, "fresh /odometry_raw")
        return self.raw_odometry[-1]

    def publish_observation(
        self,
        odometry: Odometry,
        sequence: int,
        correction_x: float = 0.0,
        accepted: bool = True,
        status: int = RelocalizationObservation.STATUS_ACCEPTED,
        message: str = "synthetic accepted observation",
    ) -> None:
        observation = RelocalizationObservation()
        observation.header.stamp = odometry.header.stamp
        observation.header.frame_id = "map"
        observation.child_frame_id = "gimbal_yaw_odom"
        observation.sequence = sequence
        observation.accepted = accepted
        observation.status = status
        observation.inlier_count = 500
        observation.source_points = 1000
        observation.registration_error = 1.0
        observation.quality = 0.5
        observation.message = message
        observation.pose.pose = deepcopy(odometry.pose.pose)
        observation.pose.pose.position.x += correction_x
        self.observation_pub.publish(observation)

    @staticmethod
    def command_norm(command) -> float:
        return math.hypot(command[0], command[1]) + abs(command[2]) if command else 0.0

    def commands_are_zero(self) -> bool:
        if self.latest_cmd is None:
            return False
        return self.command_norm(self.latest_cmd) < 1e-3

    def send_goal(self):
        self.wait_for(
            lambda: self.action_client.wait_for_server(timeout_sec=0.1),
            60.0,
            "ATS action server",
        )
        goal = NavigateToPose.Goal()
        goal.goal_pose.header.frame_id = "map"
        goal.goal_pose.pose.position.x = -9.0
        goal.goal_pose.pose.position.y = 1.47
        goal.goal_pose.pose.orientation.w = 1.0
        goal.timeout.sec = 120
        send_future = self.action_client.send_goal_async(goal)
        self.wait_for(lambda: send_future.done(), 10.0, "goal acceptance")
        handle = send_future.result()
        if handle is None or not handle.accepted:
            raise RuntimeError("ATS action goal was rejected")
        return handle, handle.get_result_async()

    def inject_fault(self, sequence: int) -> tuple[int, int]:
        # 每次注入都使用真实 odometry stamp；只有 delayed 用已确认的旧样本。
        if self.fault == "odometry_stale":
            self.relay_enabled = False
            return sequence, 1
        if self.fault == "tf_loss":
            self.wait_for(
                lambda: self.find_process("localization_fusion_node") is not None,
                3.0,
                "localization fusion process",
            )
            self.fusion_pid = self.find_process("localization_fusion_node")
            self.status_count_before_tf_loss = len(self.localization_statuses)
            os.kill(self.fusion_pid, signal.SIGSTOP)
            return sequence, 1
        if self.fault == "epoch_jump":
            current = self.wait_for_fresh_raw()
            self.publish_observation(current, sequence, correction_x=0.20)
            return sequence + 1, 2
        if self.fault == "delayed":
            if len(self.raw_odometry) < 5:
                raise RuntimeError(
                    "insufficient odometry history for delayed injection"
                )
            delayed = self.raw_odometry[0]
            for _ in range(3):
                self.publish_observation(delayed, sequence)
                sequence += 1
                time.sleep(0.05)
            return sequence, 1

        correction = 3.0 if self.fault == "false_match" else 0.0
        accepted = self.fault == "false_match"
        observation_status = (
            RelocalizationObservation.STATUS_ACCEPTED
            if accepted
            else RelocalizationObservation.STATUS_REJECTED
        )
        for _ in range(3):
            previous = None
            if self.raw_odometry:
                stamp = self.raw_odometry[-1].header.stamp
                previous = (stamp.sec, stamp.nanosec)
            current = self.wait_for_fresh_raw(previous)
            self.publish_observation(
                current,
                sequence,
                correction_x=correction,
                accepted=accepted,
                status=observation_status,
                message=f"synthetic {self.fault}",
            )
            sequence += 1
        return sequence, 1

    def recover(self, sequence: int, expected_epoch: int) -> int:
        # stale 恢复 relay；其他拒绝类故障用可信同位姿观测清零 rejection 计数。
        if self.fault == "odometry_stale":
            self.relay_enabled = True
        if self.fault == "tf_loss":
            self.resume_fusion()
            self.wait_for(
                lambda: len(self.localization_statuses)
                > self.status_count_before_tf_loss
                and self.latest_status().odometry_silence_sec < 0.5,
                5.0,
                "fresh odometry status after TF recovery",
            )
        if self.fault != "epoch_jump":
            current = self.wait_for_fresh_raw()
            self.publish_observation(current, sequence)
            sequence += 1
        self.wait_for(
            lambda: self.latest_status() is not None
            and self.latest_status().state == LocalizationStatus.STATE_TRACKING
            and self.latest_status().epoch == expected_epoch,
            5.0,
            "localization recovery",
        )
        self.maintenance_sequence = sequence
        self.maintenance_correction_x = 0.20 if self.fault == "epoch_jump" else 0.0
        return sequence

    def run(self) -> dict:
        self.wait_for(lambda: bool(self.raw_odometry), 30.0, "/odometry_raw")
        self.wait_for(
            lambda: self.latest_status() is not None
            and self.latest_status().state == LocalizationStatus.STATE_TRACKING
            and self.latest_status().epoch == 1,
            30.0,
            "initial localization tracking",
        )
        self.wait_for(
            lambda: self.latest_map_status() is not None
            and self.latest_map_status().ready
            and self.latest_map_status().localization_epoch == 1,
            120.0,
            "initial epoch-1 planning map",
        )

        handle, result_future = self.send_goal()
        self.wait_for(
            lambda: self.command_norm(self.latest_cmd) > 0.02,
            30.0,
            "initial non-zero control",
        )

        current = self.wait_for_fresh_raw()
        sequence = 1
        self.publish_observation(current, sequence)
        sequence += 1
        self.wait_for(
            lambda: self.latest_status() is not None
            and self.latest_status().state == LocalizationStatus.STATE_TRACKING
            and self.latest_status().observation_sequence == 1,
            3.0,
            "baseline accepted observation",
        )
        reference_count_before = len(self.references)
        planner_count_before = len(self.planner_goals)

        sequence, expected_epoch = self.inject_fault(sequence)
        if self.fault == "epoch_jump":
            self.wait_for(
                lambda: self.latest_status() is not None
                and self.latest_status().epoch == 2,
                3.0,
                "localization epoch jump",
            )
        elif self.fault != "tf_loss":
            expected_state = (
                LocalizationStatus.STATE_LOST
                if self.fault == "odometry_stale"
                else LocalizationStatus.STATE_DEGRADED
            )
            self.wait_for(
                lambda: self.latest_status() is not None
                and self.latest_status().state == expected_state,
                3.0,
                "localization fault state",
            )

        stop_count = len(self.stop_states)
        stop_search_start = max(0, stop_count - 1)
        self.wait_for(
            lambda: True in self.stop_states[stop_search_start:],
            3.0,
            "emergency stop",
        )
        self.wait_for(
            self.commands_are_zero,
            3.0,
            "/cmd_vel/selected zero after localization fault",
        )
        stop_reference_count = len(self.references)
        hold_deadline = time.monotonic() + 0.4
        while time.monotonic() < hold_deadline:
            rclpy.spin_once(self, timeout_sec=0.05)
            status = self.latest_status()
            if (
                self.fault != "tf_loss"
                and status is not None
                and status.state == LocalizationStatus.STATE_TRACKING
            ):
                break
            if (
                len(self.references) != stop_reference_count
                or not self.commands_are_zero()
            ):
                raise RuntimeError(
                    "old reference revived while localization was unhealthy"
                )

        sequence = self.recover(sequence, expected_epoch)
        self.wait_for(
            lambda: self.latest_map_status() is not None
            and self.latest_map_status().ready
            and self.latest_map_status().localization_epoch == expected_epoch,
            30.0,
            "recovered planning map",
        )
        self.wait_for(
            lambda: result_future.done()
            or (
                len(self.planner_goals) > planner_count_before
                and self.planner_goals[-1].localization_epoch == expected_epoch
            ),
            10.0,
            "replanned goal",
        )
        if result_future.done():
            early_result = result_future.result()
            latest_status = self.latest_status()
            latest_map_status = self.latest_map_status()
            raise RuntimeError(
                "goal terminated before localization recovery replan: "
                f"code={early_result.result.result_code} "
                f"message={early_result.result.message!r} "
                f"localization_state={getattr(latest_status, 'state', None)} "
                f"localization_epoch={getattr(latest_status, 'epoch', None)} "
                f"map_ready={getattr(latest_map_status, 'ready', None)} "
                "map_localization_epoch="
                f"{getattr(latest_map_status, 'localization_epoch', None)}"
            )
        self.wait_for(
            lambda: len(self.references) > reference_count_before,
            30.0,
            "new reference after recovery",
        )
        self.wait_for(
            lambda: self.command_norm(self.latest_cmd) > 0.02,
            15.0,
            "motion after fresh reference",
        )
        self.wait_for(lambda: result_future.done(), 120.0, "recovered goal result")
        wrapped = result_future.result()
        if (
            wrapped is None
            or wrapped.result.result_code != NavigateToPose.Result.RESULT_SUCCEEDED
        ):
            result_code = None if wrapped is None else wrapped.result.result_code
            result_message = None if wrapped is None else wrapped.result.message
            raise RuntimeError(
                "goal did not succeed after localization recovery: "
                f"code={result_code} message={result_message!r}"
            )
        self.wait_for(
            self.commands_are_zero,
            3.0,
            "goal completion /cmd_vel/selected zero",
        )

        final_pose = wrapped.result.final_pose.pose.position
        return {
            "fault": self.fault,
            "expected_epoch": expected_epoch,
            "final_epoch": int(self.latest_status().epoch),
            "final_result_code": int(wrapped.result.result_code),
            "final_x": float(final_pose.x),
            "final_y": float(final_pose.y),
            "final_distance": float(wrapped.result.final_distance),
            "references_before_fault": reference_count_before,
            "references_after_recovery": len(self.references),
            "planner_goals_before_fault": planner_count_before,
            "planner_goals_after_recovery": len(self.planner_goals),
            "sequence_after_test": self.maintenance_sequence or sequence,
        }


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--fault", choices=FAULTS, required=True)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()

    rclpy.init()
    node = LocalizationFaultEvaluator(args.fault)
    try:
        result = node.run()
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(
            json.dumps(result, ensure_ascii=False, indent=2) + "\n",
            encoding="utf-8",
        )
        print(json.dumps(result, ensure_ascii=False, indent=2))
        return 0
    finally:
        node.resume_fusion()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    raise SystemExit(main())
