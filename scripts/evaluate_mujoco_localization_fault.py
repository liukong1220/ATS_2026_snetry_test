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
from ats_navigation_interfaces.msg import ExecutionCommand
from ats_navigation_interfaces.msg import PlannerGoal
from ats_navigation_interfaces.msg import PlannerStatus
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
    def __init__(self, fault: str, relocalization_mode: str = "real") -> None:
        super().__init__("mujoco_localization_fault_evaluator")
        if relocalization_mode not in ("real", "synthetic"):
            raise ValueError("relocalization mode must be real or synthetic")
        if relocalization_mode == "real" and fault != "odometry_stale":
            raise ValueError("mutation faults require explicit synthetic mode")
        self.relocalization_mode = relocalization_mode
        self.stage = "initialization"
        self.execution_commands = []
        self.observations = []
        self.latest_cmd_received_at = 0.0
        self.baseline_epoch = None
        self.recovery_observation_floor = 0
        self.recovery_stamp_floor = 0
        self.fault_started_at = 0.0
        self.fault = fault
        self.relay_enabled = True
        self.raw_odometry = deque(maxlen=100)
        self.localization_statuses = []
        self.map_statuses = []
        self.planner_goals = []
        self.planner_statuses = []
        self.recovery_request_floor = 0
        self.recovery_planner_offset = 0
        self.recovery_status_offset = 0
        self.recovery_correlation_evidence = {}
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
        self.observation_pub = (
            self.create_publisher(
                RelocalizationObservation, "/relocalization_observation", 10
            ) if relocalization_mode == "synthetic" else None
        )
        self.create_subscription(
            RelocalizationObservation, "/relocalization_observation",
            self.observations.append, 10,
        )
        self.create_subscription(
            ExecutionCommand, "/planner/execution_command",
            self.execution_commands.append, transient_qos,
        )
        self.create_subscription(
            PlannerStatus, "/minco/planning_status", self.planner_statuses.append, 20,
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
        self.latest_cmd_received_at = time.monotonic()
        self.latest_cmd = (
            float(message.linear.x),
            float(message.linear.y),
            float(message.angular.z),
        )

    def _publish_maintenance_observation(self) -> None:
        if self.relocalization_mode != "synthetic":
            return
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
        self.stage = label
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
        if self.observation_pub is None:
            raise RuntimeError("synthetic observation forbidden in real GICP mode")
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

    def commands_are_zero(self, since: float = 0.0) -> bool:
        if self.latest_cmd is None or self.latest_cmd_received_at <= since:
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
        goal.goal_pose.pose.position.x = 1.0
        goal.goal_pose.pose.position.y = 0.06
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
            return sequence, self.baseline_epoch
        if self.fault == "tf_loss":
            self.wait_for(
                lambda: self.find_process("localization_fusion_node") is not None,
                3.0,
                "localization fusion process",
            )
            self.fusion_pid = self.find_process("localization_fusion_node")
            self.status_count_before_tf_loss = len(self.localization_statuses)
            os.kill(self.fusion_pid, signal.SIGSTOP)
            return sequence, self.baseline_epoch
        if self.fault == "epoch_jump":
            current = self.wait_for_fresh_raw()
            self.publish_observation(current, sequence, correction_x=0.20)
            return sequence + 1, self.baseline_epoch + 1
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
            return sequence, self.baseline_epoch

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
        return sequence, self.baseline_epoch

    @staticmethod
    def stamp_ns(stamp) -> int:
        return stamp.sec * 1_000_000_000 + stamp.nanosec

    def assert_topic_ownership(self, *, allow_missing: bool = False) -> bool:
        expected_observer = (
            self.get_name() if self.relocalization_mode == "synthetic"
            else "small_gicp_relocalization"
        )
        missing = []
        for topic, expected in (
            ("/odometry", self.get_name()),
            ("/odometry_raw", "ats_mujoco_sim"),
            ("/relocalization_observation", expected_observer),
        ):
            infos = list(self.get_publishers_info_by_topic(topic))
            names = [info.node_name for info in infos]
            if not names:
                missing.append(f"{topic} publisher {expected}")
                continue
            # Fast-DDS often reports a sole local publisher as _NODE_NAME_UNKNOWN_
            # before the participant graph resolves. That is not a second owner.
            # Multiple endpoints, or a single resolved foreign name, still fail closed.
            if len(names) == 1 and names[0] in (expected, "_NODE_NAME_UNKNOWN_"):
                continue
            if names == [expected]:
                continue
            raise RuntimeError(
                f"{topic} must have sole publisher {expected}; observed publishers={names!r}"
            )
        subscribers = [info.node_name for info in
                       self.get_subscriptions_info_by_topic("/odometry")]
        if "localization_fusion" not in subscribers and "_NODE_NAME_UNKNOWN_" not in subscribers:
            missing.append(
                f"/odometry subscriber localization_fusion; observed subscribers={subscribers!r}"
            )
        if missing and not allow_missing:
            raise RuntimeError("missing topic ownership prerequisites: " + "; ".join(missing))
        return not missing


    def fresh_real_observation(self) -> bool:
        status = self.latest_status()
        return status is not None and any(
            observation.accepted
            and observation.status == RelocalizationObservation.STATUS_ACCEPTED
            and observation.sequence > self.recovery_observation_floor
            and observation.sequence == status.observation_sequence
            and self.stamp_ns(observation.header.stamp) > self.recovery_stamp_floor
            for observation in self.observations
        )

    @staticmethod
    def candidate_digest(content: str) -> bytes:
        # Match Goal Manager fillCandidateDigest: complete hex pairs, then zero-fill.
        result = bytearray(32)
        for index in range(min(len(content) // 2, len(result))):
            pair = content[index * 2:index * 2 + 2]
            if any(character not in "0123456789abcdefABCDEF" for character in pair):
                break
            result[index] = int(pair, 16)
        return bytes(result)

    @staticmethod
    def execution_identity(command) -> dict:
        return {
            "mode": int(command.mode),
            "manager_incarnation": int(command.manager_incarnation),
            "command_sequence": int(command.command_sequence),
            "goal_id": int(command.goal_id),
            "localization_epoch": int(command.localization_epoch),
            "map_generation": int(command.map_generation),
            "map_publication_sequence": int(command.map_publication_sequence),
            "planner_incarnation": int(command.planner_incarnation),
            "planner_candidate_sequence": int(command.planner_candidate_sequence),
            "candidate_content_digest": bytes(command.candidate_content_digest).hex(),
            "reference_stamp_ns": LocalizationFaultEvaluator.stamp_ns(command.reference.header.stamp),
        }

    def correlated_plan_status(self, command, stopped):
        stop_stamp = self.stamp_ns(stopped.header.stamp)
        for status in reversed(self.planner_statuses[self.recovery_status_offset:]):
            if not (
                status.state == PlannerStatus.STATE_REFERENCE_READY
                and status.failure_reason == PlannerStatus.FAILURE_NONE
                and status.goal_id == command.goal_id
                and status.localization_epoch == command.localization_epoch
                and status.map_generation == command.map_generation
                and status.map_publication_sequence == command.map_publication_sequence
                and status.plan_request_sequence > self.recovery_request_floor
                and stop_stamp < self.stamp_ns(status.reference_stamp)
                <= self.stamp_ns(command.reference.header.stamp)
                and status.content_digest
                and self.candidate_digest(status.content_digest) == bytes(command.candidate_content_digest)
            ):
                continue
            if any(
                request.goal_id == status.goal_id
                and request.localization_epoch == status.localization_epoch
                and request.plan_request_sequence == status.plan_request_sequence
                and request.map_publication_sequence == status.map_publication_sequence
                and stop_stamp < self.stamp_ns(request.header.stamp)
                <= self.stamp_ns(status.reference_stamp)
                for request in self.planner_goals[self.recovery_planner_offset:]
            ):
                return status
        return None

    def fresh_execution(self, offset: int, stopped, epoch: int):
        rejected = []
        for command in reversed(self.execution_commands[offset:]):
            if command.mode != ExecutionCommand.MODE_EXECUTE:
                continue
            checks = {
                "manager_incarnation": command.manager_incarnation == stopped.manager_incarnation,
                "post_stop_command_sequence": command.command_sequence > stopped.command_sequence,
                "same_nonzero_goal": command.goal_id == stopped.goal_id and command.goal_id != 0,
                "current_localization_epoch": command.localization_epoch == epoch,
                "reference_poses": len(command.reference.poses) >= 2,
                "post_stop_reference_stamp": self.stamp_ns(command.reference.header.stamp)
                    > self.stamp_ns(stopped.header.stamp),
                "fresh_request_ready_status_digest": self.correlated_plan_status(command, stopped) is not None,
            }
            failures = [name for name, passed in checks.items() if not passed]
            if not failures:
                self.recovery_correlation_evidence = {
                    "expected_epoch": int(epoch), "request_floor": int(self.recovery_request_floor),
                    "stopped": self.execution_identity(stopped),
                    "accepted": self.execution_identity(command),
                }
                return command
            if len(rejected) < 8:
                rejected.append({"identity": self.execution_identity(command), "rejected_by": failures})
        self.recovery_correlation_evidence = {
            "expected_epoch": int(epoch), "request_floor": int(self.recovery_request_floor),
            "stopped": self.execution_identity(stopped), "rejected_executions": rejected,
            "planner_goals": [
                {"goal_id": int(request.goal_id), "epoch": int(request.localization_epoch),
                 "request_sequence": int(request.plan_request_sequence),
                 "map_publication_sequence": int(request.map_publication_sequence)}
                for request in self.planner_goals[self.recovery_planner_offset:][-8:]
            ],
            "planner_statuses": [
                {"goal_id": int(status.goal_id), "epoch": int(status.localization_epoch),
                 "request_sequence": int(status.plan_request_sequence), "state": int(status.state),
                 "map_generation": int(status.map_generation),
                 "map_publication_sequence": int(status.map_publication_sequence),
                 "content_digest": status.content_digest,
                 "reference_stamp_ns": self.stamp_ns(status.reference_stamp)}
                for status in self.planner_statuses[self.recovery_status_offset:][-8:]
            ],
        }
        return None

    def recover(self, sequence: int, expected_epoch: int) -> int:
        self.assert_topic_ownership()
        if self.relocalization_mode == "real":
            self.recovery_observation_floor = self.latest_status().observation_sequence
            self.recovery_stamp_floor = self.stamp_ns(self.raw_odometry[-1].header.stamp)
            self.relay_enabled = True
            self.wait_for(
                lambda: self.latest_status().state == LocalizationStatus.STATE_TRACKING
                and self.latest_status().odometry_silence_sec < 0.5
                and self.fresh_real_observation(),
                30.0,
                "fresh real GICP observation accepted by fusion after relay recovery",
            )
            return sequence
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
            lambda: self.assert_topic_ownership(allow_missing=True)
            and self.latest_status() is not None,
            30.0, "localization fusion status and topic ownership discovery",
        )
        if self.relocalization_mode == "synthetic":
            # Maintain the accepted-observation lease before waiting for map startup.
            self.maintenance_sequence = 1
        self.wait_for(
            lambda: self.latest_status().state == LocalizationStatus.STATE_TRACKING
            and self.latest_status().observation_sequence > 0
            and (self.relocalization_mode == "synthetic" or self.fresh_real_observation()),
            30.0,
            "initial accepted observation and localization tracking",
        )
        self.wait_for(
            lambda: self.latest_map_status() is not None
            and self.latest_map_status().ready
            and self.latest_status().state == LocalizationStatus.STATE_TRACKING
            and self.latest_map_status().localization_epoch == self.latest_status().epoch,
            120.0,
            "initial current-epoch planning map",
        )
        initial_commands = len(self.execution_commands)
        _, result_future = self.send_goal()
        self.wait_for(
            lambda: self.command_norm(self.latest_cmd) > 0.02
            and any(command.mode == ExecutionCommand.MODE_EXECUTE
                    for command in self.execution_commands[initial_commands:]),
            30.0,
            "initial authorized non-zero control",
        )
        self.assert_topic_ownership()
        self.baseline_epoch = self.latest_status().epoch
        baseline_observation_sequence = self.latest_status().observation_sequence
        sequence = self.maintenance_sequence or 1
        self.maintenance_sequence = None
        reference_count_before = len(self.references)
        planner_count_before = len(self.planner_goals)
        stop_count_before = len(self.stop_states)
        command_count_before = len(self.execution_commands)
        initial_execute = next(command for command in reversed(self.execution_commands)
                               if command.mode == ExecutionCommand.MODE_EXECUTE)
        raw_stamp_before = self.stamp_ns(self.raw_odometry[-1].header.stamp)
        self.fault_started_at = time.monotonic()
        sequence, expected_epoch = self.inject_fault(sequence)
        if self.fault == "epoch_jump":
            self.wait_for(lambda: self.latest_status().epoch == expected_epoch,
                          3.0, "localization epoch jump")
        elif self.fault != "tf_loss":
            expected_state = (LocalizationStatus.STATE_LOST if self.fault == "odometry_stale"
                              else LocalizationStatus.STATE_DEGRADED)
            self.wait_for(lambda: self.latest_status().state == expected_state,
                          3.0, "localization fault state")
        if self.fault == "odometry_stale":
            self.wait_for(
                lambda: self.stamp_ns(self.raw_odometry[-1].header.stamp) > raw_stamp_before
                and self.latest_status().odometry_silence_sec > 0.5,
                3.0, "raw odometry still live while fusion input is stale",
            )
        self.wait_for(lambda: True in self.stop_states[stop_count_before:],
                      3.0, "fresh emergency stop")
        self.wait_for(
            lambda: any(command.mode == ExecutionCommand.MODE_STOP
                        and command.manager_incarnation == initial_execute.manager_incarnation
                        and command.command_sequence > initial_execute.command_sequence
                        and command.goal_id == initial_execute.goal_id
                        for command in self.execution_commands[command_count_before:]),
            3.0, "fresh correlated execution STOP",
        )
        stopped = next(command for command in reversed(self.execution_commands)
                       if command.mode == ExecutionCommand.MODE_STOP
                       and command.goal_id == initial_execute.goal_id)
        self.wait_for(lambda: self.commands_are_zero(self.fault_started_at),
                      3.0, "fresh /cmd_vel/selected zero after localization fault")
        stopped_at = self.latest_cmd_received_at
        fault_status = self.latest_status()
        stop_reference_count = len(self.references)
        stop_command_count = len(self.execution_commands)
        hold_deadline = time.monotonic() + 0.4
        while time.monotonic() < hold_deadline:
            rclpy.spin_once(self, timeout_sec=0.05)
            if (self.fault not in ("odometry_stale", "tf_loss")
                    and self.latest_status().state == LocalizationStatus.STATE_TRACKING):
                break
            if (len(self.references) != stop_reference_count
                    or not self.commands_are_zero(self.fault_started_at)
                    or any(command.mode == ExecutionCommand.MODE_EXECUTE
                           for command in self.execution_commands[stop_command_count:])):
                raise RuntimeError("old reference revived while localization was unhealthy")
        recovery_command_count = len(self.execution_commands)
        recovery_planner_count = len(self.planner_goals)
        self.recovery_planner_offset = recovery_planner_count
        self.recovery_status_offset = len(self.planner_statuses)
        self.recovery_request_floor = max(
            (request.plan_request_sequence for request in self.planner_goals
             if request.goal_id == initial_execute.goal_id), default=0)
        if self.recovery_request_floor == 0:
            raise RuntimeError("missing baseline PlannerGoal request identity")
        sequence = self.recover(sequence, expected_epoch)
        if self.relocalization_mode == "real":
            expected_epoch = self.latest_status().epoch
        recovered_observation_sequence = self.latest_status().observation_sequence
        def recovery_epoch():
            return self.latest_status().epoch if self.relocalization_mode == "real" else expected_epoch
        self.wait_for(
            lambda: self.latest_map_status() is not None
            and self.latest_map_status().ready
            and self.latest_status().state == LocalizationStatus.STATE_TRACKING
            and self.latest_map_status().localization_epoch == recovery_epoch(),
            30.0, "recovered planning map",
        )
        self.wait_for(
            lambda: result_future.done() or (
                len(self.planner_goals) > recovery_planner_count
                and self.planner_goals[-1].localization_epoch == recovery_epoch()),
            10.0, "fresh replanned goal",
        )
        if result_future.done():
            early = result_future.result()
            raise RuntimeError(
                "goal terminated before localization recovery replan: "
                f"code={early.result.result_code} message={early.result.message!r}"
            )
        self.wait_for(
            lambda: len(self.references) > stop_reference_count
            and self.latest_status().state == LocalizationStatus.STATE_TRACKING
            and self.fresh_execution(recovery_command_count, stopped, recovery_epoch()) is not None,
            30.0, "fresh correlated execution authorization after recovery",
        )
        recovered_execute = self.fresh_execution(recovery_command_count, stopped, recovery_epoch())
        expected_epoch = recovered_execute.localization_epoch
        recovered_plan = self.correlated_plan_status(recovered_execute, stopped)
        authorized_at = time.monotonic()
        self.wait_for(lambda: self.command_norm(self.latest_cmd) > 0.02
                      and self.latest_cmd_received_at > authorized_at,
                      15.0, "fresh motion after recovery authorization")
        self.wait_for(lambda: result_future.done(), 120.0, "recovered goal result")
        wrapped = result_future.result()
        if wrapped is None or wrapped.result.result_code != NavigateToPose.Result.RESULT_SUCCEEDED:
            raise RuntimeError(f"goal did not succeed after localization recovery: {wrapped}")
        completed_at = time.monotonic()
        self.wait_for(lambda: self.commands_are_zero(completed_at),
                      3.0, "fresh goal completion /cmd_vel/selected zero")
        self.assert_topic_ownership()
        final_pose = wrapped.result.final_pose.pose.position
        return {
            "status": "passed",
            "fault": self.fault,
            "relocalization_mode": self.relocalization_mode,
            "synthetic_observation_assistance": self.relocalization_mode == "synthetic",
            "real_gicp_recovery_verified": self.relocalization_mode == "real",
            "baseline_epoch": int(self.baseline_epoch),
            "baseline_observation_sequence": int(baseline_observation_sequence),
            "recovered_observation_sequence": int(recovered_observation_sequence),
            "recovery_observation_stamp_floor_ns": self.recovery_stamp_floor,
            "fault_localization_state": int(fault_status.state),
            "fault_odometry_silence_sec": float(fault_status.odometry_silence_sec),
            "fault_stop_latency_sec": stopped_at - self.fault_started_at,
            "stop_command_sequence": int(stopped.command_sequence),
            "recovery_command_sequence": int(recovered_execute.command_sequence),
            "baseline_plan_request_sequence": int(self.recovery_request_floor),
            "recovery_plan_request_sequence": int(recovered_plan.plan_request_sequence),
            "recovery_correlation": self.recovery_correlation_evidence,
            "recovery_goal_id": int(recovered_execute.goal_id),
            "expected_epoch": int(expected_epoch),
            "final_epoch": int(self.latest_status().epoch),
            "final_result_code": int(wrapped.result.result_code),
            "final_x": float(final_pose.x),
            "final_y": float(final_pose.y),
            "final_distance": float(wrapped.result.final_distance),
            "references_before_fault": reference_count_before,
            "references_after_recovery": len(self.references),
            "planner_goals_before_fault": planner_count_before,
            "planner_goals_after_recovery": len(self.planner_goals),
        }


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--fault", choices=FAULTS, required=True)
    parser.add_argument("--relocalization-mode", choices=("real", "synthetic"), default="real")
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    rclpy.init()
    node = None
    exit_code = 1
    try:
        node = LocalizationFaultEvaluator(args.fault, args.relocalization_mode)
        result = node.run()
        exit_code = 0
    except Exception as error:
        status = node.latest_status() if node is not None else None
        result = {
            "status": "failed", "fault": args.fault,
            "relocalization_mode": args.relocalization_mode,
            "synthetic_observation_assistance": args.relocalization_mode == "synthetic",
            "real_gicp_recovery_verified": False,
            "stage": node.stage if node is not None else "initialization",
            "error": f"{type(error).__name__}: {error}",
            "raw_odometry_count": len(node.raw_odometry) if node is not None else 0,
            "localization_state": getattr(status, "state", None),
            "localization_epoch": getattr(status, "epoch", None),
            "observation_sequence": getattr(status, "observation_sequence", None),
            "odometry_silence_sec": getattr(status, "odometry_silence_sec", None),
            "latest_selected_command": node.latest_cmd if node is not None else None,
            "recovery_correlation": node.recovery_correlation_evidence if node is not None else {},
        }
    finally:
        if node is not None:
            node.resume_fusion()
            node.destroy_node()
        rclpy.shutdown()
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(result, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
    print(json.dumps(result, ensure_ascii=False, indent=2))
    return exit_code


if __name__ == "__main__":
    raise SystemExit(main())
