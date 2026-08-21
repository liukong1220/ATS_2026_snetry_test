#!/usr/bin/env python3
"""Exercise P4 runtime swept-footprint fail-stop behavior in MuJoCo."""

import argparse
import json
import math
import time
from pathlib import Path

from ats_navigation_interfaces.action import NavigateToPose
from ats_navigation_interfaces.msg import ExecutionCommand
from ats_navigation_interfaces.msg import PlannerStatus
from ats_navigation_interfaces.msg import PlanningMapStatus
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rcl_interfaces.srv import SetParameters
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import DurabilityPolicy
from rclpy.qos import QoSProfile
from rclpy.qos import ReliabilityPolicy
from rclpy.qos import qos_profile_sensor_data
from std_msgs.msg import Bool


FAULTS = (
    "mid_segment",
    "pure_rotation",
    "unknown",
    "outside",
    "map_after_commit",
    "old_generation",
    "repair_after_unsafe",
)


class UnsafeTrajectoryEvaluator(Node):
    """Inject only through test-only adapter parameters after a real execute commit."""

    def __init__(self, fault: str) -> None:
        super().__init__("mujoco_unsafe_trajectory_evaluator")
        self.fault = fault
        self.localization = None
        self.map_statuses = []
        self.execution_commands = []
        self.planner_statuses = []
        self.stop_states = []
        self.latest_cmd = None
        self.transient_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.create_subscription(
            Odometry, "/localization", self._on_localization, qos_profile_sensor_data
        )
        self.create_subscription(
            PlanningMapStatus,
            "/rog_map_adapter/status",
            self.map_statuses.append,
            self.transient_qos,
        )
        self.create_subscription(
            ExecutionCommand,
            "/planner/execution_command",
            self.execution_commands.append,
            self.transient_qos,
        )
        self.create_subscription(
            PlannerStatus, "/minco/planning_status", self.planner_statuses.append, 20
        )
        self.create_subscription(
            Bool,
            "/planner/emergency_stop",
            lambda message: self.stop_states.append(message.data),
            self.transient_qos,
        )
        self.create_subscription(Twist, "/cmd_vel_mpc", self._on_command, 20)
        self.action_client = ActionClient(self, NavigateToPose, "/ats_navigate_to_pose")
        self.adapter_parameters = self.create_client(
            SetParameters, "/ats_rog_map_adapter/set_parameters"
        )

    def _on_localization(self, message: Odometry) -> None:
        self.localization = message

    def _on_command(self, message: Twist) -> None:
        self.latest_cmd = (
            float(message.linear.x),
            float(message.linear.y),
            float(message.angular.z),
        )

    def wait_for(self, predicate, timeout: float, label: str) -> None:
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.05)
            if predicate():
                return
        raise RuntimeError(f"timeout waiting for {label}")

    @staticmethod
    def command_norm(command) -> float:
        return math.hypot(command[0], command[1]) + abs(command[2]) if command else 0.0

    def commands_are_zero(self) -> bool:
        return self.latest_cmd is not None and self.command_norm(self.latest_cmd) < 1e-3

    def set_adapter_parameters(self, **values) -> None:
        self.wait_for(
            lambda: self.adapter_parameters.service_is_ready(),
            20.0,
            "ROGMap adapter parameter service",
        )
        parameters = [Parameter(name, value=value) for name, value in values.items()]
        request = SetParameters.Request()
        request.parameters = [parameter.to_parameter_msg() for parameter in parameters]
        future = self.adapter_parameters.call_async(request)
        self.wait_for(lambda: future.done(), 5.0, "adapter parameter update")
        response = future.result()
        results = None if response is None else response.results
        if results is None or not all(result.successful for result in results):
            reasons = [] if results is None else [result.reason for result in results]
            raise RuntimeError(f"adapter rejected fault parameters: {reasons}")

    def send_goal(self, pure_rotation: bool = False, outside: bool = False):
        self.wait_for(lambda: self.localization is not None, 30.0, "/localization")
        self.wait_for(
            lambda: self.action_client.wait_for_server(timeout_sec=0.1),
            60.0,
            "ATS action server",
        )
        goal = NavigateToPose.Goal()
        goal.goal_pose.header.frame_id = "map"
        goal.goal_pose.pose.position.x = float(self.localization.pose.pose.position.x)
        goal.goal_pose.pose.position.y = float(self.localization.pose.pose.position.y)
        if outside:
            goal.goal_pose.pose.position.x += 50.0
            goal.goal_pose.pose.position.y += 50.0
        elif not pure_rotation:
            # This free westward corridor gives the post-commit map update a stable tracking window.
            goal.goal_pose.pose.position.x -= 1.80
        if pure_rotation:
            goal.goal_pose.pose.orientation.z = 1.0
            goal.goal_pose.pose.orientation.w = 0.0
        else:
            goal.goal_pose.pose.orientation.w = 1.0
        goal.timeout.sec = 45
        send_future = self.action_client.send_goal_async(goal)
        self.wait_for(lambda: send_future.done(), 10.0, "goal acceptance")
        handle = send_future.result()
        if handle is None or not handle.accepted:
            raise RuntimeError("unsafe-fault action goal was rejected")
        return handle, handle.get_result_async()

    def latest_execute(self, command_count: int):
        for command in reversed(self.execution_commands[command_count:]):
            if (
                command.mode == ExecutionCommand.MODE_EXECUTE
                and command.goal_id != 0
                and len(command.reference.poses) >= 2
            ):
                return command
        return None

    def select_future_pose(self, command: ExecutionCommand):
        now_ns = self.get_clock().now().nanoseconds
        target_ns = now_ns + 400_000_000
        for pose in command.reference.poses:
            stamp_ns = pose.header.stamp.sec * 1_000_000_000 + pose.header.stamp.nanosec
            if stamp_ns >= target_ns:
                return pose
        return command.reference.poses[len(command.reference.poses) // 2]

    @staticmethod
    def pose_yaw(pose) -> float:
        q = pose.pose.orientation
        return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))

    def inject_dynamic_obstacle(self, command: ExecutionCommand, pure_rotation: bool) -> None:
        pose = self.select_future_pose(command)
        x = float(pose.pose.position.x)
        y = float(pose.pose.position.y)
        if pure_rotation:
            # Put the obstacle at a future expanded rectangle corner.  The start and
            # end yaw endpoints remain clear; the swept rotation must detect it.
            yaw = self.pose_yaw(pose)
            half_length = 0.40
            half_width = 0.325
            x += math.cos(yaw) * half_length - math.sin(yaw) * half_width
            y += math.sin(yaw) * half_length + math.cos(yaw) * half_width
        self.set_adapter_parameters(
            test_dynamic_obstacle_x=x,
            test_dynamic_obstacle_y=y,
            test_dynamic_obstacle_radius=0.06,
            test_inject_dynamic_obstacle=True,
        )

    def cancel_and_hold_stop(self, handle, old_sequence: int) -> None:
        cancel_future = handle.cancel_goal_async()
        self.wait_for(lambda: cancel_future.done(), 5.0, "fault action cancellation")
        self.set_adapter_parameters(test_inject_dynamic_obstacle=False)
        old_execute_count = len(self.execution_commands)
        deadline = time.monotonic() + 0.8
        while time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.05)
            if any(
                command.mode == ExecutionCommand.MODE_EXECUTE
                and command.command_sequence == old_sequence
                for command in self.execution_commands[old_execute_count:]
            ):
                raise RuntimeError("old execution command revived after the stop")
            if not self.commands_are_zero():
                raise RuntimeError("motion resumed after cancellation without a new goal")

    def run_outside(self) -> dict:
        before_stop = len(self.stop_states)
        handle, result_future = self.send_goal(outside=True)
        self.wait_for(lambda: result_future.done(), 20.0, "outside-map action result")
        result = result_future.result()
        if result is None or result.result.result_code == NavigateToPose.Result.RESULT_SUCCEEDED:
            raise RuntimeError("outside-map goal unexpectedly succeeded")
        self.wait_for(lambda: True in self.stop_states[before_stop:], 5.0, "outside-map stop")
        self.wait_for(
            self.commands_are_zero,
            5.0,
            "outside-map /cmd_vel_mpc zero",
        )
        return {
            "fault": self.fault,
            "result_code": int(result.result.result_code),
        }

    def run(self) -> dict:
        self.wait_for(lambda: self.localization is not None, 30.0, "/localization")
        self.wait_for(
            lambda: bool(self.map_statuses) and self.map_statuses[-1].ready,
            120.0,
            "ready planning map",
        )
        if self.fault == "outside":
            return self.run_outside()
        if self.fault == "unknown":
            raise RuntimeError(
                "P4 evaluator refuses the retired post-fusion unknown injection; "
                "run the P2 source-unknown closure in test_mujoco_minco_mpc_chain.sh"
            )

        before_commands = len(self.execution_commands)
        before_stops = len(self.stop_states)
        before_statuses = len(self.planner_statuses)
        handle, _ = self.send_goal(pure_rotation=self.fault == "pure_rotation")
        self.wait_for(
            lambda: self.latest_execute(before_commands) is not None,
            30.0,
            "committed structured execution command",
        )
        command = self.latest_execute(before_commands)
        self.wait_for(
            lambda: self.command_norm(self.latest_cmd) > 0.02,
            20.0,
            "initial non-zero control",
        )

        self.inject_dynamic_obstacle(command, self.fault == "pure_rotation")

        self.wait_for(
            lambda: True in self.stop_states[before_stops:],
            8.0,
            "Goal Manager emergency stop",
        )
        self.wait_for(
            self.commands_are_zero,
            5.0,
            "/cmd_vel_mpc zero after unsafe trajectory",
        )
        self.wait_for(
            lambda: any(
                status.state == PlannerStatus.STATE_FAILED
                and status.failure_reason == PlannerStatus.FAILURE_RUNTIME_UNSAFE
                and status.goal_id == command.goal_id
                for status in self.planner_statuses[before_statuses:]
            ),
            5.0,
            "runtime swept unsafe planner status",
        )

        self.cancel_and_hold_stop(handle, int(command.command_sequence))
        return {
            "fault": self.fault,
            "goal_id": int(command.goal_id),
            "old_command_sequence": int(command.command_sequence),
            "old_map_generation": int(command.map_generation),
            "old_map_publication_sequence": int(command.map_publication_sequence),
            "planner_statuses_after_fault": len(self.planner_statuses) - before_statuses,
        }


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--fault", choices=FAULTS, required=True)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()

    rclpy.init()
    node = UnsafeTrajectoryEvaluator(args.fault)
    try:
        result = node.run()
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(json.dumps(result, ensure_ascii=False, indent=2) + "\n")
        print(json.dumps(result, ensure_ascii=False, indent=2))
        return 0
    except Exception as error:
        print(f"FAIL: {error}")
        return 1
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    raise SystemExit(main())
