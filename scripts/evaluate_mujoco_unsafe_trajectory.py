#!/usr/bin/env python3
"""Exercise P4 runtime swept-footprint fail-stop behavior in MuJoCo."""

import argparse
import json
import math
import time
from pathlib import Path

from types import SimpleNamespace
from ats_navigation_interfaces.action import NavigateToPose
from ats_navigation_interfaces.msg import ExecutionCommand
from ats_navigation_interfaces.msg import PlannerStatus
from ats_navigation_interfaces.msg import PlanningMapStatus
from geometry_msgs.msg import Twist
from ats_navigation_interfaces.msg import PlanningMapSnapshot
from nav_msgs.msg import Odometry
from nav_msgs.msg import OccupancyGrid
from query_occupancy_grid import cell_is_traversable, validate_grid, world_to_grid
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
    "occupied",
)


class UnsafeTrajectoryEvaluator(Node):
    """Inject only through test-only adapter parameters after a real execute commit."""

    def __init__(self, fault: str) -> None:
        super().__init__("mujoco_unsafe_trajectory_evaluator")
        self.fault = fault
        self.localization = None
        self.map_statuses = []
        self.map_snapshots = []
        self.execution_commands = []
        self.execution_received_at = {}
        self.planner_statuses = []
        self.stop_states = []
        self.latest_cmd = None
        self.latest_cmd_at = 0.0
        self.grid = None
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
            self._on_execution,
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
        self.create_subscription(Twist, "/cmd_vel/selected", self._on_command, qos_profile_sensor_data)
        self.create_subscription(OccupancyGrid, '/rc_esdf/planning_grid',
                                 self._on_grid, self.transient_qos)
        self.create_subscription(
            PlanningMapSnapshot, '/rog_map_adapter/planning_snapshot',
            self._on_snapshot, self.transient_qos)
        self.action_client = ActionClient(self, NavigateToPose, "/ats_navigate_to_pose")
        self.adapter_parameters = self.create_client(
            SetParameters, "/ats_rog_map_adapter/set_parameters"
        )

    def _on_execution(self, message):
        self.execution_received_at[message.command_sequence] = time.monotonic()
        self.execution_commands.append(message)

    @staticmethod
    def stamp_ns(stamp):
        return stamp.sec * 1_000_000_000 + stamp.nanosec

    def _on_snapshot(self, message):
        # Keep identity and target-cell evidence, not copies of the full numeric map.
        grid = SimpleNamespace(header=message.header, info=message.info, data=message.occupancy)
        cell = world_to_grid(grid, 1.0, 0.06) if validate_grid(grid) else None
        self.map_snapshots.append(dict(
            publication=int(message.publication_sequence), epoch=int(message.localization_epoch),
            source_generation=int(message.source_generation), ready=bool(message.ready),
            stamp=self.stamp_ns(message.header.stamp),
            occupied=(message.header.frame_id == 'map' and cell is not None and
                      message.occupancy[cell[1] * message.info.width + cell[0]] >= 100)))

    def _on_grid(self, message):
        self.grid = message

    def nominal_goal_cell(self):
        if self.grid is None or not validate_grid(self.grid) or self.grid.header.frame_id != 'map':
            return None
        return world_to_grid(self.grid, 1.0, 0.06)

    def nominal_goal_free(self):
        cell = self.nominal_goal_cell()
        return cell is not None and cell_is_traversable(self.grid, *cell, 100, 0.42)

    def nominal_goal_occupied(self):
        cell = self.nominal_goal_cell()
        return cell is not None and self.grid.data[cell[1] * self.grid.info.width + cell[0]] >= 100

    def _on_localization(self, message: Odometry) -> None:
        self.localization = message

    def _on_command(self, message: Twist) -> None:
        self.latest_cmd_at = time.monotonic()
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

    def commands_are_zero(self, after: float = 0.0) -> bool:
        return (self.latest_cmd is not None and self.latest_cmd_at > after
                and time.monotonic() - self.latest_cmd_at < 0.5
                and self.command_norm(self.latest_cmd) < 1e-3)

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
        if self.fault in ('occupied', 'map_after_commit'):
            goal.goal_pose.pose.position.x = 1.0
            goal.goal_pose.pose.position.y = 0.06
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
        if self.fault == 'map_after_commit':
            x, y = 1.0, 0.06
        self.set_adapter_parameters(
            test_dynamic_obstacle_x=x,
            test_dynamic_obstacle_y=y,
            test_dynamic_obstacle_radius=0.20 if self.fault == 'map_after_commit' else 0.06,
            test_inject_dynamic_obstacle=True,
        )
        if self.fault == 'map_after_commit':
            self.wait_for(self.nominal_goal_occupied, 15.0, 'committed route target became occupied')

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

    def run_occupied(self):
        self.wait_for(self.nominal_goal_free, 30.0, 'known-free nominal goal before injection')
        publication = self.map_statuses[-1].publication_sequence
        self.set_adapter_parameters(test_dynamic_obstacle_x=1.0, test_dynamic_obstacle_y=0.06,
                                    test_dynamic_obstacle_radius=0.20, test_inject_dynamic_obstacle=True)
        self.wait_for(lambda: self.nominal_goal_occupied() and
                      self.map_statuses[-1].publication_sequence > publication,
                      15.0, 'source-owned occupied goal publication')
        command_count = len(self.execution_commands)
        stop_count = len(self.stop_states)
        planner_count = len(self.planner_statuses)
        handle, future = self.send_goal()
        self.wait_for(lambda: future.done(), 50.0, 'occupied goal terminal rejection')
        result = future.result()
        if result is None or result.result.result_code != NavigateToPose.Result.RESULT_PLANNING_FAILED:
            raise RuntimeError('occupied goal did not terminate with planning failure')
        self.wait_for(lambda: any(status.state == PlannerStatus.STATE_FAILED and
                      status.failure_reason == PlannerStatus.FAILURE_START_OR_GOAL_OCCUPIED
                      for status in self.planner_statuses[planner_count:]),
                      5.0, 'occupied-cell planner rejection (not timeout/TF/unready)')
        self.wait_for(lambda: True in self.stop_states[stop_count:] and self.commands_are_zero(),
                      5.0, 'occupied goal stop and fresh selected zero')
        if self.latest_execute(command_count) is not None:
            raise RuntimeError('occupied goal produced executable reference')
        return dict(fault=self.fault, result_code=int(result.result.result_code),
                    occupied_publication=int(self.map_statuses[-1].publication_sequence),
                    executable_reference=False)

    def recover_map_goal(self, old_sequence):
        self.wait_for(self.nominal_goal_free, 15.0, 'known-free goal after obstacle removal')
        before_commands = len(self.execution_commands)
        handle, future = self.send_goal()
        self.wait_for(lambda: self.latest_execute(before_commands) is not None, 30.0,
                      'fresh recovery execution')
        fresh = self.latest_execute(before_commands)
        if fresh.command_sequence <= old_sequence:
            raise RuntimeError('map recovery reused old execution sequence')
        self.wait_for(lambda: future.done(), 50.0, 'map recovery action success')
        result = future.result()
        if result is None or result.result.result_code != NavigateToPose.Result.RESULT_SUCCEEDED:
            raise RuntimeError('new goal did not succeed after map recovery')
        if any(command.mode == ExecutionCommand.MODE_EXECUTE and
               command.command_sequence <= old_sequence
               for command in self.execution_commands[before_commands:]):
            raise RuntimeError('old execution revived during map recovery')
        self.wait_for(self.commands_are_zero, 5.0, 'selected zero after recovery success')
        return dict(recovery_command_sequence=int(fresh.command_sequence),
                    recovery_result_code=int(result.result.result_code))

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
            "outside-map /cmd_vel/selected zero",
        )
        return {
            "fault": self.fault,
            "result_code": int(result.result.result_code),
        }

    def committed_status(self, command):
        return next((status for status in reversed(self.planner_statuses)
                     if status.state == PlannerStatus.STATE_REFERENCE_READY
                     and status.goal_id == command.goal_id
                     and status.localization_epoch == command.localization_epoch
                     and status.map_generation == command.map_generation
                     and status.map_publication_sequence == command.map_publication_sequence
                     and self.stamp_ns(status.reference_stamp) ==
                     self.stamp_ns(command.reference.header.stamp)), None)

    def map_invalidation(self, command, committed, first_status):
        for status in self.planner_statuses[first_status:]:
            if (status.state != PlannerStatus.STATE_FAILED
                    or status.goal_id != command.goal_id
                    or status.localization_epoch != command.localization_epoch
                    or status.plan_request_sequence != committed.plan_request_sequence
                    or status.map_publication_sequence != command.map_publication_sequence
                    or status.map_generation <= command.map_generation):
                continue
            if (status.failure_reason == PlannerStatus.FAILURE_SNAPSHOT_CHANGED
                    and self.stamp_ns(status.reference_stamp) ==
                    self.stamp_ns(command.reference.header.stamp)):
                return status
            if status.failure_reason == PlannerStatus.FAILURE_RUNTIME_UNSAFE:
                return status
        return None

    def occupied_rejection(self, command, committed, invalidated, first_status, first_snapshot):
        for status in self.planner_statuses[first_status:]:
            if (status.state != PlannerStatus.STATE_FAILED
                    or status.failure_reason != PlannerStatus.FAILURE_START_OR_GOAL_OCCUPIED
                    or status.goal_id != command.goal_id
                    or status.localization_epoch != command.localization_epoch
                    or status.plan_request_sequence <= committed.plan_request_sequence
                    or status.map_generation < invalidated.map_generation
                    or status.map_publication_sequence <= command.map_publication_sequence):
                continue
            if any(snapshot['ready'] and snapshot['occupied']
                   and snapshot['epoch'] == status.localization_epoch
                   and snapshot['publication'] == status.map_publication_sequence
                   for snapshot in self.map_snapshots[first_snapshot:]):
                return status
        return None

    def assert_no_old_execution(self, old, first_command):
        for command in self.execution_commands[first_command:]:
            if command.mode != ExecutionCommand.MODE_EXECUTE:
                continue
            if (command.manager_incarnation != old.manager_incarnation
                    or command.command_sequence <= old.command_sequence
                    or (command.goal_id == old.goal_id
                        and command.localization_epoch == old.localization_epoch
                        and (command.map_generation <= old.map_generation
                             or command.map_publication_sequence <= old.map_publication_sequence
                             or self.stamp_ns(command.reference.header.stamp) ==
                             self.stamp_ns(old.reference.header.stamp)))):
                raise RuntimeError('old execution authorization revived after map invalidation')

    def fresh_map_stop(self, old, first_command):
        # suspendActiveGoal intentionally zeros epoch/map/reason on STOP;
        # goal, manager incarnation, sequence and lease stamp identify this stop.
        return next((command for command in self.execution_commands[first_command:]
                     if command.mode == ExecutionCommand.MODE_STOP
                     and command.manager_incarnation == old.manager_incarnation
                     and command.command_sequence > old.command_sequence
                     and command.goal_id == old.goal_id
                     and self.stamp_ns(command.header.stamp) >
                     self.stamp_ns(old.header.stamp)), None)

    def run_map_after_commit(self):
        self.wait_for(self.nominal_goal_free, 30.0, 'known-free nominal goal')
        first_command = len(self.execution_commands)
        handle, _ = self.send_goal()
        self.wait_for(lambda: self.latest_execute(first_command) is not None, 30.0,
                      'committed structured execution command')
        self.wait_for(lambda: self.command_norm(self.latest_cmd) > 0.02, 20.0,
                      'initial non-zero control')
        old = self.latest_execute(first_command)
        self.wait_for(lambda: self.committed_status(old) is not None, 5.0,
                      'exact committed planner request identity')
        committed = self.committed_status(old)
        first_status = len(self.planner_statuses)
        first_snapshot = len(self.map_snapshots)
        fault_command = len(self.execution_commands)
        fault_stop = len(self.stop_states)
        fault_at = time.monotonic()
        self.inject_dynamic_obstacle(old, False)
        self.wait_for(lambda: self.map_invalidation(old, committed, first_status) is not None,
                      5.0, 'identity-correlated old snapshot invalidation')
        invalidated = self.map_invalidation(old, committed, first_status)
        self.wait_for(lambda: self.occupied_rejection(
            old, committed, invalidated, first_status, first_snapshot) is not None,
            5.0, 'new occupied snapshot rejects a fresh planner request')
        rejected = self.occupied_rejection(old, committed, invalidated, first_status, first_snapshot)
        fresh_stop = lambda: self.fresh_map_stop(old, fault_command)
        self.wait_for(lambda: fresh_stop() is not None and True in self.stop_states[fault_stop:]
                      and self.commands_are_zero(max(fault_at,
                          self.execution_received_at[fresh_stop().command_sequence])), 8.0,
                      'fresh correlated STOP and post-injection selected zero')
        stopped = fresh_stop()
        after_stop = self.execution_commands.index(stopped) + 1
        self.assert_no_old_execution(old, after_stop)
        cancel = handle.cancel_goal_async()
        self.wait_for(lambda: cancel.done(), 5.0, 'map fault action cancellation')
        hold_start = len(self.execution_commands)
        self.set_adapter_parameters(test_inject_dynamic_obstacle=False)
        self.wait_for(self.nominal_goal_free, 15.0, 'known-free goal after obstacle removal')
        deadline = time.monotonic() + 0.8
        while time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.05)
            self.assert_no_old_execution(old, after_stop)
            if self.latest_execute(hold_start) is not None or not self.commands_are_zero(fault_at):
                raise RuntimeError('execution resumed without a fresh recovery goal')
        before_recovery = len(self.execution_commands)
        recovery = self.recover_map_goal(int(old.command_sequence))
        fresh = self.latest_execute(before_recovery)
        if (fresh is None or fresh.goal_id == old.goal_id
                or fresh.map_generation <= old.map_generation
                or fresh.map_publication_sequence <= old.map_publication_sequence
                or self.stamp_ns(fresh.reference.header.stamp) <=
                self.stamp_ns(old.reference.header.stamp)):
            raise RuntimeError('map recovery did not authorize a fresh goal, map and reference')
        self.assert_no_old_execution(old, after_stop)
        return dict(fault=self.fault, goal_id=int(old.goal_id),
                    old_command_sequence=int(old.command_sequence),
                    old_plan_request_sequence=int(committed.plan_request_sequence),
                    old_map_generation=int(old.map_generation),
                    old_map_publication_sequence=int(old.map_publication_sequence),
                    invalidation_reason=int(invalidated.failure_reason),
                    invalidated_generation=int(invalidated.map_generation),
                    occupied_publication=int(rejected.map_publication_sequence),
                    occupied_plan_request_sequence=int(rejected.plan_request_sequence),
                    stop_command_sequence=int(stopped.command_sequence),
                    post_fault_zero=True, old_execution_revived=False, **recovery)

    def run(self) -> dict:
        self.wait_for(lambda: self.localization is not None, 30.0, "/localization")
        self.wait_for(
            lambda: bool(self.map_statuses) and self.map_statuses[-1].ready,
            120.0,
            "ready planning map",
        )
        if self.fault == 'occupied':
            return self.run_occupied()
        if self.fault == 'map_after_commit':
            return self.run_map_after_commit()
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
            "/cmd_vel/selected zero after unsafe trajectory",
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
