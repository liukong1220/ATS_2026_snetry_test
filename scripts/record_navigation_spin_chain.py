#!/usr/bin/env python3
"""Read-only spin-chain trace with independent ROS and steady receive timestamps.

The MPC horizon contains states only. Its yaw derivatives must use the actual
MPC dt parameter; neither receive time nor repeated pose headers is that dt.
Subscriber records are not an atomic snapshot of the controller internals.
"""
import argparse
import collections
import json
import math
import os
from pathlib import Path
import time


def path_record(msg):
    return {
        'frame': msg.header.frame_id,
        'stamp_ns': msg.header.stamp.sec * 10**9 + msg.header.stamp.nanosec,
        'poses': [
            [p.header.stamp.sec * 10**9 + p.header.stamp.nanosec,
             p.pose.position.x, p.pose.position.y,
             math.atan2(2 * (p.pose.orientation.w * p.pose.orientation.z +
                             p.pose.orientation.x * p.pose.orientation.y),
                        1 - 2 * (p.pose.orientation.y**2 + p.pose.orientation.z**2))]
            for p in msg.poses],
    }


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--idle-timeout', type=float, default=60.0)
    parser.add_argument('--startup-timeout', type=float, default=240.0)
    parser.add_argument('--duration', type=float, default=7200.0)
    args = parser.parse_args()
    if args.output.exists() or args.output.with_suffix('.summary.json').exists():
        parser.error('choose a fresh output path')
    if not os.environ.get('ROS_DOMAIN_ID'):
        parser.error('set ROS_DOMAIN_ID explicitly')
    import rclpy
    from rclpy.parameter import Parameter
    from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy, qos_profile_sensor_data
    from rosidl_runtime_py.convert import message_to_ordereddict
    from rosgraph_msgs.msg import Clock
    from nav_msgs.msg import Path as PathMsg, Odometry
    from geometry_msgs.msg import Twist
    from ats_navigation_interfaces.msg import ExecutionCommand, SwerveTelemetry
    from manda_can_control.msg import MotionCtrl

    rclpy.init()
    node = rclpy.create_node('navigation_spin_chain_evaluator', parameter_overrides=[
        Parameter('use_sim_time', value=True)])
    args.output.parent.mkdir(parents=True, exist_ok=True)
    stream = args.output.open('x')
    counts = collections.Counter()
    received = collections.Counter()
    start = time.monotonic()
    last_clock = [start]
    identity = [None]

    def write(kind, data):
        counts[kind] += 1
        stream.write(json.dumps({'kind': kind, 'received_steady_s': time.monotonic(),
                                'ros_time_ns': node.get_clock().now().nanoseconds,
                                'data': data}) + '\n')
        stream.flush()

    def clock(msg):
        received['clock'] += 1
        last_clock[0] = time.monotonic()

    def execution(msg):
        received['execution'] += 1
        stamp = msg.reference.header.stamp
        key = (msg.manager_incarnation, msg.mode, msg.goal_id, msg.localization_epoch,
               stamp.sec, stamp.nanosec, msg.yaw_authority, msg.failure_reason)
        if key == identity[0]:
            return
        identity[0] = key
        data = {name: getattr(msg, name) for name in (
            'manager_incarnation', 'command_sequence', 'mode', 'goal_id',
            'localization_epoch', 'map_generation', 'map_publication_sequence',
            'yaw_authority', 'failure_reason')}
        data['reference'] = path_record(msg.reference)
        write('execution', data)

    def generic(kind, msg):
        received[kind] += 1
        write(kind, message_to_ordereddict(msg))

    def horizon(msg):
        received['horizon'] += 1
        write('horizon', path_record(msg))

    subscriptions = [
        node.create_subscription(Clock, '/clock', clock, qos_profile_sensor_data),
        node.create_subscription(ExecutionCommand, '/planner/execution_command', execution,
                                 QoSProfile(depth=100, reliability=ReliabilityPolicy.RELIABLE,
                                            durability=DurabilityPolicy.TRANSIENT_LOCAL)),
        node.create_subscription(PathMsg, '/ats_swerve_mpc/reference_horizon', horizon,
                                 qos_profile_sensor_data),
    ]
    for kind, topic, msg_type in (
        ('localization', '/localization', Odometry),
        ('autonomy_raw', '/cmd_vel/autonomy_raw', Twist),
        ('selected', '/cmd_vel/selected', Twist),
        ('motion_control', '/motion_control', MotionCtrl),
        ('telemetry', '/swerve/telemetry', SwerveTelemetry),
    ):
        subscriptions.append(node.create_subscription(
            msg_type, topic, lambda msg, key=kind: generic(key, msg), qos_profile_sensor_data))
    reason = 'duration'
    try:
        while time.monotonic() - start < args.duration:
            rclpy.spin_once(node, timeout_sec=0.2)
            limit = args.idle_timeout if received['clock'] else args.startup_timeout
            if time.monotonic() - last_clock[0] > limit:
                reason = 'clock_receive_timeout'
                break
    finally:
        stream.close()
        args.output.with_suffix('.summary.json').write_text(json.dumps({
            'domain': os.environ['ROS_DOMAIN_ID'], 'pid': os.getpid(), 'pgid': os.getpgrp(),
            'elapsed_wall_s': time.monotonic() - start, 'reason': reason,
            'received': received, 'written': counts,
            'scope': 'subscriber receive trace; execution references deduplicated; '
                     'not an atomic internal MPC snapshot; horizon dt requires parameter capture',
        }, indent=2) + '\n')
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
