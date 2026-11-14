#!/usr/bin/env python3
"""Run the pinned historical NavigateToPose baseline in an isolated overlay.

This is a standalone acceptance runner, not a pytest module. All output paths
must be new. The current ATS navigation launch is disabled in this experiment.
"""
__test__ = False

import argparse
import hashlib
import json
import math
import os
from pathlib import Path
import signal
import subprocess
import time

ROOT = Path(__file__).resolve().parents[1]
GOAL = (-9.0, 1.47)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--compat-workspace', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--startup-timeout', type=float, default=420.0)
    parser.add_argument('--goal-timeout', type=float, default=600.0, help='wall seconds')
    parser.add_argument('--strict-xy-goal-tolerance', type=float,
                        help='explicit stricter goal-checker variant, at most 0.08 m')
    args = parser.parse_args()
    if int(os.environ.get('ROS_DOMAIN_ID', '-1')) not in set(range(233)) - {5, 77}:
        parser.error('allocate a fresh nonreserved ROS_DOMAIN_ID first')
    if os.environ.get('ROS_LOCALHOST_ONLY') != '0' or 'wlan' not in os.environ.get('CYCLONEDDS_URI', ''):
        parser.error('source the workspace WLAN DDS environment first')
    args.output = args.output.resolve()
    from prepare_nav2_compat import validate_map_points
    geometry_check = validate_map_points(
        args.compat_workspace / 'historical_scene/maps/rmuc_2026.yaml',
        [(-10.66, 1.47), GOAL])
    args.output.mkdir(parents=True, exist_ok=False)
    command = ['ros2', 'launch', str(ROOT / 'scripts/nav2_compat.launch.py'),
               f'compat_workspace:={args.compat_workspace.resolve()}',
               f'evidence_output:={args.output}']
    if args.strict_xy_goal_tolerance is not None:
        if not math.isfinite(args.strict_xy_goal_tolerance) or not 0 < args.strict_xy_goal_tolerance <= 0.08:
            parser.error('strict goal-checker tolerance must be in (0, 0.08]')
        command.append(f'strict_xy_goal_tolerance:={args.strict_xy_goal_tolerance}')
    manifest = {'command': command, 'domain': os.environ['ROS_DOMAIN_ID'], 'rviz': False,
                'goal_map_xy': GOAL, 'start_map_xy': [-10.66, 1.47], 'map_preflight': geometry_check,
                'scope': 'historical RMUC2026 truth/Nav2; not LIO, P2/P3, or current 2025/2027 acceptance',
                'strict_goal_checker_variant_m': args.strict_xy_goal_tolerance,
                'hashes': {str(p): hashlib.sha256(p.read_bytes()).hexdigest() for p in (
                    args.compat_workspace / 'historical_scene/manifest.json', Path(__file__),
                    ROOT / 'scripts/nav2_compat.launch.py', ROOT / 'scripts/prepare_nav2_compat.py',
                    args.compat_workspace / 'manifest.json')},
                'heads': {name: subprocess.check_output(['git', '-C', str(repo), 'rev-parse', 'HEAD'],
                           text=True).strip() for name, repo in (
                    ('root', ROOT), ('navigation', ROOT / 'src/ats_sentry_nav'),
                    ('simulation', ROOT / 'src/sim/ats_mujoco_sim'))}}
    (args.output / 'command.json').write_text(json.dumps(manifest, indent=2) + '\n')
    import rclpy
    from rclpy.action import ActionClient
    from rclpy.qos import QoSProfile, DurabilityPolicy, qos_profile_sensor_data
    from rosidl_runtime_py.convert import message_to_ordereddict
    from ats_navigation_interfaces.msg import LocalizationStatus
    from geometry_msgs.msg import Twist
    from manda_can_control.msg import MotionCtrl
    from nav_msgs.msg import Path as RosPath, Odometry, OccupancyGrid
    from nav2_msgs.action import NavigateToPose
    from lifecycle_msgs.srv import GetState
    from sensor_msgs.msg import PointCloud2

    rclpy.init()
    node = rclpy.create_node('nav2_compat_evaluator')
    durable = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
    last, counts, received = {}, {}, {}
    events = (args.output / 'events.jsonl').open('w')
    subscriptions = []

    def receive(topic, msg):
        last[topic], received[topic] = msg, time.monotonic()
        counts[topic] = counts.get(topic, 0) + 1
        if topic in ('/localization/status', '/odometry',
                     '/cmd_vel_nav2_result', '/motion_control', '/plan', '/local_elastic_path'):
            events.write(json.dumps({'wall_monotonic': time.monotonic(), 'topic': topic,
                                     'message': message_to_ordereddict(msg)}) + '\n')
    for kind, topic, qos in [
            (LocalizationStatus, '/localization/status', durable),
            (Odometry, '/odometry', qos_profile_sensor_data),
            (Odometry, '/localization', qos_profile_sensor_data),
            (PointCloud2, '/registered_scan', qos_profile_sensor_data),
            (PointCloud2, '/terrain_map', qos_profile_sensor_data),
            (PointCloud2, '/terrain_map_ext', qos_profile_sensor_data),
            (OccupancyGrid, '/traversability_grid', 10),
            (OccupancyGrid, '/rc_esdf/planning_grid', durable),
            (RosPath, '/plan', 10), (RosPath, '/local_elastic_path', durable),
            (Twist, '/cmd_vel_nav2_result', 10), (MotionCtrl, '/motion_control', 10)]:
        subscriptions.append(node.create_subscription(kind, topic, lambda m, t=topic: receive(t, m), qos))
    client = ActionClient(node, NavigateToPose, '/navigate_to_pose')
    lifecycle_clients = {name: node.create_client(GetState, f'/{name}/get_state') for name in (
        'controller_server', 'planner_server', 'smoother_server', 'behavior_server',
        'bt_navigator', 'waypoint_follower', 'velocity_smoother')}
    lifecycle_states, lifecycle_requests = {}, {}
    next_state_poll = 0.0
    zero = node.create_publisher(Twist, '/cmd_vel_nav2_result', 10)
    # Only publish an evaluator stop during cleanup; graph checks exclude it
    # explicitly and never mistake this safety publisher for the control owner.
    stream = (args.output / 'launch.log').open('w')
    process = subprocess.Popen(command, stdout=stream, stderr=subprocess.STDOUT, start_new_session=True)
    (args.output / 'process.json').write_text(json.dumps({'pid': process.pid, 'pgid': process.pid}) + '\n')
    summary = {'passed': False, 'physical_contact_acceptance': 'unverified',
               'strict_goal_checker_variant_m': args.strict_xy_goal_tolerance,
               'historical_nav2_result': None, 'truth_error_m': None}
    handle = None
    last_report = time.monotonic()

    def pump():
        nonlocal last_report
        rclpy.spin_once(node, timeout_sec=0.2)
        if (args.output / 'STOP_REQUESTED').exists():
            raise RuntimeError('operator requested stop for this owned experiment')
        truth = last.get('/odometry')
        if truth is not None and not -0.2 <= truth.pose.pose.position.z <= 2.0:
            raise RuntimeError('truth height left physical field bounds')
        if process.poll() is not None:
            raise RuntimeError(f'launch exited {process.returncode}')
        if time.monotonic() - last_report >= 30:
            status = last.get('/localization/status')
            print(json.dumps({'phase': summary.get('phase'), 'localization_state':
                              None if status is None else status.state, 'counts': counts}), flush=True)
            events.flush()
            last_report = time.monotonic()

    try:
        summary['phase'] = 'startup'
        deadline = time.monotonic() + args.startup_timeout
        while time.monotonic() < deadline:
            pump()
            for name, future in list(lifecycle_requests.items()):
                if future.done():
                    lifecycle_states[name] = future.result().current_state.id
                    del lifecycle_requests[name]
            if time.monotonic() >= next_state_poll:
                for name, service in lifecycle_clients.items():
                    if name not in lifecycle_requests and service.service_is_ready():
                        lifecycle_requests[name] = service.call_async(GetState.Request())
                next_state_poll = time.monotonic() + 0.5
            if (client.server_is_ready() and
                    len(lifecycle_states) == len(lifecycle_clients) and
                    all(state == 3 for state in lifecycle_states.values()) and
                    all(t in last for t in ('/registered_scan', '/terrain_map', '/terrain_map_ext',
                                           '/rc_esdf/planning_grid', '/odometry')) and
                    time.monotonic() - received['/odometry'] < 0.5):
                break
        else:
            summary['lifecycle_states'] = lifecycle_states
            raise RuntimeError('startup did not receive fresh truth, Nav2 action server and map inputs')
        summary['lifecycle_states'] = lifecycle_states
        graph = {}
        for topic in ('/motion_control', '/cmd_vel_nav2_result', '/rc_esdf/planning_grid', '/local_elastic_path'):
            graph[topic] = {'publishers': [i.node_name for i in node.get_publishers_info_by_topic(topic)],
                            'subscribers': [i.node_name for i in node.get_subscriptions_info_by_topic(topic)]}
        summary['graph'] = graph
        if graph['/motion_control']['publishers'] != ['twist_to_motion_ctrl']:
            raise RuntimeError('motion_control publisher ownership mismatch')
        if graph['/rc_esdf/planning_grid']['publishers'] != ['rc_esdf_map']:
            raise RuntimeError('legacy planning-grid ownership mismatch')
        names = {name for name, _ in node.get_node_names_and_namespaces()}
        if names & {'ats_goal_manager', 'ats_swerve_mpc', 'minco_planner', 'ats_rog_map', 'cmd_vel_arbiter'}:
            raise RuntimeError('production navigation leaked into isolated Nav2 experiment')
        goal = NavigateToPose.Goal()
        goal.pose.header.frame_id = 'map'
        goal.pose.pose.position.x, goal.pose.pose.position.y = GOAL
        goal.pose.pose.orientation.w = 1.0
        request = client.send_goal_async(goal)
        while not request.done():
            pump()
            if time.monotonic() >= deadline:
                raise RuntimeError('goal acceptance timed out')
        handle = request.result()
        if not handle.accepted:
            raise RuntimeError('NavigateToPose rejected goal')
        result = handle.get_result_async()
        summary['phase'] = 'navigate'
        deadline = time.monotonic() + args.goal_timeout
        while not result.done():
            pump()
            if time.monotonic() - received['/odometry'] > 1.0:
                raise RuntimeError('simulation truth heartbeat stale during navigation')
            if time.monotonic() >= deadline:
                raise RuntimeError('NavigateToPose wall-time budget exhausted')
        summary['historical_nav2_result'] = result.result().status
        if result.result().status != 4:
            raise RuntimeError(f'NavigateToPose result status {result.result().status}')
        plan_count = counts.get('/plan', 0)
        until = time.monotonic() + 4.0
        while time.monotonic() < until:
            pump()
        summary['post_goal_plan_count_4_wall_s'] = counts.get('/plan', 0) - plan_count
        truth = last['/odometry'].pose.pose.position
        summary['truth_xyz'] = [truth.x, truth.y, truth.z]
        summary['truth_error_m'] = math.hypot(truth.x - GOAL[0], truth.y - GOAL[1])
        text = (args.output / 'launch.log').read_text()
        errors = []
        if summary['truth_error_m'] > 0.08:
            errors.append('truth endpoint exceeds 0.08 m')
        if not last.get('/local_elastic_path') or not last['/local_elastic_path'].poses:
            errors.append('no nonempty elastic path')
        if not counts.get('/cmd_vel_nav2_result') or not counts.get('/motion_control'):
            errors.append('missing downstream velocity messages')
        if summary['post_goal_plan_count_4_wall_s'] > 1:
            errors.append('global plan still periodic after completion')
        if 'Resulting plan has 0 poses' in text or '[follow_path] [ActionServer] Aborting handle.' in text:
            errors.append('controller empty path or FollowPath abort')
        summary['failures'] = errors
        summary['stage_truth_gate_passed'] = summary['truth_error_m'] <= 0.08
        summary['tf_lookup_errors'] = text.count('No Transform available')
        if summary['tf_lookup_errors']:
            errors.append('compatibility TF lookup failed')
        protocol_passed = not [e for e in errors if e != 'truth endpoint exceeds 0.08 m']
        summary['compatibility_protocol_passed'] = protocol_passed
        summary['historical_baseline_passed'] = (
            protocol_passed if args.strict_xy_goal_tolerance is None else None)
        summary['passed'] = not errors
    except (Exception, KeyboardInterrupt) as exc:
        summary['error'] = str(exc)
    finally:
        if handle is not None and not summary['passed']:
            handle.cancel_goal_async()
        zero.publish(Twist())
        for _ in range(5):
            rclpy.spin_once(node, timeout_sec=0.1)
        for sig, timeout in ((signal.SIGINT, 15), (signal.SIGTERM, 5), (signal.SIGKILL, 3)):
            if process.poll() is not None:
                break
            os.killpg(process.pid, sig)
            try:
                process.wait(timeout=timeout)
            except subprocess.TimeoutExpired:
                continue
        stream.close()
        events.close()
        summary['counts'] = counts
        summary['launch_exit'] = process.returncode
        (args.output / 'summary.json').write_text(json.dumps(summary, indent=2) + '\n')
        node.destroy_node()
        rclpy.shutdown()
    print(json.dumps(summary, indent=2), flush=True)
    return 0 if summary['passed'] else 1


if __name__ == '__main__':
    raise SystemExit(main())
