#!/usr/bin/env python3
"""Isolated historical Nav2 test. Never included by the production launch."""
import importlib.util
import json
from pathlib import Path

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node



ROOT = Path(__file__).resolve().parents[1]
REVISION = '305b985213c655d72101d3d8281efda7fcec2d68'


def make_nodes(context):
    lc = context.launch_configurations
    fixture = Path(lc['compat_workspace']).resolve()
    output = Path(lc['evidence_output']).resolve()
    manifest = json.loads((fixture / 'manifest.json').read_text())
    if manifest['source_revision'] != REVISION:
        raise ValueError('unexpected Nav2 compatibility source revision')
    # The overlay must be explicitly sourced; never load a similarly named
    # package from the production workspace by accident.
    for package in ('trajectory_optimizer', 'ats_nav2_plugins'):
        if fixture not in Path(get_package_share_directory(package)).resolve().parents:
            raise ValueError(f'{package} is not from the isolated compatibility overlay')
    scene = fixture / 'historical_scene'
    scene_manifest = json.loads((scene / 'manifest.json').read_text())
    if scene_manifest['revision'] != '71262e7b45e4659189cd35715397ec65a0549aad':
        raise ValueError('unexpected historical simulation scene')
    sim_share = Path(get_package_share_directory('ats_mujoco_sim'))
    old_share = fixture / 'historical/ats_nav_bringup'
    text = (old_share / 'config/simulation/nav2_params.yaml').read_text()
    text = text.replace('$(find-pkg-share ats_nav_bringup)', str(old_share))
    text = text.replace('<robot_namespace>/', '/')
    document = yaml.safe_load(text)
    def wall_clock(value):
        if isinstance(value, dict):
            for key, child in value.items():
                if key == 'use_sim_time':
                    value[key] = False
                else:
                    wall_clock(child)
        elif isinstance(value, list):
            for child in value:
                wall_clock(child)
    wall_clock(document)
    if lc.get('strict_xy_goal_tolerance', ''):
        spec = importlib.util.spec_from_file_location(
            'nav2_compat_preparation', Path(__file__).with_name('prepare_nav2_compat.py'))
        preparation = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(preparation)
        variant = preparation.tighten_goal_checker(document, float(lc['strict_xy_goal_tolerance']))
        (output / 'goal_checker_variant.json').write_text(json.dumps(variant, indent=2) + '\n')
    legacy = output / 'nav2_runtime.yaml'
    # A run gets its own immutable parameter copy, including resolved BT paths.
    with legacy.open('x') as stream:
        yaml.safe_dump(document, stream, sort_keys=False)
    legacy_params = [str(legacy), {'use_sim_time': False}]

    def node(package, executable, name, parameters, **kwargs):
        return Node(package=package, executable=executable, name=name,
                    output='screen', parameters=parameters, **kwargs)

    # The deleted baseline used the centered 2026 field and simulated truth
    # odometry. Preserve those coordinates and geometry, including the old
    # physical robot, only inside this historical comparison experiment.
    model = str(scene / 'models/rmuc_2026_swerve.xml')
    physics = IncludeLaunchDescription(PythonLaunchDescriptionSource(
        str(sim_share / 'launch/ats_mujoco_sim.launch.py')),
        launch_arguments={
            'model_path': model, 'scene_file': model, 'map_ready_file': '',
            'map_wait_timeout_sec': '0.0', 'start_x': '-10.66', 'start_y': '1.47',
            'start_z': '0.42', 'start_yaw': '0.0', 'localization_source': 'truth',
            'use_viewer': 'false', 'show_viewer': 'false', 'launch_mujoco_rviz': 'false',
            'enable_static_walls': 'true', 'enable_lidar': 'true', 'lidar_backend': 'cpu',
            'lidar_model': 'mid360', 'lidar_downsample': '64', 'enable_tof': 'false',
            'lidar_topic': '/local_pointcloud', 'lidar_frame_id': 'front_mid360',
            'registered_scan_topic': '/registered_scan', 'registered_scan_frame_id': 'odom',
            'odom_topic': '/odometry', 'publish_map_to_odom_tf': 'true',
            'lidar_mount_preset': 'legacy_gimbal',
        }.items())
    localization = [
        node('ats_mujoco_sim', 'static_map_publisher', 'static_map_publisher', [{
            'use_sim_time': False, 'map_yaml_file': str(scene / 'maps/rmuc_2026.yaml'),
            'map_topic': '/map', 'frame_id': 'map'}]),
        node('terrain_analysis', 'terrainAnalysis', 'terrain_analysis', legacy_params),
        node('terrain_analysis_ext', 'terrainAnalysisExt', 'terrain_analysis_ext', legacy_params),
        Node(package='tf2_ros', executable='static_transform_publisher',
             name='nav2_compat_fake_yaw_identity', output='screen',
             arguments=['--frame-id', 'gimbal_yaw_odom', '--child-frame-id', 'gimbal_yaw_fake']),
        Node(package='tf2_ros', executable='static_transform_publisher',
             name='nav2_compat_base_link', output='screen',
             arguments=['--frame-id', 'base_footprint', '--child-frame-id', 'base_link']),
    ]
    navigation = [node('trajectory_optimizer', exe, name, legacy_params)
                  for exe, name in [('rc_esdf_map_node', 'rc_esdf_map'),
                                    ('trajectory_optimizer_node', 'trajectory_optimizer'),
                                    ('trajectory_speed_governor_node', 'trajectory_speed_governor')]]
    servers = [('nav2_controller', 'controller_server', [('cmd_vel', 'cmd_vel_controller')]),
               ('nav2_smoother', 'smoother_server', []),
               ('nav2_planner', 'planner_server', []),
               ('nav2_behaviors', 'behavior_server', [('cmd_vel', 'cmd_vel_nav2_result')]),
               ('nav2_bt_navigator', 'bt_navigator', []),
               ('nav2_waypoint_follower', 'waypoint_follower', []),
               ('nav2_velocity_smoother', 'velocity_smoother',
                [('cmd_vel', 'cmd_vel_controller_governed'), ('cmd_vel_smoothed', 'cmd_vel_nav2_result')])]
    for package, name, remappings in servers:
        navigation.append(node(package, name, name, legacy_params, remappings=remappings))
    navigation.extend([
        node('nav2_lifecycle_manager', 'lifecycle_manager', 'lifecycle_manager_navigation', [{
            'use_sim_time': False, 'autostart': True, 'node_names': [s[1] for s in servers]}]),
        node('ats_mujoco_sim', 'twist_to_motion_ctrl', 'twist_to_motion_ctrl', [{
            'use_sim_time': False, 'input_topic': '/cmd_vel_nav2_result',
            'output_topic': '/motion_control', 'max_linear_speed': 3.0, 'max_angular_z': 6.0}]),
    ])
    return [physics, TimerAction(period=2.0, actions=localization),
            TimerAction(period=9.0, actions=navigation)]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('compat_workspace'), DeclareLaunchArgument('evidence_output'),
        DeclareLaunchArgument('strict_xy_goal_tolerance', default_value=''),
        OpaqueFunction(function=make_nodes),
    ])
