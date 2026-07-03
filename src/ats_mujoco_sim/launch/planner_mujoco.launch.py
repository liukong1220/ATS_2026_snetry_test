#!/usr/bin/env python3
"""Generate a map and launch only the swerve MuJoCo simulation."""

from __future__ import annotations

import uuid
from pathlib import Path

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import ExecuteProcess
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.substitutions import PathJoinSubstitution
from launch.substitutions import PythonExpression


def _package_root() -> Path:
    return Path(__file__).resolve().parents[1]


def _default_map_root() -> str:
    return "/tmp/ats_mujoco_sim_maps"


def _default_map_config() -> str:
    return (_package_root() / "config" / "random_map.yaml").as_posix()


def _quoted(value):
    return ["'", value, "'"]


def _map_directory_substitution(
    output_root: LaunchConfiguration,
    map_name: LaunchConfiguration,
    seed: LaunchConfiguration,
):
    return PythonExpression(
        [
            "__import__('ats_mujoco_sim.terrain_assets', "
            "fromlist=['map_output_dir']).map_output_dir(",
            *_quoted(output_root),
            ", ",
            *_quoted(map_name),
            ", int(",
            *_quoted(seed),
            ")).as_posix()",
        ]
    )


def _map_generation_process():
    return ExecuteProcess(
        name="generate_ats_mujoco_map_assets",
        cmd=[
            "python3",
            "-m",
            "ats_mujoco_sim.terrain_assets",
            "--config",
            LaunchConfiguration("map_config"),
            "--output-root",
            LaunchConfiguration("output_root"),
            "--map-name",
            LaunchConfiguration("map_name"),
            "--seed",
            LaunchConfiguration("seed"),
            "--ready-token",
            LaunchConfiguration("map_ready_token"),
        ],
        output="screen",
    )


def generate_launch_description() -> LaunchDescription:
    ld = LaunchDescription()

    declare_launch_arguments = [
        DeclareLaunchArgument("map_config", default_value=_default_map_config()),
        DeclareLaunchArgument("output_root", default_value=_default_map_root()),
        DeclareLaunchArgument("map_name", default_value=""),
        # seed=-1 每次重新随机，并覆盖 map/random 目录。
        DeclareLaunchArgument("seed", default_value="-1"),
        DeclareLaunchArgument("resolution", default_value="0.03"),
        DeclareLaunchArgument("map_ready_token", default_value=str(uuid.uuid4())),
        DeclareLaunchArgument("map_wait_timeout_sec", default_value="30.0"),
        DeclareLaunchArgument("odom_topic", default_value="/localization"),
        DeclareLaunchArgument("pose_cmd_topic", default_value="/simulation/PoseSub"),
        DeclareLaunchArgument("start_x", default_value="0.0"),
        DeclareLaunchArgument("start_y", default_value="0.0"),
        DeclareLaunchArgument("start_z", default_value="0.18"),
        DeclareLaunchArgument("start_yaw", default_value="0.0"),
        DeclareLaunchArgument("use_viewer", default_value="true"),
        DeclareLaunchArgument("show_viewer", default_value=LaunchConfiguration("use_viewer")),
        DeclareLaunchArgument("sim_rate_hz", default_value="300.0"),
        DeclareLaunchArgument("feedback_rate_hz", default_value="10.0"),
        DeclareLaunchArgument("truth_rate_hz", default_value="10.0"),
        DeclareLaunchArgument("command_timeout", default_value="0.5"),
        DeclareLaunchArgument("enable_lidar", default_value="true"),
        DeclareLaunchArgument("lidar_backend", default_value="gpu"),
        DeclareLaunchArgument("lidar_line_mode", default_value="96"),   # 48
        DeclareLaunchArgument("lidar_rate_hz", default_value="10.0"),
        DeclareLaunchArgument("lidar_rate_clock", default_value="wall"),
        DeclareLaunchArgument("lidar_state_rate_hz", default_value="30.0"),
        DeclareLaunchArgument("lidar_horizontal_resolution_deg", default_value="0.4"),
        DeclareLaunchArgument(
            "lidar_topic",
            default_value="/local_pointcloud",
        ),
        DeclareLaunchArgument("enable_tof", default_value="true"),
        DeclareLaunchArgument("tof_backend", default_value="cpu"),
        DeclareLaunchArgument("tof_range", default_value="1.0"),
        DeclareLaunchArgument("tof_min_range", default_value="0.03"),
        DeclareLaunchArgument("tof_rate_hz", default_value="10.0"),
        DeclareLaunchArgument("tof_width", default_value="248"),
        DeclareLaunchArgument("tof_height", default_value="180"),
        DeclareLaunchArgument("tof_horizontal_fov_deg", default_value="98.0"),
        DeclareLaunchArgument("tof_vertical_fov_deg", default_value="72.0"),
        DeclareLaunchArgument("tof_footprint_length", default_value="0.73"),
        DeclareLaunchArgument("tof_footprint_width", default_value="0.54"),
        DeclareLaunchArgument("tof_footprint_expand", default_value="1.0"),
        DeclareLaunchArgument(
            "tof_footprint_resolution",
            default_value=LaunchConfiguration("resolution"),
        ),
        DeclareLaunchArgument("tof_footprint_z_min", default_value="0.1"),
        DeclareLaunchArgument("tof_footprint_z_max", default_value="0.65"),
        DeclareLaunchArgument(
            "merged_tof_topic",
            default_value="/perception/tof/points_merged",
        ),
    ]
    for action in declare_launch_arguments:
        ld.add_action(action)

    output_root = LaunchConfiguration("output_root")
    map_name = LaunchConfiguration("map_name")
    seed = LaunchConfiguration("seed")
    map_directory = _map_directory_substitution(output_root, map_name, seed)
    map_manifest_path = PathJoinSubstitution([map_directory, "map_manifest.yaml"])
    map_ready_file = PathJoinSubstitution([map_directory, "assets.ready"])
    scene_file = PathJoinSubstitution([map_directory, "model", "swerve.xml"])

    sim_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            (_package_root() / "launch" / "ats_mujoco_sim.launch.py").as_posix()
        ),
        launch_arguments={
            "model_path": scene_file,
            "scene_file": scene_file,
            "map_dir": map_directory,
            "map_manifest_path": map_manifest_path,
            "map_ready_file": map_ready_file,
            "map_ready_token": LaunchConfiguration("map_ready_token"),
            "map_wait_timeout_sec": LaunchConfiguration("map_wait_timeout_sec"),
            "odom_topic": LaunchConfiguration("odom_topic"),
            "pose_cmd_topic": LaunchConfiguration("pose_cmd_topic"),
            "start_x": LaunchConfiguration("start_x"),
            "start_y": LaunchConfiguration("start_y"),
            "start_z": LaunchConfiguration("start_z"),
            "start_yaw": LaunchConfiguration("start_yaw"),
            "use_viewer": LaunchConfiguration("use_viewer"),
            "show_viewer": LaunchConfiguration("show_viewer"),
            "sim_rate_hz": LaunchConfiguration("sim_rate_hz"),
            "feedback_rate_hz": LaunchConfiguration("feedback_rate_hz"),
            "truth_rate_hz": LaunchConfiguration("truth_rate_hz"),
            "command_timeout": LaunchConfiguration("command_timeout"),
            "enable_lidar": LaunchConfiguration("enable_lidar"),
            "lidar_backend": LaunchConfiguration("lidar_backend"),
            "lidar_line_mode": LaunchConfiguration("lidar_line_mode"),
            "lidar_rate_hz": LaunchConfiguration("lidar_rate_hz"),
            "lidar_rate_clock": LaunchConfiguration("lidar_rate_clock"),
            "lidar_state_rate_hz": LaunchConfiguration("lidar_state_rate_hz"),
            "lidar_horizontal_resolution_deg": LaunchConfiguration(
                "lidar_horizontal_resolution_deg"
            ),
            "lidar_topic": LaunchConfiguration("lidar_topic"),
            "enable_tof": LaunchConfiguration("enable_tof"),
            "tof_backend": LaunchConfiguration("tof_backend"),
            "tof_range": LaunchConfiguration("tof_range"),
            "tof_min_range": LaunchConfiguration("tof_min_range"),
            "tof_rate_hz": LaunchConfiguration("tof_rate_hz"),
            "tof_width": LaunchConfiguration("tof_width"),
            "tof_height": LaunchConfiguration("tof_height"),
            "tof_horizontal_fov_deg": LaunchConfiguration("tof_horizontal_fov_deg"),
            "tof_vertical_fov_deg": LaunchConfiguration("tof_vertical_fov_deg"),
            "tof_footprint_length": LaunchConfiguration("tof_footprint_length"),
            "tof_footprint_width": LaunchConfiguration("tof_footprint_width"),
            "tof_footprint_expand": LaunchConfiguration("tof_footprint_expand"),
            "tof_footprint_resolution": LaunchConfiguration("tof_footprint_resolution"),
            "tof_footprint_z_min": LaunchConfiguration("tof_footprint_z_min"),
            "tof_footprint_z_max": LaunchConfiguration("tof_footprint_z_max"),
            "merged_tof_topic": LaunchConfiguration("merged_tof_topic"),
        }.items(),
    )

    ld.add_action(_map_generation_process())
    ld.add_action(sim_launch)
    return ld
