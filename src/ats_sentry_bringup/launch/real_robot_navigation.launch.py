"""ATS real-robot navigation entrypoint.

The formal graph is fixed to the self-developed navigation chain:
ROGMap adapter -> ATS action/Goal Manager -> JPS/MINCO -> SE2 MPC.
There is no legacy-stack fallback in this entrypoint.  The static map is published by
``static_map_publisher.py`` with the original ``/map`` transient-local contract.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    bringup_dir = get_package_share_directory("ats_sentry_bringup")
    world = LaunchConfiguration("world")
    namespace = LaunchConfiguration("namespace")
    params_file = LaunchConfiguration("params_file")
    require_gimbal_status = LaunchConfiguration("require_gimbal_status")
    launch_behavior = LaunchConfiguration("launch_behavior")
    use_rviz = LaunchConfiguration("use_rviz")
    launch_fake_vel_transform = LaunchConfiguration("launch_fake_vel_transform")
    launch_chassis_vel_transform = LaunchConfiguration("launch_chassis_vel_transform")
    log_level = LaunchConfiguration("log_level")

    declarations = [
        DeclareLaunchArgument("world", default_value="rmuc_2026"),
        DeclareLaunchArgument("namespace", default_value=""),
        DeclareLaunchArgument(
            "params_file",
            default_value=os.path.join(bringup_dir, "params", "node_params.yaml"),
        ),
        DeclareLaunchArgument("require_gimbal_status", default_value="True"),
        DeclareLaunchArgument("launch_behavior", default_value="True"),
        DeclareLaunchArgument("use_rviz", default_value="False"),
        DeclareLaunchArgument("launch_fake_vel_transform", default_value="True"),
        DeclareLaunchArgument("launch_chassis_vel_transform", default_value="True"),
        DeclareLaunchArgument("log_level", default_value="info"),
    ]
    bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(bringup_dir, "launch", "bringup.launch.py")),
        launch_arguments={
            "world": world,
            "namespace": namespace,
            "params_file": params_file,
            "use_sim_time": "False",
            "launch_fake_vel_transform": launch_fake_vel_transform,
            "launch_chassis_vel_transform": launch_chassis_vel_transform,
            "require_gimbal_status": require_gimbal_status,
            "launch_behavior": launch_behavior,
            "launch_small_gicp_relocalization": "True",
            "launch_localization_fusion": "True",
            "use_rviz": use_rviz,
            "log_level": log_level,
        }.items(),
    )
    ld = LaunchDescription()
    for declaration in declarations:
        ld.add_action(declaration)
    ld.add_action(bringup)
    return ld
