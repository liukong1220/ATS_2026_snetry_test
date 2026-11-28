"""ATS real-robot navigation entrypoint.

The formal graph is fixed to the self-developed navigation chain:
ROGMap adapter -> ATS action/Goal Manager -> JPS/MINCO -> SE2 MPC.
There is no legacy-stack fallback in this entrypoint.  The static map is published by
``static_map_publisher.py`` with the original ``/map`` transient-local contract.
"""

import os

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression, TextSubstitution


def _load_real_robot_defaults(bringup_dir):
    cfg_path = os.path.join(bringup_dir, "params", "launch_defaults.yaml")
    with open(cfg_path, "r", encoding="utf-8") as handle:
        raw = yaml.safe_load(handle) or {}
    merged = {}
    for section in ("common", "localization", "auto_mapping"):
        for k, v in (raw.get(section) or {}).items():
            merged[str(k)] = str(v)
    for k, v in ((raw.get("profiles") or {}).get("real_robot") or {}).items():
        merged[str(k)] = str(v)
    return merged


def generate_launch_description():
    bringup_dir = get_package_share_directory("ats_sentry_bringup")
    defaults = _load_real_robot_defaults(bringup_dir)
    world = LaunchConfiguration("world")
    prior_pcd_file = LaunchConfiguration("prior_pcd_file")
    namespace = LaunchConfiguration("namespace")
    params_file = LaunchConfiguration("params_file")
    require_gimbal_status = LaunchConfiguration("require_gimbal_status")
    launch_behavior = LaunchConfiguration("launch_behavior")
    use_rviz = LaunchConfiguration("use_rviz")
    lidar_mount_mode = LaunchConfiguration("lidar_mount_mode")
    gimbal_use_base_footprint_odom = LaunchConfiguration("gimbal_use_base_footprint_odom")
    launch_fake_vel_transform = LaunchConfiguration("launch_fake_vel_transform")
    launch_chassis_vel_transform = LaunchConfiguration("launch_chassis_vel_transform")
    slam = LaunchConfiguration("slam")
    launch_small_gicp_relocalization = LaunchConfiguration("launch_small_gicp_relocalization")
    launch_localization_fusion = LaunchConfiguration("launch_localization_fusion")
    log_level = LaunchConfiguration("log_level")

    declarations = [
        DeclareLaunchArgument("world", default_value=defaults["world"]),
        DeclareLaunchArgument(
            "prior_pcd_file",
            default_value=[TextSubstitution(text=os.path.join(bringup_dir, "pcd", "")), world, TextSubstitution(text=".pcd")],
            description="Matching-world prior PCD already expressed in map_frame; required before hardware startup.",
        ),
        DeclareLaunchArgument("namespace", default_value=defaults["namespace"]),
        DeclareLaunchArgument(
            "params_file",
            default_value=os.path.join(bringup_dir, "params", "node_params.yaml"),
        ),
        DeclareLaunchArgument("require_gimbal_status", default_value=defaults["require_gimbal_status"]),
        DeclareLaunchArgument("launch_behavior", default_value=defaults["launch_behavior"]),
        DeclareLaunchArgument("use_rviz", default_value=defaults["use_rviz"]),
        DeclareLaunchArgument("slam", default_value=defaults["slam"]),
        DeclareLaunchArgument(
            "launch_small_gicp_relocalization",
            default_value=defaults["launch_small_gicp_relocalization"],
        ),
        DeclareLaunchArgument(
            "launch_localization_fusion",
            default_value=defaults["launch_localization_fusion"],
        ),
        DeclareLaunchArgument(
            "lidar_mount_mode",
            default_value="gimbal",
            description="Real-robot LiDAR mount mode: 'gimbal' (dual-yaw big-yaw mount, default) or 'chassis' (fixed on chassis).",
        ),
        DeclareLaunchArgument(
            "gimbal_use_base_footprint_odom",
            default_value="True",
            description="In gimbal mode, output odometry/localization in base_footprint (chassis frame) and disable linear big_yaw rotation.",
        ),
        DeclareLaunchArgument("launch_fake_vel_transform", default_value="False"),
        DeclareLaunchArgument(
            "launch_chassis_vel_transform",
            default_value=PythonExpression([
                "'True' if '", lidar_mount_mode, "'.lower() == 'gimbal' else 'False'",
            ]),
        ),
        DeclareLaunchArgument("log_level", default_value=defaults["log_level"]),
    ]
    bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(bringup_dir, "launch", "bringup.launch.py")),
        launch_arguments={
            "world": world,
            "prior_pcd_file": prior_pcd_file,
            "namespace": namespace,
            "params_file": params_file,
            "use_sim_time": "False",
            "slam": slam,
            "lidar_mount_mode": lidar_mount_mode,
            "gimbal_use_base_footprint_odom": gimbal_use_base_footprint_odom,
            "launch_fake_vel_transform": launch_fake_vel_transform,
            "launch_chassis_vel_transform": launch_chassis_vel_transform,
            "require_serial_link": "True",
            "require_gimbal_status": require_gimbal_status,
            "launch_behavior": launch_behavior,
            "launch_small_gicp_relocalization": launch_small_gicp_relocalization,
            "launch_localization_fusion": launch_localization_fusion,
            "use_rviz": use_rviz,
            "log_level": log_level,
        }.items(),
    )
    ld = LaunchDescription()
    for declaration in declarations:
        ld.add_action(declaration)
    ld.add_action(bringup)
    return ld
