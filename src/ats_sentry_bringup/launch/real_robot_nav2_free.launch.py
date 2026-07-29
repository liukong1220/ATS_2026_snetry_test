"""
实车 Nav2-free 自研链入口（P6 零.1）.

本入口与 Nav2 对照 profile 互斥：它把 `launch_nav2` 钉死为 `False`，
因此运行图中不出现任何 Nav2 节点、任何 lifecycle manager，也不存在 `/plan`
的生产者或消费者；`/map` 由 `ats_nav_bringup/static_map_publisher.py` 以
transient-local 契约发布。

命令通路是唯一的：

    minco_planner -> ats_goal_manager(/planner/execution_command)
      -> ats_swerve_mpc -> /cmd_vel -> standard_robot_pp_ros2 speed_vector

MPC 之后不存在任何级：`fake_vel_transform` 与 `chassis_vel_transform` 在本
入口被结构性禁止（它们会按云台 yaw 二次旋转 `[vx, vy]`，或叠加 `cmd_spin`
的车体 `wz`），因此 `[vx, vy, wz]` 的数值与方向从 MPC 输出到串口不被改变。

参数权威：三条自研链节点只从各自包内的 `*_reality.yaml` 读取，
`node_params.yaml` 与 `reality/nav2_params.yaml` 不再给出同名段。

本入口只用于本轮允许的两级：不通电检查、抬轮 HIL。落地行走不在本轮范围内。
`require_gimbal_status` 默认保持 `True`；只有受控 HIL 才可显式改为 `False`，
且此时禁止 `BODY_YAW_FOLLOW`，禁止伪造云台 ack。
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
    planning_grid_owner = LaunchConfiguration("planning_grid_owner")
    require_gimbal_status = LaunchConfiguration("require_gimbal_status")
    launch_behavior = LaunchConfiguration("launch_behavior")
    use_rviz = LaunchConfiguration("use_rviz")
    use_composition = LaunchConfiguration("use_composition")
    log_level = LaunchConfiguration("log_level")

    declare_world_cmd = DeclareLaunchArgument(
        "world",
        default_value="rmuc_2026",
        description=(
            "Field asset name shared by the map YAML and the prior PCD. "
            "params/map/<world>.yaml must exist; pcd/<world>.pcd is downloaded "
            "separately per pcd/readme.md and its absence is a stop condition "
            "for the localization chain, not a degraded mode."
        ),
    )

    declare_namespace_cmd = DeclareLaunchArgument(
        "namespace", default_value="", description="Top-level namespace"
    )

    declare_params_file_cmd = DeclareLaunchArgument(
        "params_file",
        default_value=os.path.join(bringup_dir, "params", "node_params.yaml"),
        description=(
            "Shared parameter file for sensors, localization and the serial "
            "driver. The MINCO/goal-manager/MPC sections live only in their "
            "own *_reality.yaml files."
        ),
    )

    declare_planning_grid_owner_cmd = DeclareLaunchArgument(
        "planning_grid_owner",
        default_value="rog_map",
        choices=["rc_esdf", "rog_map"],
        description="Single /rc_esdf/planning_grid owner for this profile",
    )

    declare_require_gimbal_status_cmd = DeclareLaunchArgument(
        "require_gimbal_status",
        default_value="True",
        description=(
            "Keep True unless running a controlled wheels-up HIL profile. "
            "While False, BODY_YAW_FOLLOW is forbidden and no ack may be faked."
        ),
    )

    declare_launch_behavior_cmd = DeclareLaunchArgument(
        "launch_behavior",
        default_value="True",
        description=(
            "Set False for the wheels-up HIL profile. The behavior tree issues "
            "navigation goals on its own, so leaving it running makes the "
            "measured zeroing latency impossible to attribute to one "
            "authorization transition."
        ),
    )

    declare_use_rviz_cmd = DeclareLaunchArgument(
        "use_rviz", default_value="False", description="Whether to start RViz"
    )

    declare_use_composition_cmd = DeclareLaunchArgument(
        "use_composition",
        default_value="False",
        description="Whether to use composed bringup",
    )

    declare_log_level_cmd = DeclareLaunchArgument(
        "log_level", default_value="info", description="log level"
    )

    bringup_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(bringup_dir, "launch", "bringup.launch.py")
        ),
        launch_arguments={
            "world": world,
            "namespace": namespace,
            "params_file": params_file,
            "use_sim_time": "False",
            # 本 profile 的互斥核心：没有 Nav2，就没有 lifecycle manager，
            # 也没有 /plan；自研链由 launch_swerve_mpc 打开。
            "launch_nav2": "False",
            "launch_swerve_mpc": "True",
            # MPC 之后不允许任何变换级。
            "launch_fake_vel_transform": "False",
            "launch_chassis_vel_transform": "False",
            "mpc_cmd_vel_topic": "/cmd_vel",
            # Nav2 对照链专属节点（依赖 /plan 与 Nav2 cmd_vel）在本 profile 关闭。
            "launch_trajectory_optimizer": "False",
            "launch_joy_teleop": "False",
            "planning_grid_owner": planning_grid_owner,
            "require_gimbal_status": require_gimbal_status,
            # 抬轮 HIL 用 launch_behavior:=False，让授权只由测试脚本触发。
            "launch_behavior": launch_behavior,
            "launch_small_gicp_relocalization": "True",
            "launch_localization_fusion": "True",
            "use_rviz": use_rviz,
            "use_composition": use_composition,
            "log_level": log_level,
        }.items(),
    )

    ld = LaunchDescription()
    ld.add_action(declare_world_cmd)
    ld.add_action(declare_namespace_cmd)
    ld.add_action(declare_params_file_cmd)
    ld.add_action(declare_planning_grid_owner_cmd)
    ld.add_action(declare_require_gimbal_status_cmd)
    ld.add_action(declare_launch_behavior_cmd)
    ld.add_action(declare_use_rviz_cmd)
    ld.add_action(declare_use_composition_cmd)
    ld.add_action(declare_log_level_cmd)
    ld.add_action(bringup_cmd)
    return ld
