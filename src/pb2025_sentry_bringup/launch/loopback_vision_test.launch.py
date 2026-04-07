import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    bringup_dir = get_package_share_directory("pb2025_sentry_bringup")
    behavior_dir = get_package_share_directory("pb2025_sentry_behavior")

    behavior_params_file = LaunchConfiguration("behavior_params_file")
    use_rviz = LaunchConfiguration("use_rviz")
    initial_x = LaunchConfiguration("initial_x")
    initial_y = LaunchConfiguration("initial_y")
    initial_yaw = LaunchConfiguration("initial_yaw")
    publish_decision_mode = LaunchConfiguration("publish_decision_mode")
    decision_mode = LaunchConfiguration("decision_mode")
    publish_vision_target = LaunchConfiguration("publish_vision_target")
    vision_tracking = LaunchConfiguration("vision_tracking")
    vision_nav_hold = LaunchConfiguration("vision_nav_hold")
    vision_fire_permitted = LaunchConfiguration("vision_fire_permitted")
    vision_target_id = LaunchConfiguration("vision_target_id")
    vision_suggested_goal_index = LaunchConfiguration("vision_suggested_goal_index")
    vision_confidence = LaunchConfiguration("vision_confidence")
    vision_target_distance = LaunchConfiguration("vision_target_distance")
    vision_target_yaw = LaunchConfiguration("vision_target_yaw")
    vision_target_pitch = LaunchConfiguration("vision_target_pitch")
    vision_target_position_gimbal_x = LaunchConfiguration(
        "vision_target_position_gimbal_x"
    )
    vision_target_position_gimbal_y = LaunchConfiguration(
        "vision_target_position_gimbal_y"
    )
    vision_target_position_gimbal_z = LaunchConfiguration(
        "vision_target_position_gimbal_z"
    )

    # 这是给视觉融合链专门准备的快捷入口：
    # 1. 默认加载 vision_test 行为树；
    # 2. 默认开启假视觉目标；
    # 3. 同时把常用视觉仿真参数透传出来，便于直接在 launch 命令里改。
    loopback_vision_test = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(bringup_dir, "launch", "loopback_decision_sim.launch.py")
        ),
        launch_arguments={
            "behavior_params_file": behavior_params_file,
            "use_rviz": use_rviz,
            "initial_x": initial_x,
            "initial_y": initial_y,
            "initial_yaw": initial_yaw,
            "publish_decision_mode": publish_decision_mode,
            "decision_mode": decision_mode,
            "publish_vision_target": publish_vision_target,
            "vision_tracking": vision_tracking,
            "vision_nav_hold": vision_nav_hold,
            "vision_fire_permitted": vision_fire_permitted,
            "vision_target_id": vision_target_id,
            "vision_suggested_goal_index": vision_suggested_goal_index,
            "vision_confidence": vision_confidence,
            "vision_target_distance": vision_target_distance,
            "vision_target_yaw": vision_target_yaw,
            "vision_target_pitch": vision_target_pitch,
            "vision_target_position_gimbal_x": vision_target_position_gimbal_x,
            "vision_target_position_gimbal_y": vision_target_position_gimbal_y,
            "vision_target_position_gimbal_z": vision_target_position_gimbal_z,
        }.items(),
    )

    ld = LaunchDescription()

    ld.add_action(
        DeclareLaunchArgument(
            "behavior_params_file",
            default_value=os.path.join(
                behavior_dir, "params", "sentry_behavior_vision_test.yaml"
            ),
            description="视觉融合测试时使用的行为树参数文件。",
        )
    )
    ld.add_action(
        DeclareLaunchArgument(
            "use_rviz",
            default_value="True",
            description="是否同时启动 RViz 观察导航与视觉接管效果。",
        )
    )
    ld.add_action(
        DeclareLaunchArgument(
            "initial_x",
            default_value="0.0",
            description="loopback 机器人初始 x 坐标。",
        )
    )
    ld.add_action(
        DeclareLaunchArgument(
            "initial_y",
            default_value="0.0",
            description="loopback 机器人初始 y 坐标。",
        )
    )
    ld.add_action(
        DeclareLaunchArgument(
            "initial_yaw",
            default_value="0.0",
            description="loopback 机器人初始 yaw。",
        )
    )
    ld.add_action(
        DeclareLaunchArgument(
            "publish_decision_mode",
            default_value="True",
            description="是否继续发布 decision/sim_mode，便于测试视觉失效后的回退分支。",
        )
    )
    ld.add_action(
        DeclareLaunchArgument(
            "decision_mode",
            default_value="patrol",
            description="视觉分支失效时，底层仿真决策模式默认值。",
        )
    )
    ld.add_action(
        DeclareLaunchArgument(
            "publish_vision_target",
            default_value="True",
            description="是否发布假视觉目标。",
        )
    )
    ld.add_action(
        DeclareLaunchArgument(
            "vision_tracking",
            default_value="True",
            description="假视觉是否认为当前已经稳定追踪到目标。",
        )
    )
    ld.add_action(
        DeclareLaunchArgument(
            "vision_nav_hold",
            default_value="True",
            description="假视觉是否请求行为树进入视觉接管分支。",
        )
    )
    ld.add_action(
        DeclareLaunchArgument(
            "vision_fire_permitted",
            default_value="False",
            description="假视觉是否允许发弹，当前主要用于接口观测。",
        )
    )
    ld.add_action(
        DeclareLaunchArgument(
            "vision_target_id",
            default_value="7",
            description="假视觉目标编号。",
        )
    )
    ld.add_action(
        DeclareLaunchArgument(
            "vision_suggested_goal_index",
            default_value="-1",
            description="假视觉直接指定的建议导航点编号，-1 表示回退到锚点。",
        )
    )
    ld.add_action(
        DeclareLaunchArgument(
            "vision_confidence",
            default_value="1.0",
            description="假视觉置信度。",
        )
    )
    ld.add_action(
        DeclareLaunchArgument(
            "vision_target_distance",
            default_value="3.0",
            description="假视觉目标距离，单位 m。",
        )
    )
    ld.add_action(
        DeclareLaunchArgument(
            "vision_target_yaw",
            default_value="0.0",
            description="假视觉给出的云台 yaw 指令，单位 rad。",
        )
    )
    ld.add_action(
        DeclareLaunchArgument(
            "vision_target_pitch",
            default_value="0.0",
            description="假视觉给出的云台 pitch 指令，单位 rad。",
        )
    )
    ld.add_action(
        DeclareLaunchArgument(
            "vision_target_position_gimbal_x",
            default_value="1.0",
            description="假视觉目标在云台坐标系下的 x 坐标。",
        )
    )
    ld.add_action(
        DeclareLaunchArgument(
            "vision_target_position_gimbal_y",
            default_value="0.0",
            description="假视觉目标在云台坐标系下的 y 坐标。",
        )
    )
    ld.add_action(
        DeclareLaunchArgument(
            "vision_target_position_gimbal_z",
            default_value="0.0",
            description="假视觉目标在云台坐标系下的 z 坐标。",
        )
    )
    ld.add_action(loopback_vision_test)
    return ld
