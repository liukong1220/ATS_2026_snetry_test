
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    SetEnvironmentVariable,
    TimerAction,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node


def generate_launch_description():
    bringup_dir = get_package_share_directory("pb2025_sentry_bringup")
    behavior_dir = get_package_share_directory("pb2025_sentry_behavior")
    nav_bringup_dir = get_package_share_directory("pb2025_nav_bringup")
    loopback_dir = get_package_share_directory("nav2_loopback_sim")

    map_yaml_file = LaunchConfiguration("map")
    nav2_params_file = LaunchConfiguration("nav2_params_file")
    behavior_params_file = LaunchConfiguration("behavior_params_file")
    autostart = LaunchConfiguration("autostart")
    use_respawn = LaunchConfiguration("use_respawn")
    use_rviz = LaunchConfiguration("use_rviz")
    rviz_config_file = LaunchConfiguration("rviz_config_file")
    log_level = LaunchConfiguration("log_level")
    initial_x = LaunchConfiguration("initial_x")
    initial_y = LaunchConfiguration("initial_y")
    initial_yaw = LaunchConfiguration("initial_yaw")
    publish_decision_mode = LaunchConfiguration("publish_decision_mode")
    decision_mode = LaunchConfiguration("decision_mode")
    decision_mode_topic = LaunchConfiguration("decision_mode_topic")
    publish_referee_inputs = LaunchConfiguration("publish_referee_inputs")
    current_hp = LaunchConfiguration("current_hp")
    projectile_allowance_17mm = LaunchConfiguration("projectile_allowance_17mm")
    publish_vision_target = LaunchConfiguration("publish_vision_target")
    vision_tracking = LaunchConfiguration("vision_tracking")
    vision_nav_hold = LaunchConfiguration("vision_nav_hold")
    vision_fire_permitted = LaunchConfiguration("vision_fire_permitted")
    vision_target_type = LaunchConfiguration("vision_target_type")
    vision_target_id = LaunchConfiguration("vision_target_id")
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
    vision_target_position_map_x = LaunchConfiguration("vision_target_position_map_x")
    vision_target_position_map_y = LaunchConfiguration("vision_target_position_map_y")
    vision_target_position_map_z = LaunchConfiguration("vision_target_position_map_z")
    vision_has_target_position_map = LaunchConfiguration("vision_has_target_position_map")
    vision_target_position_map_frame = LaunchConfiguration(
        "vision_target_position_map_frame"
    )

    stdout_linebuf_envvar = SetEnvironmentVariable(
        "RCUTILS_LOGGING_BUFFERED_STREAM", "1"
    )
    colorized_output_envvar = SetEnvironmentVariable("RCUTILS_COLORIZED_OUTPUT", "1")

    declare_map_yaml_cmd = DeclareLaunchArgument(
        "map",
        default_value=os.path.join(bringup_dir, "map", "rmuc_2025.yaml"),
        description="Full path to the map yaml file used by loopback simulation.",
    )

    declare_nav2_params_cmd = DeclareLaunchArgument(
        "nav2_params_file",
        default_value=os.path.join(loopback_dir, "nav2_params.yaml"),
        description="Parameter file for Nav2 loopback simulation.",
    )

    declare_behavior_params_cmd = DeclareLaunchArgument(
        "behavior_params_file",
        default_value=os.path.join(
            behavior_dir, "params", "sentry_behavior_loopback.yaml"
        ),
        description="Parameter file for the sentry behavior tree nodes.",
    )

    declare_autostart_cmd = DeclareLaunchArgument(
        "autostart",
        default_value="true",
        description="Automatically startup the Nav2 stack.",
    )

    declare_use_respawn_cmd = DeclareLaunchArgument(
        "use_respawn",
        default_value="False",
        description="Whether to respawn nodes on crash when composition is disabled.",
    )

    declare_use_rviz_cmd = DeclareLaunchArgument(
        "use_rviz",
        default_value="True",
        description="Whether to start RViz.",
    )

    declare_rviz_config_file_cmd = DeclareLaunchArgument(
        "rviz_config_file",
        default_value=os.path.join(bringup_dir, "rviz", "sentry_default_view.rviz"),
        description="Full path to the RViz config file to use.",
    )

    declare_log_level_cmd = DeclareLaunchArgument(
        "log_level",
        default_value="info",
        description="Log level for launched nodes.",
    )

    declare_initial_x_cmd = DeclareLaunchArgument(
        "initial_x",
        default_value="-0.0",
        description="Initial robot x position in map frame.",
    )

    declare_initial_y_cmd = DeclareLaunchArgument(
        "initial_y",
        default_value="-0.0",
        description="Initial robot y position in map frame.",
    )

    declare_initial_yaw_cmd = DeclareLaunchArgument(
        "initial_yaw",
        default_value="-0.3",
        description="Initial robot yaw in map frame.",
    )

    declare_publish_decision_mode_cmd = DeclareLaunchArgument(
        "publish_decision_mode",
        default_value="True",
        description="Whether the fake loopback node publishes decision/sim_mode.",
    )

    declare_decision_mode_cmd = DeclareLaunchArgument(
        "decision_mode",
        default_value="patrol",
        description="Default decision simulation mode: patrol/anchor/retreat/safe.",
    )

    declare_decision_mode_topic_cmd = DeclareLaunchArgument(
        "decision_mode_topic",
        default_value="decision/sim_mode",
        description="Topic used by the fake loopback node to publish simulation mode.",
    )

    declare_publish_referee_inputs_cmd = DeclareLaunchArgument(
        "publish_referee_inputs",
        default_value="True",
        description="Whether the fake loopback node publishes referee topics. Keep true to test the same resource policy used on the real robot.",
    )

    declare_current_hp_cmd = DeclareLaunchArgument(
        "current_hp",
        default_value="400",
        description="Fake referee current_hp used by the unified resource decision policy.",
    )

    declare_projectile_allowance_17mm_cmd = DeclareLaunchArgument(
        "projectile_allowance_17mm",
        default_value="200",
        description="Fake referee 17mm ammo used by the unified resource decision policy.",
    )

    declare_publish_vision_target_cmd = DeclareLaunchArgument(
        "publish_vision_target",
        default_value="False",
        description="Whether the fake loopback node publishes vision/target.",
    )

    declare_vision_tracking_cmd = DeclareLaunchArgument(
        "vision_tracking",
        default_value="False",
        description="Whether the fake vision target is considered tracked.",
    )

    declare_vision_nav_hold_cmd = DeclareLaunchArgument(
        "vision_nav_hold",
        default_value="True",
        description="Whether the fake vision target asks navigation to enter hold/override mode.",
    )

    declare_vision_fire_permitted_cmd = DeclareLaunchArgument(
        "vision_fire_permitted",
        default_value="False",
        description="Whether the fake vision target allows firing.",
    )

    declare_vision_target_type_cmd = DeclareLaunchArgument(
        "vision_target_type",
        default_value="0",
        description="Fake vision target type. 0=unknown/default and 7=outpost both disable vision follow.",
    )

    declare_vision_target_id_cmd = DeclareLaunchArgument(
        "vision_target_id",
        default_value="7",
        description="Fake vision target id.",
    )

    declare_vision_confidence_cmd = DeclareLaunchArgument(
        "vision_confidence",
        default_value="1.0",
        description="Fake vision confidence.",
    )

    declare_vision_target_distance_cmd = DeclareLaunchArgument(
        "vision_target_distance",
        default_value="3.0",
        description="Fake vision target distance in meters.",
    )

    declare_vision_target_yaw_cmd = DeclareLaunchArgument(
        "vision_target_yaw",
        default_value="0.0",
        description="Fake vision yaw command in rad.",
    )

    declare_vision_target_pitch_cmd = DeclareLaunchArgument(
        "vision_target_pitch",
        default_value="0.0",
        description="Fake vision pitch command in rad.",
    )

    declare_vision_target_position_gimbal_x_cmd = DeclareLaunchArgument(
        "vision_target_position_gimbal_x",
        default_value="1.0",
        description="Fake vision target gimbal-frame x position in meters.",
    )

    declare_vision_target_position_gimbal_y_cmd = DeclareLaunchArgument(
        "vision_target_position_gimbal_y",
        default_value="0.0",
        description="Fake vision target gimbal-frame y position in meters.",
    )

    declare_vision_target_position_gimbal_z_cmd = DeclareLaunchArgument(
        "vision_target_position_gimbal_z",
        default_value="0.0",
        description="Fake vision target gimbal-frame z position in meters.",
    )

    declare_vision_target_position_map_x_cmd = DeclareLaunchArgument(
        "vision_target_position_map_x",
        default_value="0.0",
        description="Fake vision target map x position in meters.",
    )

    declare_vision_target_position_map_y_cmd = DeclareLaunchArgument(
        "vision_target_position_map_y",
        default_value="0.0",
        description="Fake vision target map y position in meters.",
    )

    declare_vision_target_position_map_z_cmd = DeclareLaunchArgument(
        "vision_target_position_map_z",
        default_value="0.0",
        description="Fake vision target map z position in meters.",
    )
    declare_vision_has_target_position_map_cmd = DeclareLaunchArgument(
        "vision_has_target_position_map",
        default_value="False",
        description="Whether fake vision publishes a valid target_position_map.",
    )
    declare_vision_target_position_map_frame_cmd = DeclareLaunchArgument(
        "vision_target_position_map_frame",
        default_value="map",
        description="Frame id used by fake vision target_position_map.",
    )

    map_server_cmd = Node(
        package="nav2_map_server",
        executable="map_server",
        name="map_server",
        output="screen",
        parameters=[nav2_params_file, {"yaml_filename": map_yaml_file}],
        arguments=["--ros-args", "--log-level", log_level],
    )

    map_server_lifecycle_cmd = Node(
        package="nav2_lifecycle_manager",
        executable="lifecycle_manager",
        name="lifecycle_manager_map_server",
        output="screen",
        parameters=[
            {"use_sim_time": True},
            {"autostart": autostart},
            {"node_names": ["map_server"]},
        ],
        arguments=["--ros-args", "--log-level", log_level],
    )

    loopback_sim_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(loopback_dir, "launch", "loopback_simulation.launch.py")
        ),
        launch_arguments={
            "params_file": nav2_params_file,
            "scan_frame_id": "base_scan",
        }.items(),
    )

    nav2_bringup_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(bringup_dir, "launch", "loopback_navigation.launch.py")
        ),
        launch_arguments={
            "params_file": nav2_params_file,
            "autostart": autostart,
            "use_respawn": use_respawn,
            "log_level": log_level,
        }.items(),
    )

    fake_inputs_cmd = Node(
        package="pb2025_sentry_bringup",
        executable="fake_decision_sim_inputs.py",
        name="fake_decision_sim_inputs",
        output="screen",
        parameters=[
            {
                "use_sim_time": True,
                "initial_x": initial_x,
                "initial_y": initial_y,
                "initial_yaw": initial_yaw,
                "publish_decision_mode": publish_decision_mode,
                "decision_mode": decision_mode,
                "decision_mode_topic": decision_mode_topic,
                "publish_referee_inputs": publish_referee_inputs,
                "current_hp": current_hp,
                "projectile_allowance_17mm": projectile_allowance_17mm,
                "publish_vision_target": publish_vision_target,
                "vision_tracking": vision_tracking,
                "vision_nav_hold": vision_nav_hold,
                "vision_fire_permitted": vision_fire_permitted,
                "vision_target_type": vision_target_type,
                "vision_target_id": vision_target_id,
                "vision_confidence": vision_confidence,
                "vision_target_distance": vision_target_distance,
                "vision_target_yaw": vision_target_yaw,
                "vision_target_pitch": vision_target_pitch,
                "vision_target_position_gimbal_x": vision_target_position_gimbal_x,
                "vision_target_position_gimbal_y": vision_target_position_gimbal_y,
                "vision_target_position_gimbal_z": vision_target_position_gimbal_z,
                "vision_target_position_map_x": vision_target_position_map_x,
                "vision_target_position_map_y": vision_target_position_map_y,
                "vision_target_position_map_z": vision_target_position_map_z,
                "vision_has_target_position_map": vision_has_target_position_map,
                "vision_target_position_map_frame": vision_target_position_map_frame,
            }
        ],
        arguments=["--ros-args", "--log-level", log_level],
    )

    behavior_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(behavior_dir, "launch", "pb2025_sentry_behavior_launch.py")
        ),
        launch_arguments={
            "namespace": "",
            "use_sim_time": "True",
            "params_file": behavior_params_file,
            "log_level": log_level,
        }.items(),
    )

    delayed_behavior_cmd = TimerAction(period=3.0, actions=[behavior_cmd])

    rviz_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav_bringup_dir, "launch", "rviz_launch.py")
        ),
        condition=IfCondition(
            PythonExpression(
                [
                    "'",
                    use_rviz,
                    "'.strip().lower() in ['true', '1', 'ture', 'yes', 'on']",
                ]
            )
        ),
        launch_arguments={
            "namespace": "",
            "rviz_config": rviz_config_file,
        }.items(),
    )

    ld = LaunchDescription()

    ld.add_action(stdout_linebuf_envvar)
    ld.add_action(colorized_output_envvar)

    ld.add_action(declare_map_yaml_cmd)
    ld.add_action(declare_nav2_params_cmd)
    ld.add_action(declare_behavior_params_cmd)
    ld.add_action(declare_autostart_cmd)
    ld.add_action(declare_use_respawn_cmd)
    ld.add_action(declare_use_rviz_cmd)
    ld.add_action(declare_rviz_config_file_cmd)
    ld.add_action(declare_log_level_cmd)
    ld.add_action(declare_initial_x_cmd)
    ld.add_action(declare_initial_y_cmd)
    ld.add_action(declare_initial_yaw_cmd)
    ld.add_action(declare_publish_decision_mode_cmd)
    ld.add_action(declare_decision_mode_cmd)
    ld.add_action(declare_decision_mode_topic_cmd)
    ld.add_action(declare_publish_referee_inputs_cmd)
    ld.add_action(declare_current_hp_cmd)
    ld.add_action(declare_projectile_allowance_17mm_cmd)
    ld.add_action(declare_publish_vision_target_cmd)
    ld.add_action(declare_vision_tracking_cmd)
    ld.add_action(declare_vision_nav_hold_cmd)
    ld.add_action(declare_vision_fire_permitted_cmd)
    ld.add_action(declare_vision_target_type_cmd)
    ld.add_action(declare_vision_target_id_cmd)
    ld.add_action(declare_vision_confidence_cmd)
    ld.add_action(declare_vision_target_distance_cmd)
    ld.add_action(declare_vision_target_yaw_cmd)
    ld.add_action(declare_vision_target_pitch_cmd)
    ld.add_action(declare_vision_target_position_gimbal_x_cmd)
    ld.add_action(declare_vision_target_position_gimbal_y_cmd)
    ld.add_action(declare_vision_target_position_gimbal_z_cmd)
    ld.add_action(declare_vision_target_position_map_x_cmd)
    ld.add_action(declare_vision_target_position_map_y_cmd)
    ld.add_action(declare_vision_target_position_map_z_cmd)
    ld.add_action(declare_vision_has_target_position_map_cmd)
    ld.add_action(declare_vision_target_position_map_frame_cmd)

    ld.add_action(map_server_cmd)
    ld.add_action(map_server_lifecycle_cmd)
    ld.add_action(loopback_sim_cmd)
    ld.add_action(nav2_bringup_cmd)
    ld.add_action(fake_inputs_cmd)
    ld.add_action(delayed_behavior_cmd)
    ld.add_action(rviz_cmd)

    return ld
