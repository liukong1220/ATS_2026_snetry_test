
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
    publish_vision_target = LaunchConfiguration("publish_vision_target")
    vision_tracking = LaunchConfiguration("vision_tracking")
    vision_nav_hold = LaunchConfiguration("vision_nav_hold")
    vision_target_id = LaunchConfiguration("vision_target_id")
    vision_suggested_goal_index = LaunchConfiguration("vision_suggested_goal_index")
    vision_target_yaw = LaunchConfiguration("vision_target_yaw")
    vision_target_pitch = LaunchConfiguration("vision_target_pitch")
    vision_target_position_map_x = LaunchConfiguration("vision_target_position_map_x")
    vision_target_position_map_y = LaunchConfiguration("vision_target_position_map_y")
    vision_target_position_map_z = LaunchConfiguration("vision_target_position_map_z")

    stdout_linebuf_envvar = SetEnvironmentVariable(
        "RCUTILS_LOGGING_BUFFERED_STREAM", "1"
    )
    colorized_output_envvar = SetEnvironmentVariable("RCUTILS_COLORIZED_OUTPUT", "1")

    declare_map_yaml_cmd = DeclareLaunchArgument(
        "map",
        default_value=os.path.join(bringup_dir, "map", "rmul.yaml"),
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
        default_value=os.path.join(nav_bringup_dir, "rviz", "nav2_default_view.rviz"),
        description="Full path to the RViz config file to use.",
    )

    declare_log_level_cmd = DeclareLaunchArgument(
        "log_level",
        default_value="info",
        description="Log level for launched nodes.",
    )

    declare_initial_x_cmd = DeclareLaunchArgument(
        "initial_x",
        default_value="0.0",
        description="Initial robot x position in map frame.",
    )

    declare_initial_y_cmd = DeclareLaunchArgument(
        "initial_y",
        default_value="0.0",
        description="Initial robot y position in map frame.",
    )

    declare_initial_yaw_cmd = DeclareLaunchArgument(
        "initial_yaw",
        default_value="0.0",
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
        default_value="False",
        description="Whether the fake loopback node publishes referee topics.",
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

    declare_vision_target_id_cmd = DeclareLaunchArgument(
        "vision_target_id",
        default_value="7",
        description="Fake vision target id.",
    )

    declare_vision_suggested_goal_index_cmd = DeclareLaunchArgument(
        "vision_suggested_goal_index",
        default_value="-1",
        description="Suggested navigation goal index from fake vision target.",
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

    static_tf_base_link_cmd = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="static_tf_base_link",
        arguments=["--frame-id", "base_footprint", "--child-frame-id", "base_link"],
        output="screen",
    )

    static_tf_base_scan_cmd = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="static_tf_base_scan",
        arguments=["--frame-id", "base_link", "--child-frame-id", "base_scan"],
        output="screen",
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
            os.path.join(loopback_dir, "loopback_simulation.launch.py")
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
                "initial_x": initial_x,
                "initial_y": initial_y,
                "initial_yaw": initial_yaw,
                "publish_decision_mode": publish_decision_mode,
                "decision_mode": decision_mode,
                "decision_mode_topic": decision_mode_topic,
                "publish_referee_inputs": publish_referee_inputs,
                "publish_vision_target": publish_vision_target,
                "vision_tracking": vision_tracking,
                "vision_nav_hold": vision_nav_hold,
                "vision_target_id": vision_target_id,
                "vision_suggested_goal_index": vision_suggested_goal_index,
                "vision_target_yaw": vision_target_yaw,
                "vision_target_pitch": vision_target_pitch,
                "vision_target_position_map_x": vision_target_position_map_x,
                "vision_target_position_map_y": vision_target_position_map_y,
                "vision_target_position_map_z": vision_target_position_map_z,
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
    ld.add_action(declare_publish_vision_target_cmd)
    ld.add_action(declare_vision_tracking_cmd)
    ld.add_action(declare_vision_nav_hold_cmd)
    ld.add_action(declare_vision_target_id_cmd)
    ld.add_action(declare_vision_suggested_goal_index_cmd)
    ld.add_action(declare_vision_target_yaw_cmd)
    ld.add_action(declare_vision_target_pitch_cmd)
    ld.add_action(declare_vision_target_position_map_x_cmd)
    ld.add_action(declare_vision_target_position_map_y_cmd)
    ld.add_action(declare_vision_target_position_map_z_cmd)

    ld.add_action(static_tf_base_link_cmd)
    ld.add_action(static_tf_base_scan_cmd)
    ld.add_action(map_server_cmd)
    ld.add_action(map_server_lifecycle_cmd)
    ld.add_action(loopback_sim_cmd)
    ld.add_action(nav2_bringup_cmd)
    ld.add_action(fake_inputs_cmd)
    ld.add_action(delayed_behavior_cmd)
    ld.add_action(rviz_cmd)

    return ld
