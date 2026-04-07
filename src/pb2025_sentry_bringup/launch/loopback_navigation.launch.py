
import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    params_file = LaunchConfiguration("params_file")
    autostart = LaunchConfiguration("autostart")
    use_respawn = LaunchConfiguration("use_respawn")
    log_level = LaunchConfiguration("log_level")

    stdout_linebuf_envvar = SetEnvironmentVariable(
        "RCUTILS_LOGGING_BUFFERED_STREAM", "1"
    )

    declare_params_file_cmd = DeclareLaunchArgument(
        "params_file",
        description="Full path to the Nav2 parameter file.",
    )

    declare_autostart_cmd = DeclareLaunchArgument(
        "autostart",
        default_value="true",
        description="Automatically startup the Nav2 stack.",
    )

    declare_use_respawn_cmd = DeclareLaunchArgument(
        "use_respawn",
        default_value="False",
        description="Whether to respawn nodes if they crash.",
    )

    declare_log_level_cmd = DeclareLaunchArgument(
        "log_level",
        default_value="info",
        description="Log level for launched nodes.",
    )

    lifecycle_nodes = [
        "controller_server",
        "smoother_server",
        "planner_server",
        "behavior_server",
        "bt_navigator",
        "waypoint_follower",
    ]

    controller_server_cmd = Node(
        package="nav2_controller",
        executable="controller_server",
        name="controller_server",
        output="screen",
        respawn=use_respawn,
        respawn_delay=2.0,
        parameters=[params_file],
        arguments=["--ros-args", "--log-level", log_level],
    )

    smoother_server_cmd = Node(
        package="nav2_smoother",
        executable="smoother_server",
        name="smoother_server",
        output="screen",
        respawn=use_respawn,
        respawn_delay=2.0,
        parameters=[params_file],
        arguments=["--ros-args", "--log-level", log_level],
    )

    planner_server_cmd = Node(
        package="nav2_planner",
        executable="planner_server",
        name="planner_server",
        output="screen",
        respawn=use_respawn,
        respawn_delay=2.0,
        parameters=[params_file],
        arguments=["--ros-args", "--log-level", log_level],
    )

    behavior_server_cmd = Node(
        package="nav2_behaviors",
        executable="behavior_server",
        name="behavior_server",
        output="screen",
        respawn=use_respawn,
        respawn_delay=2.0,
        parameters=[params_file],
        arguments=["--ros-args", "--log-level", log_level],
    )

    bt_navigator_cmd = Node(
        package="nav2_bt_navigator",
        executable="bt_navigator",
        name="bt_navigator",
        output="screen",
        respawn=use_respawn,
        respawn_delay=2.0,
        parameters=[params_file],
        arguments=["--ros-args", "--log-level", log_level],
    )

    waypoint_follower_cmd = Node(
        package="nav2_waypoint_follower",
        executable="waypoint_follower",
        name="waypoint_follower",
        output="screen",
        respawn=use_respawn,
        respawn_delay=2.0,
        parameters=[params_file],
        arguments=["--ros-args", "--log-level", log_level],
    )

    lifecycle_manager_cmd = Node(
        package="nav2_lifecycle_manager",
        executable="lifecycle_manager",
        name="lifecycle_manager_navigation",
        output="screen",
        parameters=[
            params_file,
            {"use_sim_time": True},
            {"autostart": autostart},
            {"node_names": lifecycle_nodes},
        ],
        arguments=["--ros-args", "--log-level", log_level],
    )

    ld = LaunchDescription()

    ld.add_action(stdout_linebuf_envvar)

    ld.add_action(declare_params_file_cmd)
    ld.add_action(declare_autostart_cmd)
    ld.add_action(declare_use_respawn_cmd)
    ld.add_action(declare_log_level_cmd)

    ld.add_action(controller_server_cmd)
    ld.add_action(smoother_server_cmd)
    ld.add_action(planner_server_cmd)
    ld.add_action(behavior_server_cmd)
    ld.add_action(bt_navigator_cmd)
    ld.add_action(waypoint_follower_cmd)
    ld.add_action(lifecycle_manager_cmd)

    return ld
