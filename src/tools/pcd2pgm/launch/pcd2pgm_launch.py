
import os

from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    SetEnvironmentVariable,
)
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    # Get the launch directory
    bringup_dir = get_package_share_directory("pcd2pgm")

    # Create the launch configuration variables``
    params_file = LaunchConfiguration("params_file")
    rviz_config_file = LaunchConfiguration("rviz_config_file")

    stdout_linebuf_envvar = SetEnvironmentVariable(
        "RCUTILS_LOGGING_BUFFERED_STREAM", "1"
    )

    colorized_output_envvar = SetEnvironmentVariable("RCUTILS_COLORIZED_OUTPUT", "1")

    declare_params_file_cmd = DeclareLaunchArgument(
        "params_file",
        default_value=os.path.join(bringup_dir, "config", "pcd2pgm.yaml"),
        description="Full path to the ROS2 parameters file to use for all launched nodes",
    )

    declare_rviz_config_file_cmd = DeclareLaunchArgument(
        "rviz_config_file",
        default_value=os.path.join(bringup_dir, "rviz", "pcd2pgm.rviz"),
        description="Full path to the RVIZ config file to use",
    )

    start_pcd2pgm_cmd = Node(
        package="pcd2pgm",
        executable="pcd2pgm_node",
        name="pcd2pgm",
        output="screen",
        parameters=[params_file],
    )

    start_rviz_cmd = Node(
        package="rviz2",
        executable="rviz2",
        arguments=["-d", rviz_config_file],
        output="screen",
        remappings=[
            ("/tf", "tf"),
            ("/tf_static", "tf_static"),
        ],
    )

    # Create the launch description and populate
    ld = LaunchDescription()

    # Set environment variables
    ld.add_action(stdout_linebuf_envvar)
    ld.add_action(colorized_output_envvar)

    # Declare the launch options
    ld.add_action(declare_params_file_cmd)
    ld.add_action(declare_rviz_config_file_cmd)

    # Add the actions to launch all of the navigation nodes
    ld.add_action(start_pcd2pgm_cmd)
    ld.add_action(start_rviz_cmd)

    return ld
