import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # Get the launch directory
    bringup_dir = get_package_share_directory("ats_nav_bringup")
    workspace_dir = os.path.abspath(
        os.path.join(bringup_dir, os.pardir, os.pardir, os.pardir, os.pardir)
    )
    ros_home_dir = os.path.join(workspace_dir, ".ros")
    ros_log_dir = os.path.join(ros_home_dir, "log")
    rviz_home_dir = workspace_dir

    # Create the launch configuration variables
    namespace = LaunchConfiguration("namespace")
    rviz_config_file = LaunchConfiguration("rviz_config")
    rviz_force_software = LaunchConfiguration("rviz_force_software")

    # Declare the launch arguments
    declare_namespace_cmd = DeclareLaunchArgument(
        "namespace",
        default_value="",
        description=(
            "Top-level namespace. The value will be used to replace the "
            "<robot_namespace> keyword on the RViz config file."
        ),
    )

    declare_rviz_config_file_cmd = DeclareLaunchArgument(
        "rviz_config",
        default_value=os.path.join(bringup_dir, "rviz", "nav2_default_view.rviz"),
        description="Full path to the RViz config file to use",
    )

    declare_rviz_force_software_cmd = DeclareLaunchArgument(
        "rviz_force_software",
        default_value="0",
        description="Force RViz to use Mesa software rendering when set to 1",
    )

    rviz_software_rendering_env = SetEnvironmentVariable(
        "LIBGL_ALWAYS_SOFTWARE", rviz_force_software
    )
    rviz_dri3_env = SetEnvironmentVariable("LIBGL_DRI3_DISABLE", "1")
    rviz_ros_home_env = SetEnvironmentVariable("ROS_HOME", ros_home_dir)
    rviz_ros_log_dir_env = SetEnvironmentVariable("ROS_LOG_DIR", ros_log_dir)
    rviz_home_env = SetEnvironmentVariable("HOME", rviz_home_dir)

    # Launch rviz
    start_rviz_cmd = Node(
        package="rviz2",
        executable="rviz2",
        namespace=namespace,
        arguments=["-d", rviz_config_file],
        output="screen",
        remappings=[
            ("/tf", "tf"),
            ("/tf_static", "tf_static"),
        ],
    )

    # Create the launch description and populate
    ld = LaunchDescription()

    # Declare the launch options
    ld.add_action(declare_namespace_cmd)
    ld.add_action(declare_rviz_config_file_cmd)
    ld.add_action(declare_rviz_force_software_cmd)

    # Add any conditioned actions
    ld.add_action(rviz_home_env)
    ld.add_action(rviz_ros_home_env)
    ld.add_action(rviz_ros_log_dir_env)
    ld.add_action(rviz_dri3_env)
    ld.add_action(rviz_software_rendering_env)
    ld.add_action(start_rviz_cmd)

    return ld
