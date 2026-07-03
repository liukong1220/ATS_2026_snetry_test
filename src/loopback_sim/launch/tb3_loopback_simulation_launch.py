import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description() -> LaunchDescription:
    bringup_dir = get_package_share_directory("ats_sentry_bringup")
    loopback_sim_dir = get_package_share_directory("nav2_loopback_sim")

    map_yaml_file = LaunchConfiguration("map")
    params_file = LaunchConfiguration("params_file")
    autostart = LaunchConfiguration("autostart")
    use_respawn = LaunchConfiguration("use_respawn")
    rviz_config_file = LaunchConfiguration("rviz_config_file")
    use_rviz = LaunchConfiguration("use_rviz")

    declare_map_yaml_cmd = DeclareLaunchArgument(
        "map",
        default_value=os.path.join(bringup_dir, "map", "rmul.yaml"),
        description="Map yaml file for loopback navigation.",
    )

    declare_params_file_cmd = DeclareLaunchArgument(
        "params_file",
        default_value=os.path.join(loopback_sim_dir, "nav2_params.yaml"),
        description="Full path to Nav2 loopback parameter file.",
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

    declare_rviz_config_file_cmd = DeclareLaunchArgument(
        "rviz_config_file",
        default_value=os.path.join(bringup_dir, "rviz", "sentry_default_view.rviz"),
        description="RViz config file.",
    )

    declare_use_rviz_cmd = DeclareLaunchArgument(
        "use_rviz",
        default_value="True",
        description="Whether to start RViz.",
    )

    nav_only_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(bringup_dir, "launch", "loopback_nav_only.launch.py")
        ),
        launch_arguments={
            "map": map_yaml_file,
            "nav2_params_file": params_file,
            "autostart": autostart,
            "use_respawn": use_respawn,
            "use_rviz": use_rviz,
            "rviz_config_file": rviz_config_file,
        }.items(),
    )

    ld = LaunchDescription()
    ld.add_action(declare_map_yaml_cmd)
    ld.add_action(declare_params_file_cmd)
    ld.add_action(declare_autostart_cmd)
    ld.add_action(declare_use_respawn_cmd)
    ld.add_action(declare_rviz_config_file_cmd)
    ld.add_action(declare_use_rviz_cmd)
    ld.add_action(nav_only_cmd)
    return ld
