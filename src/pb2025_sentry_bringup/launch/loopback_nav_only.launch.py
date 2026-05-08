import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node


def generate_launch_description():
    bringup_dir = get_package_share_directory("pb2025_sentry_bringup")
    loopback_dir = get_package_share_directory("nav2_loopback_sim")
    nav_bringup_dir = get_package_share_directory("pb2025_nav_bringup")

    map_yaml_file = LaunchConfiguration("map")
    nav2_params_file = LaunchConfiguration("nav2_params_file")
    autostart = LaunchConfiguration("autostart")
    use_respawn = LaunchConfiguration("use_respawn")
    use_rviz = LaunchConfiguration("use_rviz")
    rviz_config_file = LaunchConfiguration("rviz_config_file")
    log_level = LaunchConfiguration("log_level")
    initial_x = LaunchConfiguration("initial_x")
    initial_y = LaunchConfiguration("initial_y")
    initial_yaw = LaunchConfiguration("initial_yaw")

    declare_map_yaml_cmd = DeclareLaunchArgument(
        "map",
        default_value=os.path.join(bringup_dir, "map", "rmul.yaml"),
        description="Map yaml for pure navigation loopback.",
    )

    declare_nav2_params_cmd = DeclareLaunchArgument(
        "nav2_params_file",
        default_value=os.path.join(loopback_dir, "nav2_params.yaml"),
        description="Nav2 loopback parameter file.",
    )

    declare_autostart_cmd = DeclareLaunchArgument(
        "autostart",
        default_value="true",
        description="Automatically startup the Nav2 stack.",
    )

    declare_use_respawn_cmd = DeclareLaunchArgument(
        "use_respawn",
        default_value="False",
        description="Whether to respawn nodes on crash.",
    )

    declare_use_rviz_cmd = DeclareLaunchArgument(
        "use_rviz",
        default_value="True",
        description="Whether to start RViz.",
    )

    declare_rviz_config_file_cmd = DeclareLaunchArgument(
        "rviz_config_file",
        default_value=os.path.join(bringup_dir, "rviz", "sentry_default_view.rviz"),
        description="RViz config file.",
    )

    declare_log_level_cmd = DeclareLaunchArgument(
        "log_level",
        default_value="info",
        description="Log level for launched nodes.",
    )

    declare_initial_x_cmd = DeclareLaunchArgument(
        "initial_x",
        default_value="0.0",
        description="Default loopback initial x in map frame.",
    )

    declare_initial_y_cmd = DeclareLaunchArgument(
        "initial_y",
        default_value="0.0",
        description="Default loopback initial y in map frame.",
    )

    declare_initial_yaw_cmd = DeclareLaunchArgument(
        "initial_yaw",
        default_value="0.0",
        description="Default loopback initial yaw in map frame.",
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
                "initial_pose_repeats": 1,
                "initial_pose_period": 0.5,
                "publish_decision_mode": False,
                "publish_referee_inputs": False,
            }
        ],
    )

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
    ld.add_action(declare_map_yaml_cmd)
    ld.add_action(declare_nav2_params_cmd)
    ld.add_action(declare_autostart_cmd)
    ld.add_action(declare_use_respawn_cmd)
    ld.add_action(declare_use_rviz_cmd)
    ld.add_action(declare_rviz_config_file_cmd)
    ld.add_action(declare_log_level_cmd)
    ld.add_action(declare_initial_x_cmd)
    ld.add_action(declare_initial_y_cmd)
    ld.add_action(declare_initial_yaw_cmd)
    ld.add_action(map_server_cmd)
    ld.add_action(map_server_lifecycle_cmd)
    ld.add_action(loopback_sim_cmd)
    ld.add_action(nav2_bringup_cmd)
    ld.add_action(fake_inputs_cmd)
    ld.add_action(rviz_cmd)
    return ld
