import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, TextSubstitution
from launch_ros.actions import SetParameter

def generate_launch_description():
    bringup_dir = get_package_share_directory("pb2025_sentry_bringup")
    behavior_dir = get_package_share_directory("pb2025_sentry_behavior")

    robot_name = LaunchConfiguration("robot_name")
    slam = LaunchConfiguration("slam")
    world = LaunchConfiguration("world")
    map_yaml_file = LaunchConfiguration("map")
    prior_pcd_file = LaunchConfiguration("prior_pcd_file")
    namespace = LaunchConfiguration("namespace")
    use_sim_time = LaunchConfiguration("use_sim_time")
    params_file = LaunchConfiguration("params_file")
    behavior_params_file = LaunchConfiguration("behavior_params_file")
    rviz_config_file = LaunchConfiguration("rviz_config_file")
    rviz_force_software = LaunchConfiguration("rviz_force_software")
    use_robot_state_pub = LaunchConfiguration("use_robot_state_pub")
    use_rviz = LaunchConfiguration("use_rviz")
    launch_joy_teleop = LaunchConfiguration("launch_joy_teleop")
    use_composition = LaunchConfiguration("use_composition")
    use_respawn = LaunchConfiguration("use_respawn")
    log_level = LaunchConfiguration("log_level")

    include_bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(bringup_dir, "launch", "bringup.launch.py")
        ),
        launch_arguments={
            "robot_name": robot_name,
            "slam": slam,
            "world": world,
            "map": map_yaml_file,
            "prior_pcd_file": prior_pcd_file,
            "namespace": namespace,
            "use_sim_time": use_sim_time,
            "params_file": params_file,
            "behavior_params_file": behavior_params_file,
            "rviz_config_file": rviz_config_file,
            "rviz_force_software": rviz_force_software,
            "use_robot_state_pub": use_robot_state_pub,
            "use_rviz": use_rviz,
            "launch_joy_teleop": launch_joy_teleop,
            "use_composition": use_composition,
            "use_respawn": use_respawn,
            "log_level": log_level,
        }.items(),
    )

    ld = LaunchDescription()

    ld.add_action(
        DeclareLaunchArgument(
            "robot_name",
            default_value="pb2025_sentry_robot",
            description="The file name of the robot xmacro to be used",
        )
    )
    ld.add_action(
        DeclareLaunchArgument(
            "slam",
            default_value="False",
            description="Whether run a SLAM. If True, it will disable small_gicp and send static tf (map->odom)",
        )
    )
    ld.add_action(
        DeclareLaunchArgument(
            "world",
            default_value="ats",
            description="Select world. Map and PCD file share the same name as this parameter",
        )
    )
    ld.add_action(
        DeclareLaunchArgument(
            "map",
            default_value=[
                TextSubstitution(text=os.path.join(bringup_dir, "map", "")),
                world,
                TextSubstitution(text=".yaml"),
            ],
            description="Full path to map file to load",
        )
    )
    ld.add_action(
        DeclareLaunchArgument(
            "prior_pcd_file",
            default_value=[
                TextSubstitution(text=os.path.join(bringup_dir, "pcd", "")),
                world,
                TextSubstitution(text=".pcd"),
            ],
            description="Full path to prior pcd file to load",
        )
    )
    ld.add_action(
        DeclareLaunchArgument(
            "namespace", default_value="", description="Top-level namespace"
        )
    )
    ld.add_action(
        DeclareLaunchArgument(
            "use_sim_time",
            default_value="False",
            description="Use simulation (Gazebo) clock if true",
        )
    )
    ld.add_action(
        DeclareLaunchArgument(
            "params_file",
            default_value=os.path.join(bringup_dir, "params", "node_params.yaml"),
            description="Full path to the ROS2 parameters file to use for all launched nodes",
        )
    )
    ld.add_action(
        DeclareLaunchArgument(
            "behavior_params_file",
            default_value=os.path.join(
                behavior_dir, "params", "sentry_behavior_vision_test.yaml"
            ),
            description="Behavior tree params for no-referee vision-follow testing",
        )
    )
    ld.add_action(
        DeclareLaunchArgument(
            "rviz_config_file",
            default_value=os.path.join(bringup_dir, "rviz", "sentry_default_view.rviz"),
            description="Full path to the RViz config file to use",
        )
    )
    ld.add_action(
        DeclareLaunchArgument(
            "rviz_force_software",
            default_value="0",
            description="Force RViz to use Mesa software rendering when set to 1",
        )
    )
    ld.add_action(
        DeclareLaunchArgument(
            "use_robot_state_pub",
            default_value="False",
            description="Whether to start the robot state publisher",
        )
    )
    ld.add_action(
        DeclareLaunchArgument(
            "use_rviz", default_value="True", description="Whether to start RViz"
        )
    )
    ld.add_action(
        DeclareLaunchArgument(
            "launch_joy_teleop",
            default_value="False",
            description="Whether to start joystick teleop nodes that can publish to cmd_vel",
        )
    )
    ld.add_action(
        DeclareLaunchArgument(
            "use_composition",
            default_value="True",
            description="Whether to use composed bringup",
        )
    )
    ld.add_action(
        DeclareLaunchArgument(
            "use_respawn",
            default_value="True",
            description="Whether to respawn if a node crashes. Applied when composition is disabled.",
        )
    )
    ld.add_action(
        DeclareLaunchArgument(
            "log_level", default_value="info", description="log level"
        )
    )

    # 直接把行为树客户端切到 vision_test.xml 中定义的树，
    # 这样整车导航链仍沿用 bringup 主入口，但不会再跑 rmul_2026 的比赛门控。
    ld.add_action(SetParameter(name="target_tree", value="vision_test"))
    ld.add_action(include_bringup)

    return ld
