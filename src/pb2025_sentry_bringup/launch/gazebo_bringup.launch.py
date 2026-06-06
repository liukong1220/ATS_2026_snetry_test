import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, TextSubstitution


def generate_launch_description():
    bringup_dir = get_package_share_directory("pb2025_sentry_bringup")
    nav_bringup_dir = get_package_share_directory("pb2025_nav_bringup")
    behavior_bringup_dir = get_package_share_directory("pb2025_sentry_behavior")
    simulator_dir = get_package_share_directory("rmu_gazebo_simulator")

    namespace = LaunchConfiguration("namespace")
    sim_world = LaunchConfiguration("sim_world")
    nav_world = LaunchConfiguration("nav_world")
    use_rviz = LaunchConfiguration("use_rviz")
    rviz_config_file = LaunchConfiguration("rviz_config_file")
    rviz_force_software = LaunchConfiguration("rviz_force_software")
    params_file = LaunchConfiguration("params_file")
    behavior_params_file = LaunchConfiguration("behavior_params_file")
    launch_behavior = LaunchConfiguration("launch_behavior")
    launch_joy_teleop = LaunchConfiguration("launch_joy_teleop")
    launch_trajectory_optimizer = LaunchConfiguration("launch_trajectory_optimizer")
    use_respawn = LaunchConfiguration("use_respawn")
    log_level = LaunchConfiguration("log_level")

    declare_namespace = DeclareLaunchArgument(
        "namespace",
        default_value="red_standard_robot1",
        description="Robot namespace to control in Gazebo",
    )
    declare_sim_world = DeclareLaunchArgument(
        "sim_world",
        default_value="rmuc_2025",
        description="Gazebo world to spawn, e.g. rmul_2025 or rmuc_2025",
    )
    declare_nav_world = DeclareLaunchArgument(
        "nav_world",
        default_value="rmul",
        description="Navigation map/pcd asset basename under pb2025_sentry_bringup/{map,pcd}",
    )
    declare_use_rviz = DeclareLaunchArgument(
        "use_rviz", default_value="True", description="Whether to start RViz"
    )
    declare_rviz_config = DeclareLaunchArgument(
        "rviz_config_file",
        default_value=os.path.join(bringup_dir, "rviz", "sentry_default_view.rviz"),
        description="RViz config file",
    )
    declare_rviz_force_software = DeclareLaunchArgument(
        "rviz_force_software",
        default_value="0",
        description="Force RViz to use Mesa software rendering when set to 1",
    )
    declare_params_file = DeclareLaunchArgument(
        "params_file",
        default_value=os.path.join(
            nav_bringup_dir, "config", "simulation", "nav2_params.yaml"
        ),
        description="Simulation navigation params file",
    )
    declare_behavior_params = DeclareLaunchArgument(
        "behavior_params_file",
        default_value=os.path.join(
            behavior_bringup_dir, "params", "sentry_behavior.yaml"
        ),
        description="Behavior params file",
    )
    declare_launch_behavior = DeclareLaunchArgument(
        "launch_behavior",
        default_value="False",
        description="Whether to start sentry behavior nodes",
    )
    declare_launch_joy_teleop = DeclareLaunchArgument(
        "launch_joy_teleop",
        default_value="False",
        description="Whether to start joystick teleop nodes",
    )
    declare_launch_trajectory_optimizer = DeclareLaunchArgument(
        "launch_trajectory_optimizer",
        default_value="True",
        description="Whether to start visualization trajectory optimizer node",
    )
    declare_use_respawn = DeclareLaunchArgument(
        "use_respawn",
        default_value="False",
        description="Whether to respawn nodes when they crash",
    )
    declare_log_level = DeclareLaunchArgument(
        "log_level", default_value="info", description="log level"
    )

    simulator_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(simulator_dir, "launch", "bringup_sim.launch.py")
        ),
        launch_arguments={
            "world": sim_world,
            "gz_world_path": os.path.join(simulator_dir, "config", "gz_world.yaml"),
        }.items(),
    )

    navigation_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav_bringup_dir, "launch", "rm_navigation_simulation_launch.py")
        ),
        launch_arguments={
            "namespace": namespace,
            "world": nav_world,
            "map": [
                TextSubstitution(text=os.path.join(bringup_dir, "map", "")),
                nav_world,
                TextSubstitution(text=".yaml"),
            ],
            "prior_pcd_file": [
                TextSubstitution(text=os.path.join(bringup_dir, "pcd", "")),
                nav_world,
                TextSubstitution(text=".pcd"),
            ],
            "use_sim_time": "True",
            "params_file": params_file,
            "use_rviz": use_rviz,
            "rviz_config_file": rviz_config_file,
            "rviz_force_software": rviz_force_software,
            "launch_joy_teleop": launch_joy_teleop,
            "launch_trajectory_optimizer": launch_trajectory_optimizer,
            "use_respawn": use_respawn,
            "log_level": log_level,
        }.items(),
    )

    behavior_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                behavior_bringup_dir, "launch", "pb2025_sentry_behavior_launch.py"
            )
        ),
        condition=IfCondition(launch_behavior),
        launch_arguments={
            "namespace": namespace,
            "use_sim_time": "True",
            "params_file": behavior_params_file,
            "log_level": log_level,
        }.items(),
    )

    ld = LaunchDescription()
    ld.add_action(declare_namespace)
    ld.add_action(declare_sim_world)
    ld.add_action(declare_nav_world)
    ld.add_action(declare_use_rviz)
    ld.add_action(declare_rviz_config)
    ld.add_action(declare_rviz_force_software)
    ld.add_action(declare_params_file)
    ld.add_action(declare_behavior_params)
    ld.add_action(declare_launch_behavior)
    ld.add_action(declare_launch_joy_teleop)
    ld.add_action(declare_launch_trajectory_optimizer)
    ld.add_action(declare_use_respawn)
    ld.add_action(declare_log_level)
    ld.add_action(simulator_launch)
    ld.add_action(navigation_launch)
    ld.add_action(behavior_launch)
    return ld
