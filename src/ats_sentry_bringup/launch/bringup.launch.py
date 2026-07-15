
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    SetEnvironmentVariable,
)
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    IfElseSubstitution,
    LaunchConfiguration,
    PythonExpression,
    TextSubstitution,
)
from launch_ros.actions import Node
from launch_ros.descriptions import ParameterFile
from nav2_common.launch import RewrittenYaml


def generate_launch_description():
    # Get the launch directory
    bringup_dir = get_package_share_directory("ats_sentry_bringup")

    serial_bringup_dir = get_package_share_directory("standard_robot_pp_ros2")
    navigation_bringup_dir = get_package_share_directory("ats_nav_bringup")
    bt_bringup_dir = get_package_share_directory("ats_sentry_behavior")

    # Create the launch configuration variables
    ## Serial
    robot_name = LaunchConfiguration("robot_name")
    ## Navigation
    slam = LaunchConfiguration("slam")
    world = LaunchConfiguration("world")
    map_yaml_file = LaunchConfiguration("map")
    prior_pcd_file = LaunchConfiguration("prior_pcd_file")
    ## Common
    namespace = LaunchConfiguration("namespace")
    use_sim_time = LaunchConfiguration("use_sim_time")
    params_file = LaunchConfiguration("params_file")
    behavior_params_file = LaunchConfiguration("behavior_params_file")
    rviz_config_file = LaunchConfiguration("rviz_config_file")
    rviz_force_software = LaunchConfiguration("rviz_force_software")
    use_robot_state_pub = LaunchConfiguration("use_robot_state_pub")
    use_rviz = LaunchConfiguration("use_rviz")
    launch_joy_teleop = LaunchConfiguration("launch_joy_teleop")
    launch_rosbag_recorder = LaunchConfiguration("launch_rosbag_recorder")
    launch_trajectory_optimizer = LaunchConfiguration("launch_trajectory_optimizer")
    launch_small_gicp_relocalization = LaunchConfiguration("launch_small_gicp_relocalization")
    launch_fake_vel_transform = LaunchConfiguration("launch_fake_vel_transform")
    launch_chassis_vel_transform = LaunchConfiguration("launch_chassis_vel_transform")
    nav_cmd_vel_topic = LaunchConfiguration("nav_cmd_vel_topic")
    fake_vel_output_topic = LaunchConfiguration("fake_vel_output_topic")
    chassis_vel_input_topic = LaunchConfiguration("chassis_vel_input_topic")
    launch_rog_map = LaunchConfiguration("launch_rog_map")
    use_composition = LaunchConfiguration("use_composition")
    use_respawn = LaunchConfiguration("use_respawn")
    log_level = LaunchConfiguration("log_level")

    any_velocity_transform = PythonExpression([
        "'", launch_fake_vel_transform, "'.lower() == 'true' or '",
        launch_chassis_vel_transform, "'.lower() == 'true'",
    ])

    configured_params = ParameterFile(
        RewrittenYaml(
            source_file=params_file,
            root_key=namespace,
            param_rewrites={},
            convert_types=True,
        ),
        allow_substs=True,
    )

    stdout_linebuf_envvar = SetEnvironmentVariable(
        "RCUTILS_LOGGING_BUFFERED_STREAM", "1"
    )

    colorized_output_envvar = SetEnvironmentVariable("RCUTILS_COLORIZED_OUTPUT", "1")

    declare_robot_name_cmd = DeclareLaunchArgument(
        "robot_name",
        default_value="ats_sentry_robot",
        description="The file name of the robot xmacro to be used",
    )

    declare_slam_cmd = DeclareLaunchArgument(
        "slam",
        default_value="False",
        description="Whether run a SLAM. If True, it will disable small_gicp and send static tf (map->odom)",
    )

    declare_world_cmd = DeclareLaunchArgument(
        "world",
        default_value="",
        description="Select world. Map and PCD file share the same name as this parameter",
    )

    declare_map_yaml_cmd = DeclareLaunchArgument(
        "map",
        default_value=[
            TextSubstitution(text=os.path.join(bringup_dir, "map", "")),
            world,
            TextSubstitution(text=".yaml"),
        ],
        description="Full path to map file to load",
    )

    declare_prior_pcd_file_cmd = DeclareLaunchArgument(
        "prior_pcd_file",
        default_value=[
            TextSubstitution(text=os.path.join(bringup_dir, "pcd", "")),
            world,
            TextSubstitution(text=".pcd"),
        ],
        description="Full path to prior pcd file to load",
    )

    declare_namespace_cmd = DeclareLaunchArgument(
        "namespace", default_value="", description="Top-level namespace"
    )

    declare_use_sim_time_cmd = DeclareLaunchArgument(
        "use_sim_time",
        default_value="False",
        description="Use simulation clock if true",
    )

    declare_params_file_cmd = DeclareLaunchArgument(
        "params_file",
        default_value=os.path.join(bringup_dir, "params", "node_params.yaml"),
        description="Full path to the ROS2 parameters file to use for all launched nodes",
    )

    declare_behavior_params_file_cmd = DeclareLaunchArgument(
        "behavior_params_file",
        default_value=os.path.join(
            bt_bringup_dir,
            "params",
            "sentry_behavior.yaml",
        ),
        description="Full path to the ROS2 parameters file used by behavior tree nodes",
    )

    declare_rviz_config_file_cmd = DeclareLaunchArgument(
        "rviz_config_file",
        default_value=os.path.join(bringup_dir, "rviz", "sentry_default_view.rviz"),
        description="Full path to the RViz config file to use",
    )

    declare_rviz_force_software_cmd = DeclareLaunchArgument(
        "rviz_force_software",
        default_value="0",
        description="Force RViz to use Mesa software rendering when set to 1",
    )

    declare_use_robot_state_pub_cmd = DeclareLaunchArgument(
        "use_robot_state_pub",
        default_value="False",
        description="Whether to start the robot state publisher",
    )

    declare_use_rviz_cmd = DeclareLaunchArgument(
        "use_rviz", default_value="False", description="Whether to start RViz"
    )

    declare_launch_joy_teleop_cmd = DeclareLaunchArgument(
        "launch_joy_teleop",
        default_value="False",
        description="Whether to start joystick teleop nodes that can publish to cmd_vel",
    )

    declare_launch_rosbag_recorder_cmd = DeclareLaunchArgument(
        "launch_rosbag_recorder",
        default_value="False",
        description="Whether to start lightweight rosbag recorder node",
    )

    declare_launch_trajectory_optimizer_cmd = DeclareLaunchArgument(
        "launch_trajectory_optimizer",
        default_value="True",
        description="Whether to start the RC-ESDF local elastic path optimizer",
    )

    declare_launch_small_gicp_relocalization_cmd = DeclareLaunchArgument(
        "launch_small_gicp_relocalization",
        default_value="True",
        description="Whether to start small_gicp map->odom relocalization",
    )

    declare_launch_fake_vel_transform_cmd = DeclareLaunchArgument(
        "launch_fake_vel_transform",
        default_value="True",
        description="Keep the fake-yaw command-frame transform enabled for the gimbal-mounted lidar.",
    )

    declare_launch_chassis_vel_transform_cmd = DeclareLaunchArgument(
        "launch_chassis_vel_transform",
        default_value="True",
        description="Keep the gimbal-yaw to chassis command transform enabled.",
    )

    declare_nav_cmd_vel_topic_cmd = DeclareLaunchArgument(
        "nav_cmd_vel_topic",
        default_value=IfElseSubstitution(
            any_velocity_transform, "cmd_vel_nav2_result", "/cmd_vel"
        ),
        description="Nav2 velocity output selected for the enabled transform chain",
    )

    declare_fake_vel_output_topic_cmd = DeclareLaunchArgument(
        "fake_vel_output_topic",
        default_value=IfElseSubstitution(
            launch_chassis_vel_transform, "cmd_vel_gimbal_yaw_odom", "/cmd_vel"
        ),
        description="Fake-yaw adapter output topic",
    )

    declare_chassis_vel_input_topic_cmd = DeclareLaunchArgument(
        "chassis_vel_input_topic",
        default_value=IfElseSubstitution(
            launch_fake_vel_transform, "cmd_vel_gimbal_yaw_odom", "cmd_vel_nav2_result"
        ),
        description="Chassis-frame adapter input topic",
    )

    declare_launch_rog_map_cmd = DeclareLaunchArgument(
        "launch_rog_map",
        default_value="False",
        description="Start ROGMap 3D occupancy perception without changing the RC-ESDF planning owner.",
    )

    declare_use_composition_cmd = DeclareLaunchArgument(
        "use_composition",
        default_value="True",
        description="Whether to use composed bringup",
    )

    declare_use_respawn_cmd = DeclareLaunchArgument(
        "use_respawn",
        default_value="True",
        description="Whether to respawn if a node crashes. Applied when composition is disabled.",
    )

    declare_log_level_cmd = DeclareLaunchArgument(
        "log_level", default_value="info", description="log level"
    )

    # Specify the actions
    start_serial_driver_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                serial_bringup_dir, "launch", "standard_robot_pp_ros2.launch.py"
            )
        ),
        condition=UnlessCondition(use_robot_state_pub),
        launch_arguments={
            "robot_name": robot_name,
            "namespace": namespace,
            "params_file": params_file,
            "launch_robot_decision": "False",
            "use_rviz": "False",
            "use_respawn": use_respawn,
            "log_level": log_level,
        }.items(),
    )

    static_tf_base_link_cmd = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="static_tf_base_link",
        condition=UnlessCondition(use_robot_state_pub),
        arguments=["--frame-id", "base_footprint", "--child-frame-id", "base_link"],
        output="screen",
    )

    start_chassis_vel_transform_cmd = Node(
        package="sentry_chassis_vel_transform",
        executable="chassis_vel_transform_node",
        name="chassis_vel_transform",
        condition=IfCondition(launch_chassis_vel_transform),
        namespace=namespace,
        output="screen",
        respawn=use_respawn,
        respawn_delay=2.0,
        parameters=[
            configured_params,
            {"input_cmd_vel_topic": chassis_vel_input_topic},
        ],
        arguments=["--ros-args", "--log-level", log_level],
    )

    start_rog_map_cmd = Node(
        package="ats_rog_map",
        executable="ats_rog_map_node",
        name="ats_rog_map",
        namespace=namespace,
        condition=IfCondition(launch_rog_map),
        output="screen",
        respawn=use_respawn,
        respawn_delay=2.0,
        parameters=[configured_params],
        arguments=["--ros-args", "--log-level", log_level],
    )

    start_navigation_launch_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                navigation_bringup_dir, "launch", "rm_navigation_reality_launch.py"
            )
        ),
        launch_arguments={
            "slam": slam,
            "map": map_yaml_file,
            "prior_pcd_file": prior_pcd_file,
            "namespace": namespace,
            "use_sim_time": use_sim_time,
            "params_file": params_file,
            "use_robot_state_pub": use_robot_state_pub,
            "use_rviz": "False",
            "launch_joy_teleop": launch_joy_teleop,
            "launch_trajectory_optimizer": launch_trajectory_optimizer,
            "launch_small_gicp_relocalization": launch_small_gicp_relocalization,
            "launch_fake_vel_transform": launch_fake_vel_transform,
            "launch_chassis_vel_transform": "False",
            "nav_cmd_vel_topic": nav_cmd_vel_topic,
            "fake_vel_output_topic": fake_vel_output_topic,
            "chassis_vel_input_topic": chassis_vel_input_topic,
            "use_composition": use_composition,
            "use_respawn": use_respawn,
            "log_level": log_level,
        }.items(),
    )

    start_behavior_launch_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(bt_bringup_dir, "launch", "ats_sentry_behavior_launch.py")
        ),
        launch_arguments={
            "namespace": namespace,
            "use_sim_time": use_sim_time,
            "params_file": behavior_params_file,
            "log_level": log_level,
        }.items(),
    )

    start_rviz_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(bringup_dir, "launch", "rviz_launch.py")
        ),
        condition=IfCondition(use_rviz),
        launch_arguments={
            "namespace": namespace,
            "use_sim_time": use_sim_time,
            "rviz_config": rviz_config_file,
            "rviz_force_software": rviz_force_software,
        }.items(),
    )

    record_rosbag_cmd = Node(
        package="rosbag2_composable_recorder",
        executable="composable_recorder_node",
        name="rosbag_recorder",
        condition=IfCondition(launch_rosbag_recorder),
        output="screen",
        respawn=use_respawn,
        respawn_delay=2.0,
        parameters=[configured_params],
        arguments=["--ros-args", "--log-level", log_level],
    )

    # Create the launch description and populate
    ld = LaunchDescription()

    # Set environment variables
    ld.add_action(stdout_linebuf_envvar)
    ld.add_action(colorized_output_envvar)

    # Declare the launch options
    ld.add_action(declare_robot_name_cmd)
    ld.add_action(declare_slam_cmd)
    ld.add_action(declare_world_cmd)
    ld.add_action(declare_map_yaml_cmd)
    ld.add_action(declare_prior_pcd_file_cmd)
    ld.add_action(declare_namespace_cmd)
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_params_file_cmd)
    ld.add_action(declare_behavior_params_file_cmd)
    ld.add_action(declare_rviz_config_file_cmd)
    ld.add_action(declare_rviz_force_software_cmd)
    ld.add_action(declare_use_robot_state_pub_cmd)
    ld.add_action(declare_use_rviz_cmd)
    ld.add_action(declare_launch_joy_teleop_cmd)
    ld.add_action(declare_launch_rosbag_recorder_cmd)
    ld.add_action(declare_launch_trajectory_optimizer_cmd)
    ld.add_action(declare_launch_small_gicp_relocalization_cmd)
    ld.add_action(declare_launch_fake_vel_transform_cmd)
    ld.add_action(declare_launch_chassis_vel_transform_cmd)
    ld.add_action(declare_nav_cmd_vel_topic_cmd)
    ld.add_action(declare_fake_vel_output_topic_cmd)
    ld.add_action(declare_chassis_vel_input_topic_cmd)
    ld.add_action(declare_launch_rog_map_cmd)
    ld.add_action(declare_use_composition_cmd)
    ld.add_action(declare_use_respawn_cmd)
    ld.add_action(declare_log_level_cmd)

    # Add the actions to launch all of the navigation nodes
    ld.add_action(start_rviz_cmd)
    ld.add_action(start_serial_driver_cmd)
    # navigation_launch owns base_footprint -> base_link to keep one TF authority.
    ld.add_action(start_chassis_vel_transform_cmd)
    ld.add_action(start_rog_map_cmd)
    ld.add_action(start_navigation_launch_cmd)
    ld.add_action(start_behavior_launch_cmd)
    ld.add_action(record_rosbag_cmd)

    return ld
