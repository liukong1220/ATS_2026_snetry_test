import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import IfElseSubstitution, LaunchConfiguration, PythonExpression, TextSubstitution
from launch_ros.actions import Node


def generate_launch_description():
    bringup_dir = get_package_share_directory("ats_sentry_bringup")
    serial_bringup_dir = get_package_share_directory("standard_robot_pp_ros2")
    navigation_bringup_dir = get_package_share_directory("ats_nav_bringup")
    robot_name = LaunchConfiguration("robot_name")
    world = LaunchConfiguration("world")
    map_yaml_file = LaunchConfiguration("map")
    prior_pcd_file = LaunchConfiguration("prior_pcd_file")
    namespace = LaunchConfiguration("namespace")
    use_sim_time = LaunchConfiguration("use_sim_time")
    params_file = LaunchConfiguration("params_file")
    rviz_config_file = LaunchConfiguration("rviz_config_file")
    rviz_force_software = LaunchConfiguration("rviz_force_software")
    use_robot_state_pub = LaunchConfiguration("use_robot_state_pub")
    use_rviz = LaunchConfiguration("use_rviz")
    launch_joy_teleop = LaunchConfiguration("launch_joy_teleop")
    launch_rosbag_recorder = LaunchConfiguration("launch_rosbag_recorder")
    launch_small_gicp_relocalization = LaunchConfiguration("launch_small_gicp_relocalization")
    launch_localization_fusion = LaunchConfiguration("launch_localization_fusion")
    launch_fake_vel_transform = LaunchConfiguration("launch_fake_vel_transform")
    launch_chassis_vel_transform = LaunchConfiguration("launch_chassis_vel_transform")
    fake_vel_output_topic = LaunchConfiguration("fake_vel_output_topic")
    chassis_vel_input_topic = LaunchConfiguration("chassis_vel_input_topic")
    mpc_cmd_vel_topic = LaunchConfiguration("mpc_cmd_vel_topic")
    require_gimbal_status = LaunchConfiguration("require_gimbal_status")
    launch_behavior = LaunchConfiguration("launch_behavior")
    launch_lidar_static_tf = LaunchConfiguration("launch_lidar_static_tf")
    lidar_static_tf_x = LaunchConfiguration("lidar_static_tf_x")
    lidar_static_tf_y = LaunchConfiguration("lidar_static_tf_y")
    lidar_static_tf_z = LaunchConfiguration("lidar_static_tf_z")
    lidar_static_tf_roll = LaunchConfiguration("lidar_static_tf_roll")
    lidar_static_tf_pitch = LaunchConfiguration("lidar_static_tf_pitch")
    lidar_static_tf_yaw = LaunchConfiguration("lidar_static_tf_yaw")
    use_respawn = LaunchConfiguration("use_respawn")
    log_level = LaunchConfiguration("log_level")

    lidar_static_tf_enabled = PythonExpression([
        "'", use_sim_time, "'.lower() == 'false' and '", launch_lidar_static_tf,
        "'.lower() == 'true'",
    ])
    declarations = [
        DeclareLaunchArgument("robot_name", default_value="ats_sentry_robot"),
        DeclareLaunchArgument("world", default_value=""),
        DeclareLaunchArgument(
            "map",
            default_value=[TextSubstitution(text=os.path.join(bringup_dir, "map", "")), world, TextSubstitution(text=".yaml")],
        ),
        DeclareLaunchArgument(
            "prior_pcd_file",
            default_value=[TextSubstitution(text=os.path.join(bringup_dir, "pcd", "")), world, TextSubstitution(text=".pcd")],
        ),
        DeclareLaunchArgument("namespace", default_value=""),
        DeclareLaunchArgument("use_sim_time", default_value="False"),
        DeclareLaunchArgument(
            "params_file", default_value=os.path.join(bringup_dir, "params", "node_params.yaml"),
            description="Formal serial, behavior, transform, localization, map, planning and control parameters.",
        ),
        DeclareLaunchArgument(
            "rviz_config_file", default_value=os.path.join(bringup_dir, "rviz", "sentry_default_view.rviz"),
        ),
        DeclareLaunchArgument("rviz_force_software", default_value="0"),
        DeclareLaunchArgument("use_robot_state_pub", default_value="False"),
        DeclareLaunchArgument("use_rviz", default_value="False"),
        DeclareLaunchArgument("launch_joy_teleop", default_value="False"),
        DeclareLaunchArgument("launch_rosbag_recorder", default_value="False"),
        DeclareLaunchArgument("launch_small_gicp_relocalization", default_value="True"),
        DeclareLaunchArgument("launch_localization_fusion", default_value="True"),
        DeclareLaunchArgument("launch_fake_vel_transform", default_value="True"),
        DeclareLaunchArgument("launch_chassis_vel_transform", default_value="True"),
        DeclareLaunchArgument(
            "fake_vel_output_topic",
            default_value=IfElseSubstitution(launch_chassis_vel_transform, "cmd_vel_gimbal_yaw_odom", "/cmd_vel"),
        ),
        DeclareLaunchArgument(
            "chassis_vel_input_topic",
            default_value=IfElseSubstitution(launch_fake_vel_transform, "cmd_vel_gimbal_yaw_odom", mpc_cmd_vel_topic),
        ),
        DeclareLaunchArgument(
            "mpc_cmd_vel_topic",
            default_value=IfElseSubstitution(
                launch_fake_vel_transform,
                "/cmd_vel_mpc",
                IfElseSubstitution(launch_chassis_vel_transform, "cmd_vel_gimbal_yaw_odom", "/cmd_vel"),
            ),
        ),
        DeclareLaunchArgument("require_gimbal_status", default_value="True"),
        DeclareLaunchArgument("launch_behavior", default_value="True"),
        DeclareLaunchArgument("launch_lidar_static_tf", default_value="True"),
        DeclareLaunchArgument("lidar_static_tf_x", default_value="-0.2"),
        DeclareLaunchArgument("lidar_static_tf_y", default_value="0.0"),
        DeclareLaunchArgument("lidar_static_tf_z", default_value="0.0"),
        DeclareLaunchArgument("lidar_static_tf_roll", default_value="0.0"),
        DeclareLaunchArgument("lidar_static_tf_pitch", default_value="0.0"),
        DeclareLaunchArgument("lidar_static_tf_yaw", default_value="-1.0646508437165408"),
        DeclareLaunchArgument("use_respawn", default_value="True"),
        DeclareLaunchArgument("log_level", default_value="info"),
    ]
    serial_driver = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(serial_bringup_dir, "launch", "standard_robot_pp_ros2.launch.py")),
        condition=UnlessCondition(use_robot_state_pub),
        launch_arguments={
            "robot_name": robot_name, "namespace": namespace, "params_file": params_file,
            "launch_robot_decision": "False", "use_rviz": "False", "use_respawn": use_respawn,
            "log_level": log_level,
        }.items(),
    )
    lidar_static_tf = Node(
        package="tf2_ros", executable="static_transform_publisher",
        name="static_tf_gimbal_yaw_odom_to_front_mid360", namespace=namespace,
        condition=IfCondition(lidar_static_tf_enabled), output="screen",
        arguments=[
            "--x", lidar_static_tf_x, "--y", lidar_static_tf_y, "--z", lidar_static_tf_z,
            "--roll", lidar_static_tf_roll, "--pitch", lidar_static_tf_pitch, "--yaw", lidar_static_tf_yaw,
            "--frame-id", "gimbal_yaw_odom", "--child-frame-id", "front_mid360",
        ], parameters=[{"use_sim_time": use_sim_time}],
    )
    gimbal_status_bridge = Node(
        package="standard_robot_pp_ros2", executable="gimbal_yaw_status_bridge_node",
        name="gimbal_yaw_status_bridge", namespace=namespace,
        condition=IfCondition(require_gimbal_status), output="screen", respawn=use_respawn, respawn_delay=2.0,
        parameters=[params_file, {"use_sim_time": use_sim_time}], arguments=["--ros-args", "--log-level", log_level],
    )
    rog_map = Node(
        package="ats_rog_map", executable="ats_rog_map_node", name="ats_rog_map", namespace=namespace,
        output="screen", respawn=use_respawn, respawn_delay=2.0,
        parameters=[params_file, {"use_sim_time": use_sim_time}],
        arguments=["--ros-args", "--log-level", log_level],
    )
    rog_map_adapter = Node(
        package="ats_rog_map_adapter", executable="ats_rog_map_adapter_node", name="ats_rog_map_adapter", namespace=namespace,
        output="screen", respawn=use_respawn, respawn_delay=2.0,
        parameters=[params_file, {
            "use_sim_time": use_sim_time,
            "require_localization_status": launch_localization_fusion,
        }], arguments=["--ros-args", "--log-level", log_level],
    )
    navigation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(navigation_bringup_dir, "launch", "rm_navigation_reality_launch.py")),
        launch_arguments={
            "assets_dir": bringup_dir,
            "world": world, "map": map_yaml_file, "prior_pcd_file": prior_pcd_file,
            "namespace": namespace, "use_sim_time": use_sim_time, "params_file": params_file,
            "use_robot_state_pub": use_robot_state_pub, "use_rviz": "False",
            "launch_joy_teleop": launch_joy_teleop,
            "launch_small_gicp_relocalization": launch_small_gicp_relocalization,
            "launch_localization_fusion": launch_localization_fusion,
            "launch_fake_vel_transform": launch_fake_vel_transform,
            "launch_chassis_vel_transform": launch_chassis_vel_transform,
            "fake_vel_output_topic": fake_vel_output_topic,
            "chassis_vel_input_topic": chassis_vel_input_topic,
            "mpc_cmd_vel_topic": mpc_cmd_vel_topic,
            "require_gimbal_status": require_gimbal_status, "use_respawn": use_respawn,
            "log_level": log_level,
        }.items(),
    )
    behavior = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory("ats_sentry_behavior"), "launch", "ats_sentry_behavior_launch.py")
        ),
        condition=IfCondition(launch_behavior),
        launch_arguments={
            "namespace": namespace, "use_sim_time": use_sim_time, "params_file": params_file,
            "log_level": log_level,
        }.items(),
    )
    rviz = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(bringup_dir, "launch", "rviz_launch.py")),
        condition=IfCondition(use_rviz),
        launch_arguments={
            "namespace": namespace, "use_sim_time": use_sim_time,
            "rviz_config": rviz_config_file, "rviz_force_software": rviz_force_software,
        }.items(),
    )
    rosbag = Node(
        package="rosbag2_composable_recorder", executable="composable_recorder_node", name="rosbag_recorder",
        namespace=namespace, condition=IfCondition(launch_rosbag_recorder), output="screen",
        respawn=use_respawn, respawn_delay=2.0, parameters=[params_file, {"use_sim_time": use_sim_time}],
        arguments=["--ros-args", "--log-level", log_level],
    )

    ld = LaunchDescription()
    ld.add_action(SetEnvironmentVariable("RCUTILS_LOGGING_BUFFERED_STREAM", "1"))
    ld.add_action(SetEnvironmentVariable("RCUTILS_COLORIZED_OUTPUT", "1"))
    for declaration in declarations:
        ld.add_action(declaration)
    for action in (serial_driver, lidar_static_tf, gimbal_status_bridge, rog_map, rog_map_adapter,
                   navigation, behavior, rviz, rosbag):
        ld.add_action(action)
    return ld
