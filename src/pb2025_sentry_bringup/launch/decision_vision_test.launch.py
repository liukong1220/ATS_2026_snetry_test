import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression, TextSubstitution
from launch_ros.actions import Node


def generate_launch_description():
    bringup_dir = get_package_share_directory("pb2025_sentry_bringup")
    behavior_dir = get_package_share_directory("pb2025_sentry_behavior")

    namespace = LaunchConfiguration("namespace")
    robot_name = LaunchConfiguration("robot_name")
    slam = LaunchConfiguration("slam")
    world = LaunchConfiguration("world")
    map_yaml_file = LaunchConfiguration("map")
    prior_pcd_file = LaunchConfiguration("prior_pcd_file")
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

    publish_fake_inputs = LaunchConfiguration("publish_fake_inputs")
    initial_x = LaunchConfiguration("initial_x")
    initial_y = LaunchConfiguration("initial_y")
    initial_yaw = LaunchConfiguration("initial_yaw")
    publish_decision_mode = LaunchConfiguration("publish_decision_mode")
    decision_mode = LaunchConfiguration("decision_mode")
    publish_vision_target = LaunchConfiguration("publish_vision_target")
    vision_tracking = LaunchConfiguration("vision_tracking")
    vision_nav_hold = LaunchConfiguration("vision_nav_hold")
    vision_fire_permitted = LaunchConfiguration("vision_fire_permitted")
    vision_target_id = LaunchConfiguration("vision_target_id")
    vision_confidence = LaunchConfiguration("vision_confidence")
    vision_target_distance = LaunchConfiguration("vision_target_distance")
    vision_target_yaw = LaunchConfiguration("vision_target_yaw")
    vision_target_pitch = LaunchConfiguration("vision_target_pitch")
    vision_target_position_gimbal_x = LaunchConfiguration(
        "vision_target_position_gimbal_x"
    )
    vision_target_position_gimbal_y = LaunchConfiguration(
        "vision_target_position_gimbal_y"
    )
    vision_target_position_gimbal_z = LaunchConfiguration(
        "vision_target_position_gimbal_z"
    )
    vision_target_position_map_x = LaunchConfiguration("vision_target_position_map_x")
    vision_target_position_map_y = LaunchConfiguration("vision_target_position_map_y")
    vision_target_position_map_z = LaunchConfiguration("vision_target_position_map_z")
    vision_has_target_position_map = LaunchConfiguration("vision_has_target_position_map")
    vision_target_position_map_frame = LaunchConfiguration(
        "vision_target_position_map_frame"
    )

    truthy_fake_inputs = PythonExpression(
        [
            "'",
            publish_fake_inputs,
            "'.strip().lower() in ['true', '1', 'yes', 'on']",
        ]
    )

    bringup_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(bringup_dir, "launch", "bringup.launch.py")
        ),
        launch_arguments={
            "namespace": namespace,
            "robot_name": robot_name,
            "slam": slam,
            "world": world,
            "map": map_yaml_file,
            "prior_pcd_file": prior_pcd_file,
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

    fake_inputs_cmd = Node(
        package="pb2025_sentry_bringup",
        executable="fake_decision_sim_inputs.py",
        name="fake_decision_sim_inputs",
        output="screen",
        condition=IfCondition(truthy_fake_inputs),
        parameters=[
            {
                "use_sim_time": use_sim_time,
                "initial_x": initial_x,
                "initial_y": initial_y,
                "initial_yaw": initial_yaw,
                "publish_decision_mode": publish_decision_mode,
                "decision_mode": decision_mode,
                "publish_referee_inputs": False,
                "publish_vision_target": publish_vision_target,
                "vision_tracking": vision_tracking,
                "vision_nav_hold": vision_nav_hold,
                "vision_fire_permitted": vision_fire_permitted,
                "vision_target_id": vision_target_id,
                "vision_confidence": vision_confidence,
                "vision_target_distance": vision_target_distance,
                "vision_target_yaw": vision_target_yaw,
                "vision_target_pitch": vision_target_pitch,
                "vision_target_position_gimbal_x": vision_target_position_gimbal_x,
                "vision_target_position_gimbal_y": vision_target_position_gimbal_y,
                "vision_target_position_gimbal_z": vision_target_position_gimbal_z,
                "vision_target_position_map_x": vision_target_position_map_x,
                "vision_target_position_map_y": vision_target_position_map_y,
                "vision_target_position_map_z": vision_target_position_map_z,
                "vision_has_target_position_map": vision_has_target_position_map,
                "vision_target_position_map_frame": vision_target_position_map_frame,
            }
        ],
        arguments=["--ros-args", "--log-level", log_level],
    )

    ld = LaunchDescription()

    ld.add_action(
        DeclareLaunchArgument(
            "namespace",
            default_value="",
            description="Top-level namespace.",
        )
    )
    ld.add_action(
        DeclareLaunchArgument(
            "robot_name",
            default_value="pb2025_sentry_robot",
            description="Robot xacro name used by the shared sentry bringup.",
        )
    )
    ld.add_action(
        DeclareLaunchArgument(
            "slam",
            default_value="True",
            description="True for mapping mode, False for localization/navigation mode.",
        )
    )
    ld.add_action(
        DeclareLaunchArgument(
            "world",
            default_value="rmul",
            description="Map/PCD basename used by navigation mode.",
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
            description="Full path to map yaml used when slam is False.",
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
            description="Full path to prior PCD used when slam is False.",
        )
    )
    ld.add_action(
        DeclareLaunchArgument(
            "use_sim_time",
            default_value="False",
            description="Use simulated clock.",
        )
    )
    ld.add_action(
        DeclareLaunchArgument(
            "params_file",
            default_value=os.path.join(bringup_dir, "params", "node_params.yaml"),
            description="Full path to navigation and robot node parameters.",
        )
    )
    ld.add_action(
        DeclareLaunchArgument(
            "behavior_params_file",
            default_value=os.path.join(
                behavior_dir, "params", "sentry_behavior_decision_vision_test.yaml"
            ),
            description="Behavior parameters for referee-free decision and vision follow test.",
        )
    )
    ld.add_action(
        DeclareLaunchArgument(
            "rviz_config_file",
            default_value=os.path.join(bringup_dir, "rviz", "sentry_default_view.rviz"),
            description="RViz config file.",
        )
    )
    ld.add_action(
        DeclareLaunchArgument(
            "rviz_force_software",
            default_value="0",
            description="Force RViz to use Mesa software rendering when set to 1.",
        )
    )
    ld.add_action(
        DeclareLaunchArgument(
            "use_robot_state_pub",
            default_value="False",
            description="Whether to start robot_state_publisher.",
        )
    )
    ld.add_action(
        DeclareLaunchArgument(
            "use_rviz",
            default_value="False",
            description="Whether to start RViz.",
        )
    )
    ld.add_action(
        DeclareLaunchArgument(
            "launch_joy_teleop",
            default_value="False",
            description="Whether to start joystick teleop.",
        )
    )
    ld.add_action(
        DeclareLaunchArgument(
            "use_composition",
            default_value="True",
            description="Whether to use composed Nav2 bringup.",
        )
    )
    ld.add_action(
        DeclareLaunchArgument(
            "use_respawn",
            default_value="True",
            description="Whether to respawn launched nodes on crash.",
        )
    )
    ld.add_action(
        DeclareLaunchArgument(
            "log_level",
            default_value="info",
            description="Log level.",
        )
    )
    ld.add_action(
        DeclareLaunchArgument(
            "publish_fake_inputs",
            default_value="False",
            description="Publish fake decision mode and/or fake vision target for bench tests.",
        )
    )
    ld.add_action(
        DeclareLaunchArgument(
            "initial_x",
            default_value="0.0",
            description="Fake initial pose x, only used when publish_fake_inputs is True.",
        )
    )
    ld.add_action(
        DeclareLaunchArgument(
            "initial_y",
            default_value="0.0",
            description="Fake initial pose y, only used when publish_fake_inputs is True.",
        )
    )
    ld.add_action(
        DeclareLaunchArgument(
            "initial_yaw",
            default_value="0.0",
            description="Fake initial pose yaw, only used when publish_fake_inputs is True.",
        )
    )
    ld.add_action(
        DeclareLaunchArgument(
            "publish_decision_mode",
            default_value="True",
            description="Whether fake input node publishes decision/sim_mode.",
        )
    )
    ld.add_action(
        DeclareLaunchArgument(
            "decision_mode",
            default_value="patrol",
            description="Fake decision mode: patrol/anchor/retreat/safe.",
        )
    )
    ld.add_action(
        DeclareLaunchArgument(
            "publish_vision_target",
            default_value="False",
            description="Whether fake input node publishes vision/target.",
        )
    )
    ld.add_action(
        DeclareLaunchArgument(
            "vision_tracking",
            default_value="False",
            description="Fake vision tracking flag.",
        )
    )
    ld.add_action(
        DeclareLaunchArgument(
            "vision_nav_hold",
            default_value="True",
            description="Fake vision nav_hold flag.",
        )
    )
    ld.add_action(
        DeclareLaunchArgument(
            "vision_fire_permitted",
            default_value="False",
            description="Fake vision fire permission flag.",
        )
    )
    ld.add_action(
        DeclareLaunchArgument(
            "vision_target_id",
            default_value="7",
            description="Fake vision target id.",
        )
    )
    ld.add_action(
        DeclareLaunchArgument(
            "vision_confidence",
            default_value="1.0",
            description="Fake vision confidence.",
        )
    )
    ld.add_action(
        DeclareLaunchArgument(
            "vision_target_distance",
            default_value="3.0",
            description="Fake vision target distance in meters.",
        )
    )
    ld.add_action(
        DeclareLaunchArgument(
            "vision_target_yaw",
            default_value="0.0",
            description="Fake vision yaw command in rad.",
        )
    )
    ld.add_action(
        DeclareLaunchArgument(
            "vision_target_pitch",
            default_value="0.0",
            description="Fake vision pitch command in rad.",
        )
    )
    ld.add_action(
        DeclareLaunchArgument(
            "vision_target_position_gimbal_x",
            default_value="1.0",
            description="Fake vision target gimbal-frame x position.",
        )
    )
    ld.add_action(
        DeclareLaunchArgument(
            "vision_target_position_gimbal_y",
            default_value="0.0",
            description="Fake vision target gimbal-frame y position.",
        )
    )
    ld.add_action(
        DeclareLaunchArgument(
            "vision_target_position_gimbal_z",
            default_value="0.0",
            description="Fake vision target gimbal-frame z position.",
        )
    )
    ld.add_action(
        DeclareLaunchArgument(
            "vision_target_position_map_x",
            default_value="0.0",
            description="Fake vision target map-frame x position.",
        )
    )
    ld.add_action(
        DeclareLaunchArgument(
            "vision_target_position_map_y",
            default_value="0.0",
            description="Fake vision target map-frame y position.",
        )
    )
    ld.add_action(
        DeclareLaunchArgument(
            "vision_target_position_map_z",
            default_value="0.0",
            description="Fake vision target map-frame z position.",
        )
    )
    ld.add_action(
        DeclareLaunchArgument(
            "vision_has_target_position_map",
            default_value="False",
            description="Whether fake target_position_map is valid.",
        )
    )
    ld.add_action(
        DeclareLaunchArgument(
            "vision_target_position_map_frame",
            default_value="map",
            description="Frame id of fake target_position_map.",
        )
    )

    ld.add_action(bringup_cmd)
    ld.add_action(fake_inputs_cmd)
    return ld
