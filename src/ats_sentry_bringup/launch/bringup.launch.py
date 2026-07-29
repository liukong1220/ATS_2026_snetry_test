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
    launch_small_gicp_relocalization = LaunchConfiguration(
        "launch_small_gicp_relocalization"
    )
    launch_localization_fusion = LaunchConfiguration("launch_localization_fusion")
    launch_fake_vel_transform = LaunchConfiguration("launch_fake_vel_transform")
    launch_chassis_vel_transform = LaunchConfiguration("launch_chassis_vel_transform")
    nav_cmd_vel_topic = LaunchConfiguration("nav_cmd_vel_topic")
    fake_vel_output_topic = LaunchConfiguration("fake_vel_output_topic")
    chassis_vel_input_topic = LaunchConfiguration("chassis_vel_input_topic")
    launch_nav2 = LaunchConfiguration("launch_nav2")
    launch_swerve_mpc = LaunchConfiguration("launch_swerve_mpc")
    minco_params_file = LaunchConfiguration("minco_params_file")
    goal_manager_params_file = LaunchConfiguration("goal_manager_params_file")
    mpc_params_file = LaunchConfiguration("mpc_params_file")
    mpc_cmd_vel_topic = LaunchConfiguration("mpc_cmd_vel_topic")
    require_gimbal_status = LaunchConfiguration("require_gimbal_status")
    launch_lidar_static_tf = LaunchConfiguration("launch_lidar_static_tf")
    lidar_static_tf_x = LaunchConfiguration("lidar_static_tf_x")
    lidar_static_tf_y = LaunchConfiguration("lidar_static_tf_y")
    lidar_static_tf_z = LaunchConfiguration("lidar_static_tf_z")
    lidar_static_tf_roll = LaunchConfiguration("lidar_static_tf_roll")
    lidar_static_tf_pitch = LaunchConfiguration("lidar_static_tf_pitch")
    lidar_static_tf_yaw = LaunchConfiguration("lidar_static_tf_yaw")
    planning_grid_owner = LaunchConfiguration("planning_grid_owner")
    launch_rog_map = LaunchConfiguration("launch_rog_map")
    rog_map_config_file = LaunchConfiguration("rog_map_config_file")
    rog_map_adapter_params_file = LaunchConfiguration("rog_map_adapter_params_file")
    use_composition = LaunchConfiguration("use_composition")
    use_respawn = LaunchConfiguration("use_respawn")
    log_level = LaunchConfiguration("log_level")

    # Nav2-free profile 下 MPC 之后不允许任何旋转级或增益级，
    # 因此速度变换级的存在性判断必须同时要求 launch_nav2 为真。
    any_velocity_transform = PythonExpression(
        [
            "'",
            launch_nav2,
            "'.lower() == 'true' and ('",
            launch_fake_vel_transform,
            "'.lower() == 'true' or '",
            launch_chassis_vel_transform,
            "'.lower() == 'true')",
        ]
    )

    chassis_vel_transform_enabled = PythonExpression(
        [
            "'",
            launch_nav2,
            "'.lower() == 'true' and '",
            launch_chassis_vel_transform,
            "'.lower() == 'true'",
        ]
    )

    # 实车 GimbalYawStatus 桥只在真的需要 ack 时启动，且实车上是该话题的唯一
    # 发布者。MuJoCo 的模拟 ack 属于仿真 profile，两者不得同时运行。
    gimbal_status_bridge_enabled = PythonExpression(
        [
            "'",
            require_gimbal_status,
            "'.lower() == 'true'",
        ]
    )

    # `gimbal_yaw_odom -> front_mid360` 在实车上原本没有任何发布者：
    # 仿真里由 `ats_mujoco_sim/sim_node.py:_publish_static_transforms()` 发布，
    # 实车侧 grep 全仓（排除 `src/sim/`）没有任何 TransformBroadcaster 发这条边。
    # 缺这条边的后果不止是雷达坐标系缺失：`sensor_scan_generation` 的
    # `odometryHandler` 在 `lidar_frame->robot_base_frame` 或
    # `lidar_frame->base_frame` 查询失败时直接 return，于是
    # `odom->gimbal_yaw_odom` 与 `odom->base_footprint` 也一起消失，
    # `loam_interface` 的 `base_frame_to_lidar_initialized_` 永远不会置真。
    # 因此实车必须有唯一一份静态 TF 发布者；仿真 profile 下必须关掉以免出现第二个发布者。
    lidar_static_tf_enabled = PythonExpression(
        [
            "'",
            use_sim_time,
            "'.lower() == 'false' and '",
            launch_lidar_static_tf,
            "'.lower() == 'true'",
        ]
    )

    # ROGMap 只在显式开启或它就是规划栅格所有者时启动；adapter 只在它是所有者时启动，
    # 保证 /rc_esdf/planning_grid 始终只有一个发布者。
    rog_map_enabled = PythonExpression(
        [
            "'",
            launch_rog_map,
            "'.lower() == 'true' or '",
            planning_grid_owner,
            "'.lower() == 'rog_map'",
        ]
    )
    rog_map_adapter_enabled = PythonExpression(
        [
            "'",
            planning_grid_owner,
            "'.lower() == 'rog_map'",
        ]
    )

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

    declare_launch_localization_fusion_cmd = DeclareLaunchArgument(
        "launch_localization_fusion",
        default_value="True",
        description="Whether localization_fusion owns map->odom and /localization",
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

    declare_launch_nav2_cmd = DeclareLaunchArgument(
        "launch_nav2",
        default_value="True",
        description=(
            "Whether to start the Nav2 comparison stack. False selects the "
            "Nav2-free MINCO+MPC profile."
        ),
    )

    declare_launch_swerve_mpc_cmd = DeclareLaunchArgument(
        "launch_swerve_mpc",
        default_value="False",
        description=(
            "Whether to start minco_planner/ats_goal_manager/ats_swerve_mpc. "
            "Only effective together with launch_nav2:=False."
        ),
    )

    declare_minco_params_file_cmd = DeclareLaunchArgument(
        "minco_params_file",
        default_value=os.path.join(
            get_package_share_directory("minco_planner"),
            "config",
            "minco_planner_reality.yaml",
        ),
        description="Authoritative minco_planner parameter file for this profile",
    )

    declare_goal_manager_params_file_cmd = DeclareLaunchArgument(
        "goal_manager_params_file",
        default_value=os.path.join(
            get_package_share_directory("ats_goal_manager"),
            "config",
            "ats_goal_manager_reality.yaml",
        ),
        description="Authoritative ats_goal_manager parameter file for this profile",
    )

    declare_mpc_params_file_cmd = DeclareLaunchArgument(
        "mpc_params_file",
        default_value=os.path.join(
            get_package_share_directory("ats_swerve_mpc"),
            "config",
            "ats_swerve_mpc_reality.yaml",
        ),
        description="Authoritative ats_swerve_mpc parameter file for this profile",
    )

    declare_mpc_cmd_vel_topic_cmd = DeclareLaunchArgument(
        "mpc_cmd_vel_topic",
        default_value="/cmd_vel",
        description=(
            "MPC body-frame [vx, vy, wz] output topic; the serial chassis is the "
            "single consumer and no stage may follow the MPC."
        ),
    )

    declare_require_gimbal_status_cmd = DeclareLaunchArgument(
        "require_gimbal_status",
        default_value="True",
        description=(
            "Whether ats_goal_manager/ats_swerve_mpc require a fresh "
            "GimbalYawStatus ack. Only a controlled HIL profile may set False, "
            "and BODY_YAW_FOLLOW is forbidden while it is False."
        ),
    )

    declare_launch_lidar_static_tf_cmd = DeclareLaunchArgument(
        "launch_lidar_static_tf",
        default_value="True",
        description=(
            "Publish the single authoritative real-robot gimbal_yaw_odom -> "
            "front_mid360 static TF. Must stay False in simulation, where "
            "ats_mujoco_sim/sim_node.py already owns this edge."
        ),
    )

    # 数值来源：`ats_robot_description/resource/xmacro/ats_sentry_robot.sdf.xmacro:45`
    # 的 `pose="-0.2 -0.0 0.0 0.0 0 -${61*pi/180}"`（仿真安装位姿），
    # 与 `ats_mujoco_sim/sim_node.py:105-108` 的常量一致。
    # 【未实测】这四个数字是仿真模型口径，不是实车卷尺/标定结果。
    # 停止条件：抬轮 HIL 之后、落地行走之前必须用实测外参替换，
    # 否则不得据此推断任何定位精度或障碍物距离。
    declare_lidar_static_tf_x_cmd = DeclareLaunchArgument(
        "lidar_static_tf_x",
        default_value="-0.2",
        description="gimbal_yaw_odom -> front_mid360 x [m] (UNMEASURED, from xmacro).",
    )

    declare_lidar_static_tf_y_cmd = DeclareLaunchArgument(
        "lidar_static_tf_y",
        default_value="0.0",
        description="gimbal_yaw_odom -> front_mid360 y [m] (UNMEASURED, from xmacro).",
    )

    declare_lidar_static_tf_z_cmd = DeclareLaunchArgument(
        "lidar_static_tf_z",
        default_value="0.0",
        description="gimbal_yaw_odom -> front_mid360 z [m] (UNMEASURED, from xmacro).",
    )

    declare_lidar_static_tf_roll_cmd = DeclareLaunchArgument(
        "lidar_static_tf_roll",
        default_value="0.0",
        description="gimbal_yaw_odom -> front_mid360 roll [rad] (UNMEASURED).",
    )

    declare_lidar_static_tf_pitch_cmd = DeclareLaunchArgument(
        "lidar_static_tf_pitch",
        default_value="0.0",
        description="gimbal_yaw_odom -> front_mid360 pitch [rad] (UNMEASURED).",
    )

    declare_lidar_static_tf_yaw_cmd = DeclareLaunchArgument(
        "lidar_static_tf_yaw",
        # -61 deg = -1.0646508437165408 rad，与 xmacro 的 -${61*pi/180} 同值。
        default_value="-1.0646508437165408",
        description="gimbal_yaw_odom -> front_mid360 yaw [rad] (UNMEASURED, -61 deg).",
    )

    declare_planning_grid_owner_cmd = DeclareLaunchArgument(
        "planning_grid_owner",
        default_value="rc_esdf",
        choices=["rc_esdf", "rog_map"],
        description="Single /rc_esdf/planning_grid owner: rc_esdf or rog_map",
    )

    declare_launch_rog_map_cmd = DeclareLaunchArgument(
        "launch_rog_map",
        default_value="False",
        description="Start ROGMap 3D occupancy perception without changing the RC-ESDF planning owner.",
    )

    declare_rog_map_config_file_cmd = DeclareLaunchArgument(
        "rog_map_config_file",
        default_value=os.path.join(
            get_package_share_directory("ats_rog_map"), "config", "rog_map.yaml"
        ),
        description="ROGMap probabilistic/ESDF map configuration file",
    )

    declare_rog_map_adapter_params_file_cmd = DeclareLaunchArgument(
        "rog_map_adapter_params_file",
        default_value=os.path.join(
            get_package_share_directory("ats_rog_map_adapter"),
            "config",
            "rog_map_ground_planning.yaml",
        ),
        description="ROGMap ground-projection adapter parameter file",
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

    # 实车 `gimbal_yaw_odom -> front_mid360` 的唯一发布者。
    #
    # 为什么必须在这里：Point-LIO 的 `mapping.extrinsic_T/extrinsic_R` 只描述
    # IMU->LiDAR 的机内外参，不产生任何 TF；`sensor_scan_generation` 与
    # `loam_interface` 都要求 `front_mid360` 在 TF 树里可查，查不到就整帧 return。
    # 因此这条边只能由静态 TF 提供，且实车只允许一份（仿真由 sim_node.py 发布）。
    #
    # 【未实测占位】数值取自仿真模型（xmacro / sim_node.py），不是实车标定结果。
    # 这属于本轮的停止条件之一：抬轮 HIL 可以用它把 TF 树接通并做连通性检查，
    # 但不得据此声明定位精度达标，也不得进入落地行走。
    static_tf_lidar_cmd = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="static_tf_gimbal_yaw_odom_to_front_mid360",
        condition=IfCondition(lidar_static_tf_enabled),
        namespace=namespace,
        arguments=[
            "--x",
            lidar_static_tf_x,
            "--y",
            lidar_static_tf_y,
            "--z",
            lidar_static_tf_z,
            "--roll",
            lidar_static_tf_roll,
            "--pitch",
            lidar_static_tf_pitch,
            "--yaw",
            lidar_static_tf_yaw,
            "--frame-id",
            "gimbal_yaw_odom",
            "--child-frame-id",
            "front_mid360",
        ],
        parameters=[{"use_sim_time": use_sim_time}],
        output="screen",
    )

    static_tf_base_link_cmd = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="static_tf_base_link",
        condition=UnlessCondition(use_robot_state_pub),
        arguments=["--frame-id", "base_footprint", "--child-frame-id", "base_link"],
        output="screen",
    )

    # Nav2-free profile 下这一级必须不存在：它会在 MPC 之后按云台 yaw 再旋转一次
    # [vx, vy]，并且缺少 serial/gimbal_joint_state 时会静默直通。
    start_chassis_vel_transform_cmd = Node(
        package="sentry_chassis_vel_transform",
        executable="chassis_vel_transform_node",
        name="chassis_vel_transform",
        condition=IfCondition(chassis_vel_transform_enabled),
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

    # 实车云台 ack 的唯一来源：串口 serial/gimbal_joint_state 的新鲜度 +
    # odom->gimbal_yaw_odom、odom->base_footprint 两条实测 TF。
    # tf_healthy 与 locked 都是实测量，禁止伪造 ack。
    start_gimbal_yaw_status_bridge_cmd = Node(
        package="standard_robot_pp_ros2",
        executable="gimbal_yaw_status_bridge_node",
        name="gimbal_yaw_status_bridge",
        condition=IfCondition(gimbal_status_bridge_enabled),
        namespace=namespace,
        output="screen",
        respawn=use_respawn,
        respawn_delay=2.0,
        parameters=[configured_params],
        arguments=["--ros-args", "--log-level", log_level],
    )

    # 实车 ROGMap：frame 与话题在此处显式给出（node_params.yaml 无 rog_map 段），
    # 避免出现第二处不同数值的地图参数来源。
    start_rog_map_cmd = Node(
        package="ats_rog_map",
        executable="ats_rog_map_node",
        name="ats_rog_map",
        namespace=namespace,
        condition=IfCondition(rog_map_enabled),
        output="screen",
        respawn=use_respawn,
        respawn_delay=2.0,
        parameters=[
            {
                "use_sim_time": use_sim_time,
                "map_frame": "odom",
                "base_frame": "gimbal_yaw_odom",
                "sensor_frame": "front_mid360",
                "odom_topic": "/localization",
                "cloud_topic": "/registered_scan",
                "map_config_file": rog_map_config_file,
                "cloud_timeout_sec": 2.0,
                "odom_timeout_sec": 2.0,
            }
        ],
        arguments=["--ros-args", "--log-level", log_level],
    )

    # 只有 planning_grid_owner=rog_map 时才启动 adapter，
    # 保证 /rc_esdf/planning_grid 只有一个发布者。
    start_rog_map_adapter_cmd = Node(
        package="ats_rog_map_adapter",
        executable="ats_rog_map_adapter_node",
        name="ats_rog_map_adapter",
        namespace=namespace,
        condition=IfCondition(rog_map_adapter_enabled),
        output="screen",
        respawn=use_respawn,
        respawn_delay=2.0,
        parameters=[
            ParameterFile(rog_map_adapter_params_file, allow_substs=True),
            {
                "use_sim_time": use_sim_time,
                # 实车必须要求定位健康，投影快照不得跨 epoch 复用。
                "require_localization_status": launch_localization_fusion,
            },
        ],
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
            "launch_localization_fusion": launch_localization_fusion,
            "launch_fake_vel_transform": launch_fake_vel_transform,
            "launch_chassis_vel_transform": "False",
            "nav_cmd_vel_topic": nav_cmd_vel_topic,
            "fake_vel_output_topic": fake_vel_output_topic,
            "chassis_vel_input_topic": chassis_vel_input_topic,
            "launch_nav2": launch_nav2,
            "launch_swerve_mpc": launch_swerve_mpc,
            "minco_params_file": minco_params_file,
            "goal_manager_params_file": goal_manager_params_file,
            "mpc_params_file": mpc_params_file,
            "mpc_cmd_vel_topic": mpc_cmd_vel_topic,
            "require_gimbal_status": require_gimbal_status,
            "planning_grid_owner": planning_grid_owner,
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
    ld.add_action(declare_launch_localization_fusion_cmd)
    ld.add_action(declare_launch_fake_vel_transform_cmd)
    ld.add_action(declare_launch_chassis_vel_transform_cmd)
    ld.add_action(declare_nav_cmd_vel_topic_cmd)
    ld.add_action(declare_fake_vel_output_topic_cmd)
    ld.add_action(declare_chassis_vel_input_topic_cmd)
    ld.add_action(declare_launch_rog_map_cmd)
    ld.add_action(declare_launch_nav2_cmd)
    ld.add_action(declare_launch_swerve_mpc_cmd)
    ld.add_action(declare_minco_params_file_cmd)
    ld.add_action(declare_goal_manager_params_file_cmd)
    ld.add_action(declare_mpc_params_file_cmd)
    ld.add_action(declare_mpc_cmd_vel_topic_cmd)
    ld.add_action(declare_require_gimbal_status_cmd)
    ld.add_action(declare_launch_lidar_static_tf_cmd)
    ld.add_action(declare_lidar_static_tf_x_cmd)
    ld.add_action(declare_lidar_static_tf_y_cmd)
    ld.add_action(declare_lidar_static_tf_z_cmd)
    ld.add_action(declare_lidar_static_tf_roll_cmd)
    ld.add_action(declare_lidar_static_tf_pitch_cmd)
    ld.add_action(declare_lidar_static_tf_yaw_cmd)
    ld.add_action(declare_planning_grid_owner_cmd)
    ld.add_action(declare_rog_map_config_file_cmd)
    ld.add_action(declare_rog_map_adapter_params_file_cmd)
    ld.add_action(declare_use_composition_cmd)
    ld.add_action(declare_use_respawn_cmd)
    ld.add_action(declare_log_level_cmd)

    # Add the actions to launch all of the navigation nodes
    ld.add_action(start_rviz_cmd)
    ld.add_action(start_serial_driver_cmd)
    # navigation_launch owns base_footprint -> base_link to keep one TF authority.
    # 实车侧 gimbal_yaw_odom -> front_mid360 只有这一个发布者（仿真下条件为假）。
    ld.add_action(static_tf_lidar_cmd)
    ld.add_action(start_chassis_vel_transform_cmd)
    ld.add_action(start_gimbal_yaw_status_bridge_cmd)
    ld.add_action(start_rog_map_cmd)
    ld.add_action(start_rog_map_adapter_cmd)
    ld.add_action(start_navigation_launch_cmd)
    ld.add_action(start_behavior_launch_cmd)
    ld.add_action(record_rosbag_cmd)

    return ld
