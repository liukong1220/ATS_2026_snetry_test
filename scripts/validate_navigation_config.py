#!/usr/bin/env python3
"""Static validator locking formal single-source navigation config and contracts."""

from validate_navigation_rviz_helpers import *  # noqa: F401,F403


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "params_file",
        nargs="?",
        type=Path,
        default=Path("src/ats_sentry_bringup/params/node_params.yaml"),
    )
    args = parser.parse_args()
    workspace = Path(__file__).resolve().parents[1]
    params_path = (
        args.params_file
        if args.params_file.is_absolute()
        else workspace / args.params_file
    )
    document = load_yaml(params_path)
    assert_shared_profile_contract(workspace, document)
    navigation_root_nodes = (
        "bt_navigator",
        "controller_server",
        "planner_server",
        "behavior_server",
        "map_server",
        "smoother_server",
        "waypoint_follower",
        "velocity_smoother",
        "local_costmap",
        "global_costmap",
    )
    for node_name in navigation_root_nodes + (
        "terrain_analysis",
        "pointcloud_to_laserscan",
        "slam_toolbox",
    ):
        assert node_name not in document, f"root YAML retains {node_name}"

    rog_map = parameters(document, "ats_rog_map")
    adapter = parameters(document, "ats_rog_map_adapter")
    minco = parameters(document, "minco_planner")
    goal_manager = parameters(document, "ats_goal_manager")
    mpc = parameters(document, "ats_swerve_mpc")
    joint_state_publisher = parameters(document, "joint_state_publisher")
    gimbal_yaw_status_bridge = parameters(document, "gimbal_yaw_status_bridge")
    behavior_server = parameters(document, "ats_sentry_behavior_server")
    behavior_client = parameters(document, "ats_sentry_behavior_client")
    point_lio = parameters(document, "point_lio")
    gicp = parameters(document, "small_gicp_relocalization")
    loc_fusion = parameters(document, "localization_fusion")

    assert rog_map["map_frame"] == "odom"
    assert rog_map["debug_bounds_topic"] == "/rog_map/bounds"
    assert rog_map["debug_viz_topic"] == "/rog_map/viz"
    assert rog_map["debug_viz_stride"] == 1
    assert rog_map["debug_viz_include_unknown"] is False
    assert rog_map["core.esdf.enable"] is True
    assert rog_map["core.esdf.update_interval_updates"] == 1
    assert rog_map["core.raycasting.p_occupied"] == 0.80
    assert rog_map["core.raycasting.p_miss"] == 0.30
    assert rog_map["core.map_size"] == [10.0, 10.0, 1.0]
    assert "map_config_file" not in rog_map
    # M3: the projection is an in-process call; no service name is configurable.
    assert "projection_service" not in adapter
    for removed in ("planning_grid_topic", "signed_distance_grid_topic",
                    "footprint_clearance_grid_topic"):
        assert removed not in adapter, f"adapter retains removed parameter {removed}"
    assert adapter["planning_grid_viz_topic"] == PLANNING_GRID_VIZ_TOPIC
    assert adapter["esdf_cloud_topic"] == ESDF_CLOUD_VIZ_TOPIC
    assert "grid_topic" not in minco, "MINCO plans on planning_snapshot only"
    assert adapter["unknown_is_obstacle"] is True
    assert adapter["require_localization_status"] is True
    assert_planning_snapshot_lease_contract(adapter, goal_manager)
    assert minco["goal_topic"] == ""
    assert "global_plan_topic" not in minco
    assert minco["goal_request_topic"] == "/ats_goal_manager/planner_goal"
    assert minco["map_ready_topic"] == "/rog_map_adapter/ready"
    # MINCO consumes the owner's numeric snapshot and refuses one whose blocking
    # policy differs from its own, so the two parameter sets must agree.
    assert minco["planning_snapshot_topic"] == adapter["planning_snapshot_topic"]
    assert minco["obstacle_value_threshold"] == adapter["terrain_obstacle_value_threshold"]
    assert minco["unknown_is_obstacle"] == adapter["unknown_is_obstacle"]
    assert minco["unknown_is_obstacle"] is True
    assert minco["publish_unsafe_trajectory"] is False
    assert minco["raw_path_topic"] == "minco/raw_path"
    assert minco["reference_path_topic"] == "minco/reference_path"
    assert minco["candidate_reference_path_topic"] == "/minco/reference_path_candidate"
    assert minco["planner_manages_emergency_stop"] is False
    assert goal_manager["action_name"] == "/ats_navigate_to_pose"
    assert goal_manager["planner_goal_topic"] == minco["goal_request_topic"]
    assert goal_manager["candidate_reference_topic"] == minco["candidate_reference_path_topic"]
    assert goal_manager["reference_path_topic"] == "/minco/reference_path"
    assert mpc["trajectory_topic"] == goal_manager["reference_path_topic"]
    assert mpc["frame_id"] == rog_map["map_frame"]
    fake_transform = parameters(document, "fake_vel_transform")
    chassis_transform = parameters(document, "chassis_vel_transform")
    serial = parameters(document, "standard_robot_pp_ros2")
    arbiter = parameters(document, "cmd_vel_arbiter")
    assert mpc["command_topic"] == "/cmd_vel/autonomy_raw"
    assert fake_transform["input_cmd_vel_topic"] == mpc["command_topic"]
    assert fake_transform["output_cmd_vel_topic"] == "/cmd_vel/autonomy_gimbal"
    assert (
        chassis_transform["input_cmd_vel_topic"]
        == fake_transform["output_cmd_vel_topic"]
    )
    assert chassis_transform["output_cmd_vel_topic"] == "/cmd_vel/autonomy"
    assert arbiter["manual_cmd_vel_topic"] == "/cmd_vel"
    assert arbiter["autonomy_cmd_vel_topic"] == chassis_transform["output_cmd_vel_topic"]
    assert arbiter["selected_cmd_vel_topic"] == "/cmd_vel/selected"
    assert arbiter["planner_status_topic"] == minco["planner_status_topic"]
    assert arbiter["planner_status_topic"] == "/minco/planning_status"
    assert arbiter["map_ready_topic"] == "/rog_map_adapter/ready"
    assert arbiter["execution_command_topic"] == "/planner/execution_command"
    assert arbiter["link_health_topic"] == "/serial/link_up"
    assert arbiter["require_serial_link"] is True
    assert arbiter["link_timeout_ms"] == 300
    assert arbiter["manual_timeout_ms"] == serial["cmd_vel_watchdog_timeout_ms"]
    assert serial["cmd_vel_topic"] == arbiter["selected_cmd_vel_topic"]
    assert serial["require_execution_authorization"] is False
    assert serial["execution_command_topic"] == ""
    serial_default_config = parameters(
        load_yaml(
            workspace
            / "src/standard_robot_pp_ros2/config/standard_robot_pp_ros2.yaml"
        ),
        "standard_robot_pp_ros2",
    )
    assert serial_default_config["cmd_vel_topic"] == arbiter["selected_cmd_vel_topic"]
    assert serial_default_config["require_execution_authorization"] is False
    assert serial_default_config["execution_command_topic"] == ""
    serial_node_source = (
        workspace / "src/standard_robot_pp_ros2/src/standard_robot_pp_ros2.cpp"
    ).read_text(encoding="utf-8")
    serial_node_header = (
        workspace
        / "src/standard_robot_pp_ros2/include/standard_robot_pp_ros2/standard_robot_pp_ros2.hpp"
    ).read_text(encoding="utf-8")
    assert 'declare_parameter("cmd_vel_topic", std::string("/cmd_vel/selected"))' in serial_node_source
    assert 'declare_parameter("execution_command_topic", std::string(""))' in serial_node_source
    assert 'std::string cmd_vel_topic_{ "/cmd_vel/selected" };' in serial_node_header
    assert "if (!execution_command_topic_.empty())" in serial_node_source
    assert joint_state_publisher["source_list"] == ["serial/gimbal_joint_state"]
    assert chassis_transform["joint_state_topic"] == "serial/gimbal_joint_state"
    assert (
        gimbal_yaw_status_bridge["joint_state_topic"]
        == "serial/gimbal_joint_state"
    )
    assert gimbal_yaw_status_bridge["gimbal_status_topic"] == "/gimbal/yaw_status"
    assert (
        gimbal_yaw_status_bridge["yaw_authority_request_topic"]
        == "/gimbal/yaw_authority_request"
    )
    assert mpc["max_vy"] > 0.0
    # 控制周期与预测离散步长解耦，但控制频率不得慢于预测离散步长，且仲裁器不低于 MPC 频率。
    assert 0.0 < 1.0 / mpc["control_rate_hz"] <= mpc["dt"] + 1e-9
    assert arbiter["control_rate_hz"] >= mpc["control_rate_hz"]
    # 终点区限速必须比底盘约 0.4 s 的响应滞后的 e 倍更慢，且只覆盖终点附近（报告 5.1.10）。
    assert mpc["terminal_approach_time_constant"] >= 1.1
    assert mpc["goal_position_tolerance"] < mpc["terminal_approach_radius"] <= 1.0
    # 换代宽限：MPC 与 arbiter 必须一致，否则一侧放行、另一侧撤销；且不得长于执行授权租约。
    assert mpc["supersede_grace_sec"] == arbiter["supersede_grace_sec"]
    assert 0.0 <= mpc["supersede_grace_sec"] < mpc["execution_command_timeout"]
    assert arbiter["supersede_grace_sec"] < arbiter["execution_command_timeout"]
    assert mpc["solver_mode"] == "qp"
    assert mpc["planning_snapshot_topic"] == adapter["planning_snapshot_topic"]
    assert mpc["require_planning_snapshot"] is True
    assert float(mpc["planning_snapshot_timeout_sec"]) == float(
        mpc["map_ready_timeout_sec"]
    ) == float(goal_manager["planning_snapshot_timeout_sec"])
    assert float(mpc["qp_time_limit_ms"]) < 1000.0 / float(mpc["control_rate_hz"])
    assert float(mpc["lag_weight"]) > 0.0 and float(mpc["contour_weight"]) >= float(mpc["lag_weight"])
    assert mpc["rollout_safety_enabled"] is True and mpc["qp_esdf_corridor_enabled"] is True
    assert mpc["use_derivative_feedforward"] is True
    assert 0.0 <= float(mpc["feedforward_accel_lead_sec"]) <= 0.10
    assert minco["rolling_horizon_enabled"] is True
    assert float(minco["rolling_horizon_length_m"]) >= 4.0
    assert float(minco["rolling_min_tail_length_m"]) >= 0.5
    assert float(minco["joint_time_budget_ms"]) <= float(minco["minco_stage_budget_ms"]) <= 20.0
    assert float(minco["plan_wall_budget_ms"]) <= 30.0
    assert float(minco["max_acceleration"]) == float(mpc["max_ax"])
    assert int(minco["far_conflict_confirm_snapshots"]) >= 1
    assert goal_manager["rolling_replan_enabled"] is True
    assert 5.0 <= float(goal_manager["rolling_replan_hz"]) <= 15.0
    assert float(goal_manager["rolling_replan_advance_m"]) >= 0.5
    assert float(goal_manager["retarget_cruise_threshold"]) >= float(goal_manager["retarget_threshold"])
    assert float(goal_manager["replan_advised_cooldown_sec"]) >= 0.0
    assert isinstance(rog_map["odom_layer.tunnel_free_enabled"], bool)
    assert int(rog_map["odom_layer.tunnel_free_confirm_frames"]) >= 1
    assert isinstance(adapter["tunnel_free_suppresses_terrain"], bool)
    assert isinstance(minco["yaw_tunnel_sidewall_pca_enabled"], bool)
    assert isinstance(minco["yaw_tunnel_dynamic_pre_align_enabled"], bool)
    assert isinstance(minco["yaw_tunnel_terminal_goal_axial_override_enabled"], bool)
    assert isinstance(minco["yaw_tunnel_lock_gimbal"], bool)
    assert isinstance(minco["adaptive_lipschitz_swept_enabled"], bool)
    assert 0.005 <= float(minco["swept_min_corner_step_m"]) <= 0.5 * float(
        minco["footprint_safety_margin"]
    ) + 1e-9
    assert float(minco["spin_resume_after"]) >= 0.30
    assert adapter["use_static_map_obstacles"] is True
    assert isinstance(adapter["use_static_map_geofence_only"], bool)
    assert float(adapter["static_map_geofence_border_m"]) >= 0.0
    assert minco["partial_subgoal_continuation_enabled"] is True
    assert minco["partial_subgoal_only_without_prior"] is True
    assert float(minco["partial_subgoal_min_advance_m"]) >= 0.15
    assert 1 <= int(point_lio["mapping"]["batch_update_size"]) <= 64
    assert point_lio["mapping"]["adaptive_downsample_en"] is True
    assert int(point_lio["mapping"]["max_surf_points"]) >= 200
    assert point_lio["mapping"]["satu_soft_degrade_en"] is True
    assert point_lio["odometry"]["imu_odom_en"] is True
    assert 50.0 <= float(point_lio["odometry"]["imu_odom_rate_hz"]) <= 400.0
    assert isinstance(gicp["local_submap.enable"], bool)
    assert float(gicp["local_submap.radius_m"]) >= 8.0
    assert float(gicp["degeneracy.weak_translation_eigenvalue"]) >= 0.0
    assert float(gicp["degeneracy.max_translation_condition_ratio"]) > 1.0
    assert float(gicp["degeneracy.weak_translation_per_inlier"]) >= 0.0
    assert isinstance(gicp["scan_context.enable"], bool)
    assert int(gicp["scan_context.top_k"]) >= 1
    assert isinstance(loc_fusion["directional_fusion.enable"], bool)
    assert 0.0 <= float(loc_fusion["directional_fusion.weak_axis_damping"]) <= 1.0
    assert float(loc_fusion["directional_fusion.max_strong_nis"]) >= 0.0
    assert loc_fusion["tracking_filter.enable"] is True
    assert float(loc_fusion["tracking_filter.deadband_translation"]) >= 0.0
    assert int(loc_fusion["tracking_filter.motion_jump_confirm_count"]) >= 1
    assert behavior_server["action_name"] == "ats_sentry_behavior"
    assert "planning_grid" not in behavior_server["decision"]["inputs"]
    assert behavior_server["decision"]["inputs"]["planning_snapshot"]["topic"] == (
        "/rog_map_adapter/planning_snapshot"
    )
    assert behavior_server["decision"]["inputs"]["localization"]["topic"] == "/localization"
    assert behavior_server["decision"]["pose"]["expected_frame"] == "map"
    assert behavior_server["decision"]["decision_config"]["ats_action_server"] == "/ats_navigate_to_pose"
    assert behavior_server["use_sim_time"] is False
    assert behavior_client["target_tree"] == "rmuc_2026_mapping"
    assert behavior_client["use_sim_time"] is False

    root_launch = workspace / "src/ats_sentry_bringup/launch/bringup.launch.py"
    root_text = root_launch.read_text(encoding="utf-8")
    for forbidden in (
        "nav2_common",
        "RewrittenYaml",
        "launch_nav2",
        "launch_swerve_mpc",
        "launch_trajectory_optimizer",
        "nav_cmd_vel_topic",
        "planning_grid_owner",
        "map_config_file",
        "behavior_params_file",
    ):
        assert forbidden not in root_text, f"formal root launch retains {forbidden}"
    assert 'executable="ats_rog_map_node"' in root_text
    # The planning-map owner runs inside ats_rog_map_node (M3).
    assert "ats_rog_map_adapter_node" not in root_text
    assert 'package="ats_rog_map_adapter"' not in root_text
    assert "rog_map_params_file" not in root_text
    assert "rog_map_adapter_params_file" not in root_text
    assert "parameters=[params_file" in root_text

    behavior_launch = workspace / "src/ats_sentry_behavior/launch/ats_sentry_behavior_launch.py"
    behavior_launch_defaults = launch_defaults(behavior_launch)
    assert "params_file" in behavior_launch_defaults
    assert behavior_launch_defaults["params_file"] is None, (
        "standalone behavior launch must require an explicit params_file"
    )
    assert not (
        workspace / "src/ats_sentry_behavior/params/sentry_behavior.yaml"
    ).exists()
    behavior_example = workspace / "src/ats_sentry_behavior/params/sentry_behavior.example.yaml"
    assert behavior_example.exists()

    nav_real = (
        workspace
        / "src/ats_sentry_nav/ats_nav_bringup/launch/rm_navigation_reality_launch.py"
    )
    nav_bringup = (
        workspace / "src/ats_sentry_nav/ats_nav_bringup/launch/bringup_launch.py"
    )
    nav_launch = (
        workspace / "src/ats_sentry_nav/ats_nav_bringup/launch/navigation_launch.py"
    )
    assert 'assets_dir = LaunchConfiguration("assets_dir")' in nav_real.read_text(encoding="utf-8")
    for launch_path in (nav_real, nav_bringup, nav_launch):
        launch_text = launch_path.read_text(encoding="utf-8")
        for forbidden in (
            "nav2_common",
            "RewrittenYaml",
            "launch_nav2",
            "launch_swerve_mpc",
            "launch_trajectory_optimizer",
            "nav_cmd_vel_topic",
            "planning_grid_owner",
            "map_server",
            "lifecycle_manager",
            "trajectory_optimizer",
            "ats_nav2_plugins",
        ):
            assert forbidden not in launch_text, f"{launch_path} retains {forbidden}"
        for forbidden_source in (
            "minco_params_file",
            "goal_manager_params_file",
            "mpc_params_file",
            "minco_planner_reality.yaml",
            "ats_goal_manager_reality.yaml",
            "ats_swerve_mpc_reality.yaml",
        ):
            assert forbidden_source not in launch_text, (
                f"{launch_path} retains a second navigation parameter source"
            )
    assert "parameters=[params_file" in nav_launch.read_text(encoding="utf-8")

    for removed_path in (
        workspace / "src/ats_sentry_nav/trajectory_optimizer",
        workspace / "src/ats_sentry_nav/ats_nav2_plugins",
        workspace / "src/ats_sentry_nav/ats_nav_bringup/config/reality/nav2_params.yaml",
        workspace / "src/ats_sentry_nav/ats_nav_bringup/config/simulation/nav2_params.yaml",
    ):
        assert not removed_path.exists(), f"Nav2-only resource still exists: {removed_path}"

    mpc_topic_default = (
        "IfElseSubstitution(launch_fake_vel_transform, '/cmd_vel/autonomy_raw', "
        "IfElseSubstitution(launch_chassis_vel_transform, "
        "'/cmd_vel/autonomy_gimbal', '/cmd_vel/autonomy'))"
    )
    mpc_package_dir = workspace / "src/ats_sentry_nav/ats_swerve_mpc"
    for relative_path in (
        "config/ats_swerve_mpc.yaml",
        "config/ats_swerve_mpc_reality.yaml",
        "include/ats_swerve_mpc/ats_swerve_mpc_node.hpp",
        "include/ats_swerve_mpc/qp/control_cycle_snapshot.hpp",
        "src/main.cpp",
        "src/ats_swerve_mpc_node.cpp",
        "src/se2_mpc_controller.cpp",
    ):
        mpc_source = (mpc_package_dir / relative_path).read_text(encoding="utf-8")
        assert "/cmd_vel_mpc" not in mpc_source
    for config_name in ("ats_swerve_mpc.yaml", "ats_swerve_mpc_reality.yaml"):
        assert parameters(
            load_yaml(mpc_package_dir / "config" / config_name), "ats_swerve_mpc"
        )["command_topic"] == "/cmd_vel/autonomy_raw"
    assert '"/cmd_vel/autonomy_raw"' in (
        mpc_package_dir / "src/ats_swerve_mpc_node.cpp"
    ).read_text(encoding="utf-8")
    chassis_input_default = (
        "IfElseSubstitution(launch_fake_vel_transform, "
        "'/cmd_vel/autonomy_gimbal', mpc_cmd_vel_topic)"
    )
    for launch_path in (root_launch, nav_real, nav_launch):
        assert_default_expression(launch_path, "mpc_cmd_vel_topic", mpc_topic_default)
    for launch_path in (root_launch, nav_real, nav_launch):
        assert_default_expression(launch_path, "chassis_vel_input_topic", chassis_input_default)
    serial_link_default = (
        "IfElseSubstitution(use_robot_state_pub, 'False', 'True')"
    )
    for launch_path in (root_launch, nav_real, nav_bringup, nav_launch):
        assert_default_expression(launch_path, "require_serial_link", serial_link_default)
    assert '"require_serial_link": require_serial_link' in nav_launch.read_text(
        encoding="utf-8"
    )
    nav_launch_text = nav_launch.read_text(encoding="utf-8")
    assert 'executable="cmd_vel_arbiter_node"' in nav_launch_text
    assert 'name="cmd_vel_arbiter"' in nav_launch_text

    mujoco_launch = workspace / "src/sim/ats_mujoco_sim/launch/rmuc_2025_mujoco.launch.py"
    mujoco_launch_text = mujoco_launch.read_text(encoding="utf-8")
    assert '"command_topic": "/cmd_vel/autonomy_raw"' in mujoco_launch_text
    assert '"input_topic": "/cmd_vel/selected"' in mujoco_launch_text
    assert 'executable="cmd_vel_arbiter_node"' in mujoco_launch_text
    assert '"require_serial_link": False' in mujoco_launch_text
    assert '"autonomy_cmd_vel_topic": "/cmd_vel/autonomy_raw"' in mujoco_launch_text
    assert '"manual_cmd_vel_topic": "/cmd_vel"' in mujoco_launch_text

    gazebo_dir = workspace / "src/sim/gazebo_simulator/rmu_gazebo_simulator"
    gazebo_launch_text = (gazebo_dir / "launch/ats_gazebo_nav.launch.py").read_text(
        encoding="utf-8"
    )
    assert '"command_topic": "/cmd_vel/autonomy_raw"' in gazebo_launch_text
    assert 'executable="cmd_vel_arbiter_node"' in gazebo_launch_text
    assert '"manual_cmd_vel_topic": "/cmd_vel"' in gazebo_launch_text
    assert '"autonomy_cmd_vel_topic": "/cmd_vel/autonomy_raw"' in gazebo_launch_text
    assert '"selected_cmd_vel_topic": "/cmd_vel/selected"' in gazebo_launch_text
    assert '"require_serial_link": False' in gazebo_launch_text
    assert '"input_topic": "/cmd_vel/selected"' in gazebo_launch_text
    assert '"lidar_frame": "front_mid360"' in gazebo_launch_text
    assert '"robot_base_frame": "gimbal_yaw_odom"' in gazebo_launch_text
    assert '"/cmd_vel_mpc"' not in gazebo_launch_text
    gazebo_package = (gazebo_dir / "package.xml").read_text(encoding="utf-8")
    assert "<exec_depend>ats_cmd_vel_arbiter</exec_depend>" in gazebo_package
    gazebo_adapter = (gazebo_dir / "scripts/ats_bridge/chassis_cmd_adapter.py").read_text(
        encoding="utf-8"
    )
    assert 'self.declare_parameter("input_topic", "/cmd_vel/selected")' in gazebo_adapter
    assert "from chassis_command_logic import transform_command" in gazebo_adapter
    assert "output = transform_command(" in gazebo_adapter
    assert "/cmd_vel_mpc" not in gazebo_adapter
    recorder_source = (gazebo_dir / "src/ats_navigation_evidence_recorder.cpp").read_text(
        encoding="utf-8"
    )
    for required in (
        '"/cmd_vel/selected"',
        "selected_cmd_vel_nonzero",
        "selected_cmd_vel_publisher_max",
        "selected_cmd_vel_subscriber_max",
        "observeGazeboLidarPublicationSequence",
        "gazebo_lidar_dds_publication_sequence_missing_count",
    ):
        assert required in recorder_source
    assert "/cmd_vel_mpc" not in recorder_source
    cancel_source = (gazebo_dir / "src/ats_navigation_cancel_on_command_client.cpp").read_text(
        encoding="utf-8"
    )
    assert '"/cmd_vel/selected"' in cancel_source
    assert "/cmd_vel_mpc" not in cancel_source
    gazebo_bridge = (gazebo_dir / "config/ros_gz_bridge.yaml").read_text(
        encoding="utf-8"
    )
    assert "/cmd_vel/selected" in gazebo_bridge
    assert "/cmd_vel_mpc" not in gazebo_bridge

    loopback_launch = workspace / "src/sim/loopback_sim/launch/loopback_simulation.launch.py"
    loopback_node = workspace / "src/sim/loopback_sim/nav2_loopback_sim/loopback_simulator.py"
    loopback_launch_text = loopback_launch.read_text(
        encoding="utf-8"
    )
    assert "'command_topic': '/cmd_vel/selected'" in loopback_launch_text
    assert "package='ats_cmd_vel_arbiter'" in loopback_launch_text
    assert "executable='cmd_vel_arbiter_node'" in loopback_launch_text
    assert "'manual_cmd_vel_topic': '/cmd_vel'" in loopback_launch_text
    assert "'autonomy_cmd_vel_topic': '/cmd_vel/autonomy_raw'" in loopback_launch_text
    assert "'selected_cmd_vel_topic': '/cmd_vel/selected'" in loopback_launch_text
    assert "'require_serial_link': False" in loopback_launch_text
    assert "self.declare_parameter('command_topic', '/cmd_vel/selected')" in (
        loopback_node.read_text(encoding="utf-8")
    )
    assert "'enable_stamped_cmd_vel': False" in loopback_launch.read_text(
        encoding="utf-8"
    )
    assert "self.declare_parameter('enable_stamped_cmd_vel', False)" in (
        loopback_node.read_text(encoding="utf-8")
    )
    loopback_setup = workspace / "src/sim/loopback_sim/setup.py"
    assert "os.path.join('share', package_name, 'maps'), glob('maps/*')" in (
        loopback_setup.read_text(encoding="utf-8")
    )
    loopback_package = workspace / "src/sim/loopback_sim/package.xml"
    assert "<exec_depend>ats_cmd_vel_arbiter</exec_depend>" in loopback_package.read_text(
        encoding="utf-8"
    )

    mujoco_twist_bridge = workspace / "src/sim/ats_mujoco_sim/ats_mujoco_sim/twist_to_motion_ctrl.py"
    assert 'self.declare_parameter("input_topic", "/cmd_vel/selected")' in (
        mujoco_twist_bridge.read_text(encoding="utf-8")
    )

    real_robot_navigation = (
        workspace / "src/ats_sentry_bringup/launch/real_robot_navigation.launch.py"
    )
    assert not (
        workspace / "src/ats_sentry_bringup/launch/real_robot_nav2_free.launch.py"
    ).exists()
    real_robot_text = real_robot_navigation.read_text(encoding="utf-8")
    for forbidden in ("nav2", "launch_swerve_mpc", "planning_grid_owner"):
        assert forbidden not in real_robot_text.lower(), (
            f"neutral real-robot entry retains {forbidden}"
        )
    assert '"lidar_mount_mode": lidar_mount_mode' in real_robot_text
    assert '"gimbal_use_base_footprint_odom": gimbal_use_base_footprint_odom' in real_robot_text
    assert '"launch_fake_vel_transform": launch_fake_vel_transform' in real_robot_text
    assert (
        '"launch_chassis_vel_transform": launch_chassis_vel_transform' in real_robot_text
    )
    assert '"require_serial_link": "True"' in real_robot_text
    real_robot_defaults = launch_defaults(real_robot_navigation)
    assert isinstance(real_robot_defaults["lidar_mount_mode"], ast.Constant)
    assert real_robot_defaults["lidar_mount_mode"].value == "gimbal"
    assert isinstance(real_robot_defaults["gimbal_use_base_footprint_odom"], ast.Constant)
    assert real_robot_defaults["gimbal_use_base_footprint_odom"].value == "True"
    assert isinstance(real_robot_defaults["launch_fake_vel_transform"], ast.Constant)
    assert real_robot_defaults["launch_fake_vel_transform"].value == "False"
    chassis_vel_mode_default = (
        "PythonExpression([\"'True' if '\", lidar_mount_mode, "
        "\"'.lower() == 'gimbal' else 'False'\"])"
    )
    for launch_path in (real_robot_navigation, root_launch, nav_real, nav_bringup, nav_launch):
        assert_default_expression(
            launch_path, "launch_chassis_vel_transform", chassis_vel_mode_default
        )
    root_defaults = launch_defaults(root_launch)
    assert isinstance(root_defaults["lidar_mount_mode"], ast.Constant)
    assert root_defaults["lidar_mount_mode"].value == "chassis"
    assert isinstance(root_defaults["gimbal_use_base_footprint_odom"], ast.Constant)
    assert root_defaults["gimbal_use_base_footprint_odom"].value == "True"
    assert isinstance(root_defaults["launch_fake_vel_transform"], ast.Constant)
    assert root_defaults["launch_fake_vel_transform"].value == "False"
    # Gimbal status timing hierarchy: T_pub < T_joint <= T_tf < T_lease.
    gimbal_pub_sec = 1.0 / float(gimbal_yaw_status_bridge["publish_rate_hz"])
    gimbal_joint_sec = float(gimbal_yaw_status_bridge["joint_state_timeout_ms"]) / 1000.0
    gimbal_tf_sec = float(gimbal_yaw_status_bridge["tf_timeout_ms"]) / 1000.0
    assert gimbal_pub_sec < gimbal_joint_sec <= gimbal_tf_sec < float(
        goal_manager["gimbal_status_timeout_sec"]
    )
    assert gimbal_tf_sec < float(mpc["gimbal_status_timeout"])

    behavior_text = behavior_example.read_text(encoding="utf-8")
    assert "nav2_action_server" not in behavior_text
    assert "nav2_to_pose_action_server" not in behavior_text
    assert 'ats_action_server: "/ats_navigate_to_pose"' in behavior_text

    rog_source = workspace / "src/ats_sentry_nav/ats_rog_map/src/ats_rog_map_node.cpp"
    rog_text = rog_source.read_text(encoding="utf-8")
    assert "map_config_file" not in rog_text
    assert "const auto core_parameters = declareCoreParameters(*this);" in rog_text
    assert "makeRogMapConfig(core_parameters)" in rog_text
    assert "intensity_required_ = core_parameters.intensity_threshold > 0;" in rog_text
    assert "decodePointCloudForRogMap(*msg, intensity_required_)" in rog_text
    assert "No map update is committed; stale-map safety applies." in rog_text
    for required in (
        '"rog_map/occ"',
        '"rog_map/inf_occ"',
        '"rog_map/unk"',
        '"rog_map/esdf"',
        '"debug_viz_topic"',
        "makeVoxelDebugCloud",
        "collectVoxelDebugInBox",
        "debug_bounds_topic_",
        "create_publisher<visualization_msgs::msg::MarkerArray>",
        "makeBoundsMarkers",
        "DebugSnapshot",
        "viz_serialize_ms",
        "map_lock_hold_ms",
    ):
        assert required in rog_text, f"ROGMap visualization producer missing {required}"

    minco_source = (
        workspace / "src/ats_sentry_nav/minco_planner/src/nodes/minco_planner_node.cpp"
    ).read_text(encoding="utf-8")
    assert "raw_path_pub_ = create_publisher<nav_msgs::msg::Path>(raw_path_topic_" in minco_source
    assert "candidate_reference_path_pub_ = create_publisher<nav_msgs::msg::Path>(" in minco_source
    assert "if (planner_manages_emergency_stop_)" in minco_source

    goal_manager_source = (
        workspace / "src/ats_sentry_nav/ats_goal_manager/src/ats_goal_manager_node.cpp"
    ).read_text(encoding="utf-8")
    assert "reference_path_pub_ = create_publisher<nav_msgs::msg::Path>(" in goal_manager_source
    assert "reference_path_pub_->publish(committed);" in goal_manager_source

    mpc_source = (
        workspace / "src/ats_sentry_nav/ats_swerve_mpc/src/ats_swerve_mpc_node.cpp"
    ).read_text(encoding="utf-8")
    for required in (
        'create_publisher<nav_msgs::msg::Path>("~/predicted_path", rclcpp::QoS(1))',
        '"~/reference_horizon", rclcpp::QoS(1)',
    ):
        assert required in mpc_source, f"MPC visualization producer missing {required}"

    rviz = load_yaml(workspace / "src/ats_sentry_bringup/rviz/sentry_default_view.rviz")
    assert_navigation_rviz_contract(rviz, rog_map["map_frame"], "default RViz")
    assert_esdf_cloud_display(rviz, "default RViz")
    assert_no_removed_rc_esdf_topics(rviz, "default RViz")
    assert_robot_model_display(rviz, "robot_description", "default RViz")
    assert_rviz_groups_and_clearance_contours(rviz, "default RViz")
    rviz_text = (
        workspace / "src/ats_sentry_bringup/rviz/sentry_default_view.rviz"
    ).read_text(encoding="utf-8")
    assert "nav2_rviz_plugins" not in rviz_text
    assert "rviz_default_plugins/SetGoal" in rviz_text
    assert "Value: /goal_pose" in rviz_text
    for forbidden in ("\n        Value: /plan\n", "transformed_global_plan", "GoalTool"):
        assert forbidden not in rviz_text, f"RViz retains Nav2 display/tool: {forbidden}"

    mujoco_rviz_path = workspace / "src/sim/ats_mujoco_sim/rviz/mujoco_navigation.rviz"
    mujoco_rviz = load_yaml(mujoco_rviz_path)
    assert PLANNING_GRID_VIZ_TOPIC in set(collect_topic_values(mujoco_rviz)), (
        f"MuJoCo RViz config missing {PLANNING_GRID_VIZ_TOPIC}"
    )
    assert_no_removed_rc_esdf_topics(mujoco_rviz, "MuJoCo RViz")
    assert_navigation_rviz_contract(
        mujoco_rviz, rog_map["map_frame"], "MuJoCo RViz", MUJOCO_PATH_DISPLAY_STYLE,
        allow_rog_map_voxel_topics=True,
        allow_disabled_esdf_guide=True,
    )
    rolling_anchor = single_display_for_topic(mujoco_rviz, "/minco/rolling_anchor", "MuJoCo RViz")
    assert rolling_anchor["Class"] == "rviz_default_plugins/MarkerArray"
    assert rolling_anchor["Enabled"] is True
    assert_rog_map_native_displays(mujoco_rviz, "MuJoCo RViz")
    # The vehicle shape comes from robot_state_publisher (ats_mujoco_sim/robot_description.py).
    assert_robot_model_display(mujoco_rviz, "robot_description", "MuJoCo RViz")
    assert_esdf_cloud_display(mujoco_rviz, "MuJoCo RViz")
    assert_rviz_groups_and_clearance_contours(mujoco_rviz, "MuJoCo RViz")
    assert_mujoco_sensor_view(mujoco_rviz)
    mujoco_rviz_text = mujoco_rviz_path.read_text(encoding="utf-8")
    localization_displays = displays_for_topic(mujoco_rviz, "/localization")
    assert len(localization_displays) == 1, "MuJoCo RViz must have one localization display"
    assert all(
        display["Class"] == "rviz_default_plugins/Odometry"
        and display["Topic"]["Reliability Policy"] == "Best Effort"
        for display in localization_displays
    )
    assert "Value: /goal_pose" in mujoco_rviz_text, "MuJoCo RViz must publish /goal_pose"
    for forbidden in ("\n        Value: /plan\n", "transformed_global_plan", "GoalTool", "nav2_rviz_plugins"):
        assert forbidden not in mujoco_rviz_text, f"MuJoCo RViz retains Nav2 display/tool: {forbidden}"

    gazebo_rviz_path = workspace / "src/sim/gazebo_simulator/rmu_gazebo_simulator/rviz/ats_gazebo_nav.rviz"
    gazebo_rviz = load_yaml(gazebo_rviz_path)
    # Gazebo retains its legacy palette contract while MuJoCo uses the upgraded high-contrast palette.
    gazebo_path_style = {
        **GAZEBO_PATH_DISPLAY_STYLE,
        "/ats_swerve_mpc/predicted_path": {
            **GAZEBO_PATH_DISPLAY_STYLE["/ats_swerve_mpc/predicted_path"],
            "name": "MPC Prediction / iLQR Follow Rollout",
        },
    }
    assert_navigation_rviz_contract(
        gazebo_rviz, rog_map["map_frame"], "Gazebo RViz", gazebo_path_style
    )
    # The Gazebo repository is not migrated to the M4 topics yet (follow-up);
    # its ESDF displays are not checked here.
    localization_displays = displays_for_topic(gazebo_rviz, "/localization")
    assert len(localization_displays) == 1, "Gazebo RViz must have one localization display"
    assert localization_displays[0]["Class"] == "rviz_default_plugins/Odometry"
    assert localization_displays[0]["Topic"]["Reliability Policy"] == "Best Effort"
    # spawn_robots.launch.py namespaces robot_state_publisher under the robot.
    assert_robot_model_display(
        gazebo_rviz, "/red_standard_robot1/robot_description", "Gazebo RViz"
    )

    print("PASS: formal single-source behavior, navigation configuration, and ROGMap visualization contract")


if __name__ == "__main__":
    try:
        main()
    except (AssertionError, OSError, yaml.YAMLError) as error:
        print(f"FAIL: {error}", file=sys.stderr)
        sys.exit(1)
