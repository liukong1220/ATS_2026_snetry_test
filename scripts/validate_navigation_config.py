#!/usr/bin/env python3
"""Validate the formal navigation parameter and visualization contract."""

import argparse
import ast
import sys
from pathlib import Path

import yaml


ROG_MAP_BOUNDS_DISPLAY_NAME = "ROGMap Bounds: Orange Local / Green Update"
GLOBAL_FUSED_ESDF_DISPLAY_NAME = "Global Fused RC-ESDF (ROGMap + Static + Terrain)"
GLOBAL_FUSED_ESDF_TOPIC = "/rc_esdf/signed_distance_grid"
# Voxel point clouds stay available for ros2 topic echo / ad-hoc RViz, but the
# shipped navigation views draw the local ROGMap as its range boxes only.
ROG_MAP_VOXEL_TOPICS = (
    "/rog_map/occ",
    "/rog_map/inf_occ",
    "/rog_map/unk",
    "/rog_map/esdf",
    "/rog_map/viz",
)
NAVIGATION_PATH_DISPLAY_CONTRACTS = {
    "/minco/raw_path": {
        "name": "Global Planning / JPS Search Path",
        "color": "0; 220; 255",
        "line_style": "Lines",
        "line_width": 0.025,
        "offset_z": 0.02,
    },
    "/minco/reference_path": {
        "name": "Local Control / MINCO Timed Reference",
        "color": "50; 255; 80",
        "line_style": "Lines",
        "line_width": 0.05,
        "offset_z": 0.04,
    },
    "/ats_swerve_mpc/reference_horizon": {
        "name": "MPC Follow / Active Reference Horizon",
        "color": "255; 196; 0",
        "line_style": "Lines",
        "line_width": 0.03,
        "offset_z": 0.08,
    },
    "/ats_swerve_mpc/predicted_path": {
        "name": "MPC Prediction / iLQR Follow Rollout",
        "color": "255; 80; 255",
        "line_style": "Billboards",
        "line_width": 0.075,
        "offset_z": 0.12,
    },
}
# MuJoCo 视图用白底 ESDF 配色(品红障碍/橙带/青绿净空),默认深色视图里的青色 JPS、
# 亮绿参考在白底和青绿净空上看不清,这里只换颜色与线宽,话题、名称、分层 Z 偏移不变。
MUJOCO_PATH_DISPLAY_STYLE = {
    "/minco/raw_path": {"color": "0; 160; 0", "line_style": "Billboards", "line_width": 0.05},
    "/minco/reference_path": {"color": "230; 20; 30", "line_style": "Billboards", "line_width": 0.1},
    "/ats_swerve_mpc/reference_horizon": {"color": "255; 140; 0", "line_style": "Lines", "line_width": 0.03},
    "/ats_swerve_mpc/predicted_path": {"color": "30; 90; 255", "line_style": "Billboards", "line_width": 0.05},
}
NAVIGATION_PATH_DISPLAY_NAMES = tuple(
    contract["name"] for contract in NAVIGATION_PATH_DISPLAY_CONTRACTS.values()
)
MINCO_GUIDE_TOPICS = (
    "/minco/preprocessed_guide",
    "/minco/esdf_refined_guide",
)


class DuplicateKeyLoader(yaml.SafeLoader):
    """Reject duplicate mapping keys instead of silently keeping the last value."""


def _construct_mapping(loader, node, deep=False):
    mapping = {}
    for key_node, value_node in node.value:
        key = loader.construct_object(key_node, deep=deep)
        if key in mapping:
            raise yaml.constructor.ConstructorError(
                "while constructing a mapping",
                node.start_mark,
                f"duplicate key: {key!r}",
                key_node.start_mark,
            )
        mapping[key] = loader.construct_object(value_node, deep=deep)
    return mapping


DuplicateKeyLoader.add_constructor(
    yaml.resolver.BaseResolver.DEFAULT_MAPPING_TAG, _construct_mapping
)


def load_yaml(path: Path):
    with path.open(encoding="utf-8") as stream:
        value = yaml.load(stream, Loader=DuplicateKeyLoader)
    if not isinstance(value, dict):
        raise AssertionError(f"{path} must contain a top-level mapping")
    return value


def parameters(document, node_name: str):
    node = document.get(node_name)
    if not isinstance(node, dict) or not isinstance(node.get("ros__parameters"), dict):
        raise AssertionError(f"missing {node_name}.ros__parameters")
    return node["ros__parameters"]


def assert_planning_snapshot_lease_contract(adapter, goal_manager):
    projection_rate_hz = float(adapter["projection_rate_hz"])
    snapshot_timeout_sec = float(goal_manager["planning_snapshot_timeout_sec"])
    map_ready_timeout_sec = float(goal_manager["map_ready_timeout_sec"])
    if projection_rate_hz <= 0.0:
        raise AssertionError("adapter projection_rate_hz must be positive")
    publication_period_sec = 1.0 / projection_rate_hz
    if snapshot_timeout_sec < publication_period_sec:
        raise AssertionError(
            "planning snapshot lease expires before the adapter publication period: "
            f"{snapshot_timeout_sec:.3f} < {publication_period_sec:.3f} s"
        )
    if snapshot_timeout_sec != map_ready_timeout_sec:
        raise AssertionError(
            "planning_snapshot_timeout_sec must match map_ready_timeout_sec so "
            "Goal Manager uses one adapter heartbeat lease"
        )


def launch_defaults(path: Path):
    tree = ast.parse(path.read_text(encoding="utf-8"), filename=str(path))
    result = {}
    for node in ast.walk(tree):
        if not isinstance(node, ast.Call):
            continue
        if (
            not isinstance(node.func, ast.Name)
            or node.func.id != "DeclareLaunchArgument"
        ):
            continue
        if not node.args or not isinstance(node.args[0], ast.Constant):
            continue
        default = next(
            (
                keyword.value
                for keyword in node.keywords
                if keyword.arg == "default_value"
            ),
            None,
        )
        result[node.args[0].value] = default
    return result


def assert_default_expression(path: Path, argument_name: str, expected: str):
    default = launch_defaults(path).get(argument_name)
    if default is None:
        raise AssertionError(f"{path}: missing launch argument {argument_name}")
    actual = ast.unparse(default)
    if actual != expected:
        raise AssertionError(
            f"{path}: {argument_name} default is {actual!r}, expected {expected!r}"
        )


def collect_topic_values(value):
    topics = []
    if isinstance(value, dict):
        topic = value.get("Topic")
        if isinstance(topic, dict) and isinstance(topic.get("Value"), str):
            topics.append(topic["Value"])
        for child in value.values():
            topics.extend(collect_topic_values(child))
    elif isinstance(value, list):
        for child in value:
            topics.extend(collect_topic_values(child))
    return topics


def named_display(value, name: str):
    if isinstance(value, dict):
        if value.get("Name") == name:
            return value
        for child in value.values():
            result = named_display(child, name)
            if result is not None:
                return result
    elif isinstance(value, list):
        for child in value:
            result = named_display(child, name)
            if result is not None:
                return result
    return None


def displays_for_topic(value, topic: str):
    displays = []
    if isinstance(value, dict):
        display_topic = value.get("Topic")
        if isinstance(display_topic, dict) and display_topic.get("Value") == topic:
            displays.append(value)
        for child in value.values():
            displays.extend(displays_for_topic(child, topic))
    elif isinstance(value, list):
        for child in value:
            displays.extend(displays_for_topic(child, topic))
    return displays


def single_display_for_topic(document, topic: str, context: str):
    displays = displays_for_topic(document, topic)
    assert len(displays) == 1, f"{context} must have one {topic} display"
    return displays[0]


def assert_navigation_rviz_contract(
    document, fixed_frame: str, context: str, path_style_overrides=None
):
    manager = document["Visualization Manager"]
    assert manager["Global Options"]["Fixed Frame"] == fixed_frame, (
        f"{context} fixed frame must match ROGMap frame {fixed_frame!r}"
    )

    for topic in (*ROG_MAP_VOXEL_TOPICS, *MINCO_GUIDE_TOPICS):
        assert not displays_for_topic(document, topic), (
            f"{context} must not display {topic}; the navigation view is minimal"
        )

    bounds = single_display_for_topic(document, "/rog_map/bounds", context)
    assert bounds["Class"] == "rviz_default_plugins/MarkerArray", (
        f"{context} /rog_map/bounds must be a MarkerArray display"
    )
    assert bounds["Name"] == ROG_MAP_BOUNDS_DISPLAY_NAME, (
        f"{context} /rog_map/bounds must describe the local/update ROGMap bounds"
    )
    assert bounds["Topic"]["Reliability Policy"] == "Best Effort", (
        f"{context} /rog_map/bounds must use Best Effort"
    )

    for topic, base_contract in NAVIGATION_PATH_DISPLAY_CONTRACTS.items():
        contract = {**base_contract, **(path_style_overrides or {}).get(topic, {})}
        display = single_display_for_topic(document, topic, context)
        assert display["Class"] == "rviz_default_plugins/Path", (
            f"{context} {topic} must be a Path display"
        )
        assert display["Name"] == contract["name"], (
            f"{context} {topic} display name must be {contract['name']!r}"
        )
        assert display["Topic"]["Reliability Policy"] == "Reliable", (
            f"{context} {topic} must use Reliable"
        )
        assert display["Color"] == contract["color"], (
            f"{context} {topic} color must be {contract['color']!r}"
        )
        assert display["Line Style"] == contract["line_style"], (
            f"{context} {topic} line style must be {contract['line_style']!r}"
        )
        assert display["Line Width"] == contract["line_width"], (
            f"{context} {topic} line width must be {contract['line_width']}"
        )
        assert display["Offset"]["Z"] == contract["offset_z"], (
            f"{context} {topic} Z offset must be {contract['offset_z']}"
        )
        assert display["Enabled"] is True, f"{context} {topic} must be enabled"


def assert_global_fused_esdf_display(document, context: str, expect_enabled: bool = True):
    """Lock the display-only global RC-ESDF layer without changing ESDF ownership."""
    display = single_display_for_topic(document, GLOBAL_FUSED_ESDF_TOPIC, context)
    assert display["Class"] == "rviz_default_plugins/Map", (
        f"{context} global fused ESDF must be a Map display"
    )
    assert display["Name"] == GLOBAL_FUSED_ESDF_DISPLAY_NAME, (
        f"{context} global fused ESDF display name must be {GLOBAL_FUSED_ESDF_DISPLAY_NAME!r}"
    )
    assert display["Topic"]["Reliability Policy"] == "Reliable", (
        f"{context} global fused ESDF must use Reliable"
    )
    assert display["Topic"]["Durability Policy"] == "Transient Local", (
        f"{context} global fused ESDF must use Transient Local"
    )
    assert display["Topic"]["History Policy"] == "Keep Last", (
        f"{context} global fused ESDF must keep only the latest grid"
    )
    assert display["Topic"]["Depth"] == 1, (
        f"{context} global fused ESDF depth must be one"
    )
    assert display["Color Scheme"] == "costmap", (
        f"{context} global fused ESDF must use costmap colors"
    )
    assert display["Alpha"] >= 0.55 and display["Alpha"] <= 0.70, (
        f"{context} global fused ESDF alpha must stay in [0.55, 0.70]"
    )
    assert display["Draw Behind"] is True, (
        f"{context} global fused ESDF must draw behind local diagnostics"
    )
    assert display["Enabled"] is expect_enabled, (
        f"{context} global fused ESDF must be {'enabled' if expect_enabled else 'disabled'}"
    )


def assert_robot_model_display(document, description_topic: str, context: str):
    display = named_display(document, "Robot Model")
    assert display is not None and display["Class"] == "rviz_default_plugins/RobotModel", (
        f"{context} must show the robot model"
    )
    assert display["Description Topic"]["Value"] == description_topic, (
        f"{context} robot model must read {description_topic}"
    )
    assert display["Description Topic"]["Durability Policy"] == "Transient Local", (
        f"{context} robot model description must be Transient Local"
    )
    assert display["TF Prefix"] == "", f"{context} robot model must use the global TF tree"


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
    for node_name in navigation_root_nodes:
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
    assert adapter["projection_service"] == "/rog_map/get_ground_projection"
    assert adapter["planning_grid_topic"] == "/rc_esdf/planning_grid"
    assert adapter["unknown_is_obstacle"] is True
    assert adapter["require_localization_status"] is True
    assert_planning_snapshot_lease_contract(adapter, goal_manager)
    assert minco["goal_topic"] == ""
    assert "global_plan_topic" not in minco
    assert minco["goal_request_topic"] == "/ats_goal_manager/planner_goal"
    assert minco["map_ready_topic"] == "/rog_map_adapter/ready"
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
    assert abs(mpc["dt"] - 1.0 / mpc["control_rate_hz"]) < 1e-9
    assert behavior_server["action_name"] == "ats_sentry_behavior"
    assert behavior_server["decision"]["inputs"]["planning_grid"]["topic"] == "/rc_esdf/planning_grid"
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
    assert 'executable="ats_rog_map_adapter_node"' in root_text
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
    gazebo_runner = workspace / "scripts/test_gazebo_minco_mpc_chain.sh"
    gazebo_runner_text = gazebo_runner.read_text(encoding="utf-8")
    assert "/cmd_vel_mpc" not in gazebo_runner_text
    assert "selected_cmd_vel_publisher_max" in gazebo_runner_text
    assert "selected_cmd_vel_subscriber_max" in gazebo_runner_text
    # Gazebo launches executables from the overlay. A source tree newer than
    # that executable invalidates the runtime evidence, so the regression
    # runner must reject it before starting a nominal P1 run.
    for required in (
        "runtime_binary_is_fresh",
        "runtime_stale_critical_binary",
        "ats_cmd_vel_arbiter",
        "ats_swerve_mpc",
        "sensor_scan_generation",
        "small_gicp_relocalization",
        "rmu_gazebo_simulator",
    ):
        assert required in gazebo_runner_text

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
    assert '"launch_fake_vel_transform": launch_fake_vel_transform' in real_robot_text
    assert (
        '"launch_chassis_vel_transform": launch_chassis_vel_transform' in real_robot_text
    )
    assert '"require_serial_link": "True"' in real_robot_text
    real_robot_defaults = launch_defaults(real_robot_navigation)
    assert isinstance(real_robot_defaults["launch_fake_vel_transform"], ast.Constant)
    assert real_robot_defaults["launch_fake_vel_transform"].value == "True"
    assert isinstance(real_robot_defaults["launch_chassis_vel_transform"], ast.Constant)
    assert real_robot_defaults["launch_chassis_vel_transform"].value == "True"

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
    assert_global_fused_esdf_display(rviz, "default RViz")
    assert_robot_model_display(rviz, "robot_description", "default RViz")
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
    assert "/rc_esdf/planning_grid" in set(collect_topic_values(mujoco_rviz)), (
        "MuJoCo RViz config missing /rc_esdf/planning_grid"
    )
    assert_navigation_rviz_contract(
        mujoco_rviz, rog_map["map_frame"], "MuJoCo RViz", MUJOCO_PATH_DISPLAY_STYLE
    )
    # MuJoCo 视图用彩色 ESDF 点云代替 costmap 栅格(两者叠加会互相遮挡),栅格保留但默认关闭。
    assert_global_fused_esdf_display(mujoco_rviz, "MuJoCo RViz", expect_enabled=False)
    esdf_cloud = single_display_for_topic(mujoco_rviz, "/rc_esdf/esdf_cloud", "MuJoCo RViz")
    assert esdf_cloud["Class"] == "rviz_default_plugins/PointCloud2", (
        "MuJoCo RViz /rc_esdf/esdf_cloud must be a PointCloud2 display"
    )
    assert esdf_cloud["Enabled"] is True, "MuJoCo RViz ESDF cloud must be enabled"
    mujoco_rviz_text = mujoco_rviz_path.read_text(encoding="utf-8")
    # MuJoCo publishes no robot_description; the /localization pose stands in
    # for the robot model.
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
    assert_navigation_rviz_contract(gazebo_rviz, rog_map["map_frame"], "Gazebo RViz")
    assert_global_fused_esdf_display(gazebo_rviz, "Gazebo RViz")
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
