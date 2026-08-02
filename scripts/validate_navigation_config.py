#!/usr/bin/env python3
"""Validate the formal Nav2-free parameter and visualization contract."""

import argparse
import ast
import sys
from pathlib import Path

import yaml


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
    serial = parameters(document, "standard_robot_pp_ros2")

    assert rog_map["map_frame"] == "odom"
    assert rog_map["debug_bounds_topic"] == "/rog_map/bounds"
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
    assert minco["goal_topic"] == ""
    assert "global_plan_topic" not in minco
    assert minco["goal_request_topic"] == "/ats_goal_manager/planner_goal"
    assert minco["map_ready_topic"] == "/rog_map_adapter/ready"
    assert minco["unknown_is_obstacle"] is True
    assert minco["publish_unsafe_trajectory"] is False
    assert goal_manager["action_name"] == "/ats_navigate_to_pose"
    assert goal_manager["planner_goal_topic"] == minco["goal_request_topic"]
    fake_transform = parameters(document, "fake_vel_transform")
    chassis_transform = parameters(document, "chassis_vel_transform")
    assert mpc["command_topic"] == "/cmd_vel_mpc"
    assert fake_transform["input_cmd_vel_topic"] == mpc["command_topic"]
    assert fake_transform["output_cmd_vel_topic"] == "cmd_vel_gimbal_yaw_odom"
    assert (
        chassis_transform["input_cmd_vel_topic"]
        == fake_transform["output_cmd_vel_topic"]
    )
    assert chassis_transform["output_cmd_vel_topic"] == "/cmd_vel"
    assert mpc["max_vy"] > 0.0
    assert abs(mpc["dt"] - 1.0 / mpc["control_rate_hz"]) < 1e-9
    assert serial["execution_command_timeout"] == mpc["execution_command_timeout"]
    assert (
        serial["cmd_vel_watchdog_timeout_ms"] / 1000.0
        <= mpc["execution_command_timeout"]
    )

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
    ):
        assert forbidden not in root_text, f"formal root launch retains {forbidden}"
    assert 'executable="ats_rog_map_node"' in root_text
    assert 'executable="ats_rog_map_adapter_node"' in root_text
    assert "rog_map_params_file" not in root_text
    assert "rog_map_adapter_params_file" not in root_text
    assert "parameters=[params_file" in root_text

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
        "IfElseSubstitution(launch_fake_vel_transform, '/cmd_vel_mpc', "
        "IfElseSubstitution(launch_chassis_vel_transform, "
        "'cmd_vel_gimbal_yaw_odom', '/cmd_vel'))"
    )
    chassis_input_default = (
        "IfElseSubstitution(launch_fake_vel_transform, "
        "'cmd_vel_gimbal_yaw_odom', mpc_cmd_vel_topic)"
    )
    for launch_path in (root_launch, nav_real, nav_launch):
        assert_default_expression(launch_path, "mpc_cmd_vel_topic", mpc_topic_default)
    for launch_path in (root_launch, nav_real, nav_launch):
        assert_default_expression(launch_path, "chassis_vel_input_topic", chassis_input_default)

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
    real_robot_defaults = launch_defaults(real_robot_navigation)
    assert isinstance(real_robot_defaults["launch_fake_vel_transform"], ast.Constant)
    assert real_robot_defaults["launch_fake_vel_transform"].value == "True"
    assert isinstance(real_robot_defaults["launch_chassis_vel_transform"], ast.Constant)
    assert real_robot_defaults["launch_chassis_vel_transform"].value == "True"

    behavior_path = workspace / "src/ats_sentry_behavior/params/sentry_behavior.yaml"
    behavior_text = behavior_path.read_text(encoding="utf-8")
    assert "nav2_action_server" not in behavior_text
    assert "nav2_to_pose_action_server" not in behavior_text
    assert 'ats_action_server: "/ats_navigate_to_pose"' in behavior_text

    rog_source = workspace / "src/ats_sentry_nav/ats_rog_map/src/ats_rog_map_node.cpp"
    rog_text = rog_source.read_text(encoding="utf-8")
    assert "map_config_file" not in rog_text
    assert "makeRogMapConfig(declareCoreParameters(*this))" in rog_text
    for required in (
        '"rog_map/occ"',
        '"rog_map/inf_occ"',
        '"rog_map/unk"',
        '"rog_map/esdf"',
        "publishBoundsMarker",
        "debug_bounds_topic_",
    ):
        assert required in rog_text, f"ROGMap visualization producer missing {required}"

    rviz = load_yaml(workspace / "src/ats_sentry_bringup/rviz/sentry_default_view.rviz")
    rviz_topics = set(collect_topic_values(rviz))
    for topic in (
        "/rog_map/occ",
        "/rog_map/inf_occ",
        "/rog_map/unk",
        "/rog_map/esdf",
        "/rog_map/bounds",
    ):
        assert topic in rviz_topics, f"RViz config missing {topic}"
    manager = rviz["Visualization Manager"]
    assert manager["Global Options"]["Fixed Frame"] in ("map", "odom")
    rviz_text = (
        workspace / "src/ats_sentry_bringup/rviz/sentry_default_view.rviz"
    ).read_text(encoding="utf-8")
    assert "nav2_rviz_plugins" not in rviz_text
    assert "rviz_default_plugins/SetGoal" in rviz_text
    assert "Value: /goal_pose" in rviz_text
    for required in (
        "Name: ROGMap Occupied",
        "Name: ROGMap Inflated",
        "Name: ROGMap Unknown",
        "Name: ROGMap ESDF Debug",
        "Name: ROGMap Local Bounds",
        "Reliability Policy: Best Effort",
        "Style: Boxes",
        "Color Transformer: Intensity",
        "Name: Planning Grid",
        "Name: MINCO Raw Path",
        "Name: MINCO Reference",
        "Name: MPC Reference Horizon",
        "Name: MPC Predicted Path",
    ):
        assert required in rviz_text, f"RViz ROGMap display contract missing {required}"
    for forbidden in ("\n        Value: /plan\n", "costmap", "transformed_global_plan", "GoalTool"):
        assert forbidden not in rviz_text, f"RViz retains Nav2 display/tool: {forbidden}"

    print("PASS: formal Nav2-free configuration and ROGMap visualization contract")


if __name__ == "__main__":
    try:
        main()
    except (AssertionError, OSError, yaml.YAMLError) as error:
        print(f"FAIL: {error}", file=sys.stderr)
        sys.exit(1)
