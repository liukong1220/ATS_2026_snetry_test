#!/usr/bin/env python3
"""Validate the formal navigation parameter and visualization contract."""

import argparse
import ast
import math
import sys
from pathlib import Path

import yaml


ROG_MAP_BOUNDS_DISPLAY_NAME = "ROGMap Bounds: Orange Local / Green Update"
# M4: the planning snapshot is the only planning-map contract; RViz draws
# subscriber-gated views of it. /rc_esdf/* topics no longer exist.
ESDF_CLOUD_VIZ_TOPIC = "/rog_map_adapter/viz/esdf_cloud"
PLANNING_GRID_VIZ_TOPIC = "/rog_map_adapter/viz/planning_grid"
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
        "name": "MPC Prediction / OSQP SQP-RTI Rollout",
        "color": "255; 80; 255",
        "line_style": "Billboards",
        "line_width": 0.075,
        "offset_z": 0.12,
    },
}
# MuJoCo 视图采用高对比分层配色（避开彩虹 ESDF 背景撞色并防止局部线遮挡全局线）：
# 全长全局路径为品红粗底线（Z=0.02），局部滚动 MINCO 为亮金黄叠加线（Z=0.06），
# MPC 参考窗为青色，OSQP 预测轨迹为纯白；并配套 /minco/rolling_anchor 与默认折叠的引导调试项。
MUJOCO_PATH_DISPLAY_STYLE = {
    "/minco/raw_path": {"color": "255; 0; 220", "line_style": "Billboards", "line_width": 0.09, "offset_z": 0.02},
    "/minco/reference_path": {"color": "255; 215; 0", "line_style": "Billboards", "line_width": 0.055, "offset_z": 0.06},
    "/ats_swerve_mpc/reference_horizon": {"color": "0; 245; 255", "line_style": "Lines", "line_width": 0.03, "offset_z": 0.08},
    "/ats_swerve_mpc/predicted_path": {"color": "255; 255; 255", "line_style": "Billboards", "line_width": 0.045, "offset_z": 0.12},
}
# 冻结的 Gazebo 旧兼容视图保留原白底配色契约。
GAZEBO_PATH_DISPLAY_STYLE = {
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


RVIZ_DISPLAY_GROUPS = ("定位", "点云", "局部规划窗口", "距离场", "路径", "安全")
CLEARANCE_CONTOURS_TOPIC = "/rog_map_adapter/viz/clearance_contours"


def assert_rviz_groups_and_clearance_contours(document, context: str):
    top_level = [
        display.get("Name")
        for display in document["Visualization Manager"]["Displays"]
        if display.get("Class") == "rviz_common/Group"
    ]
    for group in RVIZ_DISPLAY_GROUPS:
        assert group in top_level, f"{context} missing top-level display group {group!r}"
    assert [name for name in top_level if name in RVIZ_DISPLAY_GROUPS] == list(RVIZ_DISPLAY_GROUPS), (
        f"{context} display groups must be ordered {RVIZ_DISPLAY_GROUPS}"
    )
    contours = displays_for_topic(document, CLEARANCE_CONTOURS_TOPIC)
    assert len(contours) == 1, f"{context} must have exactly one {CLEARANCE_CONTOURS_TOPIC} display"
    assert contours[0]["Class"] == "rviz_default_plugins/MarkerArray", (
        f"{context} clearance contours must be a MarkerArray display"
    )


def assert_navigation_rviz_contract(
    document, fixed_frame: str, context: str, path_style_overrides=None,
    allow_rog_map_voxel_topics: bool = False,
    allow_disabled_esdf_guide: bool = False,
):
    manager = document["Visualization Manager"]
    assert manager["Global Options"]["Fixed Frame"] == fixed_frame, (
        f"{context} fixed frame must match ROGMap frame {fixed_frame!r}"
    )

    # The MuJoCo view is a simulation tool and may draw ROG-Map's native voxel clouds
    # (assert_rog_map_native_displays pins how); the shipped views stay minimal.
    hidden = (() if allow_rog_map_voxel_topics else ROG_MAP_VOXEL_TOPICS) + tuple(
        topic for topic in MINCO_GUIDE_TOPICS
        if not (allow_disabled_esdf_guide and topic == "/minco/esdf_refined_guide")
    )
    for topic in hidden:
        assert not displays_for_topic(document, topic), (
            f"{context} must not display {topic}; the navigation view is minimal"
        )
    if allow_disabled_esdf_guide:
        esdf_guides = displays_for_topic(document, "/minco/esdf_refined_guide")
        assert len(esdf_guides) == 1 and esdf_guides[0]["Enabled"] is False, (
            f"{context} /minco/esdf_refined_guide debug display must be disabled by default"
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


def assert_esdf_cloud_display(document, context: str):
    """Lock the display-only unix_global_esdf style signed-distance cloud."""
    display = single_display_for_topic(document, ESDF_CLOUD_VIZ_TOPIC, context)
    assert display["Class"] == "rviz_default_plugins/PointCloud2", (
        f"{context} {ESDF_CLOUD_VIZ_TOPIC} must be a PointCloud2 display"
    )
    assert display["Topic"]["Reliability Policy"] == "Reliable", (
        f"{context} ESDF cloud must use Reliable"
    )
    assert display["Topic"]["Durability Policy"] == "Transient Local", (
        f"{context} ESDF cloud must use Transient Local"
    )
    assert display["Topic"]["History Policy"] == "Keep Last", (
        f"{context} ESDF cloud must keep only the latest cloud"
    )
    assert display["Topic"]["Depth"] == 1, f"{context} ESDF cloud depth must be one"
    assert display["Enabled"] is True, f"{context} ESDF cloud must be enabled"
    assert display["Color Transformer"] == "Intensity", (
        f"{context} ESDF cloud must use Intensity color transformer"
    )
    assert display["Channel Name"] == "intensity", (
        f"{context} ESDF cloud must color by 'intensity' channel"
    )
    assert display["Style"] == "Flat Squares", (
        f"{context} ESDF cloud must render as Flat Squares instead of opaque Boxes"
    )
    assert display["Use rainbow"] is True, f"{context} ESDF cloud must enable rainbow palette"
    assert display["Invert Rainbow"] is False, f"{context} ESDF cloud must not invert rainbow"
    assert display["Autocompute Intensity Bounds"] is False, (
        f"{context} ESDF cloud must use fixed intensity bounds"
    )
    assert display["Min Intensity"] == 0 and display["Max Intensity"] == 2, (
        f"{context} ESDF cloud intensity bounds must be [0, 2]"
    )


def assert_no_removed_rc_esdf_topics(document, context: str):
    removed = sorted(topic for topic in collect_topic_values(document)
                     if str(topic).startswith("/rc_esdf/"))
    assert not removed, f"{context} still displays removed topics: {removed}"


def assert_rog_map_native_displays(document, context: str):
    """ROG-Map's own voxel/ESDF clouds in the simulation view: right class, Best Effort, one each.

    The node publishes these with Best Effort QoS; a Reliable RViz subscription would show nothing.
    occ / inf_occ / unk / esdf start enabled; the /rog_map/viz debug cloud is available but off.
    """
    for topic in ROG_MAP_VOXEL_TOPICS:
        display = single_display_for_topic(document, topic, context)
        assert display["Class"] == "rviz_default_plugins/PointCloud2", (
            f"{context} {topic} must be a PointCloud2 display"
        )
        assert display["Topic"]["Reliability Policy"] == "Best Effort", (
            f"{context} {topic} must use Best Effort (the ROGMap debug publishers are Best Effort)"
        )
        assert display["Topic"]["Durability Policy"] == "Volatile", f"{context} {topic} must be Volatile"
        assert display["Enabled"] is (topic != "/rog_map/viz"), (
            f"{context} {topic} must be {'off' if topic == '/rog_map/viz' else 'enabled'}"
        )
    esdf = single_display_for_topic(document, "/rog_map/esdf", context)
    assert esdf["Color Transformer"] == "Intensity", f"{context} /rog_map/esdf colours by distance"


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


def assert_mujoco_sensor_view(document):
    """Default view shows clean rainbow ESDF + local planning + footprint without voxel box clutter."""
    for group in ("距离场", "路径", "安全"):
        assert named_display(document, group)["Enabled"] is True, (
            f"{group} navigation group must be enabled by default"
        )
    assert named_display(document, "ROG-Map native voxels (local box)")["Enabled"] is False, (
        "ROG-Map native voxels (local box) must stay disabled by default"
    )
    scan = single_display_for_topic(document, "/registered_scan", "MuJoCo sensor view")
    assert scan["Enabled"] is True and scan["Style"] == "Points"
    assert scan["Decay Time"] == 0, "sensor view must not accumulate ghost clouds"
    axes = named_display(document, "MID360 chassis-mounted sensor axes")
    assert axes["Enabled"] is True and axes["Reference Frame"] == "front_mid360"


def assert_shared_profile_contract(workspace, base):
    """Catch lower downstream limits or profile overrides of the user contract."""
    canonical = {
        "footprint_length": 0.581, "footprint_width": 0.571,
        "footprint_safety_margin": 0.02,
    }
    current = {name: dict(value.get("ros__parameters", {})) for name, value in base.items()}
    for profile in (None, "rmuc_2025_navigation.yaml", "rmuc_2027_navigation.yaml", "rmul_2027_navigation.yaml"):
        if profile:
            override = load_yaml(workspace / "src/sim/ats_mujoco_sim/config" / profile)
            for name, value in override.items():
                current.setdefault(name, {}).update(value.get("ros__parameters", {}))
        for name in ("ats_rog_map_adapter", "minco_planner", "ats_goal_manager", "ats_swerve_mpc"):
            for key, value in canonical.items():
                assert current[name][key] == value, f"{profile or 'real'} {name}.{key} mismatch"
        planner = current["minco_planner"]
        assert planner["jps_safe_distance"] == 0.36
        assert planner["yaw_tunnel_enabled"] and planner["spin_enabled"]
        assert planner["reference_speed"] == planner["max_velocity"] == 5.0
        assert planner["yaw_rate_limit"] == 3.0 and planner["yaw_acceleration_limit"] == 2.0
        assert planner["lidar_frame"] == current["ats_rog_map"]["sensor_frame"] == "front_mid360"
        assert current["terrain_analysis_ext"]["traversabilityOverheadHeight"] == 0.249
        assert current["terrain_analysis_ext"]["planarVoxelSize"] == 0.05
        assert current["ats_rog_map"]["odom_layer.robot_height"] == 0.242
        assert current["ats_rog_map"]["odom_layer.top_margin"] == 0.007
        # Unified 5 cm pixel/voxel contract across mapping, localization, terrain, and planning.
        assert current["ats_rog_map"]["core.resolution"] == 0.05
        assert current["ats_rog_map"]["core.esdf.resolution"] == 0.05
        assert current["ats_rog_map"]["core.inflation_resolution"] == 0.05
        assert current["ats_rog_map"]["odom_layer.resolution"] == 0.05
        assert current["ats_rog_map"]["odom_layer.global_coarse_resolution"] == 0.05
        assert current["ats_rog_map_adapter"]["planning_resolution"] == 0.05
        assert current["small_gicp_relocalization"]["global_leaf_size"] == 0.05
        assert current["small_gicp_relocalization"]["registered_leaf_size"] == 0.05
        assert current["point_lio"]["filter_size_surf"] == 0.05
    mpc = current["ats_swerve_mpc"]
    assert mpc["max_vx"] == mpc["max_vy"] == mpc["max_wheel_speed"] == 5.0
    assert mpc["horizon"] * mpc["dt"] >= 5.0 / min(mpc["max_ax"], mpc["max_ay"])
    inferred = 2 * math.pi * mpc["wheel_radius"] * mpc["motor_max_rpm"] / (
        60 * mpc["drive_gear_ratio"] * mpc["actuator_redundancy"])
    assert math.isclose(inferred, mpc["max_wheel_speed"], abs_tol=1e-8)
    assert current["chassis_vel_transform"]["max_linear_speed"] == 5.0
