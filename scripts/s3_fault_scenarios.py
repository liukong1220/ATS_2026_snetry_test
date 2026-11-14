#!/usr/bin/env python3
"""Isolated S3 fault experiments. Process mutations require exact domain ownership."""
from copy import deepcopy
import hashlib
import json
import math
import os
from pathlib import Path
import re
import signal
import subprocess
import struct
import time


VELOCITY_TOPICS = ("/cmd_vel/autonomy_raw", "/cmd_vel/selected", "/motion_control")


def fault_goal(profile):
    """Use an existing authoritative map waypoint, including its required yaw."""
    points = profile.get("waypoints", [])
    if not points or len(points[0]) != 3 or not all(math.isfinite(x) for x in points[0]):
        raise ValueError("fault experiment requires a finite existing profile waypoint")
    return list(points[0])


def recorded_fault_goal(probe):
    return fault_goal(json.loads((probe.directory / "command.json").read_text())["profile"])


def map_observation_pose(position, quaternion, transform, map_bias_x=0.0):
    """Synthetic observation: fixed map<-odom anchor times the measured odom pose.

    Bias is an explicit map-x translation experiment; it does not remove the
    real spawn translation, height or orientation. Quaternion order is xyzw.
    """
    import numpy as np
    p, q = np.array(position, dtype=float), np.array(quaternion, dtype=float)
    t = np.array([transform["translation"][key] for key in ("x", "y", "z")])
    r = np.array([transform["rotation"][key] for key in ("x", "y", "z", "w")])
    if (p.shape != (3,) or q.shape != (4,) or
            not all(np.isfinite(v).all() for v in (p, q, t, r)) or
            not math.isfinite(map_bias_x) or
            abs(float(q @ q) - 1.0) > 1e-3 or abs(float(r @ r) - 1.0) > 1e-3):
        raise ValueError("invalid synthetic observation pose or anchor")
    q /= np.linalg.norm(q)
    r /= np.linalg.norm(r)
    rotated = p + 2 * np.cross(r[:3], np.cross(r[:3], p) + r[3] * p)
    mapped = rotated + t + [map_bias_x, 0., 0.]
    orientation = np.r_[r[3] * q[:3] + q[3] * r[:3] + np.cross(r[:3], q[:3]),
                        r[3] * q[3] - r[:3] @ q[:3]]
    return mapped.tolist(), orientation.tolist()


def point_in_odom(position, map_from_odom):
    inverse_rotation = {key: value if key == "w" else -value
                        for key, value in map_from_odom["rotation"].items()}
    shifted = [position[i] - map_from_odom["translation"][key]
               for i, key in enumerate(("x", "y", "z"))]
    mapped, _ = map_observation_pose(shifted, [0., 0., 0., 1.],
        {"translation": {"x": 0., "y": 0., "z": 0.}, "rotation": inverse_rotation})
    return mapped


def planner_latency_ms(marker_ns, stamp_ns, commits, digest=None):
    matching = [c for c in commits if c["stamp_ns"] == stamp_ns and
                (digest is None or c["digest"] == digest) and
                c["received_ns"] >= marker_ns and c["committed_ns"] >= c["received_ns"]]
    if not matching:
        raise ValueError("no matching post-insertion MINCO map commit")
    return (min(c["committed_ns"] for c in matching) - marker_ns) / 1e6


def map_commits(directory):
    text = (directory / "launch.log").read_text(errors="replace")
    return [{"stamp_ns": int(stamp), "received_ns": int(received), "committed_ns": int(committed),
             "digest": digest}
            for stamp, received, committed, digest in re.findall(
                r"S3 map_commit stamp_ns=(\d+) received_steady_ns=(\d+) committed_steady_ns=(\d+) digest=([0-9a-f]+)", text)]


def grid_safety_digest(grid, threshold=50, unknown_is_obstacle=True):
    """Same explicit byte fields as MINCO planning_digest::gridSafetyDigest."""
    frame = grid.header.frame_id.encode()
    origin = grid.info.origin
    raw = struct.pack("<Q", len(frame)) + frame
    raw += struct.pack("<fII", grid.info.resolution, grid.info.width, grid.info.height)
    raw += struct.pack("<7d", origin.position.x, origin.position.y, origin.position.z,
                       origin.orientation.x, origin.orientation.y, origin.orientation.z,
                       origin.orientation.w)
    raw += struct.pack("<iB", threshold, int(unknown_is_obstacle))
    raw += bytes(v % 256 for v in grid.data)
    return hashlib.sha256(raw).hexdigest()[:32]


def fixture_marker(text, after_ns, position):
    for raw in re.findall(r"S3 obstacle_fixture (\{[^\n]+\})", text):
        marker = json.loads(raw)
        if marker["steady_ns"] >= after_ns and marker["position"] == list(position):
            return marker
    return None


def fixture_position(probe, position):
    from rcl_interfaces.srv import SetParametersAtomically
    from rclpy.parameter import Parameter
    client = probe.node.create_client(SetParametersAtomically, "/ats_mujoco_sim/set_parameters_atomically")
    try:
        if not client.wait_for_service(timeout_sec=10):
            raise RuntimeError("physical fixture parameter service unavailable")
        request = SetParametersAtomically.Request(parameters=[
            Parameter("test_obstacle_position", value=[float(x) for x in position]).to_parameter_msg()])
        requested_ns = time.monotonic_ns()
        future = client.call_async(request)
        require(probe, future.done, 10, "physical obstacle parameter acknowledgement")
        result = future.result().result
        if not result.successful:
            raise RuntimeError("physical fixture rejected: " + result.reason)
        # Humble returns an empty success reason for atomic parameter updates.
        # Use the simulator's application marker, never evaluator receipt time.
        def applied_marker():
            return fixture_marker((probe.directory / "launch.log").read_text(errors="replace"),
                                  requested_ns, position)
        require(probe, lambda: applied_marker() is not None, 5, "simulator application marker")
        marker = applied_marker()
        probe.event("physical_obstacle", marker)
        return marker
    finally:
        probe.node.destroy_client(client)


def obstacle_region(grid, site=None):
    # Visibility means any new obstacle cell overlapping this physical box,
    # including its sides; a centre-only patch can miss the first real return.
    # The extra half-cell covers conservative rasterisation at the boundary.
    import numpy as np
    site = site or {"odom_position": [2., 0., .3], "odom_half_extent": [.3, .3, .3]}
    resolution = grid.info.resolution
    origin = grid.info.origin
    q = getattr(origin, "orientation", None)
    heading = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y*q.y + q.z*q.z)) if q else 0.
    c, s = math.cos(heading), math.sin(heading)
    rotation = np.array([[c, -s], [s, c]])
    center = rotation.T @ (np.array(site["odom_position"][:2]) -
                            [origin.position.x, origin.position.y])
    extent = np.abs(rotation.T) @ np.array(site["odom_half_extent"][:2])
    lower = np.floor((center - extent) / resolution - 1e-9).astype(int)
    upper = np.floor((center + extent) / resolution + 1e-9).astype(int)
    clipped = bool((lower < 0).any() or upper[0] >= grid.info.width or upper[1] >= grid.info.height)
    x0, y0 = np.maximum(lower, 0)
    x1, y1 = np.minimum(upper + 1, [grid.info.width, grid.info.height])
    values = np.asarray(grid.data).reshape(grid.info.height, grid.info.width)[y0:y1, x0:x1].ravel().tolist()
    if clipped:
        values.append(-1)
    return values


def region_free(probe):
    entry = probe.latest.get("/rog_map_adapter/planning_snapshot")
    if not entry:
        return False
    values = obstacle_region(entry[0], probe.fixture_site)
    return bool(values) and all(0 <= v < 50 for v in values)


def prepare_fixture_site(probe):
    from rclpy.time import Time
    from s3_fixture_scene import fixture_site, fixture_position_for_pose
    from s3_mujoco_acceptance import yaw
    truth = probe.latest["/simulation/ground_truth/odometry"][0].pose.pose
    transform = probe.buffer.lookup_transform("map", "odom", Time())
    candidates = []
    for distance in (2., 1.5, 2.5):
        for offset in (0., .3, -.3):
            position = fixture_position_for_pose(
                probe.scene_model, [truth.position.x, truth.position.y, yaw(truth.orientation) + offset], distance)
            site = fixture_site(position, probe.message_dict(transform.transform))
            values = obstacle_region(probe.latest["/rog_map_adapter/planning_snapshot"][0], site)
            candidates.append({"site": site, "values": values})
            if values and all(0 <= value < 50 for value in values):
                site["map_from_odom"] = probe.message_dict(transform)
                probe.fixture_site = site
                probe.event("fixture_site", site)
                return site
    probe.event("fixture_site_rejected", {"candidates": candidates})
    raise RuntimeError("no fully known-free physical fixture site in front of the robot")


def obstacle_latency(probe):
    import numpy as np
    from rcl_interfaces.srv import GetParameters
    client = probe.node.create_client(GetParameters, "/minco_planner/get_parameters")
    if not client.wait_for_service(timeout_sec=10):
        raise RuntimeError("MINCO effective parameters unavailable")
    future = client.call_async(GetParameters.Request(names=["obstacle_value_threshold", "unknown_is_obstacle"]))
    require(probe, future.done, 5, "effective MINCO grid semantics")
    threshold = future.result().values[0].integer_value
    unknown_is_obstacle = future.result().values[1].bool_value
    probe.node.destroy_client(client)
    probe.event("latency_grid_semantics", {"threshold": threshold, "unknown_is_obstacle": unknown_is_obstacle})
    site = prepare_fixture_site(probe)
    require(probe, lambda: region_free(probe), 30, "known-free future obstacle surface")
    samples = []
    for index in range(20):
        marker = fixture_position(probe, site["world_position"])
        marker_s = marker["steady_ns"] * 1e-9
        observed = {}

        def new_occupied():
            for received, grid in probe.history.get("/rog_map_adapter/planning_snapshot", []):
                if received < marker_s or not any(v >= 50 for v in obstacle_region(grid, site)):
                    continue
                stamp_ns = grid.header.stamp.sec * 10**9 + grid.header.stamp.nanosec
                try:
                    latency = planner_latency_ms(marker["steady_ns"], stamp_ns,
                                                 map_commits(probe.directory),
                                                 grid_safety_digest(grid, threshold, unknown_is_obstacle))
                except ValueError:
                    continue
                observed.update(stamp_ns=stamp_ns, digest=grid_safety_digest(grid, threshold, unknown_is_obstacle),
                                latency_ms=latency, grid_received_s=received)
                return True
            return False

        require(probe, new_occupied, 10, "physical obstacle reaches committed MINCO grid")
        sample = {"index": index, **marker, **observed}
        clouds = [c for c in probe.obstacle_clouds if c["received_ns"] >= marker["steady_ns"] and
                  c["obstacle_points"] > 0]
        if clouds:
            sample["first_registered_cloud_ms"] = (clouds[0]["received_ns"] - marker["steady_ns"]) / 1e6
        samples.append(sample)
        probe.event("obstacle_latency_sample", sample)
        print("LATENCY", json.dumps(sample), flush=True)
        fixture_position(probe, [*site["world_position"][:2], -10.0])
        require(probe, lambda: region_free(probe), 30, "physical ray clearing restores free surface")
        spin_for(probe, 0.1)
    latencies = [s["latency_ms"] for s in samples]
    result = {"samples": samples, "p50_ms": float(np.percentile(latencies, 50)),
              "p95_ms": float(np.percentile(latencies, 95)), "maximum_ms": max(latencies),
              "measurement": "physical obstacle insertion to matching MINCO grid commit",
              "fixture_site": site}
    (probe.directory / "latency.json").write_text(json.dumps(result, indent=2) + "\n")
    limit = float(getattr(probe, "latency_p95_limit_ms", 250.0))
    result["p95_limit_ms"] = limit
    if result["p95_ms"] >= limit:
        raise RuntimeError(f"physical obstacle latency p95 {result['p95_ms']:.3f} ms exceeds {limit:g} ms "
                           f"(maximum {result['maximum_ms']:.3f} ms)")
    return {"passed": True, "F": result}


def saved_fixture_counts(prefix, world_position, half_size=.3):
    import numpy as np
    from s3_map_quality import read_ascii_pcd, load_map
    points = read_ascii_pcd(Path(str(prefix) + ".pcd"))
    center = np.array(world_position)
    cloud = ((np.abs(points[:, 0] - center[0]) < half_size + .15) &
             (np.abs(points[:, 1] - center[1]) < half_size + .15) &
             (points[:, 2] > center[2] - half_size + .15) &
             (points[:, 2] < center[2] + half_size + .15))
    _, cells, centres = load_map(prefix)
    region = ((np.abs(centres[:, 0] - center[0]) < half_size + .15) &
              (np.abs(centres[:, 1] - center[1]) < half_size + .15))
    return {"cloud_obstacle_points": int(cloud.sum()),
            "pgm_obstacle_cells": int(((cells >= 50) & region).sum())}


def dynamic_save(probe):
    site = prepare_fixture_site(probe)
    require(probe, lambda: region_free(probe), 30, "known-free dynamic obstacle site")
    marker = fixture_position(probe, site["world_position"])
    require(probe, lambda: any(v >= 50 for v in obstacle_region(
        probe.latest["/rog_map_adapter/planning_snapshot"][0], site)), 10, "physical dynamic obstacle visible")
    trace_last = [-1.0]

    def promoted_and_traced():
        # Per-second region values of the planning snapshot and the odom layer, to
        # show whether the fixture cells flicker before the save eligibility check.
        now = probe.sim_time()
        if now - trace_last[0] >= 1.0:
            trace_last[0] = now
            probe.event("dynamic_region_trace", {
                "sim_s": now,
                "planning": [int(v) for v in obstacle_region(probe.latest["/rog_map_adapter/planning_snapshot"][0], site)],
                "odom_layer": [int(v) for v in obstacle_region(probe.latest["/rog_map/odom_layer"][0], site)]})
        return now >= marker["sim_s"] + 7.0

    require(probe, promoted_and_traced, 30, "dynamic object exceeds unchanged 3 second promotion interval")
    before = probe.save_map("before_dynamic_removal")

    initial = saved_fixture_counts(probe.directory / "before_dynamic_removal", site["world_position"])
    if initial["cloud_obstacle_points"] == 0 or initial["pgm_obstacle_cells"] == 0:
        raise RuntimeError("dynamic fixture never demonstrated persistent saved obstacle evidence: " + str(initial))
    removed = fixture_position(probe, [*site["world_position"][:2], -10.0])
    require(probe, lambda: region_free(probe), 30, "real rays clear removed dynamic obstacle")
    spin_for(probe, 3.0)
    after = probe.save_map()
    final = saved_fixture_counts(probe.directory / "map", site["world_position"])
    result = {"inserted": marker, "removed": removed, "before_region": initial,
              "after_region": final, "before_save": before, "after_save": after, "fixture_site": site}
    probe.event("dynamic_save_result", result)
    if any(final.values()):
        raise RuntimeError("cleared dynamic obstacle survived static map save: " + str(final))
    return {"passed": True, "dynamic_obstacle_excluded": True, "C_dynamic": result}


def process_matches(command, environment, domain, owned):
    return (owned and bool(command) and Path(command[0]).name == "pointlio_mapping" and
            environment.get("ROS_DOMAIN_ID") == str(domain) and domain != 77)


def sustained_zero(samples, after, duration, tolerance=1e-6):
    start, count = None, 0
    for when, values in samples:
        if when < after:
            continue
        if any(abs(v) > tolerance or not math.isfinite(v) for v in values):
            start, count = None, 0
        else:
            start = when if start is None else start
            count += 1
    return bool(start is not None and count >= 3 and samples[-1][0] - start >= duration)


def owned_process(launch_pid, name):
    if name not in ("ats_rog_map_node", "small_gicp_relocalization_node"):
        raise ValueError("not an allowed S3 process fixture")
    domain = str(os.environ["ROS_DOMAIN_ID"])
    if domain == "77":
        raise ValueError("domain 77 is excluded")
    found = []
    for entry in Path("/proc").iterdir():
        if not entry.name.isdigit():
            continue
        try:
            command = [x.decode() for x in (entry / "cmdline").read_bytes().split(b"\0") if x]
            if not command or Path(command[0]).name != name:
                continue
            environment = dict(x.decode().split("=", 1) for x in
                               (entry / "environ").read_bytes().split(b"\0") if b"=" in x)
            if environment.get("ROS_DOMAIN_ID") == domain and belongs_to(int(entry.name), launch_pid):
                found.append((int(entry.name), command))
        except (OSError, UnicodeDecodeError):
            continue
    if len(found) != 1:
        raise RuntimeError(f"expected one owned {name}, found {len(found)}")
    return found[0]


def set_parameters(probe, node, values):
    from rcl_interfaces.srv import SetParametersAtomically
    from rclpy.parameter import Parameter
    client = probe.node.create_client(SetParametersAtomically, node + "/set_parameters_atomically")
    try:
        if not client.wait_for_service(timeout_sec=10):
            raise RuntimeError(node + " parameter service unavailable")
        request = SetParametersAtomically.Request(parameters=[
            Parameter(name, value=value).to_parameter_msg() for name, value in values.items()])
        future = client.call_async(request)
        require(probe, future.done, 10, "fault parameter acknowledgement")
        if not future.result().result.successful:
            raise RuntimeError(future.result().result.reason)
        probe.event("set_parameters", {"node": node, "values": values})
    finally:
        probe.node.destroy_client(client)


def get_parameters(probe, node, names):
    from rcl_interfaces.srv import GetParameters
    from rclpy.parameter import parameter_value_to_python
    client = probe.node.create_client(GetParameters, node + "/get_parameters")
    try:
        if not client.wait_for_service(timeout_sec=10):
            raise RuntimeError(node + " effective parameters unavailable")
        future = client.call_async(GetParameters.Request(names=names))
        require(probe, future.done, 5, "effective parameters")
        return {name: parameter_value_to_python(value)
                for name, value in zip(names, future.result().values)}
    finally:
        probe.node.destroy_client(client)


def effective_search_floor(parameters, resolution):
    p = parameters
    margin = p["footprint_safety_margin"]
    preferred = max(p["jps_safe_distance"], math.hypot(
        p["footprint_length"] / 2 + margin, p["footprint_width"] / 2 + margin))
    configured = p["search_clearance_floor"]
    automatic = min(p["footprint_length"], p["footprint_width"]) / 2 + margin + resolution / math.sqrt(2)
    return min(preferred, max(0.0, configured) if configured >= 0 else automatic)


def odom_continuity(samples):
    if len(samples) < 2:
        return {"passed": False, "samples": len(samples)}
    steps, violations = [], 0
    for a, b in zip(samples, samples[1:]):
        dt = b["stamp"] - a["stamp"]
        distance = math.hypot(b["x"] - a["x"], b["y"] - a["y"])
        # 1.5 m/s bounds the configured 1.2 m/s controller; 2 cm is an
        # explicit observation-noise allowance, not a goal-error tolerance.
        if dt <= 0 or not math.isfinite(distance) or distance > 1.5 * dt + 0.02:
            violations += 1
        steps.append(distance)
    return {"passed": violations == 0, "samples": len(samples),
            "violations": violations, "maximum_step_m": max(steps)}


def correction_case(probe, launch, delta):
    """Inject observations through the sole fusion owner, never a second TF."""
    from ats_navigation_interfaces.msg import RelocalizationObservation
    from rclpy.time import Time
    from s3_mujoco_acceptance import stamp, yaw
    matched = {}
    def find_matcher():
        try:
            pid, command = owned_process(launch.pid, "small_gicp_relocalization_node")
            matched.update(pid=pid, command=command)
            return True
        except RuntimeError:
            return False
    require(probe, find_matcher, 20, "owned matcher before first acceptance")
    probe.event("replace_matcher_for_synthetic_observations", matched)
    os.kill(matched["pid"], signal.SIGTERM)
    require(probe, lambda: not probe.node.get_publishers_info_by_topic(
        "/relocalization_observation"), 10, "no competing observation publisher")
    require(probe, lambda: "/localization/status" in probe.latest and
            probe.latest["/localization/status"][0].odom_instance_id != 0 and
            "/localization" in probe.latest, 30, "known frontend before first accepted observation")
    if stamp(probe.latest["/localization/status"][0].last_observation_stamp) > 0:
        raise RuntimeError("matcher accepted before the controlled first-acceptance gate")
    goal = recorded_fault_goal(probe)
    require(probe, lambda: probe.buffer.can_transform("map", "odom", Time()),
            5, "seeded map-to-odom anchor")
    anchor = probe.message_dict(probe.buffer.lookup_transform("map", "odom", Time()).transform)
    initial_odom = deepcopy(probe.latest["/localization"][0].pose.pose.position)
    probe.event("correction_anchor", {"map_from_odom": anchor, "goal": goal})
    handle = start_goal(probe, *goal)
    early_at = time.monotonic()
    spin_for(probe, 1.0)
    # Map readiness is independent of first global acceptance. Establish real
    # stationary obstacle evidence while the early goal is still held; a run
    # with no pre-step obstacles cannot prove that map correction preserves them.
    require(probe, lambda: "/rog_map/odom_layer" in probe.latest and
            sum(v >= 50 for v in probe.latest["/rog_map/odom_layer"][0].data) >= 10,
            25, "stationary obstacle evidence before accepting the first observation")
    if probe.goals or not all_zero(probe, early_at):
        raise RuntimeError("goal escaped before first accepted localization")
    probe.event("J_preaccept_goal_held", {"duration_wall_s": 1.0, "planner_dispatches": 0})
    state = {"bias": -delta, "sequence": 100000, "stamp": 0.0}
    publisher = probe.node.create_publisher(RelocalizationObservation, "/relocalization_observation", 10)
    def observe():
        entry = probe.latest.get("/localization")
        status_entry = probe.latest.get("/localization/status")
        if not entry or not status_entry:
            return
        odom, status = entry[0], status_entry[0]
        when = stamp(odom.header.stamp)
        if when <= state["stamp"]:
            return
        observation = RelocalizationObservation()
        observation.header.stamp, observation.header.frame_id = odom.header.stamp, "map"
        observation.child_frame_id = "gimbal_yaw_odom"
        state["sequence"] += 1
        state["stamp"] = when
        observation.sequence = state["sequence"]
        observation.odom_instance_id, observation.odom_epoch = status.odom_instance_id, status.odom_epoch
        p, q = odom.pose.pose.position, odom.pose.pose.orientation
        position, orientation = map_observation_pose(
            [p.x, p.y, p.z], [q.x, q.y, q.z, q.w], anchor, state["bias"])
        for key, value in zip(("x", "y", "z"), position):
            setattr(observation.pose.pose.position, key, value)
        for key, value in zip(("x", "y", "z", "w"), orientation):
            setattr(observation.pose.pose.orientation, key, value)
        for i in (0, 7, 14, 21, 28, 35):
            observation.pose.covariance[i] = 0.001
        observation.status, observation.accepted = observation.STATUS_ACCEPTED, True
        observation.inlier_count, observation.source_points = 1500, 2000
        observation.registration_error, observation.quality = 0.001, 0.99
        observation.message = "S3 explicit synthetic map correction experiment"
        publisher.publish(observation)
    timer = probe.node.create_timer(0.05, observe)
    try:
        require(probe, probe.ready, 30, "first synthetic acceptance and mapping ready")
        graph = probe.graph()
        if graph["/rog_map_adapter/planning_snapshot"]["publishers"] != ["ats_rog_map_adapter"]:
            raise RuntimeError("planning grid ownership changed")
        require(probe, lambda: probe.last_execute is not None and
                math.hypot(probe.latest["/localization"][0].pose.pose.position.x - initial_odom.x,
                           probe.latest["/localization"][0].pose.pose.position.y - initial_odom.y) > 0.15 and
                math.hypot(probe.latest["/cmd_vel/selected"][0].linear.x,
                           probe.latest["/cmd_vel/selected"][0].linear.y) > 0.05,
                30, "motion under initially biased accepted transform")
        before_status = deepcopy(probe.latest["/localization/status"][0])
        before_tf = probe.buffer.lookup_transform("map", "odom", Time())
        if abs(before_tf.transform.translation.x - anchor["translation"]["x"] + delta) > 0.002:
            raise RuntimeError("initial correction bias was not actually accepted")
        before_grid = deepcopy(probe.latest["/rog_map/odom_layer"][0])
        probe.save_grid("correction_before")
        previous_request = probe.goals[-1][0].plan_request_sequence
        log_offset = (probe.directory / "launch.log").stat().st_size
        step_at = time.monotonic()
        state["bias"] = 0.0
        probe.event("map_correction_step", {"delta_m": delta, "before_tf": probe.message_dict(before_tf),
                    "before_status": probe.message_dict(before_status)})
        def correction_applied():
            try:
                current = probe.buffer.lookup_transform("map", "odom", Time()).transform.translation
                return math.sqrt(sum((getattr(current, key) - anchor["translation"][key])**2
                                     for key in ("x", "y", "z"))) < 0.002
            except Exception:
                return False
        require(probe, correction_applied, 10, "fusion applied the injected correction")
        expected_odom_goal = point_in_odom([goal[0], goal[1], 0.], anchor)
        require(probe, lambda: any(g.plan_request_sequence > previous_request and
                g.goal_pose.header.frame_id == "odom" and
                math.hypot(g.goal_pose.pose.position.x - expected_odom_goal[0],
                           g.goal_pose.pose.position.y - expected_odom_goal[1]) < 0.015
                for g, _ in probe.goals),
                20, "authoritative map goal retargeted with the latest transform")
        probe.finish_goal(handle, goal, 90, final=True)
        after_status = probe.latest["/localization/status"][0]
        after_tf = probe.buffer.lookup_transform("map", "odom", Time())
        after_grid = probe.latest["/rog_map/odom_layer"][0]
        if (after_status.odom_epoch != before_status.odom_epoch or
                after_status.odom_instance_id != before_status.odom_instance_id):
            raise RuntimeError("map correction unexpectedly changed the frontend identity")
        if delta <= 0.20:
            if after_status.epoch != before_status.epoch:
                raise RuntimeError("continuous correction advanced localization epoch")
            if "S3 endpoint retarget" not in (probe.directory / "launch.log").read_bytes()[log_offset:].decode(errors="replace"):
                raise RuntimeError("endpoint drift did not trigger normal retargeting")
        elif after_status.epoch <= before_status.epoch:
            raise RuntimeError("large correction failed to invalidate localization epoch")
        geometry = lambda g: (g.header.frame_id, g.info.width, g.info.height, g.info.resolution,
                              probe.message_dict(g.info.origin))
        if geometry(before_grid) != geometry(after_grid):
            raise RuntimeError("odom obstacle geometry moved with map correction")
        retained = sum(a >= 50 and b >= 50 for a, b in zip(before_grid.data, after_grid.data))
        if retained == 0:
            raise RuntimeError("no stationary obstacle cells survived the map correction")
        odom_samples, seen = [], set()
        for received, odom in probe.history.get("/localization", []):
            when = stamp(odom.header.stamp)
            if received < step_at - 0.25 or when in seen:
                continue
            seen.add(when)
            p = odom.pose.pose.position
            odom_samples.append({"received": received, "stamp": when, "x": p.x, "y": p.y,
                                 "z": p.z, "yaw": yaw(odom.pose.pose.orientation)})
        continuity = odom_continuity(odom_samples)
        (probe.directory / "odom-correction-window.json").write_text(json.dumps(odom_samples, indent=2) + "\n")
        if not continuity["passed"]:
            raise RuntimeError("odom continuity failed: " + str(continuity))
        if delta > 0.20:
            events = [json.loads(line) for line in (probe.directory / "events.jsonl").read_text().splitlines()]
            if not any(e["wall_s"] >= step_at and e["event"] == "/planner/emergency_stop" and
                       e["data"]["data"] for e in events):
                raise RuntimeError("large correction did not stop execution")
            for topic in VELOCITY_TOPICS:
                if not any(t >= step_at and max(map(abs, values)) < 1e-6
                           for t, values in probe.history[topic]):
                    raise RuntimeError("large correction missed zero velocity: " + topic)
        result = {"passed": True, "A": True, "I": True, "J": True, "K": True,
                  "synthetic_observation": True, "delta_m": delta, "continuity": continuity,
                  "anchor": anchor, "goal": goal,
                  "stationary_occupied_cells_retained": retained,
                  "before_tf": probe.message_dict(before_tf), "after_tf": probe.message_dict(after_tf),
                  "before_epoch": before_status.epoch, "after_epoch": after_status.epoch,
                  "odom_epoch": after_status.odom_epoch, "endpoints": probe.endpoint_results}
        probe.event("correction_result", result)
        return result
    finally:
        probe.node.destroy_timer(timer)
        probe.node.destroy_publisher(publisher)


def safety_case(case, probe, launch):
    if case == "no_path":
        # A low barrier is present only in the explicit small test scene.
        # Full fields use naturally disconnected known-free cells from the live
        # grid; lack of such a cell fails the experiment without changing geometry.
        from query_occupancy_grid import find_unreachable, grid_to_world
        semantics = get_parameters(probe, "/minco_planner", ["footprint_length",
            "footprint_width", "footprint_safety_margin", "jps_safe_distance",
            "search_clearance_floor", "obstacle_value_threshold"])
        selected = {}
        def visible_unreachable():
            grid = probe.latest["/rog_map_adapter/planning_snapshot"][0]
            position = probe.latest["/localization"][0].pose.pose.position
            clearance = effective_search_floor(semantics, grid.info.resolution)
            index = find_unreachable(grid, position.x, position.y,
                                     semantics["obstacle_value_threshold"], clearance,
                                     assume_start_traversable=True)
            if index is None:
                return False
            selected.update(grid=grid, index=index, clearance=clearance)
            return True
        require(probe, visible_unreachable, 25, "known-free disconnected target on effective clearance floor")
        grid, index = selected["grid"], selected["index"]
        x, y = grid_to_world(grid, index % grid.info.width, index // grid.info.width)
        if not 0 <= grid.data[index] < 100:
            raise RuntimeError("no-path target must be a genuinely known-free grid cell")
        source_xy = [x, y]
        if grid.header.frame_id != "map":
            from rclpy.time import Time
            transform = probe.buffer.lookup_transform("map", grid.header.frame_id, Time())
            mapped, _ = map_observation_pose([x, y, 0.], [0., 0., 0., 1.],
                                             probe.message_dict(transform.transform))
            x, y = mapped[:2]
        probe.event("free_unreachable_goal", {"index": index, "value": grid.data[index],
                    "grid_frame": grid.header.frame_id, "grid_xy": source_xy, "map_xy": [x, y],
                    "effective_clearance_m": selected["clearance"],
                    "planner_parameters": semantics})
        fault_at = time.monotonic()
        handle = start_goal(probe, x=x, y=y)
    else:
        handle = start_goal(probe)
        require(probe, lambda: probe.last_execute is not None and
                "/cmd_vel/selected" in probe.latest and
                math.hypot(probe.latest["/cmd_vel/selected"][0].linear.x,
                           probe.latest["/cmd_vel/selected"][0].linear.y) > 0.05,
                25, "active execution before safety injection")
        fault_at = time.monotonic()
    stalled = False
    status_begin = len(probe.planner_statuses)
    try:
        if case == "service_timeout":
            # The projection runs in-process now. Stopping ats_rog_map_node would
            # also stop the planning publisher (that is the adapter-lease fault);
            # this stalls only the projection so the request deadline fires.
            set_parameters(probe, "/ats_rog_map_adapter", {"test_stall_projection": True})
            stalled = True
            probe.event("stall_projection", {"node": "/ats_rog_map_adapter"})
        elif case == "unknown":
            set_parameters(probe, "/ats_rog_map_adapter", {"test_mask_secondary_evidence": True})
            set_parameters(probe, "/ats_rog_map", {"test_reset_to_unknown": True})
        elif case == "unsafe":
            from s3_fixture_scene import fixture_position_for_pose
            from s3_mujoco_acceptance import yaw
            pose = probe.latest["/simulation/ground_truth/odometry"][0].pose.pose
            velocity = probe.latest["/cmd_vel/selected"][0].linear
            direction = yaw(pose.orientation) + math.atan2(velocity.y, velocity.x)
            fixture_position(probe, fixture_position_for_pose(probe.scene_model,
                [pose.position.x, pose.position.y, direction], distance=0.80))
        elif case != "no_path":
            raise ValueError(case)
        require(probe, lambda: "/planner/emergency_stop" in probe.latest and
                probe.latest["/planner/emergency_stop"][1] >= fault_at and
                probe.latest["/planner/emergency_stop"][0].data and all_zero(probe, fault_at),
                12, "fresh emergency stop and sustained zero after safety fault")
        failures = [probe.message_dict(s) for s in probe.planner_statuses[status_begin:]
                    if s.state == s.STATE_FAILED]
        if case == "no_path":
            require(probe, lambda: any(s.failure_reason == s.FAILURE_NO_PATH for s in
                    probe.planner_statuses[status_begin:]), 10, "free unreachable classified NO_PATH")
        if case == "unsafe":
            require(probe, lambda: any(s.failure_reason in (s.FAILURE_RUNTIME_UNSAFE,
                    s.FAILURE_FOOTPRINT, s.FAILURE_START_ENCLOSED, s.FAILURE_SNAPSHOT_CHANGED)
                    for s in probe.planner_statuses[status_begin:]), 10,
                    "unsafe map invalidates the committed reference")
        # A stopped simulation step cannot count as evidence of continued zeros.
        observed_sim = probe.sim_time()
        require(probe, lambda: probe.sim_time() > observed_sim + 0.6 and all_zero(probe, fault_at),
                10, "simulation progresses while all velocity owners remain stopped")
        probe.save_grid(case)
        result = {"passed": True, "fault": case, "stop_wall_s": time.monotonic() - fault_at,
                  "failure_statuses": failures, "zero_topics": list(VELOCITY_TOPICS)}
        probe.event("safety_result", result)
        return result
    finally:
        cancel = handle.cancel_goal_async()
        require(probe, cancel.done, 5, "cancel faulted goal")
        if stalled:
            set_parameters(probe, "/ats_rog_map_adapter", {"test_stall_projection": False})


def belongs_to(pid, ancestor):
    visited = set()
    while pid > 1 and pid not in visited:
        if pid == ancestor:
            return True
        visited.add(pid)
        try:
            status = Path(f"/proc/{pid}/status").read_text()
            pid = int(next(x.split()[1] for x in status.splitlines() if x.startswith("PPid:")))
        except (OSError, StopIteration, ValueError):
            return False
    return False


def frontend_process(launch_pid, directory):
    domain = int(os.environ["ROS_DOMAIN_ID"])
    matches = []
    for entry in Path("/proc").iterdir():
        if not entry.name.isdigit():
            continue
        try:
            command = [x.decode() for x in (entry / "cmdline").read_bytes().split(b"\0") if x]
            if not command or Path(command[0]).name != "pointlio_mapping":
                continue
            environment = dict(x.decode().split("=", 1) for x in
                               (entry / "environ").read_bytes().split(b"\0") if b"=" in x)
            pid = int(entry.name)
            if process_matches(command, environment, domain, belongs_to(pid, launch_pid)):
                matches.append((pid, command, environment, os.readlink(entry / "cwd")))
        except (OSError, UnicodeDecodeError):
            continue
    if len(matches) != 1:
        raise RuntimeError(f"expected exactly one owned frontend, found {len(matches)}")
    pid, command, environment, cwd = matches[0]
    # Keep the complete environment only in memory for exact process restart.
    # Persist ROS/runtime keys; unrelated credentials have no evidentiary value.
    visible = {k: v for k, v in environment.items() if k.startswith(("ROS_", "RMW_", "CYCLONEDDS_",
               "AMENT_", "COLCON_", "LD_", "OMP_")) or k in ("PATH", "PYTHONPATH")}
    (directory / "point-lio-process.json").write_text(json.dumps(
        {"pid": pid, "cmdline": command, "environ": visible, "cwd": cwd}, indent=2) + "\n")
    return matches[0]


def require(probe, predicate, timeout, label):
    if not probe.spin_until(predicate, timeout, label):
        raise RuntimeError(f"timeout: {label}")


def spin_for(probe, duration):
    end = time.monotonic() + duration
    require(probe, lambda: time.monotonic() >= end, duration + 1.0, "observation window")


def start_goal(probe, x=None, y=None, heading=None):
    from ats_navigation_interfaces.action import NavigateToPose
    default = recorded_fault_goal(probe)
    x, y, heading = [default[i] if value is None else value
                     for i, value in enumerate((x, y, heading))]
    goal = NavigateToPose.Goal()
    goal.goal_pose.header.frame_id = "map"
    goal.goal_pose.header.stamp = probe.node.get_clock().now().to_msg()
    goal.goal_pose.pose.position.x, goal.goal_pose.pose.position.y = x, y
    goal.goal_pose.pose.orientation.z = math.sin(heading / 2)
    goal.goal_pose.pose.orientation.w = math.cos(heading / 2)
    goal.timeout.sec = 120
    if not probe.action.wait_for_server(timeout_sec=10):
        raise RuntimeError("goal server unavailable")
    future = probe.action.send_goal_async(goal)
    require(probe, future.done, 10, "goal acknowledgement")
    handle = future.result()
    if not handle.accepted:
        raise RuntimeError("fault experiment goal rejected")
    return handle


def all_zero(probe, after, duration=0.4):
    return all(sustained_zero(probe.history.get(topic, []), after, duration)
               for topic in VELOCITY_TOPICS)


def restart_frontend(probe, launch):
    from ats_navigation_interfaces.msg import ExecutionCommand
    from rclpy.qos import QoSProfile, DurabilityPolicy
    handle = start_goal(probe)
    require(probe, lambda: probe.last_execute is not None and "/cmd_vel/selected" in probe.latest and
            math.hypot(probe.latest["/cmd_vel/selected"][0].linear.x,
                       probe.latest["/cmd_vel/selected"][0].linear.y) > 0.05,
            25, "motion and committed execution before restart")
    old_execution = deepcopy(probe.last_execute)
    old_status = deepcopy(probe.latest["/localization/status"][0])
    pid, command, environment, cwd = frontend_process(launch.pid, probe.directory)
    fault_at = time.monotonic()
    probe.event("kill_point_lio", {"pid": pid, "instance": old_status.odom_instance_id,
                                 "epoch": old_status.odom_epoch})
    # Sum of the existing bounded stop/identity/recovery waits, with cleanup margin.
    with probe.expect_localization_loss(150.0) as finish_expected_loss:
        os.kill(pid, signal.SIGKILL)
        require(probe, lambda: any(t >= fault_at and not ready for t, ready in
                probe.history.get("/rog_map_adapter/ready", [])), 6, "adapter not ready after frontend death")
        require(probe, lambda: probe.latest["/planner/emergency_stop"][1] >= fault_at and
                probe.latest["/planner/emergency_stop"][0].data and all_zero(probe, fault_at),
                6, "fresh emergency stop and sustained zero through both velocity stages")
        stopped_at = time.monotonic()
        cancel = handle.cancel_goal_async()
        require(probe, cancel.done, 5, "cancel interrupted goal before recovery replay")
        require(probe, lambda: not Path(f"/proc/{pid}").exists(), 5, "old frontend process reaped")
        log = (probe.directory / "point-lio-restart.log").open("w")
        process = subprocess.Popen(command, env=environment, cwd=cwd, stdout=log, stderr=log,
                                   start_new_session=True)
        try:
            probe.event("restart_point_lio", {"pid": process.pid})
            require(probe, lambda: probe.latest["/localization/status"][0].odom_instance_id >
                    old_status.odom_instance_id and probe.latest["/localization/status"][0].odom_epoch ==
                    old_status.odom_epoch + 1, 30, "new frontend identity and epoch increment")
            new_epoch = deepcopy(probe.latest["/localization/status"][0])
            require(probe, lambda: probe.ready() and
                    probe.latest["/localization/status"][0].state == old_status.STATE_TRACKING,
                    90, "fresh map and accepted registration after process restart")
            finish_expected_loss()
            recovery_at = time.monotonic()
            qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
            publisher = probe.node.create_publisher(ExecutionCommand, "/planner/execution_command", qos)
            spin_for(probe, 0.3)
            replay_at = time.monotonic()
            probe.event("replay_old_execution", {"epoch": old_execution.localization_epoch,
                "sequence": old_execution.command_sequence, "reference_stamp":
                old_execution.reference.header.stamp.sec + old_execution.reference.header.stamp.nanosec * 1e-9})
            for _ in range(12):
                publisher.publish(old_execution)
                spin_for(probe, 0.05)
            spin_for(probe, 0.5)
            probe.node.destroy_publisher(publisher)
            for topic in VELOCITY_TOPICS:
                samples = [(t, v) for t, v in probe.history[topic] if t >= recovery_at]
                if not samples or any(any(abs(a) > 1e-6 for a in values) for _, values in samples):
                    raise RuntimeError(f"old execution revived after recovery on {topic}")
            result = {"passed": True, "old_instance": old_status.odom_instance_id,
                      "new_instance": new_epoch.odom_instance_id, "old_epoch": old_status.odom_epoch,
                      "new_epoch": new_epoch.odom_epoch, "death_to_sustained_stop_wall_s": stopped_at - fault_at,
                      "recovery_wall_s": recovery_at - fault_at, "old_execution_replay_at": replay_at,
                      "old_execution_rejected": True, "zero_topics": list(VELOCITY_TOPICS)}
            probe.event("restart_result", result)
            return result
        finally:
            if process.poll() is None:
                process.send_signal(signal.SIGINT)
                try:
                    process.wait(timeout=8)
                except subprocess.TimeoutExpired:
                    process.terminate()
                    process.wait(timeout=5)
            log.close()


def run_case(case, probe, launch):
    if case.startswith("correction_"):
        return correction_case(probe, launch, float(case.split("_")[1]))
    if case == "point_lio_restart":
        return restart_frontend(probe, launch)
    if case == "obstacle_latency":
        return obstacle_latency(probe)
    if case == "dynamic_save":
        return dynamic_save(probe)
    if case in ("service_timeout", "unknown", "no_path", "unsafe"):
        return safety_case(case, probe, launch)
    raise ValueError(case)
