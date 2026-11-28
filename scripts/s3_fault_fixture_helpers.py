#!/usr/bin/env python3
"""Isolated S3 fault experiments. Process mutations require exact domain ownership."""
from contextlib import nullcontext
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


def require(probe, predicate, timeout, label):
    if not probe.spin_until(predicate, timeout, label):
        raise RuntimeError(f"timeout: {label}")


def spin_for(probe, duration):
    end = time.monotonic() + duration
    require(probe, lambda: time.monotonic() >= end, duration + 1.0, "observation window")


def spin_sim_for(probe, sim_duration):
    start_sim = probe.sim_time()
    target_sim = start_sim + sim_duration
    wall_timeout = max(30.0, sim_duration * 6.0 + 5.0)
    require(probe, lambda: probe.sim_time() >= target_sim, wall_timeout,
            f"sim-time observation window ({sim_duration:.1f}s)")


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


def region_free_all(probe):
    if not region_free(probe):
        return False
    odom_entry = probe.latest.get("/rog_map/odom_layer")
    if odom_entry:
        odom_values = obstacle_region(odom_entry[0], probe.fixture_site)
        if odom_values and any(v >= 50 for v in odom_values):
            return False
    return True


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
            # Also ensure no static wall obstacle (>= 50) lies inside saved_fixture_counts's
            # half_size + 0.15 m evaluation box.
            padded_site = {
                **site,
                "odom_half_extent": [e + 0.15 for e in site["odom_half_extent"]],
            }
            padded_values = obstacle_region(
                probe.latest["/rog_map_adapter/planning_snapshot"][0], padded_site)
            candidates.append({"site": site, "values": values})
            if values and all(0 <= value < 50 for value in values) and all(v < 50 for v in padded_values):
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
    require(probe, lambda: region_free_all(probe), 30, "post-latency fixture region fully cleared")
    spin_sim_for(probe, 4.0)
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
    require(probe, lambda: region_free_all(probe), 30, "real rays clear removed dynamic obstacle")
    spin_sim_for(probe, 4.0)
    after = probe.save_map()
    final = saved_fixture_counts(probe.directory / "map", site["world_position"])
    result = {"inserted": marker, "removed": removed, "before_region": initial,
              "after_region": final, "before_save": before, "after_save": after, "fixture_site": site}
    probe.event("dynamic_save_result", result)
    if any(final.values()):
        raise RuntimeError("cleared dynamic obstacle survived static map save: " + str(final))
    return {"passed": True, "dynamic_obstacle_excluded": True, "C_dynamic": result}
