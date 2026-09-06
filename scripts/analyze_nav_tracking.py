#!/usr/bin/env python3
"""Offline analyzer that turns nav_tracking_recorder output into a verdict.

``scripts/nav_tracking_recorder.py`` records, on one steady clock, the committed
reference next to the pose that was actually driven, the command that was
selected, the emergency-stop state, and the immutable snapshot identity that was
in force at that instant.  This module joins those records to
``scripts/footprint_evaluator.py`` -- the port that
``scripts/test_footprint_evaluator_parity.sh`` cross-validates against the
planner's own C++ checker -- and answers the three questions that a run log
cannot separate:

Q1  Was the committed reference collision-free *at publish*, judged against the
    snapshot that was in force when it was published?
Q2  Did the *actual* driven pose sequence leave that envelope during tracking?
Q3  Did the *same* pose flip free -> occupied because a later snapshot or source
    layer changed underneath it?

Those three have different owners.  Q1 failing points at the final revalidation
snapshot, sampling continuity and commit timing.  Q2 failing points at MPC
tracking error, execution latency, velocity/acceleration limiting and the
stopping envelope.  Q3 failing points at generation/heartbeat handling and the
timing of old-reference revocation.  Reporting "there were collisions" without
separating them names no owner at all.

Clock discipline mirrors the recorder: every duration here comes from the
recorder's monotonic ``mono_s`` field, and ROS stamps are compared only for
*identity*, never subtracted, because the stack runs several independent clocks
with ``use_sim_time: false``.

Frames: the recorder stores poses in their own frame and samples ``map <- odom``
every tick, because the fused planning grid inherits the static map frame while
the planner works in ``odom``.  Every pose evaluated here is transformed with the
transform recorded *at that tick*.  A tick whose transform is missing is reported
as unpaired and excluded, never silently evaluated as if the two frames were the
same.

A ``footprint_collisions == 0`` verdict from this analyzer is a geometric
statement about one snapshot.  Physical contact is separate evidence: without an
independent contact evaluator it stays unverified here.
"""

from __future__ import annotations

import argparse
import gzip
import json
import math
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import footprint_evaluator as fe  # noqa: E402


def load_jsonl(path):
    """Read a recorder JSON-lines artifact, tolerating a truncated last line.

    The recorder flushes every record precisely so that a killed leg still leaves
    the first-violation evidence on disk.  A run killed mid-write therefore ends
    in a partial line, which is skipped rather than allowed to abort the whole
    analysis -- but it is counted and reported so the artifact cannot be read as
    complete.
    """
    records = []
    truncated = 0
    if not os.path.exists(path):
        return records, None
    with open(path, "r", encoding="utf-8") as handle:
        for line in handle:
            line = line.strip()
            if not line:
                continue
            try:
                records.append(json.loads(line))
            except ValueError:
                truncated += 1
    return records, truncated


def load_json(path):
    if not os.path.exists(path):
        return None
    with open(path, "r", encoding="utf-8") as handle:
        return json.load(handle)


def load_gzip_json(path):
    with gzip.open(path, "rt") as handle:
        return json.load(handle)


# ---- frames ---------------------------------------------------------------


def transform_pose(transform, pose):
    """Apply a recorded ``map <- odom`` transform to an ``(x, y, yaw)`` pose.

    Returns ``None`` when the transform is missing, so a caller has to decide
    explicitly what to do about an unpaired tick instead of inheriting identity.
    """
    if transform is None:
        return None
    cos_yaw = math.cos(transform["yaw"])
    sin_yaw = math.sin(transform["yaw"])
    x, y, yaw = pose[0], pose[1], pose[2]
    return (
        transform["x"] + cos_yaw * x - sin_yaw * y,
        transform["y"] + sin_yaw * x + cos_yaw * y,
        fe.normalize_angle(transform["yaw"] + yaw),
    )


def transform_poses(transform, poses):
    if transform is None:
        return None
    return [transform_pose(transform, pose) for pose in poses]


def frames_agree(pose_frame, grid_frame):
    """True when a pose may be evaluated against a grid without a transform.

    An empty frame id on either side is treated as *not* agreeing: an unlabelled
    pose is an evidence gap, and assuming it matches is how a cross-frame
    comparison gets reported as a clean verdict.
    """
    return bool(pose_frame) and bool(grid_frame) and pose_frame == grid_frame


# ---- snapshot and layer payload access -------------------------------------


class PayloadStore(object):
    """Lazy, cached access to the gzipped grid payloads the recorder wrote.

    Snapshots are keyed by occupancy digest, which is also how the recorder
    de-duplicated them, so "the snapshot in force at tick N" resolves to exactly
    the bytes that were published -- not to a re-derived approximation.
    """

    def __init__(self, input_dir, snapshot_records):
        self._input_dir = input_dir
        self._cache = {}
        self._by_digest = {}
        self._missing_payload = set()
        for record in snapshot_records:
            digest = record.get("occupancy_digest")
            if digest is None:
                continue
            self._by_digest.setdefault(digest, record)
            if not record.get("payload"):
                self._missing_payload.add(digest)

    @property
    def digests(self):
        return list(self._by_digest.keys())

    def record(self, digest):
        return self._by_digest.get(digest)

    def missing_payload(self, digest):
        return digest in self._missing_payload or digest not in self._by_digest

    def payload(self, digest):
        """Full snapshot payload, or ``None`` when it was truncated away."""
        if digest in self._cache:
            return self._cache[digest]
        record = self._by_digest.get(digest)
        if record is None or not record.get("payload"):
            self._cache[digest] = None
            return None
        path = os.path.join(self._input_dir, record["payload"])
        if not os.path.exists(path):
            self._cache[digest] = None
            return None
        payload = load_gzip_json(path)
        self._cache[digest] = payload
        return payload

    def grid(self, digest):
        """``fe.Grid`` for a snapshot, carrying its identity for the artifact."""
        payload = self.payload(digest)
        if payload is None:
            return None
        identity = payload.get("identity", {})
        info = identity.get("info", {})
        return fe.Grid(
            width=int(info["width"]),
            height=int(info["height"]),
            resolution=float(info["resolution"]),
            origin_x=float(info["origin_x"]),
            origin_y=float(info["origin_y"]),
            origin_yaw=float(info["origin_yaw"]),
            data=payload["occupancy"],
            frame_id=str(identity.get("frame_id", "")),
            identity={
                "stamp_identity": identity.get("stamp_identity"),
                "source_stamp_identity": identity.get("source_stamp_identity"),
                "source_generation": identity.get("source_generation"),
                "publication_sequence": identity.get("publication_sequence"),
                "localization_epoch": identity.get("localization_epoch"),
                "occupancy_digest": identity.get("occupancy_digest"),
                "ready": identity.get("ready"),
                "unknown_is_obstacle": identity.get("unknown_is_obstacle"),
                "occupied_value_threshold": identity.get("occupied_value_threshold"),
            },
        )


def signed_distance_at(payload, grid, wx, wy):
    """Signed distance from the RC-ESDF at a world point, or ``None``.

    This is what separates "inflated near an obstacle" from "hard obstacle cell":
    the occupancy verdict alone cannot say how much room was left.
    """
    if payload is None:
        return None
    distances = payload.get("signed_distance_m")
    if not distances:
        return None
    index = fe.world_to_grid(grid, wx, wy)
    if index is None:
        return None
    flat = index[1] * grid.width + index[0]
    if flat < 0 or flat >= len(distances):
        return None
    return float(distances[flat])


def obstacle_normal_at(payload, grid, wx, wy):
    """Local outward normal of the blocking surface, from the ESDF gradient.

    The task asks for the error measured against the wall normal.  Rather than
    hard-coding a wall direction -- which would be a scenario-specific bypass --
    the direction is derived from the signed-distance field around the conflict:
    the gradient of distance-to-obstacle points *away* from the nearest surface,
    so it is the outward normal at that point.

    Returns ``None`` when the field is unavailable or locally flat, so a caller
    reports "normal unavailable" instead of projecting onto a fabricated axis.
    """
    if payload is None or not payload.get("signed_distance_m"):
        return None
    index = fe.world_to_grid(grid, wx, wy)
    if index is None:
        return None
    distances = payload["signed_distance_m"]

    def value(ix, iy):
        if ix < 0 or iy < 0 or ix >= grid.width or iy >= grid.height:
            return None
        flat = iy * grid.width + ix
        if flat < 0 or flat >= len(distances):
            return None
        return float(distances[flat])

    ix, iy = index
    # Central differences where both neighbours exist, one-sided at the border.
    def derivative(plus, minus, span):
        if plus is not None and minus is not None:
            return (plus - minus) / (span * grid.resolution)
        centre = value(ix, iy)
        if centre is None:
            return None
        if plus is not None:
            return (plus - centre) / grid.resolution
        if minus is not None:
            return (centre - minus) / grid.resolution
        return None

    d_local_x = derivative(value(ix + 1, iy), value(ix - 1, iy), 2.0)
    d_local_y = derivative(value(ix, iy + 1), value(ix, iy - 1), 2.0)
    if d_local_x is None or d_local_y is None:
        return None
    # The gradient is computed on grid axes, so rotate it out by the origin yaw
    # to express the normal in the grid frame's world axes.
    cos_yaw = math.cos(grid.origin_yaw)
    sin_yaw = math.sin(grid.origin_yaw)
    nx = cos_yaw * d_local_x - sin_yaw * d_local_y
    ny = sin_yaw * d_local_x + cos_yaw * d_local_y
    norm = math.hypot(nx, ny)
    if norm <= 1e-9:
        return None
    return {"x": nx / norm, "y": ny / norm, "gradient_magnitude": norm}


class LayerStore(object):
    """Per-source layer payloads, so a conflicting cell can be attributed.

    The fused planning grid cannot say *which* source set a cell.  The recorder
    persists one payload per distinct content per topic; here the payload in force
    at a given monotonic instant is the newest one at or before it.
    """

    def __init__(self, input_dir):
        self._payloads = {}
        directory = os.path.join(input_dir, "layers")
        if not os.path.isdir(directory):
            return
        for name in sorted(os.listdir(directory)):
            if not name.endswith(".json.gz"):
                continue
            payload = load_gzip_json(os.path.join(directory, name))
            topic = payload.get("topic", name)
            self._payloads.setdefault(topic, []).append(payload)
        for entries in self._payloads.values():
            entries.sort(key=lambda item: item.get("mono_s", 0.0))

    @property
    def topics(self):
        return sorted(self._payloads.keys())

    def in_force(self, topic, mono_s):
        entries = self._payloads.get(topic)
        if not entries:
            return None
        chosen = None
        for entry in entries:
            if entry.get("mono_s", 0.0) <= mono_s:
                chosen = entry
            else:
                break
        # A layer latched before the recorder started carries a mono_s at or near
        # zero; if every payload is later than the instant asked for, the earliest
        # is still the only evidence available and is reported with its own time
        # so the artifact shows the ordering rather than hiding it.
        return chosen if chosen is not None else entries[0]

    def provenance(self, mono_s, grid_frame, wx, wy, params):
        """Per-source value at one world point in the grid frame."""
        report = {}
        for topic in self.topics:
            entry = self.in_force(topic, mono_s)
            if entry is None:
                report[topic] = {"available": False}
                continue
            info = entry.get("info", {})
            layer_frame = entry.get("frame_id", "")
            record = {
                "available": True,
                "frame_id": layer_frame,
                "digest": entry.get("digest"),
                "stamp_identity": entry.get("stamp_identity"),
                "mono_s": entry.get("mono_s"),
            }
            if not frames_agree(layer_frame, grid_frame):
                # Never look a cell up across frames silently: the answer would be
                # geometrically meaningless while looking authoritative.
                record["frame_unpaired"] = True
                record["expected_frame"] = grid_frame
                report[topic] = record
                continue
            layer_grid = fe.Grid(
                width=int(info["width"]), height=int(info["height"]),
                resolution=float(info["resolution"]),
                origin_x=float(info["origin_x"]), origin_y=float(info["origin_y"]),
                origin_yaw=float(info["origin_yaw"]),
                data=entry.get("data", []), frame_id=layer_frame)
            index = fe.world_to_grid(layer_grid, wx, wy)
            if index is None:
                record["class"] = "outside_layer"
                report[topic] = record
                continue
            record.update(fe.classify_cell(layer_grid, params, index[0], index[1]))
            report[topic] = record
        return report


def footprint_clearance(payload, grid, params, pose):
    """Smallest RC-ESDF signed distance over the oriented footprint samples.

    The same sample lattice the gate uses is reused, so the reported clearance is
    the clearance of the shape that was judged -- not of the centre point, which
    would overstate the room available by up to the footprint half-diagonal.
    """
    if payload is None or not payload.get("signed_distance_m"):
        return {"min_clearance_m": None, "evaluated": False,
                "reason": "no signed distance field in snapshot payload"}
    samples = fe.make_rectangular_footprint_samples(
        params.length, params.width, params.safety_margin, grid.resolution)
    cos_yaw = math.cos(pose[2])
    sin_yaw = math.sin(pose[2])
    best = None
    best_point = None
    off_grid = 0
    for offset_x, offset_y in samples:
        wx = pose[0] + cos_yaw * offset_x - sin_yaw * offset_y
        wy = pose[1] + sin_yaw * offset_x + cos_yaw * offset_y
        distance = signed_distance_at(payload, grid, wx, wy)
        if distance is None:
            off_grid += 1
            continue
        if best is None or distance < best:
            best = distance
            best_point = (wx, wy)
    return {
        "min_clearance_m": best,
        "evaluated": best is not None,
        "samples_off_grid": off_grid,
        "at_x": best_point[0] if best_point else None,
        "at_y": best_point[1] if best_point else None,
    }


def summarize_verdict(verdict, payload, grid, params, layers, mono_s):
    """Compact, evidence-bearing form of one footprint_evaluator verdict."""
    summary = {
        "safe": verdict["safe"],
        "pose_count": verdict["pose_count"],
        "discrete_collisions": verdict["discrete_collision_count"],
        "swept_collisions": verdict["swept_collision_count"],
        "footprint_collisions":
            verdict["discrete_collision_count"] + verdict["swept_collision_count"],
        "discrete_samples_checked": verdict["discrete_samples_checked"],
        "swept_samples_checked": verdict["swept_samples_checked"],
        "swept_segments_checked": verdict["swept_segments_checked"],
        "snapshot_identity": grid.identity,
        "first_collision": None,
    }
    if verdict.get("reason"):
        summary["reason"] = verdict["reason"]
    first = verdict.get("first_collision")
    if first is None:
        return summary
    detail = {
        "trajectory_index": first["trajectory_index"],
        "swept": first["swept"],
        "segment_fraction": first["segment_fraction"],
        "center_x": first["center_x"],
        "center_y": first["center_y"],
        "center_yaw": first["center_yaw"],
        "sample_x": first["sample_x"],
        "sample_y": first["sample_y"],
        "cell": first["cell"],
        "clearance": footprint_clearance(
            payload, grid, params,
            (first["center_x"], first["center_y"], first["center_yaw"])),
        "signed_distance_at_sample_m":
            signed_distance_at(payload, grid, first["sample_x"], first["sample_y"]),
        "obstacle_normal":
            obstacle_normal_at(payload, grid, first["sample_x"], first["sample_y"]),
    }
    if layers is not None and mono_s is not None:
        detail["provenance"] = layers.provenance(
            mono_s, grid.frame_id, first["sample_x"], first["sample_y"], params)
    summary["first_collision"] = detail
    return summary


def first_sample_for_reference(samples, digest, stamp_identity=None):
    """Earliest tick that observed one committed reference in force.

    Geometry digests are intentionally stable across republishes.  The ROS stamp
    is therefore part of the pairing key; otherwise a replan that reuses the same
    poses could be judged against the first replan's snapshot.  Artifacts written
    before the stamp field existed remain readable when they contain only one
    un-stamped match.
    """
    matches = [sample for sample in samples
               if sample.get("reference_digest") == digest]
    if stamp_identity is None:
        return matches[0] if matches else None
    stamped = [sample for sample in matches
               if sample.get("reference_stamp_identity") == stamp_identity]
    if stamped:
        return stamped[0]
    legacy = [sample for sample in matches
              if "reference_stamp_identity" not in sample]
    return legacy[0] if len(legacy) == 1 else None


def analyze_reference_at_publish(samples, references, store, layers, params,
                                 reference_topic):
    """Q1: was each committed reference collision-free when it was published?

    The snapshot used is the one the recorder saw in force at the first tick that
    observed the reference, i.e. the map the planner had just validated against.
    Judging a reference against a *later* snapshot would answer Q3 while looking
    like an answer to Q1, so the two are kept apart deliberately.
    """
    results = []
    for reference in references:
        if reference.get("source") != reference_topic:
            continue
        digest = reference.get("digest")
        sample = first_sample_for_reference(
            samples, digest, reference.get("stamp_identity"))
        entry = {
            "reference_index": reference.get("reference_index"),
            "reference_digest": digest,
            "reference_stamp_identity": reference.get("stamp_identity"),
            "reference_frame_id": reference.get("frame_id"),
            "pose_count": reference.get("pose_count"),
            "recorded_mono_s": reference.get("mono_s"),
        }
        if sample is None:
            entry["evaluated"] = False
            entry["reason"] = "no tick observed this reference in force"
            results.append(entry)
            continue
        entry["observed_tick"] = sample.get("tick")
        entry["observed_mono_s"] = sample.get("mono_s")
        snapshot = sample.get("snapshot")
        if not snapshot:
            entry["evaluated"] = False
            entry["reason"] = "no snapshot in force at that tick"
            results.append(entry)
            continue
        snapshot_digest = snapshot.get("occupancy_digest")
        grid = store.grid(snapshot_digest)
        if grid is None:
            entry["evaluated"] = False
            entry["reason"] = "snapshot payload unavailable (truncated or missing)"
            entry["snapshot_identity"] = snapshot
            results.append(entry)
            continue
        poses = [tuple(pose) for pose in reference.get("poses", [])]
        transform = sample.get("map_from_odom")
        if frames_agree(reference.get("frame_id"), grid.frame_id):
            grid_poses = poses
            entry["transform_applied"] = False
        else:
            grid_poses = transform_poses(transform, poses)
            entry["transform_applied"] = True
            entry["map_from_odom"] = transform
        if grid_poses is None:
            entry["evaluated"] = False
            entry["frame_unpaired"] = True
            entry["reason"] = (
                "reference is in %r, snapshot is in %r, and no %s <- %s transform "
                "was recorded at that tick" % (
                    reference.get("frame_id"), grid.frame_id,
                    grid.frame_id, reference.get("frame_id")))
            results.append(entry)
            continue
        verdict = fe.check(grid, params, grid_poses, max_collisions=64)
        entry["evaluated"] = True
        entry["verdict"] = summarize_verdict(
            verdict, store.payload(snapshot_digest), grid, params, layers,
            sample.get("mono_s"))
        results.append(entry)
    return results


def actual_pose_series(samples):
    """Ticks that carry a distinct localization pose, in order.

    Consecutive ticks that re-sample the same ``/localization`` message are
    collapsed by stamp identity: keeping them would inflate the discrete sample
    count and invent zero-length swept segments, both of which would make the
    actual sequence look better checked than it was.
    """
    series = []
    previous_stamp = None
    for sample in samples:
        actual = sample.get("actual")
        if not actual:
            continue
        stamp = actual.get("stamp_identity")
        if stamp is not None and stamp == previous_stamp:
            continue
        previous_stamp = stamp
        series.append(sample)
    return series


def tick_context(sample):
    """The same-clock spine for one tick, as the report needs to cite it."""
    actual = sample.get("actual") or {}
    telemetry = sample.get("telemetry") or {}
    return {
        "tick": sample.get("tick"),
        "mono_s": sample.get("mono_s"),
        "actual_x": actual.get("x"),
        "actual_y": actual.get("y"),
        "actual_yaw": actual.get("yaw"),
        "actual_frame": actual.get("frame_id"),
        "actual_stamp_identity": actual.get("stamp_identity"),
        "actual_vx_body": actual.get("vx_body"),
        "actual_vy_body": actual.get("vy_body"),
        "actual_wz_body": actual.get("wz_body"),
        "actual_speed_mps": (
            math.hypot(actual["vx_body"], actual["vy_body"])
            if actual.get("vx_body") is not None else None),
        "selected": sample.get("selected"),
        "selected_age_s_mono": sample.get("selected_age_s_mono"),
        "autonomy_raw": sample.get("autonomy_raw"),
        "motion_control": sample.get("motion_control"),
        "motion_control_age_s_mono": sample.get("motion_control_age_s_mono"),
        "localization_age_s_mono": sample.get("actual_age_s_mono"),
        "reference_age_s_mono": sample.get("reference_age_s_mono"),
        "emergency_stop": sample.get("emergency_stop"),
        "map_ready": sample.get("map_ready"),
        "tracking_error": sample.get("tracking_error"),
        "reference_digest": sample.get("reference_digest"),
        "planner_status": sample.get("planner_status"),
        "execution_mode": (sample.get("execution") or {}).get("mode"),
        "execution_command_sequence":
            (sample.get("execution") or {}).get("command_sequence"),
        "snapshot_identity": sample.get("snapshot"),
        "telemetry_contact_violation_count":
            telemetry.get("contact_violation_count"),
        "telemetry_max_contact_force": telemetry.get("max_contact_force"),
        "drive_speed_saturation_count":
            telemetry.get("drive_speed_saturation_count"),
        "drive_acceleration_saturation_count":
            telemetry.get("drive_acceleration_saturation_count"),
        "steer_rate_saturation_count": telemetry.get("steer_rate_saturation_count"),
    }


def analyze_actual_tracking(samples, store, layers, params):
    """Q2: did the driven pose sequence leave the verified envelope?

    Every tick's pose is judged against the snapshot that was in force *at that
    tick*, and the motion between consecutive ticks is swept, so a conflict that
    exists only between two 50 Hz samples is still caught.  The tracking error the
    recorder computed against the committed reference is carried alongside, which
    is what makes "the actual left the reference's envelope" a measurement rather
    than an inference.
    """
    series = actual_pose_series(samples)
    result = {
        "ticks_with_pose": len(series),
        "ticks_evaluated": 0,
        "ticks_unpaired_frame": 0,
        "ticks_without_snapshot": 0,
        "ticks_without_snapshot_payload": 0,
        "colliding_ticks": 0,
        "swept_colliding_segments": 0,
        "first_discrete_conflict": None,
        "first_swept_conflict": None,
        "min_clearance": None,
        "max_abs_lateral_error_m": None,
        "max_abs_longitudinal_error_m": None,
        "max_abs_yaw_error_rad": None,
        "max_speed_mps": None,
    }
    previous = None
    for sample in series:
        actual = sample["actual"]
        pose = (actual["x"], actual["y"], actual["yaw"])
        error = sample.get("tracking_error") or {}
        if not error.get("frame_unpaired"):
            for key, field in (("max_abs_lateral_error_m", "lateral_m"),
                               ("max_abs_longitudinal_error_m", "longitudinal_m"),
                               ("max_abs_yaw_error_rad", "yaw_error_rad")):
                value = error.get(field)
                if value is None:
                    continue
                current = result[key]
                if current is None or abs(value) > current:
                    result[key] = abs(value)
        speed = math.hypot(actual.get("vx_body", 0.0), actual.get("vy_body", 0.0))
        if result["max_speed_mps"] is None or speed > result["max_speed_mps"]:
            result["max_speed_mps"] = speed

        snapshot = sample.get("snapshot")
        if not snapshot:
            result["ticks_without_snapshot"] += 1
            previous = None
            continue
        digest = snapshot.get("occupancy_digest")
        grid = store.grid(digest)
        if grid is None:
            result["ticks_without_snapshot_payload"] += 1
            previous = None
            continue
        if frames_agree(actual.get("frame_id"), grid.frame_id):
            grid_pose = pose
        else:
            grid_pose = transform_pose(sample.get("map_from_odom"), pose)
        if grid_pose is None:
            result["ticks_unpaired_frame"] += 1
            previous = None
            continue

        payload = store.payload(digest)
        clearance = footprint_clearance(payload, grid, params, grid_pose)
        if clearance.get("min_clearance_m") is not None:
            current = result["min_clearance"]
            if current is None or \
                    clearance["min_clearance_m"] < current["min_clearance_m"]:
                record = dict(clearance)
                record["tick"] = sample.get("tick")
                record["mono_s"] = sample.get("mono_s")
                record["snapshot_identity"] = grid.identity
                result["min_clearance"] = record

        result["ticks_evaluated"] += 1
        verdict = fe.check(grid, params, [grid_pose], max_collisions=8)
        if verdict["discrete_collision_count"] > 0:
            result["colliding_ticks"] += 1
            if result["first_discrete_conflict"] is None:
                conflict = tick_context(sample)
                conflict["grid_pose"] = {"x": grid_pose[0], "y": grid_pose[1],
                                         "yaw": grid_pose[2]}
                conflict["map_from_odom"] = sample.get("map_from_odom")
                conflict["verdict"] = summarize_verdict(
                    verdict, payload, grid, params, layers, sample.get("mono_s"))
                conflict["clearance"] = clearance
                result["first_discrete_conflict"] = conflict

        if previous is not None:
            # Swept over the motion between two consecutive samples, judged
            # against the snapshot in force at the later of the two.
            swept = fe.check(grid, params, [previous, grid_pose], max_collisions=8)
            if swept["swept_collision_count"] > 0:
                result["swept_colliding_segments"] += 1
                if result["first_swept_conflict"] is None:
                    conflict = tick_context(sample)
                    conflict["segment_start"] = {
                        "x": previous[0], "y": previous[1], "yaw": previous[2]}
                    conflict["segment_end"] = {
                        "x": grid_pose[0], "y": grid_pose[1], "yaw": grid_pose[2]}
                    conflict["verdict"] = summarize_verdict(
                        swept, payload, grid, params, layers, sample.get("mono_s"))
                    result["first_swept_conflict"] = conflict
        previous = grid_pose
    return result


def analyze_snapshot_flip(probes, store, params):
    """Q3: does one fixed pose change verdict as the snapshot sequence advances?

    Each probe pose is held constant and replayed against *every* distinct
    snapshot the recorder captured, in publication order.  A pose that is free
    under one snapshot identity and occupied under a later one is a map-update
    effect, not a tracking or planning error, and the transition is reported with
    both snapshot identities so the owner is unambiguous.
    """
    records = store.digests
    ordered = sorted(
        (record for record in (store.record(digest) for digest in records)
         if record is not None),
        key=lambda item: (item.get("publication_sequence") or 0,
                          item.get("mono_s") or 0.0))
    results = []
    for probe in probes:
        pose = probe["pose"]
        entry = {
            "label": probe["label"],
            "pose": {"x": pose[0], "y": pose[1], "yaw": pose[2]},
            "frame_id": probe.get("frame_id"),
            "source_tick": probe.get("tick"),
            "timeline": [],
            "flipped_free_to_occupied": False,
            "flipped_occupied_to_free": False,
            "transitions": [],
        }
        previous = None
        for record in ordered:
            digest = record.get("occupancy_digest")
            grid = store.grid(digest)
            if grid is None:
                entry["timeline"].append({
                    "occupancy_digest": digest,
                    "publication_sequence": record.get("publication_sequence"),
                    "source_generation": record.get("source_generation"),
                    "evaluated": False,
                    "reason": "payload unavailable",
                })
                continue
            if not frames_agree(probe.get("frame_id"), grid.frame_id):
                entry["timeline"].append({
                    "occupancy_digest": digest,
                    "publication_sequence": record.get("publication_sequence"),
                    "evaluated": False,
                    "frame_unpaired": True,
                    "probe_frame": probe.get("frame_id"),
                    "snapshot_frame": grid.frame_id,
                })
                continue
            verdict = fe.check(grid, params, [pose], max_collisions=4)
            payload = store.payload(digest)
            step = {
                "occupancy_digest": digest,
                "publication_sequence": record.get("publication_sequence"),
                "source_generation": record.get("source_generation"),
                "localization_epoch": record.get("localization_epoch"),
                "stamp_identity": record.get("stamp_identity"),
                "mono_s": record.get("mono_s"),
                "evaluated": True,
                "occupied": verdict["discrete_collision_count"] > 0,
                "discrete_collisions": verdict["discrete_collision_count"],
                "min_clearance_m":
                    footprint_clearance(payload, grid, params,
                                        pose).get("min_clearance_m"),
            }
            first = verdict.get("first_collision")
            if first is not None:
                step["first_cell"] = first["cell"]
                step["first_sample_x"] = first["sample_x"]
                step["first_sample_y"] = first["sample_y"]
            entry["timeline"].append(step)
            if previous is not None and previous["occupied"] != step["occupied"]:
                transition = {
                    "from_digest": previous["occupancy_digest"],
                    "from_publication_sequence": previous["publication_sequence"],
                    "from_source_generation": previous["source_generation"],
                    "from_occupied": previous["occupied"],
                    "to_digest": step["occupancy_digest"],
                    "to_publication_sequence": step["publication_sequence"],
                    "to_source_generation": step["source_generation"],
                    "to_occupied": step["occupied"],
                }
                entry["transitions"].append(transition)
                if step["occupied"]:
                    entry["flipped_free_to_occupied"] = True
                else:
                    entry["flipped_occupied_to_free"] = True
            previous = step
        results.append(entry)
    return results


def analyze_stop_envelope(samples, events, moving_speed, rest_speed, command_eps):
    """Braking / emergency-stop instant and the displacement to rest.

    Three candidate onsets are reported rather than one, because they answer
    different questions: the command going to zero bounds what the planner asked
    for, the emergency-stop assertion bounds when the safety path engaged, and the
    last moving sample bounds what the chassis actually did.  Picking only one
    would hide a latency between them.
    """
    series = actual_pose_series(samples)
    report = {
        "moving_speed_threshold_mps": moving_speed,
        "rest_speed_threshold_mps": rest_speed,
        "command_zero_epsilon": command_eps,
        "reached_rest": False,
        "onsets": {},
        "emergency_stop_events": [],
    }
    for event in events:
        if event.get("kind") == "emergency_stop":
            report["emergency_stop_events"].append({
                "mono_s": event.get("mono_s"),
                "value": (event.get("payload") or {}).get("value"),
            })
    if not series:
        report["reason"] = "no localization samples"
        return report

    def speed_of(sample):
        actual = sample.get("actual") or {}
        return math.hypot(actual.get("vx_body", 0.0) or 0.0,
                          actual.get("vy_body", 0.0) or 0.0)

    def command_magnitude(sample):
        selected = sample.get("selected")
        if not selected:
            return None
        return (abs(selected.get("vx", 0.0)) + abs(selected.get("vy", 0.0)) +
                abs(selected.get("wz", 0.0)))

    last_moving = None
    for index, sample in enumerate(series):
        if speed_of(sample) > moving_speed:
            last_moving = index
    if last_moving is None:
        report["reason"] = "the robot never exceeded the moving-speed threshold"
        return report

    rest_index = None
    for index in range(last_moving, len(series)):
        if speed_of(series[index]) <= rest_speed:
            rest_index = index
            break
    if rest_index is None:
        report["reason"] = "never came to rest inside the recording"
        report["last_moving"] = tick_context(series[last_moving])
        return report
    report["reached_rest"] = True
    rest = series[rest_index]
    rest_actual = rest["actual"]
    report["rest"] = tick_context(rest)

    def onset_report(label, index):
        if index is None:
            return {"found": False}
        sample = series[index]
        actual = sample["actual"]
        return {
            "found": True,
            "context": tick_context(sample),
            "displacement_to_rest_m": math.hypot(
                rest_actual["x"] - actual["x"], rest_actual["y"] - actual["y"]),
            "yaw_change_to_rest_rad": fe.normalize_angle(
                rest_actual["yaw"] - actual["yaw"]),
            "duration_to_rest_s_mono":
                (rest.get("mono_s") or 0.0) - (sample.get("mono_s") or 0.0),
        }

    # Last sample whose selected command still requested motion.
    command_index = None
    for index in range(0, rest_index + 1):
        magnitude = command_magnitude(series[index])
        if magnitude is not None and magnitude > command_eps:
            command_index = index
    # First emergency-stop assertion at or before rest.
    estop_index = None
    for index in range(0, rest_index + 1):
        if series[index].get("emergency_stop"):
            estop_index = index
            break
    report["onsets"] = {
        "last_nonzero_selected_command": onset_report(
            "last_nonzero_selected_command", command_index),
        "first_emergency_stop_true": onset_report(
            "first_emergency_stop_true", estop_index),
        "last_moving_sample": onset_report("last_moving_sample", last_moving),
    }
    return report


def grid_pose_for_sample(sample, store):
    """One tick's actual pose expressed in its snapshot's frame, or ``None``."""
    actual = sample.get("actual")
    snapshot = sample.get("snapshot")
    if not actual or not snapshot:
        return None
    grid = store.grid(snapshot.get("occupancy_digest"))
    if grid is None:
        return None
    pose = (actual["x"], actual["y"], actual["yaw"])
    if frames_agree(actual.get("frame_id"), grid.frame_id):
        return pose, grid.frame_id
    transformed = transform_pose(sample.get("map_from_odom"), pose)
    if transformed is None:
        return None
    return transformed, grid.frame_id


def find_sample(samples, tick):
    for sample in samples:
        if sample.get("tick") == tick:
            return sample
    return None


def build_probes(samples, store, tracking, references_result, stop):
    """Fixed poses worth replaying against the whole snapshot sequence.

    The set is derived from what the run actually did -- where the actual first
    conflicted, where it came to rest, and where the last committed reference
    started and first conflicted -- rather than from any hard-coded location.
    """
    probes = []
    conflict = tracking.get("first_discrete_conflict")
    if conflict and conflict.get("grid_pose"):
        pose = conflict["grid_pose"]
        probes.append({
            "label": "actual_pose_at_first_discrete_conflict",
            "pose": (pose["x"], pose["y"], pose["yaw"]),
            "frame_id": (conflict.get("verdict") or {}).get(
                "snapshot_identity", {}).get("frame_id"),
            "tick": conflict.get("tick"),
        })
    rest = (stop or {}).get("rest")
    if rest and rest.get("tick") is not None:
        sample = find_sample(samples, rest["tick"])
        if sample is not None:
            resolved = grid_pose_for_sample(sample, store)
            if resolved is not None:
                pose, frame_id = resolved
                probes.append({
                    "label": "actual_rest_pose",
                    "pose": pose,
                    "frame_id": frame_id,
                    "tick": rest.get("tick"),
                })
    for entry in reversed(references_result):
        verdict = entry.get("verdict")
        if not verdict:
            continue
        first = verdict.get("first_collision")
        if first is None:
            continue
        probes.append({
            "label": "reference_pose_at_first_collision",
            "pose": (first["center_x"], first["center_y"], first["center_yaw"]),
            "frame_id": (verdict.get("snapshot_identity") or {}).get("frame_id"),
            "tick": entry.get("observed_tick"),
        })
        break
    # Deduplicate by rounded pose so the timeline is not recomputed for a probe
    # that two sources happen to agree on.
    unique = []
    seen = set()
    for probe in probes:
        if probe.get("frame_id") is None:
            continue
        key = (probe["label"], round(probe["pose"][0], 6),
               round(probe["pose"][1], 6), round(probe["pose"][2], 6))
        if key in seen:
            continue
        seen.add(key)
        unique.append(probe)
    return unique


def answer_questions(references_result, tracking, flips):
    """The three falsifiable answers, each with the routing they imply."""
    evaluated = [entry for entry in references_result if entry.get("evaluated")]
    unsafe = [entry for entry in evaluated
              if not (entry.get("verdict") or {}).get("safe", True)]
    answers = {
        "q1_reference_collision_free_at_publish": {
            "references_evaluated": len(evaluated),
            "references_not_evaluated":
                len(references_result) - len(evaluated),
            "references_with_collisions": len(unsafe),
            "answer": ("unknown" if not evaluated else
                       ("no" if unsafe else "yes")),
            "routing": ("audit the final revalidation snapshot, sampling "
                        "continuity and commit timing"
                        if unsafe else
                        "no reference-side violation to route"),
        },
        "q2_actual_left_reference_envelope": {
            "ticks_evaluated": tracking.get("ticks_evaluated"),
            "colliding_ticks": tracking.get("colliding_ticks"),
            "swept_colliding_segments": tracking.get("swept_colliding_segments"),
            "max_abs_lateral_error_m": tracking.get("max_abs_lateral_error_m"),
            "max_abs_longitudinal_error_m":
                tracking.get("max_abs_longitudinal_error_m"),
            "max_abs_yaw_error_rad": tracking.get("max_abs_yaw_error_rad"),
            "answer": ("unknown" if not tracking.get("ticks_evaluated") else
                       ("yes" if (tracking.get("colliding_ticks") or
                                  tracking.get("swept_colliding_segments"))
                        else "no")),
            "routing": ("audit MPC tracking error, execution latency, "
                        "velocity/acceleration limiting and the stopping envelope"
                        if (tracking.get("colliding_ticks") or
                            tracking.get("swept_colliding_segments"))
                        else "no actual-side violation to route"),
        },
    }
    flipped = [flip for flip in flips if flip.get("flipped_free_to_occupied")]
    answers["q3_same_pose_flipped_free_to_occupied"] = {
        "probes": len(flips),
        "probes_flipped_free_to_occupied": len(flipped),
        "flipped_labels": [flip["label"] for flip in flipped],
        "answer": ("unknown" if not flips else ("yes" if flipped else "no")),
        "routing": ("audit source generation / adapter heartbeat and the timing of "
                    "old-reference revocation"
                    if flipped else "no map-update flip to route"),
    }
    return answers


def collect_evidence_gaps(summary, truncations, tracking, store):
    """Everything the analysis could not establish, named explicitly.

    An unpaired tick or a truncated payload is a gap, not a pass.  They are listed
    so a report cannot present a partial evaluation as a clean one.
    """
    gaps = []
    if summary is None:
        gaps.append("summary.json missing: the recorder did not shut down cleanly")
    else:
        for topic in summary.get("topics_without_messages", []):
            gaps.append("topic delivered no messages: %s" % topic)
        if summary.get("snapshot_payloads_truncated"):
            gaps.append("snapshot payloads truncated: some snapshots cannot be "
                        "re-evaluated")
        if summary.get("layer_payloads_truncated"):
            gaps.append("layer payloads truncated: some provenance is unavailable")
        if not summary.get("motion_control_message_available"):
            gaps.append("manda_can_control/MotionCtrl unavailable: /motion_control "
                        "was recorded as null")
    for name, count in truncations.items():
        if count:
            gaps.append("%s has %d unparseable line(s): the recording was cut off "
                        "mid-write" % (name, count))
    if tracking.get("ticks_unpaired_frame"):
        gaps.append("%d tick(s) had no map<-odom transform and were excluded"
                    % tracking["ticks_unpaired_frame"])
    if tracking.get("ticks_without_snapshot"):
        gaps.append("%d tick(s) had no snapshot in force and were excluded"
                    % tracking["ticks_without_snapshot"])
    if tracking.get("ticks_without_snapshot_payload"):
        gaps.append("%d tick(s) referenced a snapshot whose payload was not saved"
                    % tracking["ticks_without_snapshot_payload"])
    if not store.digests:
        gaps.append("no snapshots were captured: nothing could be evaluated")
    gaps.append("physical contact is not evaluated here; the MuJoCo contact "
                "counter is separate evidence and remains unverified by this tool")
    return gaps


def parse_args(argv=None):
    parser = argparse.ArgumentParser(
        description="Join nav_tracking_recorder output to the footprint evaluator "
                    "and answer the reference/actual/map-update questions.")
    parser.add_argument("--input-dir", required=True,
                        help="a nav_tracking_recorder output directory")
    parser.add_argument("--output", help="write the full verdict JSON here")
    parser.add_argument("--reference-topic", default="/minco/reference_path",
                        help="which recorded reference source is the committed one")
    # Footprint geometry must come from the run profile.  There is no default that
    # is right for every robot, and silently using the header defaults would judge
    # the run with a footprint it never used.
    parser.add_argument("--length", type=float, required=True)
    parser.add_argument("--width", type=float, required=True)
    parser.add_argument("--safety-margin", type=float, required=True)
    parser.add_argument("--obstacle-value-threshold", type=int,
                        help="defaults to the threshold carried by the snapshot")
    parser.add_argument("--unknown-is-obstacle", dest="unknown_is_obstacle",
                        action="store_true", default=None,
                        help="defaults to the flag carried by the snapshot")
    parser.add_argument("--known-only", dest="unknown_is_obstacle",
                        action="store_false",
                        help="treat unknown as free, overriding the snapshot flag")
    parser.add_argument("--swept-max-corner-step-cells", type=float, default=0.5)
    parser.add_argument("--moving-speed-mps", type=float, default=0.05)
    parser.add_argument("--rest-speed-mps", type=float, default=0.01)
    parser.add_argument("--command-zero-epsilon", type=float, default=1e-6)
    return parser.parse_args(argv)


def resolve_params(args, snapshots):
    """Effective footprint parameters, preferring the snapshot's own flags.

    ``unknown_is_obstacle`` and the occupied threshold are published *inside* the
    snapshot, so the run's own values are used unless the caller overrides them.
    Guessing them would change the verdict without changing the run.
    """
    threshold = args.obstacle_value_threshold
    unknown = args.unknown_is_obstacle
    source = {"obstacle_value_threshold": "cli", "unknown_is_obstacle": "cli"}
    if snapshots:
        latest = snapshots[-1]
        if threshold is None:
            threshold = latest.get("occupied_value_threshold")
            source["obstacle_value_threshold"] = "snapshot"
        if unknown is None:
            unknown = latest.get("unknown_is_obstacle")
            source["unknown_is_obstacle"] = "snapshot"
    if threshold is None:
        threshold = fe.DEFAULT_OBSTACLE_VALUE_THRESHOLD
        source["obstacle_value_threshold"] = "evaluator default"
    if unknown is None:
        unknown = fe.DEFAULT_UNKNOWN_IS_OBSTACLE
        source["unknown_is_obstacle"] = "evaluator default"
    params = fe.FootprintParams(
        length=args.length, width=args.width, safety_margin=args.safety_margin,
        obstacle_value_threshold=int(threshold),
        unknown_is_obstacle=bool(unknown),
        swept_max_corner_step_cells=args.swept_max_corner_step_cells)
    return params, source


def main(argv=None):
    args = parse_args(argv)
    input_dir = args.input_dir
    samples, samples_truncated = load_jsonl(os.path.join(input_dir, "samples.jsonl"))
    references, references_truncated = load_jsonl(
        os.path.join(input_dir, "references.jsonl"))
    snapshots, snapshots_truncated = load_jsonl(
        os.path.join(input_dir, "snapshots.jsonl"))
    events, events_truncated = load_jsonl(os.path.join(input_dir, "events.jsonl"))
    summary = load_json(os.path.join(input_dir, "summary.json"))

    if not samples:
        print("RESULT: no samples.jsonl records in %s" % input_dir)
        return 2

    store = PayloadStore(input_dir, snapshots)
    layers = LayerStore(input_dir)
    params, params_source = resolve_params(args, snapshots)

    references_result = analyze_reference_at_publish(
        samples, references, store, layers, params, args.reference_topic)
    tracking = analyze_actual_tracking(samples, store, layers, params)
    stop = analyze_stop_envelope(
        samples, events, args.moving_speed_mps, args.rest_speed_mps,
        args.command_zero_epsilon)
    probes = build_probes(samples, store, tracking, references_result, stop)
    flips = analyze_snapshot_flip(probes, store, params)
    answers = answer_questions(references_result, tracking, flips)
    gaps = collect_evidence_gaps(
        summary,
        {"samples.jsonl": samples_truncated,
         "references.jsonl": references_truncated,
         "snapshots.jsonl": snapshots_truncated,
         "events.jsonl": events_truncated},
        tracking, store)

    report = {
        "input_dir": os.path.abspath(input_dir),
        "recorder_summary": summary,
        "footprint_params": params.as_dict(),
        "footprint_params_source": params_source,
        "reference_topic": args.reference_topic,
        "snapshot_count": len(store.digests),
        "layer_topics": layers.topics,
        "answers": answers,
        "reference_at_publish": references_result,
        "actual_tracking": tracking,
        "snapshot_flip": flips,
        "stop_envelope": stop,
        "evidence_gaps": gaps,
    }
    text = json.dumps(report, indent=2, sort_keys=True)
    if args.output:
        with open(args.output, "w", encoding="utf-8") as handle:
            handle.write(text + "\n")

    print("ANALYSIS: %s" % os.path.abspath(input_dir))
    print("  footprint %.3f x %.3f + %.3f m, threshold=%d, unknown_is_obstacle=%s"
          % (params.length, params.width, params.safety_margin,
             params.obstacle_value_threshold, params.unknown_is_obstacle))
    print("  snapshots=%d  references=%d  samples=%d"
          % (len(store.digests), len(references_result), len(samples)))
    for key in ("q1_reference_collision_free_at_publish",
                "q2_actual_left_reference_envelope",
                "q3_same_pose_flipped_free_to_occupied"):
        answer = answers[key]
        print("  %-42s %s" % (key + ":", answer["answer"]))
    conflict = tracking.get("first_discrete_conflict")
    if conflict:
        print("  first actual conflict at tick %s (mono %.3f s): "
              "pose=(%.4f, %.4f, yaw=%.4f)"
              % (conflict.get("tick"), conflict.get("mono_s") or 0.0,
                 conflict["grid_pose"]["x"], conflict["grid_pose"]["y"],
                 conflict["grid_pose"]["yaw"]))
    minimum = tracking.get("min_clearance")
    if minimum and minimum.get("min_clearance_m") is not None:
        print("  minimum footprint clearance %.4f m at tick %s"
              % (minimum["min_clearance_m"], minimum.get("tick")))
    if stop.get("reached_rest"):
        for label, onset in sorted(stop.get("onsets", {}).items()):
            if not onset.get("found"):
                continue
            print("  stop from %-32s %.4f m over %.3f s"
                  % (label + ":", onset["displacement_to_rest_m"],
                     onset["duration_to_rest_s_mono"]))
    for gap in gaps:
        print("  GAP: %s" % gap)
    if args.output:
        print("  wrote %s" % args.output)

    if not tracking.get("ticks_evaluated"):
        print("RESULT: nav tracking analysis INCOMPLETE (no tick could be "
              "evaluated against a snapshot)")
        return 2
    conflicts = (tracking.get("colliding_ticks") or 0) + \
        (tracking.get("swept_colliding_segments") or 0) + \
        sum(1 for entry in references_result
            if entry.get("evaluated") and
            not (entry.get("verdict") or {}).get("safe", True))
    if conflicts:
        print("RESULT: nav tracking analysis COMPLETE, conflicts found")
        return 1
    print("RESULT: nav tracking analysis COMPLETE, no conflicts found")
    return 0


if __name__ == "__main__":
    sys.exit(main())
