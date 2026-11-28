#!/usr/bin/env python3
"""Offline analyzer that turns nav_tracking_recorder output into a verdict.

``scripts/nav_tracking_recorder.py`` records, on one steady clock, the committed
reference next to the pose that was actually driven, the command that was
selected, the emergency-stop state, and the immutable snapshot identity that was
in force at that instant.  This module joins those records to
``scripts/footprint_evaluator.py`` (a port of the planner's C++ footprint
checker) and answers the three questions that a run log
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

from array import array
import argparse
from collections import OrderedDict
import gzip
import json
import math
import os
import sys
import zlib

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import footprint_evaluator as fe  # noqa: E402
from nav_tracking_payload import execution_commands  # noqa: E402


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


SNAPSHOT_BIN_MAGIC = b"ATSSNAP1"


def load_gzip_json(path):
    with gzip.open(path, "rt") as handle:
        return json.load(handle)


def load_snapshot_payload(path):
    if path.endswith(".snap.zlib"):
        with open(path, "rb") as handle:
            raw = zlib.decompress(handle.read())
        magic_len = len(SNAPSHOT_BIN_MAGIC)
        if not raw.startswith(SNAPSHOT_BIN_MAGIC):
            raise ValueError(f"invalid snapshot binary header in {path}")
        meta_len = int.from_bytes(raw[magic_len:magic_len + 4], "little")
        meta_end = magic_len + 4 + meta_len
        identity = json.loads(raw[magic_len + 4:meta_end].decode("utf-8"))
        cell_count = int(identity["cell_count"])
        occ_end = meta_end + cell_count
        occ = array("b")
        occ.frombytes(raw[meta_end:occ_end])
        dist = array("f")
        dist.frombytes(raw[occ_end:occ_end + 4 * cell_count])
        return {
            "identity": identity,
            "occupancy": occ,
            "signed_distance_m": dist,
        }
    return load_gzip_json(path)


# ---- frames ---------------------------------------------------------------


def unit_quaternion(value):
    return (isinstance(value, (list, tuple)) and len(value) == 4 and
            all(isinstance(v, (float, int)) and math.isfinite(v) for v in value) and
            math.isclose(sum(v * v for v in value), 1.0, rel_tol=1e-3, abs_tol=1e-3))


def quaternion_yaw(q):
    x, y, z, w = q
    return math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))


def pose_se3_matches_planar(full, planar):
    return (isinstance(full, (list, tuple)) and len(full) == 7 and
            all(isinstance(v, (float, int)) and math.isfinite(v) for v in full) and
            unit_quaternion(full[3:]) and
            abs(full[0] - planar[0]) <= 1e-6 and abs(full[1] - planar[1]) <= 1e-6 and
            abs(fe.normalize_angle(quaternion_yaw(full[3:]) - planar[2])) <= 1e-6)


def transform_pose(transform, pose, pose_se3=None):
    """Apply a recorded ``map <- odom`` transform to an ``(x, y, yaw)`` pose.

    Returns ``None`` when the transform is missing, so a caller has to decide
    explicitly what to do about an unpaired tick instead of inheriting identity.
    """
    if transform is None:
        return None
    if "quaternion" in transform:
        q = transform["quaternion"]
        if (not unit_quaternion(q) or not pose_se3_matches_planar(pose_se3, pose) or
                not all(isinstance(transform.get(key), (float, int)) and
                        math.isfinite(transform[key]) for key in ("x", "y", "z"))):
            return None
        # q * [p, 0] * inverse(q), then compose the complete orientations.
        x, y, z, w = q
        px, py, pz = pose_se3[:3]
        tx, ty, tz = 2 * (y * pz - z * py), 2 * (z * px - x * pz), 2 * (x * py - y * px)
        ox, oy, oz, ow = pose_se3[3:]
        composed = (w * ox + x * ow + y * oz - z * oy,
                    w * oy - x * oz + y * ow + z * ox,
                    w * oz + x * oy - y * ox + z * ow,
                    w * ow - x * ox - y * oy - z * oz)
        return (transform["x"] + px + w * tx + y * tz - z * ty,
                transform["y"] + py + w * ty + z * tx - x * tz,
                fe.normalize_angle(quaternion_yaw(composed)))
    cos_yaw = math.cos(transform["yaw"])
    sin_yaw = math.sin(transform["yaw"])
    x, y, yaw = pose[0], pose[1], pose[2]
    return (
        transform["x"] + cos_yaw * x - sin_yaw * y,
        transform["y"] + sin_yaw * x + cos_yaw * y,
        fe.normalize_angle(transform["yaw"] + yaw),
    )


def transform_poses(transform, poses, poses_se3=None):
    if transform is None:
        return None
    if "quaternion" in transform:
        if poses_se3 is None or len(poses_se3) != len(poses):
            return None
        result = [transform_pose(transform, pose, full) for pose, full in zip(poses, poses_se3)]
        return None if any(pose is None for pose in result) else result
    return [transform_pose(transform, pose) for pose in poses]


def payload_key(snapshot):
    return snapshot.get("payload_key") or snapshot.get("occupancy_digest")


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
        self._cache = OrderedDict()
        self._by_digest = {}
        self._missing_payload = set()
        for record in snapshot_records:
            digest = payload_key(record)
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
            self._cache.move_to_end(digest)
            return self._cache[digest]
        record = self._by_digest.get(digest)
        if record is None or not record.get("payload"):
            return None
        path = os.path.join(self._input_dir, record["payload"])
        if not os.path.exists(path):
            return None
        payload = load_snapshot_payload(path)
        self._cache[digest] = payload
        while len(self._cache) > 8:
            self._cache.popitem(last=False)
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
            frame_id=str(identity.get("frame_id") or ""),
            identity={
                "frame_id": identity.get("frame_id"),
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
        self._cache = OrderedDict()
        directory = os.path.join(input_dir, "layers")
        if not os.path.isdir(directory):
            return
        for name in sorted(os.listdir(directory)):
            if not name.endswith(".json.gz"):
                continue
            path = os.path.join(directory, name)
            payload = load_gzip_json(path)
            payload.pop("data", None)
            payload["payload_file"] = path
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
        chosen = chosen if chosen is not None else entries[0]
        path = chosen["payload_file"]
        if path not in self._cache:
            self._cache[path] = load_gzip_json(path)
        self._cache.move_to_end(path)
        while len(self._cache) > 8:
            self._cache.popitem(last=False)
        return self._cache[path]

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


def first_sample_for_reference(samples, digest, stamp_identity, epoch, publication):
    """Earliest tick observing both the Path and its exact EXECUTE authority.

    Independent topic callbacks can expose the Path before its lease. Skip those
    ticks by identity, never by a nearest-time estimate or the latest snapshot.
    """
    for sample in samples:
        execution = sample.get("execution") or {}
        if (sample.get("reference_digest") == digest and
                sample.get("reference_stamp_identity") == stamp_identity and
                execution.get("mode") == 1 and
                execution.get("reference_stamp_identity") == stamp_identity and
                execution.get("localization_epoch") == epoch and
                execution.get("map_publication_sequence") == publication):
            return sample
    return None
