#!/usr/bin/env python3
"""Offline oriented-footprint evaluator that mirrors the planner's own checker.

This module is a deliberate re-implementation of
``minco_planner/src/safety/footprint_safety_checker.cpp`` plus
``minco_planner/include/minco_planner/safety/footprint_samples.hpp``.  It exists
so a *reference* trajectory and the *actual* driven pose sequence can be judged
by the same geometry against the same immutable snapshot, which the planner
itself never does (it only ever checks its own candidate).

Every rule below is copied from the C++ source, not approximated:

* ``makeRectangularFootprintSamples``: half extents are
  ``0.5 * max(0, size) + max(0, safety_margin)``; ``sample_spacing`` is
  ``max(0.02, grid.resolution)``; ``samples_x/y`` are
  ``max(2, ceil(2 * half / spacing))``; the lattice is ``(nx+1) * (ny+1)``
  inclusive of both borders, and an explicit centre sample is appended.
* ``worldToGrid``: the query is rotated into the origin pose's local axes by the
  origin yaw before dividing by resolution; a query outside ``[0, width)`` x
  ``[0, height)`` returns false, and the caller treats that as occupied.
* ``isOccupied``: an out-of-range index is occupied; ``value < 0`` defers to
  ``unknown_is_obstacle``; otherwise ``value >= obstacle_value_threshold``.
* ``sweptSubdivisions``: the bound is on maximum *corner* displacement, so pure
  rotation, lateral translation and diagonal motion share one rule.  Step size
  is ``resolution * max(1e-3, swept_max_corner_step_cells)``.
* the swept pass evaluates ``step in [1, subdivisions)`` and attributes every hit
  to the *segment start* index, with the interpolation fraction retained.
* yaw interpolation goes through ``normalize_angle`` on the delta, matching
  ``FootprintSafetyChecker::interpolate``.

A ``footprint_collisions == 0`` verdict from this evaluator is a geometric
statement about one snapshot.  It is not a claim about physical contact; the
MuJoCo rigid-body contact counter is separate evidence and neither substitutes
for the other.
"""

from __future__ import annotations

import argparse
import json
import math
import sys
from dataclasses import dataclass, field

# Defaults come from FootprintSafetyParams in footprint_safety_checker.hpp.
# They are NOT the RMUC values; callers must pass the effective run parameters.
DEFAULT_LENGTH = 0.70
DEFAULT_WIDTH = 0.55
DEFAULT_SAFETY_MARGIN = 0.05
DEFAULT_OBSTACLE_VALUE_THRESHOLD = 50
DEFAULT_UNKNOWN_IS_OBSTACLE = False
DEFAULT_SWEPT_MAX_CORNER_STEP_CELLS = 0.5


def normalize_angle(angle: float) -> float:
    """Match FootprintSafetyChecker::normalizeAngle (atan2 of sin/cos)."""
    return math.atan2(math.sin(angle), math.cos(angle))


@dataclass(frozen=True)
class FootprintParams:
    length: float = DEFAULT_LENGTH
    width: float = DEFAULT_WIDTH
    safety_margin: float = DEFAULT_SAFETY_MARGIN
    obstacle_value_threshold: int = DEFAULT_OBSTACLE_VALUE_THRESHOLD
    unknown_is_obstacle: bool = DEFAULT_UNKNOWN_IS_OBSTACLE
    swept_max_corner_step_cells: float = DEFAULT_SWEPT_MAX_CORNER_STEP_CELLS

    def as_dict(self) -> dict:
        return {
            "length": self.length,
            "width": self.width,
            "safety_margin": self.safety_margin,
            "obstacle_value_threshold": self.obstacle_value_threshold,
            "unknown_is_obstacle": self.unknown_is_obstacle,
            "swept_max_corner_step_cells": self.swept_max_corner_step_cells,
        }


@dataclass(frozen=True)
class Grid:
    """An immutable OccupancyGrid reduced to what the geometry needs."""

    width: int
    height: int
    resolution: float
    origin_x: float
    origin_y: float
    origin_yaw: float
    data: list
    frame_id: str = ""
    identity: dict = field(default_factory=dict)

    @staticmethod
    def from_dict(payload: dict) -> "Grid":
        info = payload.get("info", payload)
        origin = info.get("origin", {})
        return Grid(
            width=int(info["width"]),
            height=int(info["height"]),
            resolution=float(info["resolution"]),
            origin_x=float(origin.get("x", info.get("origin_x", 0.0))),
            origin_y=float(origin.get("y", info.get("origin_y", 0.0))),
            origin_yaw=float(origin.get("yaw", info.get("origin_yaw", 0.0))),
            data=list(payload["data"]),
            frame_id=str(payload.get("frame_id", "")),
            identity=dict(payload.get("identity", {})),
        )


def make_rectangular_footprint_samples(length, width, safety_margin, spacing):
    """Byte-for-byte port of makeRectangularFootprintSamples."""
    half_length = 0.5 * max(0.0, length) + max(0.0, safety_margin)
    half_width = 0.5 * max(0.0, width) + max(0.0, safety_margin)
    sample_spacing = max(0.02, spacing)
    samples_x = max(2, int(math.ceil((2.0 * half_length) / sample_spacing)))
    samples_y = max(2, int(math.ceil((2.0 * half_width) / sample_spacing)))

    samples = []
    for ix in range(samples_x + 1):
        x = -half_length + 2.0 * half_length * ix / float(samples_x)
        for iy in range(samples_y + 1):
            y = -half_width + 2.0 * half_width * iy / float(samples_y)
            samples.append((x, y))
    # Odd sample counts need not include the centre, but it is part of every
    # footprint.  The C++ source appends it unconditionally, so duplicates for
    # even counts are kept rather than deduplicated.
    samples.append((0.0, 0.0))
    return samples


def grid_yaw_from_quaternion(qx, qy, qz, qw):
    """Match the inline yaw extraction in FootprintSafetyChecker::worldToGrid."""
    return math.atan2(
        2.0 * (qw * qz + qx * qy),
        1.0 - 2.0 * (qy * qy + qz * qz),
    )


def world_to_grid(grid: Grid, wx: float, wy: float):
    """Port of worldToGrid.  Returns (ix, iy) or None when out of bounds."""
    if grid.resolution <= 0.0:
        return None
    dx = wx - grid.origin_x
    dy = wy - grid.origin_y
    cos_yaw = math.cos(grid.origin_yaw)
    sin_yaw = math.sin(grid.origin_yaw)
    gx = (cos_yaw * dx + sin_yaw * dy) / grid.resolution
    gy = (-sin_yaw * dx + cos_yaw * dy) / grid.resolution
    if gx < 0.0 or gy < 0.0 or gx >= float(grid.width) or gy >= float(grid.height):
        return None
    return (int(math.floor(gx)), int(math.floor(gy)))


def cell_value(grid: Grid, ix: int, iy: int):
    """Raw cell value, or None when the index is outside the grid."""
    if ix < 0 or iy < 0 or ix >= grid.width or iy >= grid.height:
        return None
    return int(grid.data[iy * grid.width + ix])


def is_occupied(grid: Grid, params: FootprintParams, ix: int, iy: int) -> bool:
    """Port of isOccupied: an out-of-range index is occupied, not free."""
    value = cell_value(grid, ix, iy)
    if value is None:
        return True
    if value < 0:
        return params.unknown_is_obstacle
    return value >= params.obstacle_value_threshold


def classify_cell(grid: Grid, params: FootprintParams, ix: int, iy: int) -> dict:
    """Per-cell provenance record for a first-conflict sample."""
    value = cell_value(grid, ix, iy)
    if value is None:
        return {
            "ix": ix, "iy": iy, "value": None,
            "class": "out_of_bounds", "occupied": True,
        }
    if value < 0:
        return {
            "ix": ix, "iy": iy, "value": value,
            "class": "unknown", "occupied": params.unknown_is_obstacle,
        }
    return {
        "ix": ix, "iy": iy, "value": value,
        "class": "occupied" if value >= params.obstacle_value_threshold else "free",
        "occupied": value >= params.obstacle_value_threshold,
    }


def sample_footprint_occupied(grid: Grid, params: FootprintParams, pose):
    """Port of sampleFootprintOccupied.

    Returns ``None`` when the whole rectangle is free, otherwise the first
    offending sample.  The sample order is the lattice order of the C++ source,
    so the reported *first* conflict matches what the planner would report.
    """
    x, y, yaw = pose
    samples = make_rectangular_footprint_samples(
        params.length, params.width, params.safety_margin, grid.resolution)
    cos_yaw = math.cos(yaw)
    sin_yaw = math.sin(yaw)
    for offset_x, offset_y in samples:
        wx = x + cos_yaw * offset_x - sin_yaw * offset_y
        wy = y + sin_yaw * offset_x + cos_yaw * offset_y
        index = world_to_grid(grid, wx, wy)
        if index is None:
            return {
                "sample_x": wx, "sample_y": wy,
                "offset_x": offset_x, "offset_y": offset_y,
                "cell": {"ix": None, "iy": None, "value": None,
                         "class": "out_of_grid", "occupied": True},
            }
        if is_occupied(grid, params, index[0], index[1]):
            return {
                "sample_x": wx, "sample_y": wy,
                "offset_x": offset_x, "offset_y": offset_y,
                "cell": classify_cell(grid, params, index[0], index[1]),
            }
    return None


def footprint_min_clearance_cells(grid: Grid, params: FootprintParams, pose):
    """Smallest distance from any footprint sample to an occupied cell centre.

    Diagnostic only: it scans the local window around the rectangle rather than
    consulting the ESDF, so it never changes a verdict.  The return value keeps
    "nothing occupied within the window" distinct from "could not evaluate",
    because collapsing those two into one None would let an artifact read a
    failed evaluation as generous clearance.
    """
    x, y, yaw = pose
    half_diag = math.hypot(
        0.5 * max(0.0, params.length) + max(0.0, params.safety_margin),
        0.5 * max(0.0, params.width) + max(0.0, params.safety_margin))
    search_radius = half_diag + 4.0 * grid.resolution
    report = {
        "min_clearance_m": None,
        "search_radius_m": search_radius,
        "evaluated": False,
        "bounded_below_by_search": False,
    }
    if grid.resolution <= 0.0 or not grid.data:
        return report
    centre = world_to_grid(grid, x, y)
    if centre is None:
        # The pose itself is off-grid, which the gate already treats as occupied.
        report["min_clearance_m"] = 0.0
        report["evaluated"] = True
        return report
    report["evaluated"] = True
    span = int(math.ceil(search_radius / grid.resolution))
    samples = make_rectangular_footprint_samples(
        params.length, params.width, params.safety_margin, grid.resolution)
    cos_yaw = math.cos(yaw)
    sin_yaw = math.sin(yaw)
    world_samples = [
        (x + cos_yaw * ox - sin_yaw * oy, y + sin_yaw * ox + cos_yaw * oy)
        for ox, oy in samples
    ]
    origin_cos = math.cos(grid.origin_yaw)
    origin_sin = math.sin(grid.origin_yaw)
    best = None
    for iy in range(centre[1] - span, centre[1] + span + 1):
        for ix in range(centre[0] - span, centre[0] + span + 1):
            if not is_occupied(grid, params, ix, iy):
                continue
            local_x = (ix + 0.5) * grid.resolution
            local_y = (iy + 0.5) * grid.resolution
            cx = grid.origin_x + origin_cos * local_x - origin_sin * local_y
            cy = grid.origin_y + origin_sin * local_x + origin_cos * local_y
            for wx, wy in world_samples:
                distance = math.hypot(cx - wx, cy - wy)
                if best is None or distance < best:
                    best = distance
    if best is None:
        # Nothing blocking inside the window: the true clearance is at least the
        # search radius, reported as a lower bound rather than as a measurement.
        report["bounded_below_by_search"] = True
        return report
    report["min_clearance_m"] = best
    return report


def swept_subdivisions(grid: Grid, params: FootprintParams, start, end) -> int:
    """Port of sweptSubdivisions: bound on maximum corner displacement."""
    half_length = 0.5 * max(0.0, params.length) + max(0.0, params.safety_margin)
    half_width = 0.5 * max(0.0, params.width) + max(0.0, params.safety_margin)
    corners = (
        (-half_length, -half_width),
        (-half_length, half_width),
        (half_length, -half_width),
        (half_length, half_width),
    )
    start_cos, start_sin = math.cos(start[2]), math.sin(start[2])
    end_cos, end_sin = math.cos(end[2]), math.sin(end[2])
    maximum = 0.0
    for cx, cy in corners:
        sx = start[0] + start_cos * cx - start_sin * cy
        sy = start[1] + start_sin * cx + start_cos * cy
        ex = end[0] + end_cos * cx - end_sin * cy
        ey = end[1] + end_sin * cx + end_cos * cy
        maximum = max(maximum, math.hypot(ex - sx, ey - sy))
    maximum_step = max(
        1e-6,
        grid.resolution * max(1e-3, params.swept_max_corner_step_cells))
    return max(1, int(math.ceil(maximum / maximum_step)))


def interpolate_pose(start, end, fraction):
    """Port of FootprintSafetyChecker::interpolate for the (x, y, yaw) part."""
    clamped = max(0.0, min(1.0, fraction))
    return (
        start[0] + (end[0] - start[0]) * clamped,
        start[1] + (end[1] - start[1]) * clamped,
        normalize_angle(start[2] + normalize_angle(end[2] - start[2]) * clamped),
    )


def check(grid: Grid, params: FootprintParams, poses, max_collisions=0):
    """Port of FootprintSafetyChecker::check over an (x, y, yaw) sequence.

    ``max_collisions`` of 0 means "record them all"; a positive value bounds the
    recorded list while still counting every hit, so a pathological trajectory
    cannot produce an unbounded artifact.
    """
    result = {
        "safe": True,
        "discrete_samples_checked": 0,
        "swept_segments_checked": 0,
        "swept_samples_checked": 0,
        "discrete_collision_count": 0,
        "swept_collision_count": 0,
        "collisions": [],
        "first_collision": None,
        "params": params.as_dict(),
        "grid": {
            "width": grid.width, "height": grid.height,
            "resolution": grid.resolution,
            "origin_x": grid.origin_x, "origin_y": grid.origin_y,
            "origin_yaw": grid.origin_yaw,
            "frame_id": grid.frame_id, "identity": grid.identity,
        },
        "pose_count": len(poses),
    }
    if not poses or not grid.data:
        # trajectory.valid() / grid.data.empty() in the C++ source both mean
        # "cannot prove safe", which is reported as unsafe rather than safe.
        result["safe"] = False
        result["reason"] = "empty poses or empty grid"
        return result

    def record(sample):
        result["safe"] = False
        if sample["swept"]:
            result["swept_collision_count"] += 1
        else:
            result["discrete_collision_count"] += 1
        if result["first_collision"] is None:
            result["first_collision"] = sample
        if max_collisions <= 0 or len(result["collisions"]) < max_collisions:
            result["collisions"].append(sample)

    for index, pose in enumerate(poses):
        result["discrete_samples_checked"] += 1
        hit = sample_footprint_occupied(grid, params, pose)
        if hit is None:
            continue
        record({
            "swept": False,
            "trajectory_index": index,
            "segment_index": index,
            "segment_fraction": 0.0,
            "center_x": pose[0], "center_y": pose[1], "center_yaw": pose[2],
            "sample_x": hit["sample_x"], "sample_y": hit["sample_y"],
            "cell": hit["cell"],
        })

    for index in range(len(poses) - 1):
        start = poses[index]
        end = poses[index + 1]
        subdivisions = swept_subdivisions(grid, params, start, end)
        result["swept_segments_checked"] += 1
        for step in range(1, subdivisions):
            fraction = float(step) / float(subdivisions)
            pose = interpolate_pose(start, end, fraction)
            result["swept_samples_checked"] += 1
            hit = sample_footprint_occupied(grid, params, pose)
            if hit is None:
                continue
            # Swept hits are attributed to the segment START index, matching the
            # C++ source.  Reporting the interpolated index instead would shift
            # every escape-prefix and repair decision by one.
            record({
                "swept": True,
                "trajectory_index": index,
                "segment_index": index,
                "segment_fraction": fraction,
                "center_x": pose[0], "center_y": pose[1], "center_yaw": pose[2],
                "sample_x": hit["sample_x"], "sample_y": hit["sample_y"],
                "cell": hit["cell"],
            })
    return result


def poses_from_path(path_payload):
    """Extract (x, y, yaw) from a recorded nav_msgs/Path payload."""
    poses = []
    for pose in path_payload.get("poses", []):
        position = pose.get("position", pose)
        orientation = pose.get("orientation", {})
        yaw = pose.get("yaw")
        if yaw is None:
            yaw = grid_yaw_from_quaternion(
                float(orientation.get("x", 0.0)), float(orientation.get("y", 0.0)),
                float(orientation.get("z", 0.0)), float(orientation.get("w", 1.0)))
        poses.append((float(position["x"]), float(position["y"]), float(yaw)))
    return poses


def parse_args(argv=None):
    parser = argparse.ArgumentParser(
        description="Evaluate reference and actual poses with the planner's own "
                    "oriented-footprint geometry against one immutable snapshot.")
    parser.add_argument("--grid", required=True,
                        help="JSON file holding the snapshot grid")
    parser.add_argument("--poses", required=True,
                        help="JSON file holding a path payload or a pose list")
    parser.add_argument("--output", help="write the verdict JSON here")
    parser.add_argument("--length", type=float, default=DEFAULT_LENGTH)
    parser.add_argument("--width", type=float, default=DEFAULT_WIDTH)
    parser.add_argument("--safety-margin", type=float, default=DEFAULT_SAFETY_MARGIN)
    parser.add_argument("--obstacle-value-threshold", type=int,
                        default=DEFAULT_OBSTACLE_VALUE_THRESHOLD)
    parser.add_argument("--unknown-is-obstacle", action="store_true",
                        default=DEFAULT_UNKNOWN_IS_OBSTACLE)
    parser.add_argument("--swept-max-corner-step-cells", type=float,
                        default=DEFAULT_SWEPT_MAX_CORNER_STEP_CELLS)
    parser.add_argument("--max-collisions", type=int, default=64,
                        help="bound on recorded collisions; 0 records all")
    return parser.parse_args(argv)


def main(argv=None) -> int:
    args = parse_args(argv)
    with open(args.grid, "r", encoding="utf-8") as handle:
        grid = Grid.from_dict(json.load(handle))
    with open(args.poses, "r", encoding="utf-8") as handle:
        payload = json.load(handle)
    poses = payload if isinstance(payload, list) else poses_from_path(payload)
    if poses and not isinstance(poses[0], tuple):
        poses = [(float(p[0]), float(p[1]), float(p[2])) for p in poses]
    params = FootprintParams(
        length=args.length, width=args.width, safety_margin=args.safety_margin,
        obstacle_value_threshold=args.obstacle_value_threshold,
        unknown_is_obstacle=args.unknown_is_obstacle,
        swept_max_corner_step_cells=args.swept_max_corner_step_cells)
    verdict = check(grid, params, poses, max_collisions=args.max_collisions)
    text = json.dumps(verdict, indent=2, sort_keys=True)
    if args.output:
        with open(args.output, "w", encoding="utf-8") as handle:
            handle.write(text + "\n")
    else:
        print(text)
    return 0 if verdict["safe"] else 1


if __name__ == "__main__":
    sys.exit(main())
