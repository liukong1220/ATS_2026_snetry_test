#!/usr/bin/env python3
"""Deterministic offline test for the oriented-footprint evaluator.

No ROS graph, no simulation.  Every expectation here is derived from the C++
source it mirrors, so a divergence between this evaluator and
``FootprintSafetyChecker`` shows up as a failure rather than as a silently
different collision count in a run artifact.
"""

from __future__ import annotations

import math
import sys

import footprint_evaluator as fe

FAILURES = []


def check(condition, message):
    if not condition:
        FAILURES.append(message)
        print("FAIL: " + message)
    else:
        print("ok: " + message)


def make_grid(width, height, resolution=0.10, origin_x=0.0, origin_y=0.0,
              origin_yaw=0.0, fill=0):
    return fe.Grid(
        width=width, height=height, resolution=resolution,
        origin_x=origin_x, origin_y=origin_y, origin_yaw=origin_yaw,
        data=[fill] * (width * height), frame_id="test")


def set_cell(grid, ix, iy, value):
    data = list(grid.data)
    data[iy * grid.width + ix] = value
    return fe.Grid(
        width=grid.width, height=grid.height, resolution=grid.resolution,
        origin_x=grid.origin_x, origin_y=grid.origin_y,
        origin_yaw=grid.origin_yaw, data=data, frame_id=grid.frame_id,
        identity=grid.identity)


# ---------------------------------------------------------------- lattice shape
# half_length = 0.5*0.60 + 0.02 = 0.32; half_width = 0.5*0.50 + 0.02 = 0.27.
# spacing = max(0.02, 0.10) = 0.10 -> samples_x = ceil(0.64/0.10) = 7,
# samples_y = ceil(0.54/0.10) = 6 -> (7+1)*(6+1) + 1 centre = 57 samples.
samples = fe.make_rectangular_footprint_samples(0.60, 0.50, 0.02, 0.10)
check(len(samples) == 57,
      "RMUC footprint lattice at 0.10 m has 57 samples (got %d)" % len(samples))
check(samples[-1] == (0.0, 0.0), "the explicit centre sample is appended last")
xs = [s[0] for s in samples[:-1]]
ys = [s[1] for s in samples[:-1]]
check(abs(min(xs) + 0.32) < 1e-12 and abs(max(xs) - 0.32) < 1e-12,
      "lattice spans the full half length including both borders")
check(abs(min(ys) + 0.27) < 1e-12 and abs(max(ys) - 0.27) < 1e-12,
      "lattice spans the full half width including both borders")

# The spacing floor is 0.02 m, so a finer grid does not produce unbounded samples.
fine = fe.make_rectangular_footprint_samples(0.60, 0.50, 0.02, 0.001)
check(len(fine) == (32 + 1) * (27 + 1) + 1,
      "spacing is floored at 0.02 m rather than following the grid resolution")

# A degenerate footprint still yields the minimum 2x2 lattice plus the centre.
degenerate = fe.make_rectangular_footprint_samples(0.0, 0.0, 0.0, 0.10)
check(len(degenerate) == (2 + 1) * (2 + 1) + 1,
      "a zero-size footprint still produces the minimum lattice")

# --------------------------------------------------------------- world_to_grid
grid = make_grid(20, 20, resolution=0.10, origin_x=-1.0, origin_y=-1.0)
check(fe.world_to_grid(grid, -1.0, -1.0) == (0, 0),
      "the origin corner maps to cell (0, 0)")
check(fe.world_to_grid(grid, -0.95, -0.95) == (0, 0),
      "a point inside the first cell still maps to (0, 0)")
check(fe.world_to_grid(grid, 0.95, 0.95) == (19, 19),
      "the last cell interior maps to (19, 19)")
check(fe.world_to_grid(grid, -1.01, 0.0) is None,
      "a query below the origin is out of bounds, not clamped")
check(fe.world_to_grid(grid, 1.0, 0.0) is None,
      "the exclusive upper edge is out of bounds")

# A rotated origin must be honoured: the same world point lands in a different
# cell once the grid is yawed, which is exactly the map origin/yaw class of bug
# the evaluator has to be able to expose rather than hide.
rotated = make_grid(20, 20, resolution=0.10, origin_x=0.0, origin_y=0.0,
                    origin_yaw=math.pi / 2.0)
check(fe.world_to_grid(rotated, 0.0, 0.35) == (3, 0),
      "a +90 deg origin yaw rotates the query into the grid's local axes")
check(fe.world_to_grid(rotated, 0.35, 0.0) is None,
      "the yawed grid does not cover +x, proving the rotation is applied")

# ------------------------------------------------------------------ occupancy
params = fe.FootprintParams(length=0.60, width=0.50, safety_margin=0.02,
                            obstacle_value_threshold=100,
                            unknown_is_obstacle=False)
occupancy_grid = make_grid(10, 10, fill=0)
occupancy_grid = set_cell(occupancy_grid, 5, 5, 99)
occupancy_grid = set_cell(occupancy_grid, 6, 5, 100)
occupancy_grid = set_cell(occupancy_grid, 7, 5, -1)
check(not fe.is_occupied(occupancy_grid, params, 5, 5),
      "value 99 is free under the RMUC threshold of 100")
check(fe.is_occupied(occupancy_grid, params, 6, 5),
      "value 100 is occupied at the RMUC threshold")
check(not fe.is_occupied(occupancy_grid, params, 7, 5),
      "unknown defers to unknown_is_obstacle=false")
strict = fe.FootprintParams(obstacle_value_threshold=100,
                            unknown_is_obstacle=True)
check(fe.is_occupied(occupancy_grid, strict, 7, 5),
      "the same unknown cell is occupied when unknown_is_obstacle=true")
check(fe.is_occupied(occupancy_grid, params, -1, 0),
      "an out-of-range index is occupied, never free")
check(fe.is_occupied(occupancy_grid, params, 10, 0),
      "an index past the width is occupied, never free")

# Provenance must name the source class, not just a boolean.
check(fe.classify_cell(occupancy_grid, params, 7, 5)["class"] == "unknown",
      "an unknown cell is classified as unknown even when it is not blocking")
check(fe.classify_cell(occupancy_grid, params, 6, 5)["class"] == "occupied",
      "a threshold cell is classified as occupied")
check(fe.classify_cell(occupancy_grid, params, 0, 0)["class"] == "free",
      "a zero cell is classified as free")
check(fe.classify_cell(occupancy_grid, params, 99, 99)["class"] == "out_of_bounds",
      "an out-of-range index is classified as out_of_bounds")

# ------------------------------------------------------------ swept subdivision
# max corner step = 0.10 * 0.5 = 0.05 m.  A pure 1.0 m translation moves every
# corner 1.0 m -> ceil(1.0/0.05) = 20 subdivisions.
translation = fe.swept_subdivisions(
    grid, params, (0.0, 0.0, 0.0), (1.0, 0.0, 0.0))
check(translation == 20,
      "a 1.0 m translation needs 20 subdivisions (got %d)" % translation)

# Pure rotation must be bounded by the same rule.  A 90 deg turn in place moves
# each corner by |c| * sqrt(2) where |c| = hypot(0.32, 0.27) = 0.41876 ->
# 0.59223 m -> ceil(0.59223/0.05) = 12.
rotation = fe.swept_subdivisions(
    grid, params, (0.0, 0.0, 0.0), (0.0, 0.0, math.pi / 2.0))
expected_rotation = int(math.ceil(
    math.hypot(0.32, 0.27) * math.sqrt(2.0) / 0.05))
check(rotation == expected_rotation,
      "pure rotation uses the same corner bound (%d == %d)"
      % (rotation, expected_rotation))
check(rotation > 1,
      "a rotation in place is never treated as a single unswept step")

# A zero-length segment must not divide by zero or produce zero subdivisions.
identical = fe.swept_subdivisions(
    grid, params, (0.5, 0.5, 0.3), (0.5, 0.5, 0.3))
check(identical == 1, "an identical pose pair yields exactly one subdivision")

# ------------------------------------------------------------------ yaw wrap
wrapped = fe.interpolate_pose((0.0, 0.0, 3.0), (0.0, 0.0, -3.0), 0.5)
check(abs(abs(wrapped[2]) - math.pi) < 1e-9,
      "yaw interpolation crosses the +-pi seam by the short way")

# --------------------------------------------------------- discrete vs swept
# A free corridor with a single obstacle cell placed BETWEEN two waypoints.  The
# discrete pass alone cannot see it; the swept pass must.
corridor = make_grid(60, 60, resolution=0.10, origin_x=-1.0, origin_y=-1.0,
                     fill=0)
free_poses = [(0.0, 0.0, 0.0), (2.0, 0.0, 0.0)]
verdict = fe.check(corridor, params, free_poses)
check(verdict["safe"] and verdict["discrete_collision_count"] == 0
      and verdict["swept_collision_count"] == 0,
      "an empty corridor is safe on both passes")
# The segment is 2.0 m long and the corner step is 0.10 * 0.5 = 0.05 m, so
# ceil(2.0 / 0.05) = 40 subdivisions and 39 interior samples.
check(verdict["swept_segments_checked"] == 1
      and verdict["swept_samples_checked"] == 39,
      "one 2.0 m segment produced 39 interior swept samples (got %d)"
      % verdict["swept_samples_checked"])

# Place a hard cell at world (1.0, 0.0): grid index ((1.0+1.0)/0.1, (0+1.0)/0.1)
# = (20, 10).  Both waypoints stay clear of it because the footprint half length
# is 0.32 m and they sit 1.0 m away.
between = set_cell(corridor, 20, 10, 100)
swept_verdict = fe.check(between, params, free_poses)
check(not swept_verdict["safe"],
      "a cell between two clear waypoints is caught")
check(swept_verdict["discrete_collision_count"] == 0,
      "the discrete pass alone does not see the between-waypoint cell")
check(swept_verdict["swept_collision_count"] > 0,
      "the swept pass is the one that catches it")
check(swept_verdict["first_collision"]["segment_index"] == 0,
      "the swept hit is attributed to the segment START index")
check(swept_verdict["first_collision"]["swept"] is True,
      "the first collision is marked as swept")
check(0.0 < swept_verdict["first_collision"]["segment_fraction"] < 1.0,
      "the swept hit retains its interpolation fraction")
check(swept_verdict["first_collision"]["cell"]["value"] == 100,
      "the first-conflict record carries the offending cell value")

# ----------------------------------------------------- discrete hit at a pose
# An obstacle inside the rectangle of the FIRST waypoint must be reported by the
# discrete pass with trajectory_index 0, which is the "already colliding at
# publish" signature the run artifacts have to be able to name.
at_start = set_cell(corridor, 12, 10, 100)  # world (0.20, 0.0), inside 0.32 m
start_verdict = fe.check(at_start, params, free_poses)
check(not start_verdict["safe"], "an obstacle inside waypoint 0 is caught")
check(start_verdict["first_collision"]["swept"] is False
      and start_verdict["first_collision"]["trajectory_index"] == 0,
      "an obstacle at waypoint 0 is a DISCRETE hit at index 0")
check(start_verdict["discrete_collision_count"] >= 1,
      "the discrete counter advances for a waypoint hit")

# ------------------------------------------------------------- fail-closed
empty_pose_verdict = fe.check(corridor, params, [])
check(not empty_pose_verdict["safe"],
      "an empty pose list is unsafe, matching trajectory.valid() == false")
empty_grid = fe.Grid(width=0, height=0, resolution=0.10, origin_x=0.0,
                     origin_y=0.0, origin_yaw=0.0, data=[], frame_id="test")
empty_grid_verdict = fe.check(empty_grid, params, free_poses)
check(not empty_grid_verdict["safe"],
      "an empty grid is unsafe, matching grid.data.empty() == false")

# A trajectory that leaves the grid entirely must be unsafe, not silently free.
outside = fe.check(corridor, params, [(100.0, 100.0, 0.0), (101.0, 100.0, 0.0)])
check(not outside["safe"],
      "poses outside the grid are unsafe because out-of-bounds is occupied")

# --------------------------------------------------------- bounded artifacts
many = corridor
for iy in range(0, 30):
    many = set_cell(many, 20, iy, 100)
bounded = fe.check(many, params, free_poses, max_collisions=3)
check(len(bounded["collisions"]) == 3,
      "max_collisions bounds the recorded list")
check(bounded["swept_collision_count"] + bounded["discrete_collision_count"] > 3,
      "the counters still reflect every hit even when the list is bounded")

# ------------------------------------------------------------ min clearance
# Cell (26, 10) has centre (1.65, 0.05) and sits inside the diagnostic window
# for a pose at (1.0, 0.0); the nearest footprint sample is (1.32, 0.0), so the
# minimum is hypot(0.33, 0.05) = 0.33242 m.
clear_grid = set_cell(
    make_grid(60, 60, resolution=0.10, origin_x=-1.0, origin_y=-1.0, fill=0),
    26, 10, 100)
clearance = fe.footprint_min_clearance_cells(clear_grid, params, (1.0, 0.0, 0.0))
check(clearance["evaluated"] and clearance["min_clearance_m"] is not None
      and abs(clearance["min_clearance_m"] - 0.33242) < 1e-4,
      "minimum clearance to the occupied cell centre is 0.33242 m (got %s)"
      % clearance["min_clearance_m"])

# "Nothing blocking inside the window" must not be reported the same way as
# "could not evaluate": an artifact reading None as generous clearance would
# turn a failed measurement into a safety claim.
open_grid = make_grid(60, 60, resolution=0.10, origin_x=-1.0, origin_y=-1.0,
                      fill=0)
open_report = fe.footprint_min_clearance_cells(open_grid, params, (1.0, 0.0, 0.0))
check(open_report["evaluated"] and open_report["bounded_below_by_search"]
      and open_report["min_clearance_m"] is None,
      "an empty window reports a lower bound, not a measured clearance")
off_grid = fe.footprint_min_clearance_cells(open_grid, params, (99.0, 0.0, 0.0))
check(off_grid["evaluated"] and off_grid["min_clearance_m"] == 0.0,
      "an off-grid pose reports zero clearance, matching the occupied verdict")
degenerate_report = fe.footprint_min_clearance_cells(
    fe.Grid(width=0, height=0, resolution=0.0, origin_x=0.0, origin_y=0.0,
            origin_yaw=0.0, data=[], frame_id="test"),
    params, (0.0, 0.0, 0.0))
check(not degenerate_report["evaluated"],
      "an unusable grid reports evaluated=false rather than a clearance")

# ----------------------------------------------------- unknown-blocking parity
# The same pose flips verdict purely on unknown_is_obstacle.  This is the gate
# asymmetry between the planner's checker and the goal manager's restart gate,
# so the evaluator must be able to reproduce both readings on one snapshot.
unknown_grid = make_grid(60, 60, resolution=0.10, origin_x=-1.0, origin_y=-1.0,
                         fill=0)
unknown_grid = set_cell(unknown_grid, 12, 10, -1)
permissive = fe.check(unknown_grid, params, [(0.0, 0.0, 0.0)])
blocking = fe.check(unknown_grid,
                    fe.FootprintParams(length=0.60, width=0.50,
                                       safety_margin=0.02,
                                       obstacle_value_threshold=100,
                                       unknown_is_obstacle=True),
                    [(0.0, 0.0, 0.0)])
check(permissive["safe"] and not blocking["safe"],
      "one unknown cell flips the verdict with unknown_is_obstacle alone")
check(blocking["first_collision"]["cell"]["class"] == "unknown",
      "the blocking verdict names unknown as the source, not 'occupied'")

if FAILURES:
    print("RESULT: footprint evaluator test FAILED (%d failure(s))" % len(FAILURES))
    sys.exit(1)
print("RESULT: footprint evaluator test PASSED")
sys.exit(0)
