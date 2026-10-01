#!/usr/bin/env python3
"""Arc-length curvature metrics for MINCO reference paths.

The planner log reports curvature on time-sampled reference points.  Time
sampling is not uniform in space: slow segments crowd points together, the
terminal in-place yaw tail stacks many points on one position, and the
three-point curvature of two nearly coincident points explodes.  That is the
kmax "spike artefact" seen in the logs.  This tool removes it by resampling
every path at a fixed arc-length step ``ds`` before computing curvature, so
baseline and candidate runs are measured with the same ruler.

Metrics per path (all curvature in 1/m, computed on the resampled polyline):

* ``k95`` / ``kmax``   95th percentile / maximum of |kappa|
* ``curvature_tv``      total variation sum |kappa_i - kappa_{i-1}|
* ``sign_flips``        sign changes of kappa, ignoring |kappa| <= deadband
* ``length``            arc length in m

Curvature at resampled point i uses the circumscribed circle through points
i-h, i, i+h (``h = stencil``), i.e. a chord of ``2 * h * ds`` metres, which
keeps linear-interpolation corners from being counted as separate spikes.

Input is a rosbag2 directory; ``--topic`` selects the nav_msgs/Path topic
(default ``/minco/reference_path``).  Output is JSON on stdout.
"""

from __future__ import annotations

import argparse
import json
import math
import sys
from typing import Iterable, List, Sequence, Tuple

Point = Tuple[float, float]

DEFAULT_DS = 0.05
DEFAULT_STENCIL = 2
DEFAULT_DEADBAND = 0.05


def dedupe(points: Iterable[Point], eps: float = 1e-6) -> List[Point]:
    """Drop consecutive points closer than ``eps`` (terminal yaw tail, holds)."""
    result: List[Point] = []
    for point in points:
        if not result or math.hypot(point[0] - result[-1][0], point[1] - result[-1][1]) > eps:
            result.append((float(point[0]), float(point[1])))
    return result


def resample_by_arc_length(points: Sequence[Point], ds: float) -> List[Point]:
    """Linearly resample a polyline at a fixed arc-length step, keeping the end point."""
    if ds <= 0.0:
        raise ValueError("ds must be positive")
    unique = dedupe(points)
    if len(unique) < 2:
        return unique
    cumulative = [0.0]
    for index in range(1, len(unique)):
        cumulative.append(
            cumulative[-1]
            + math.hypot(unique[index][0] - unique[index - 1][0],
                         unique[index][1] - unique[index - 1][1]))
    total = cumulative[-1]
    count = int(math.floor(total / ds))
    targets = [ds * i for i in range(count + 1)]
    if total - targets[-1] > 1e-9:
        targets.append(total)
    result: List[Point] = []
    segment = 1
    for target in targets:
        while segment < len(cumulative) - 1 and cumulative[segment] < target:
            segment += 1
        start_s = cumulative[segment - 1]
        span = cumulative[segment] - start_s
        u = 0.0 if span <= 0.0 else min(1.0, max(0.0, (target - start_s) / span))
        a = unique[segment - 1]
        b = unique[segment]
        result.append((a[0] + u * (b[0] - a[0]), a[1] + u * (b[1] - a[1])))
    return result


def signed_curvature(before: Point, current: Point, after: Point) -> float:
    """Signed curvature of the circle through three points (Menger curvature)."""
    ax, ay = current[0] - before[0], current[1] - before[1]
    bx, by = after[0] - current[0], after[1] - current[1]
    cx, cy = after[0] - before[0], after[1] - before[1]
    denominator = math.hypot(ax, ay) * math.hypot(bx, by) * math.hypot(cx, cy)
    if denominator <= 1e-12:
        return 0.0
    return 2.0 * (ax * by - ay * bx) / denominator


def percentile95(values: Sequence[float]) -> float:
    """Linear-interpolated 95th percentile, matching the planner's evaluator."""
    if not values:
        return 0.0
    ordered = sorted(values)
    position = 0.95 * (len(ordered) - 1)
    lower = int(math.floor(position))
    upper = int(math.ceil(position))
    return ordered[lower] + (ordered[upper] - ordered[lower]) * (position - lower)


def path_metrics(points: Sequence[Point], ds: float = DEFAULT_DS,
                 stencil: int = DEFAULT_STENCIL,
                 deadband: float = DEFAULT_DEADBAND) -> dict:
    """Compute arc-length curvature metrics for one path."""
    if stencil < 1:
        raise ValueError("stencil must be >= 1")
    resampled = resample_by_arc_length(points, ds)
    length = sum(
        math.hypot(resampled[i][0] - resampled[i - 1][0], resampled[i][1] - resampled[i - 1][1])
        for i in range(1, len(resampled)))
    curvatures = [
        signed_curvature(resampled[i - stencil], resampled[i], resampled[i + stencil])
        for i in range(stencil, len(resampled) - stencil)
    ]
    magnitudes = [abs(value) for value in curvatures]
    sign_flips = 0
    previous_sign = 0
    for value in curvatures:
        if abs(value) <= deadband:
            continue
        sign = 1 if value > 0.0 else -1
        if previous_sign != 0 and sign != previous_sign:
            sign_flips += 1
        previous_sign = sign
    total_variation = sum(
        abs(curvatures[i] - curvatures[i - 1]) for i in range(1, len(curvatures)))
    return {
        "points_in": len(points),
        "points_resampled": len(resampled),
        "length": length,
        "k95": percentile95(magnitudes),
        "kmax": max(magnitudes) if magnitudes else 0.0,
        "curvature_tv": total_variation,
        "sign_flips": sign_flips,
    }


def read_paths(bag_path: str, topic: str) -> List[Tuple[int, List[Point]]]:
    """Read every nav_msgs/Path message on ``topic`` as (receive_ns, [(x, y)])."""
    import rosbag2_py  # pylint: disable=import-outside-toplevel
    from rclpy.serialization import deserialize_message  # pylint: disable=import-outside-toplevel
    from nav_msgs.msg import Path  # pylint: disable=import-outside-toplevel

    reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py.StorageOptions(uri=bag_path, storage_id=""),
        rosbag2_py.ConverterOptions(input_serialization_format="cdr",
                                    output_serialization_format="cdr"))
    reader.set_filter(rosbag2_py.StorageFilter(topics=[topic]))
    paths: List[Tuple[int, List[Point]]] = []
    while reader.has_next():
        _, data, stamp = reader.read_next()
        message = deserialize_message(data, Path)
        points = [(pose.pose.position.x, pose.pose.position.y) for pose in message.poses]
        if len(points) >= 2:
            paths.append((stamp, points))
    return paths


def summarize(paths: Sequence[Tuple[int, List[Point]]], ds: float, stencil: int,
              deadband: float) -> dict:
    """Metrics for the first path plus the worst value over all distinct paths."""
    distinct: List[Tuple[int, List[Point]]] = []
    for stamp, points in paths:
        # Latched/republished copies of the same reference must not be counted twice.
        if distinct and distinct[-1][1] == points:
            continue
        distinct.append((stamp, points))
    per_path = [dict(path_metrics(points, ds, stencil, deadband), receive_ns=stamp)
                for stamp, points in distinct]
    summary = {
        "ds": ds,
        "stencil": stencil,
        "deadband": deadband,
        "reference_count": len(per_path),
        "first": per_path[0] if per_path else None,
        "per_reference": per_path,
    }
    if per_path:
        summary["worst"] = {
            "k95": max(item["k95"] for item in per_path),
            "kmax": max(item["kmax"] for item in per_path),
            "curvature_tv": max(item["curvature_tv"] for item in per_path),
            "sign_flips": max(item["sign_flips"] for item in per_path),
        }
    return summary


def main(argv: Sequence[str]) -> int:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("bag", help="rosbag2 directory")
    parser.add_argument("--topic", default="/minco/reference_path")
    parser.add_argument("--ds", type=float, default=DEFAULT_DS)
    parser.add_argument("--stencil", type=int, default=DEFAULT_STENCIL)
    parser.add_argument("--deadband", type=float, default=DEFAULT_DEADBAND)
    parser.add_argument("--brief", action="store_true", help="omit per_reference list")
    args = parser.parse_args(argv)
    summary = summarize(read_paths(args.bag, args.topic), args.ds, args.stencil, args.deadband)
    if args.brief:
        summary.pop("per_reference")
    json.dump(summary, sys.stdout, indent=2)
    sys.stdout.write("\n")
    return 0 if summary["reference_count"] > 0 else 1


if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))
