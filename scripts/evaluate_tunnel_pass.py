#!/usr/bin/env python3
"""Evaluate a recorded tunnel trajectory against tunnels_2027.yaml.

CSV input columns: x,y,yaw.  The evaluator intentionally checks polygon entry and
heading during the passage, so reaching the goal by going around the tunnel fails.

Axis convention (2026-10-23): vehicle +x is the heading and the short 0.571 m
face is the front, so the vehicle drives along the tunnel axis with yaw equal to
axis_yaw (modulo pi).  Before that correction the target was axis_yaw + pi/2.
"""

from __future__ import annotations

import argparse
import csv
import json
import math
from pathlib import Path

import yaml


def inside(x: float, y: float, polygon: list[list[float]]) -> bool:
    return min(p[0] for p in polygon) <= x <= max(p[0] for p in polygon) and min(
        p[1] for p in polygon
    ) <= y <= max(p[1] for p in polygon)


def angle_error(angle: float, target: float) -> float:
    # Rectangular footprint has 180 degree symmetry in a tunnel.
    return abs(0.5 * math.atan2(math.sin(2.0 * (angle - target)), math.cos(2.0 * (angle - target))))


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--tunnels-yaml", required=True)
    parser.add_argument("--trajectory", required=True)
    parser.add_argument("--tunnel", required=True)
    parser.add_argument("--goal-tolerance", type=float, default=0.2)
    parser.add_argument("--yaw-tolerance", type=float, default=None)
    args = parser.parse_args()

    config = yaml.safe_load(Path(args.tunnels_yaml).read_text(encoding="utf-8"))
    tunnel = next((item for item in config["tunnels"] if item["name"] == args.tunnel), None)
    if tunnel is None:
        raise SystemExit(f"unknown tunnel: {args.tunnel}")
    with Path(args.trajectory).open(newline="", encoding="utf-8") as stream:
        samples = [(float(row["x"]), float(row["y"]), float(row["yaw"])) for row in csv.DictReader(stream)]
    # The roof slab is only a short lintel; yaw alignment is judged over the walled corridor.
    window = tunnel.get("corridor_polygon", tunnel["polygon"])
    in_tunnel = [sample for sample in samples if inside(sample[0], sample[1], window)]
    target = float(tunnel["axis_yaw"])
    yaw_tolerance = args.yaw_tolerance
    if yaw_tolerance is None:
        # Use the same geometric bound as the planning design for 2 cm tunnel margin.
        width = float(tunnel["width"])
        yaw_tolerance = math.asin(max(-1.0, min(1.0, (width - 0.04 - 0.571) / 0.58131)))
    goal = tunnel["exit"]
    goal_error = math.hypot(samples[-1][0] - goal[0], samples[-1][1] - goal[1]) if samples else math.inf
    result = {
        "entered": bool(in_tunnel),
        "passed": bool(in_tunnel) and goal_error < args.goal_tolerance,
        "max_yaw_err_in_tunnel": max((angle_error(s[2], target) for s in in_tunnel), default=math.inf),
        "yaw_err_at_entry": angle_error(in_tunnel[0][2], target) if in_tunnel else math.inf,
        "pass_face": "0.571",
        "goal_error": goal_error,
        "yaw_tolerance": yaw_tolerance,
    }
    result["passed"] = result["passed"] and result["max_yaw_err_in_tunnel"] <= yaw_tolerance
    print(json.dumps(result, ensure_ascii=False, indent=2))
    raise SystemExit(0 if result["passed"] else 1)


if __name__ == "__main__":
    main()
