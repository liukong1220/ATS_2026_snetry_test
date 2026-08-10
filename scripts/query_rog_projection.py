#!/usr/bin/env python3
"""Structured client for ``/rog_map/get_ground_projection``.

Replaces text-scraping ``ros2 service call`` output.  The ``all-unknown`` mode
implements the strict predicate: a response only qualifies when it is
structurally valid *and* every occupancy cell is exactly ``-1``.  A response
that mixes unknown with free or occupied cells is refused with the offending
counters named, so a partially cleared map can never be reported as an
all-unknown source fault.
"""

from __future__ import annotations

import argparse
import math
import sys
import time

import rclpy
from ats_rog_map_interfaces.srv import GetRogMapProjection
from rclpy.node import Node


class ProjectionClient(Node):
    def __init__(self, service: str) -> None:
        super().__init__("ats_rog_projection_query")
        self.client = self.create_client(GetRogMapProjection, service)

    def call(self, min_height: float, max_height: float, resolution: float,
             timeout: float):
        if not self.client.wait_for_service(timeout_sec=timeout):
            return None
        request = GetRogMapProjection.Request()
        request.min_height = float(min_height)
        request.max_height = float(max_height)
        request.resolution = float(resolution)
        future = self.client.call_async(request)
        deadline = time.monotonic() + timeout
        while not future.done() and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.05)
        if not future.done():
            return None
        return future.result()


def audit(response) -> dict:
    grid = response.occupancy_grid
    cell_count = int(grid.info.width) * int(grid.info.height)
    unknown = 0
    free = 0
    occupied = 0
    out_of_range = 0
    for cell in grid.data:
        value = int(cell)
        if value == -1:
            unknown += 1
        elif value == 0:
            free += 1
        elif 0 < value <= 100:
            occupied += 1
        else:
            out_of_range += 1
    finite = 0
    for index in range(len(response.signed_distance)):
        if (
            math.isfinite(response.signed_distance[index])
            or (
                index < len(response.gradient_x)
                and math.isfinite(response.gradient_x[index])
            )
            or (
                index < len(response.gradient_y)
                and math.isfinite(response.gradient_y[index])
            )
        ):
            finite += 1
    structural_errors = []
    if not response.ready:
        structural_errors.append("ready=false")
    if response.stale:
        structural_errors.append("stale=true")
    if not grid.header.frame_id:
        structural_errors.append("empty frame_id")
    if int(grid.header.stamp.sec) <= 0 and int(grid.header.stamp.nanosec) == 0:
        structural_errors.append("non-positive stamp")
    if not math.isfinite(grid.info.resolution) or grid.info.resolution <= 0.0:
        structural_errors.append("resolution not finite and positive")
    if grid.info.width == 0 or grid.info.height == 0 or cell_count == 0:
        structural_errors.append("empty grid")
    if len(grid.data) != cell_count:
        structural_errors.append(
            f"occupancy len {len(grid.data)} != width*height {cell_count}"
        )
    for name, array in (
        ("signed_distance", response.signed_distance),
        ("gradient_x", response.gradient_x),
        ("gradient_y", response.gradient_y),
    ):
        if len(array) != cell_count:
            structural_errors.append(f"{name} len {len(array)} != cells {cell_count}")
    return {
        "generation": int(response.generation),
        "ready": bool(response.ready),
        "stale": bool(response.stale),
        "frame_id": grid.header.frame_id,
        "stamp_ns": int(grid.header.stamp.sec) * 1_000_000_000
        + int(grid.header.stamp.nanosec),
        "resolution": float(grid.info.resolution),
        "width": int(grid.info.width),
        "height": int(grid.info.height),
        "cell_count": cell_count,
        "occupancy_len": len(grid.data),
        "unknown_cells": unknown,
        "free_cells": free,
        "occupied_cells": occupied,
        "out_of_range_cells": out_of_range,
        "finite_numeric_cells": finite,
        "structural_errors": structural_errors,
    }


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument("--service", default="/rog_map/get_ground_projection")
    parser.add_argument("--min-height", type=float, default=0.1)
    parser.add_argument("--max-height", type=float, default=0.8)
    parser.add_argument("--resolution", type=float, default=0.1)
    parser.add_argument("--timeout", type=float, default=15.0)
    parser.add_argument(
        "--mode",
        choices=("generation", "all-unknown"),
        default="generation",
        help="generation: require a healthy ready/non-stale response and print "
        "its source generation. all-unknown: additionally require every cell to "
        "be strictly -1 with all-NaN numeric arrays.",
    )
    parser.add_argument(
        "--baseline-generation",
        type=int,
        default=-1,
        help="When >= 0, all-unknown additionally requires generation > baseline.",
    )
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    rclpy.init()
    node = ProjectionClient(args.service)
    try:
        response = node.call(
            args.min_height, args.max_height, args.resolution, args.timeout
        )
        if response is None:
            print("projection service call timed out", file=sys.stderr)
            return 2
        report = audit(response)
        summary = " ".join(f"{key}={value}" for key, value in report.items()
                           if key != "structural_errors")
        if report["structural_errors"]:
            print(
                f"projection structurally invalid: "
                f"{'; '.join(report['structural_errors'])} ({summary})",
                file=sys.stderr,
            )
            return 3
        if args.mode == "all-unknown":
            if report["unknown_cells"] != report["cell_count"]:
                print(
                    "projection is not strictly all-unknown: "
                    f"unknown={report['unknown_cells']} free={report['free_cells']} "
                    f"occupied={report['occupied_cells']} "
                    f"out_of_range={report['out_of_range_cells']} "
                    f"cells={report['cell_count']}",
                    file=sys.stderr,
                )
                return 4
            if report["finite_numeric_cells"] != 0:
                print(
                    "projection carries finite numeric values in "
                    f"{report['finite_numeric_cells']} unknown cells",
                    file=sys.stderr,
                )
                return 5
            if (
                args.baseline_generation >= 0
                and report["generation"] <= args.baseline_generation
            ):
                print(
                    f"generation {report['generation']} did not advance past "
                    f"baseline {args.baseline_generation}",
                    file=sys.stderr,
                )
                return 6
        print(summary)
        return 0
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    sys.exit(main())
