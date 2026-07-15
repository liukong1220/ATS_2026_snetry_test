#!/usr/bin/env python3

import argparse
import copy
import math
import sys
import time
from collections import deque

import rclpy
from nav_msgs.msg import OccupancyGrid
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy


class GridCapture(Node):
    def __init__(self, topic):
        super().__init__("ats_occupancy_grid_query")
        self.grid = None
        qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.subscription = self.create_subscription(
            OccupancyGrid, topic, self._on_grid, qos
        )

    def _on_grid(self, message):
        if self.grid is None:
            self.grid = message


def parse_args():
    parser = argparse.ArgumentParser()
    parser.add_argument("--topic", default="/rc_esdf/planning_grid")
    parser.add_argument("--timeout", type=float, default=10.0)
    subparsers = parser.add_subparsers(dest="mode", required=True)

    value_parser = subparsers.add_parser("value")
    value_parser.add_argument("--value", type=int, required=True)

    unreachable_parser = subparsers.add_parser("unreachable")
    unreachable_parser.add_argument("--start-x", type=float, required=True)
    unreachable_parser.add_argument("--start-y", type=float, required=True)
    unreachable_parser.add_argument("--threshold", type=int, default=50)
    unreachable_parser.add_argument("--clearance", type=float, default=0.57)

    publish_unknown_parser = subparsers.add_parser("publish-unknown")
    publish_unknown_parser.add_argument("--output-topic", required=True)
    publish_unknown_parser.add_argument("--count", type=int, default=3)
    publish_unknown_parser.add_argument("--period", type=float, default=0.2)
    return parser.parse_args()


def validate_grid(grid):
    count = int(grid.info.width) * int(grid.info.height)
    return (
        grid.header.frame_id
        and grid.info.width > 0
        and grid.info.height > 0
        and math.isfinite(grid.info.resolution)
        and grid.info.resolution > 0.0
        and len(grid.data) == count
    )


def grid_yaw(grid):
    orientation = grid.info.origin.orientation
    return math.atan2(
        2.0
        * (
            orientation.w * orientation.z
            + orientation.x * orientation.y
        ),
        1.0
        - 2.0
        * (
            orientation.y * orientation.y
            + orientation.z * orientation.z
        ),
    )


def grid_to_world(grid, mx, my):
    yaw = grid_yaw(grid)
    local_x = (mx + 0.5) * grid.info.resolution
    local_y = (my + 0.5) * grid.info.resolution
    return (
        grid.info.origin.position.x
        + math.cos(yaw) * local_x
        - math.sin(yaw) * local_y,
        grid.info.origin.position.y
        + math.sin(yaw) * local_x
        + math.cos(yaw) * local_y,
    )


def world_to_grid(grid, world_x, world_y):
    yaw = grid_yaw(grid)
    dx = world_x - grid.info.origin.position.x
    dy = world_y - grid.info.origin.position.y
    local_x = math.cos(yaw) * dx + math.sin(yaw) * dy
    local_y = -math.sin(yaw) * dx + math.cos(yaw) * dy
    mx = math.floor(local_x / grid.info.resolution)
    my = math.floor(local_y / grid.info.resolution)
    if mx < 0 or my < 0 or mx >= grid.info.width or my >= grid.info.height:
        return None
    return mx, my


def cell_is_traversable(grid, mx, my, threshold, clearance):
    width = int(grid.info.width)
    height = int(grid.info.height)
    radius = math.ceil(max(0.0, clearance) / grid.info.resolution)
    radius_squared = (max(0.0, clearance) / grid.info.resolution) ** 2
    for offset_y in range(-radius, radius + 1):
        for offset_x in range(-radius, radius + 1):
            if offset_x * offset_x + offset_y * offset_y > radius_squared:
                continue
            x = mx + offset_x
            y = my + offset_y
            if x < 0 or y < 0 or x >= width or y >= height:
                return False
            value = int(grid.data[y * width + x])
            if value < 0 or value >= threshold:
                return False
    return True


def find_value(grid, value):
    for index, cell in enumerate(grid.data):
        if int(cell) == value:
            return index
    return None


def find_unreachable(grid, start_x, start_y, threshold, clearance):
    start = world_to_grid(grid, start_x, start_y)
    if start is None:
        raise ValueError("start is outside the planning grid")
    width = int(grid.info.width)
    height = int(grid.info.height)
    traversable = bytearray(width * height)
    for my in range(height):
        for mx in range(width):
            index = my * width + mx
            traversable[index] = cell_is_traversable(
                grid, mx, my, threshold, clearance
            )
    start_index = start[1] * width + start[0]
    if not traversable[start_index]:
        raise ValueError("start is not traversable at the requested clearance")

    reached = bytearray(width * height)
    reached[start_index] = 1
    queue = deque([start_index])
    while queue:
        current = queue.popleft()
        x = current % width
        y = current // width
        for dy in (-1, 0, 1):
            for dx in (-1, 0, 1):
                if dx == 0 and dy == 0:
                    continue
                nx = x + dx
                ny = y + dy
                if nx < 0 or ny < 0 or nx >= width or ny >= height:
                    continue
                neighbor = ny * width + nx
                if traversable[neighbor] and not reached[neighbor]:
                    reached[neighbor] = 1
                    queue.append(neighbor)

    candidates = [
        index
        for index in range(width * height)
        if traversable[index] and not reached[index]
    ]
    if not candidates:
        return None
    return max(
        candidates,
        key=lambda index: (index % width - start[0]) ** 2
        + (index // width - start[1]) ** 2,
    )


def publish_unknown_copy(node, grid, output_topic, count, period):
    qos = QoSProfile(
        history=HistoryPolicy.KEEP_LAST,
        depth=1,
        reliability=ReliabilityPolicy.RELIABLE,
        durability=DurabilityPolicy.TRANSIENT_LOCAL,
    )
    publisher = node.create_publisher(OccupancyGrid, output_topic, qos)
    unknown_grid = copy.deepcopy(grid)
    unknown_grid.data = [-1] * len(grid.data)
    for _ in range(max(1, count)):
        publisher.publish(unknown_grid)
        rclpy.spin_once(node, timeout_sec=max(0.01, period))
    return len(unknown_grid.data)


def main():
    args = parse_args()
    rclpy.init()
    node = GridCapture(args.topic)
    try:
        deadline = time.monotonic() + max(0.1, args.timeout)
        while node.grid is None and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.2)
        if node.grid is None:
            print("planning grid timeout", file=sys.stderr)
            return 2
        grid = node.grid
        if not validate_grid(grid):
            print("invalid planning grid", file=sys.stderr)
            return 3

        if args.mode == "publish-unknown":
            cell_count = publish_unknown_copy(
                node,
                grid,
                args.output_topic,
                args.count,
                args.period,
            )
            print(f"published {cell_count} unknown cells to {args.output_topic}")
            return 0
        if args.mode == "value":
            index = find_value(grid, args.value)
        else:
            index = find_unreachable(
                grid,
                args.start_x,
                args.start_y,
                args.threshold,
                args.clearance,
            )
        if index is None:
            print(f"no cell matched mode={args.mode}", file=sys.stderr)
            return 4

        mx = index % int(grid.info.width)
        my = index // int(grid.info.width)
        world_x, world_y = grid_to_world(grid, mx, my)
        value = int(grid.data[index])
        stamp = grid.header.stamp
        print(
            f"{world_x:.6f} {world_y:.6f} {index} {value} "
            f"{grid.header.frame_id} {stamp.sec}.{stamp.nanosec:09d}"
        )
        return 0
    except ValueError as exception:
        print(str(exception), file=sys.stderr)
        return 5
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    sys.exit(main())
