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

    all_value_parser = subparsers.add_parser("all-value")
    all_value_parser.add_argument("--value", type=int, required=True)

    unreachable_parser = subparsers.add_parser("unreachable")
    unreachable_parser.add_argument("--start-x", type=float, required=True)
    unreachable_parser.add_argument("--start-y", type=float, required=True)
    unreachable_parser.add_argument("--threshold", type=int, default=50)
    unreachable_parser.add_argument("--clearance", type=float, default=0.57)
    unreachable_parser.add_argument(
        "--unknown-traversable", action="store_true"
    )
    unreachable_parser.add_argument(
        "--assume-start-traversable", action="store_true"
    )

    farthest_parser = subparsers.add_parser("farthest-free")
    farthest_parser.add_argument("--start-x", type=float, required=True)
    farthest_parser.add_argument("--start-y", type=float, required=True)
    farthest_parser.add_argument("--threshold", type=int, default=50)
    farthest_parser.add_argument("--clearance", type=float, default=0.57)
    farthest_parser.add_argument("--unknown-traversable", action="store_true")
    farthest_parser.add_argument(
        "--max-distance",
        type=float,
        default=0.0,
        help=(
            "候选格与起点的欧氏距离上限（m）；<=0 表示不设上限（旧行为）。"
            "跳数最大的可达格是全场最远角，其 JPS 路径不一定能过 MINCO 的"
            "矩形足迹门禁：domain 61 选到 (11.37, -8.49)（约 14 m），"
            "MINCO 连续以 452~622 个 footprint collisions 拒绝，前置段"
            "因此一次都没产生控制量。上限把选择限制在规划器确实能提交"
            "参考的邻域内，而“最远可达”仍由实时栅格洪泛给出。"
        ),
    )

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


def cell_is_traversable(
    grid, mx, my, threshold, clearance, unknown_traversable=False
):
    """规划器 `hasGridClearance` 的等价实现。

    环形判据、`ceil` 半径、越界视为阻塞都与 C++ 侧一致；`unknown_traversable`
    对应 `GridOccupancyPolicy::unknown_is_obstacle` 的反面，默认沿用旧的保守
    语义，只有显式传入才与 MuJoCo RMUC profile 的 `unknown_is_obstacle=false`
    对齐。故障前提必须用规划器的同一张可通行图判定，
    否则前提根本不成立。
    """
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
            if value < 0:
                if not unknown_traversable:
                    return False
                continue
            if value >= threshold:
                return False
    return True


def find_value(grid, value):
    for index, cell in enumerate(grid.data):
        if int(cell) == value:
            return index
    return None


def build_traversability(grid, threshold, clearance, unknown_traversable):
    width = int(grid.info.width)
    height = int(grid.info.height)
    traversable = bytearray(width * height)
    for my in range(height):
        for mx in range(width):
            traversable[my * width + mx] = cell_is_traversable(
                grid, mx, my, threshold, clearance, unknown_traversable
            )
    return traversable


def flood_from_start(grid, traversable, start_index, hop_distance=False):
    """从起点洪泛。起点无条件入队，对应规划器的 `assume_start_traversable`。

    机器人已经站在起点上，拒绝起点不会让它移动，所以
    起点不可通行时把整张图判成不可达是错误前提。
    `hop_distance` 为真时返回 8 邻域跳数，用于挑选真正
    需要长时间跟踪的目标，而不是直线距离最远、却可能一步到位的目标。
    """
    width = int(grid.info.width)
    height = int(grid.info.height)
    reached = bytearray(width * height)
    hops = [-1] * (width * height) if hop_distance else None
    reached[start_index] = 1
    if hops is not None:
        hops[start_index] = 0
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
                    if hops is not None:
                        hops[neighbor] = hops[current] + 1
                    queue.append(neighbor)
    return reached, hops


def find_farthest_free(
    grid,
    start_x,
    start_y,
    threshold,
    clearance,
    unknown_traversable=False,
    max_distance=0.0,
):
    """返回洪泛跳数最大的可达自由格。

    unknown 故障用例需要一个"注入生效之前不会自然完成"的
    前置目标。以前用固定的相对 1.80 m，实测该段在注入生效
    （ready=false 约 6.6 s）之前就已 SUCCEEDED，于是没有在飞
    目标可被 abort，用例只能超时。跳数最大的可达格由实时
    栅格给出，不依赖对自由空间几何的猜测。
    """
    start = world_to_grid(grid, start_x, start_y)
    if start is None:
        raise ValueError("start is outside the planning grid")
    width = int(grid.info.width)
    traversable = build_traversability(
        grid, threshold, clearance, unknown_traversable
    )
    start_index = start[1] * width + start[0]
    _, hops = flood_from_start(grid, traversable, start_index, hop_distance=True)
    # 上限只裁剪候选集，不改变“跳数最大”的判据本身：不传 --max-distance
    # 时下面的 candidates 就是全体格，行为与加上限之前逐位相同。
    resolution = float(grid.info.resolution)
    candidates = range(len(hops))
    if max_distance > 0.0:
        limit_cells_sq = (max_distance / resolution) ** 2
        candidates = [
            index
            for index in range(len(hops))
            if (index % width - start[0]) ** 2
            + (index // width - start[1]) ** 2
            <= limit_cells_sq
        ]
        if not candidates:
            raise ValueError(
                "no cell within %.3f m of the start cell" % max_distance
            )
    best_index = max(candidates, key=lambda index: hops[index])
    if hops[best_index] <= 0:
        raise ValueError("no reachable cell beyond the start cell")
    return best_index


def find_unreachable(
    grid,
    start_x,
    start_y,
    threshold,
    clearance,
    unknown_traversable=False,
    assume_start_traversable=False,
):
    """挑选一个自由但与起点不连通的格。

    前提必须以规划器最宽松的那一级 clearance 判定，而调用方
    要传的就是 `footprintConsistentClearanceFloor` 解析出的那个
    下限（内切半宽再加半个格对角线），不是内切半宽本身。
    端点松弛并不能替代这一点：`GridJps` 对欠 clearance 的目标
    格会按目标自身 clearance 重跑整张图，失败后报的仍是
    "goal occupied" 而不是 "no path"，故障原因于是落在
    FAILURE_START_OR_GOAL_OCCUPIED 上，恢复期被当作瞬时故障
    无限重试。目标格在下限那一级可通行时，失败原因才会
    收敛到连通性。

    起点不可通行时不再抛错：规划器的 `assume_start_traversable`
    会照样规划，所以前提判定也必须无条件从起点洪泛。
    """
    start = world_to_grid(grid, start_x, start_y)
    if start is None:
        raise ValueError("start is outside the planning grid")
    width = int(grid.info.width)
    traversable = build_traversability(
        grid, threshold, clearance, unknown_traversable
    )
    start_index = start[1] * width + start[0]
    if not traversable[start_index] and not assume_start_traversable:
        raise ValueError("start is not traversable at the requested clearance")

    reached, _ = flood_from_start(grid, traversable, start_index)
    candidates = [
        index
        for index in range(width * int(grid.info.height))
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
        if args.mode == "all-value":
            expected = int(args.value)
            if not all(int(cell) == expected for cell in grid.data):
                print(
                    f"grid contains a value other than {expected}", file=sys.stderr
                )
                return 4
            stamp = grid.header.stamp
            print(
                f"all {len(grid.data)} cells are {expected} "
                f"{grid.header.frame_id} {stamp.sec}.{stamp.nanosec:09d}"
            )
            return 0
        if args.mode == "value":
            index = find_value(grid, args.value)
        elif args.mode == "farthest-free":
            index = find_farthest_free(
                grid,
                args.start_x,
                args.start_y,
                args.threshold,
                args.clearance,
                args.unknown_traversable,
                args.max_distance,
            )
        else:
            index = find_unreachable(
                grid,
                args.start_x,
                args.start_y,
                args.threshold,
                args.clearance,
                args.unknown_traversable,
                args.assume_start_traversable,
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
