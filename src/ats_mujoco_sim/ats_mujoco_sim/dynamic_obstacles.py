"""Helpers for generated MuJoCo dynamic obstacles."""

from __future__ import annotations

from dataclasses import dataclass, field
import heapq
import math
from typing import Iterable

import numpy as np


@dataclass(frozen=True)
class DynamicObstacleSceneConfig:
    enabled: bool = False
    count: int = 0
    shape: str = "cylinder"
    radius: float = 0.25
    height: float = 1.0
    mass: float = 20.0
    speed: float = 0.4
    path_length: float = 6.0
    min_robot_distance: float = 1.8
    near_robot_radius: float = 4.0
    map_clearance: float = 0.35
    replan_rate_hz: float = 1.0


@dataclass(frozen=True)
class DynamicObstacleRuntimeConfig:
    name: str
    behavior: str
    center: tuple[float, float, float]
    phase: float
    speed: float
    path_length: float


@dataclass(frozen=True)
class DynamicObstaclePose:
    position: np.ndarray
    yaw: float
    linear_velocity: np.ndarray
    angular_velocity_z: float = 0.0


@dataclass
class DynamicObstacleControllerState:
    target: np.ndarray
    mode: str = ""
    last_replan_time: float = -1.0e9
    path: list[np.ndarray] = field(default_factory=list)
    waypoint_index: int = 0


class StaticMapSampler:
    """Tiny helper that samples free points from the static occupancy image."""

    def __init__(self, occupancy, resolution, origin, clearance=0.0):
        self.occupancy = np.asarray(occupancy, dtype=bool)
        self.resolution = float(resolution)
        self.origin = (float(origin[0]), float(origin[1]), float(origin[2]))
        self.clearance = max(0.0, float(clearance))
        self.height, self.width = self.occupancy.shape
        self.free_points = self._build_free_points()

    def _build_free_points(self):
        points = []
        for row in range(self.height):
            for col in range(self.width):
                if not self.grid_cell_is_free(row, col):
                    continue
                points.append(self.grid_to_world(row, col))
        if not points:
            points.append(np.array([self.origin[0], self.origin[1]], dtype=np.float64))
        return np.asarray(points, dtype=np.float64)

    def grid_to_world(self, row, col):
        x = self.origin[0] + (float(col) + 0.5) * self.resolution
        y = self.origin[1] + (float(self.height) - float(row) - 0.5) * self.resolution
        return np.array([x, y], dtype=np.float64)

    def world_to_grid(self, point):
        col = int(np.floor((float(point[0]) - self.origin[0]) / self.resolution))
        row = int(np.floor(float(self.height) - (float(point[1]) - self.origin[1]) / self.resolution))
        return row, col

    def point_is_free(self, point):
        row, col = self.world_to_grid(point)
        return self.grid_cell_is_free(row, col)

    def grid_cell_is_free(self, row, col):
        if row < 0 or row >= self.height or col < 0 or col >= self.width:
            return False
        if self.clearance <= 0.0:
            return not self.occupancy[row, col]

        radius_cells = int(np.ceil(self.clearance / self.resolution))
        r0 = max(0, row - radius_cells)
        r1 = min(self.height, row + radius_cells + 1)
        c0 = max(0, col - radius_cells)
        c1 = min(self.width, col + radius_cells + 1)
        return not bool(np.any(self.occupancy[r0:r1, c0:c1]))

    def segment_is_free(self, start, goal):
        start = np.asarray(start, dtype=np.float64)
        goal = np.asarray(goal, dtype=np.float64)
        distance = float(np.linalg.norm(goal - start))
        steps = max(1, int(np.ceil(distance / max(self.resolution * 0.5, 1.0e-3))))
        for index in range(steps + 1):
            ratio = index / steps
            point = start + (goal - start) * ratio
            if not self.point_is_free(point):
                return False
        return True

    def nearest_free_point(self, point):
        point = np.asarray(point, dtype=np.float64)
        distances = np.linalg.norm(self.free_points - point, axis=1)
        return np.array(self.free_points[int(np.argmin(distances))], dtype=np.float64)

    def random_free_point(self, rng):
        index = int(rng.integers(0, len(self.free_points)))
        return np.array(self.free_points[index], dtype=np.float64)

    def random_free_point_near(self, center, radius, rng):
        center = np.asarray(center, dtype=np.float64)
        radius = max(self.resolution, float(radius))
        distances = np.linalg.norm(self.free_points - center, axis=1)
        candidates = self.free_points[distances <= radius]
        if candidates.size == 0:
            return self.nearest_free_point(center)
        index = int(rng.integers(0, len(candidates)))
        return np.array(candidates[index], dtype=np.float64)

    def coarse_planner(self, target_resolution):
        target_resolution = max(float(target_resolution), self.resolution)
        factor = max(1, int(round(target_resolution / self.resolution)))
        if factor <= 1:
            return self

        coarse_height = int(math.ceil(self.height / factor))
        coarse_width = int(math.ceil(self.width / factor))
        coarse_occupancy = np.ones((coarse_height, coarse_width), dtype=bool)

        # 粗格只要覆盖到原图障碍就视为占用，规划更保守也更稳定。
        for row in range(coarse_height):
            r0 = row * factor
            r1 = min(self.height, r0 + factor)
            for col in range(coarse_width):
                c0 = col * factor
                c1 = min(self.width, c0 + factor)
                coarse_occupancy[row, col] = bool(np.any(self.occupancy[r0:r1, c0:c1]))

        return StaticMapSampler(
            coarse_occupancy,
            self.resolution * factor,
            self.origin,
            clearance=0.0,
        )

    def astar_path(self, start, goal, max_expansions=20000):
        start = np.asarray(start, dtype=np.float64)
        goal = np.asarray(goal, dtype=np.float64)
        if not self.point_is_free(start):
            start = self.nearest_free_point(start)
        if not self.point_is_free(goal):
            goal = self.nearest_free_point(goal)
        if self.segment_is_free(start, goal):
            return [np.array(start, dtype=np.float64), np.array(goal, dtype=np.float64)]

        start_cell = self.world_to_grid(start)
        goal_cell = self.world_to_grid(goal)
        if not self.grid_cell_is_free(*start_cell) or not self.grid_cell_is_free(*goal_cell):
            return [np.array(start, dtype=np.float64)]

        neighbors = (
            (-1, 0, 1.0),
            (1, 0, 1.0),
            (0, -1, 1.0),
            (0, 1, 1.0),
            (-1, -1, math.sqrt(2.0)),
            (-1, 1, math.sqrt(2.0)),
            (1, -1, math.sqrt(2.0)),
            (1, 1, math.sqrt(2.0)),
        )
        open_heap = [(0.0, 0, start_cell)]
        came_from = {}
        cost_so_far = {start_cell: 0.0}
        push_count = 1
        expansions = 0

        while open_heap and expansions < int(max_expansions):
            _, _, current = heapq.heappop(open_heap)
            expansions += 1
            if current == goal_cell:
                grid_path = self._reconstruct_grid_path(came_from, start_cell, goal_cell)
                return self._smooth_grid_path(grid_path, start, goal)

            row, col = current
            for drow, dcol, step_cost in neighbors:
                next_cell = (row + drow, col + dcol)
                if not self.grid_cell_is_free(*next_cell):
                    continue
                if drow != 0 and dcol != 0:
                    # 避免斜向从两个障碍格角落中间“挤过去”。
                    if not self.grid_cell_is_free(row + drow, col):
                        continue
                    if not self.grid_cell_is_free(row, col + dcol):
                        continue
                new_cost = cost_so_far[current] + step_cost
                if next_cell in cost_so_far and new_cost >= cost_so_far[next_cell]:
                    continue
                cost_so_far[next_cell] = new_cost
                priority = new_cost + self._grid_heuristic(next_cell, goal_cell)
                heapq.heappush(open_heap, (priority, push_count, next_cell))
                came_from[next_cell] = current
                push_count += 1

        return [np.array(start, dtype=np.float64)]

    def _grid_heuristic(self, cell, goal_cell):
        return math.hypot(float(cell[0] - goal_cell[0]), float(cell[1] - goal_cell[1]))

    def _reconstruct_grid_path(self, came_from, start_cell, goal_cell):
        cell = goal_cell
        path = [cell]
        while cell != start_cell:
            cell = came_from[cell]
            path.append(cell)
        path.reverse()
        return path

    def _smooth_grid_path(self, grid_path, start, goal):
        points = [self.grid_to_world(row, col) for row, col in grid_path]
        if not points:
            return [np.array(goal, dtype=np.float64)]
        points[0] = np.array(start, dtype=np.float64)
        points[-1] = np.array(goal, dtype=np.float64)

        smoothed = [points[0]]
        anchor = 0
        while anchor < len(points) - 1:
            next_index = len(points) - 1
            while next_index > anchor + 1:
                if self.segment_is_free(points[anchor], points[next_index]):
                    break
                next_index -= 1
            smoothed.append(points[next_index])
            anchor = next_index
        return smoothed


BEHAVIOR_RGBA = {
    "follower": "0.95 0.20 0.15 1",
    "wander_near_robot": "0.12 0.55 0.95 1",
    "approach_and_leave": "0.95 0.75 0.12 1",
    "global_wander": "0.35 0.85 0.35 1",
}


def behavior_rgba(behavior: str) -> str:
    return BEHAVIOR_RGBA.get(str(behavior), "0.95 0.35 0.08 1")


def behavior_speed_scale(behavior: str) -> float:
    """Give each behavior a small speed personality without extra config."""
    scales = {
        "follower": 1.0,
        "wander_near_robot": 1.35,
        "approach_and_leave": 1.15,
        "global_wander": 1.65,
    }
    return scales.get(str(behavior), 1.0)


def staggered_replan_time(index: int, count: int, replan_rate_hz: float) -> float:
    count = max(1, int(count))
    period = 1.0 / max(0.1, float(replan_rate_hz))
    return -period * (int(index) % count) / count


def bool_from_value(value) -> bool:
    if isinstance(value, bool):
        return value
    if isinstance(value, str):
        return value.strip().lower() in ("1", "true", "yes", "on")
    return bool(value)


def clamp_scene_config(config: DynamicObstacleSceneConfig) -> DynamicObstacleSceneConfig:
    return DynamicObstacleSceneConfig(
        enabled=bool_from_value(config.enabled),
        count=max(0, int(config.count)),
        shape=str(config.shape).strip().lower() or "cylinder",
        radius=max(0.05, float(config.radius)),
        height=max(0.1, float(config.height)),
        mass=max(0.1, float(config.mass)),
        speed=max(0.0, float(config.speed)),
        path_length=max(0.1, float(config.path_length)),
        min_robot_distance=max(0.1, float(config.min_robot_distance)),
        near_robot_radius=max(0.5, float(config.near_robot_radius)),
        map_clearance=max(0.0, float(config.map_clearance)),
        replan_rate_hz=max(0.1, float(config.replan_rate_hz)),
    )


def dynamic_obstacle_name(index: int) -> str:
    return f"dynamic_obstacle_{index:02d}"


def _iter_centers(
    count: int,
    map_center_x: float,
    map_center_y: float,
    map_half_x: float,
    map_half_y: float,
    height: float,
) -> Iterable[tuple[float, float, float]]:
    z = 0.5 * float(height)
    if count <= 0:
        return

    # 用黄金角分布做一个确定性的“散开”效果，避免所有障碍物挤在地图中心。
    max_radius_x = max(0.5, float(map_half_x) - 1.0)
    max_radius_y = max(0.5, float(map_half_y) - 1.0)
    golden_angle = math.pi * (3.0 - math.sqrt(5.0))
    for index in range(count):
        radius = math.sqrt((index + 0.5) / max(1, count))
        angle = index * golden_angle
        x = float(map_center_x) + math.cos(angle) * radius * max_radius_x
        y = float(map_center_y) + math.sin(angle) * radius * max_radius_y
        yield x, y, z


def build_runtime_configs(
    config: DynamicObstacleSceneConfig,
    map_center_x: float,
    map_center_y: float,
    map_half_x: float,
    map_half_y: float,
) -> list[DynamicObstacleRuntimeConfig]:
    config = clamp_scene_config(config)
    if not config.enabled or config.count <= 0:
        return []

    runtime_configs = []
    behaviors = [
        "follower",
        "wander_near_robot",
        "approach_and_leave",
        "global_wander",
    ]
    for index, center in enumerate(
        _iter_centers(
            config.count,
            map_center_x,
            map_center_y,
            map_half_x,
            map_half_y,
            config.height,
        )
    ):
        runtime_configs.append(
            DynamicObstacleRuntimeConfig(
                name=dynamic_obstacle_name(index),
                behavior=behaviors[index % len(behaviors)],
                center=center,
                phase=(index / max(1, config.count)) * 2.0 * config.path_length,
                speed=config.speed,
                path_length=config.path_length,
            )
        )
    return runtime_configs


def _unit_from_yaw(yaw):
    return np.array([math.cos(float(yaw)), math.sin(float(yaw))], dtype=np.float64)


def _normalized_or(vector, fallback):
    vector = np.asarray(vector, dtype=np.float64)
    norm = float(np.linalg.norm(vector))
    if norm < 1.0e-6:
        return np.asarray(fallback, dtype=np.float64)
    return vector / norm


def _segment_distance_to_point(start, goal, point):
    start = np.asarray(start, dtype=np.float64)
    goal = np.asarray(goal, dtype=np.float64)
    point = np.asarray(point, dtype=np.float64)
    segment = goal - start
    length_sq = float(np.dot(segment, segment))
    if length_sq < 1.0e-9:
        return float(np.linalg.norm(start - point))
    ratio = float(np.dot(point - start, segment) / length_sq)
    closest = start + np.clip(ratio, 0.0, 1.0) * segment
    return float(np.linalg.norm(closest - point))


def segment_stays_clear_of_point(start, goal, point, clearance):
    return _segment_distance_to_point(start, goal, point) >= max(0.0, float(clearance))


def _front_wander_target(current_xy, robot_xy, robot_yaw, sampler, rng, min_robot_distance, near_robot_radius):
    forward = _unit_from_yaw(robot_yaw)
    side = np.array([-forward[1], forward[0]], dtype=np.float64)
    min_front = min_robot_distance * 0.8

    # 蓝色障碍物主要在车头半圆内游走，同时不能从车后直线切过机器人。
    for _ in range(40):
        front = float(rng.uniform(min_front, near_robot_radius))
        lateral_limit = min(near_robot_radius * 0.65, max(0.2, near_robot_radius - front * 0.35))
        lateral = float(rng.uniform(-lateral_limit, lateral_limit))
        candidate = robot_xy + forward * front + side * lateral
        rel = candidate - robot_xy
        if np.dot(rel, forward) <= 0.0:
            continue
        if np.linalg.norm(rel) > near_robot_radius:
            continue
        if _segment_distance_to_point(current_xy, candidate, robot_xy) < min_robot_distance:
            continue
        if sampler.point_is_free(candidate):
            return candidate

    side_sign = 1.0
    lateral_now = float(np.dot(current_xy - robot_xy, side))
    if abs(lateral_now) > 1.0e-6:
        side_sign = 1.0 if lateral_now > 0.0 else -1.0

    for sign in (side_sign, -side_sign):
        fallback = robot_xy + forward * (min_robot_distance * 0.8)
        fallback = fallback + side * sign * min(near_robot_radius * 0.8, min_robot_distance * 1.8)
        if sampler.point_is_free(fallback):
            return fallback
    fallback = robot_xy + forward * min(near_robot_radius, max(min_robot_distance, near_robot_radius * 0.6))
    return sampler.nearest_free_point(fallback)


def _approach_target(robot_xy, current_xy, sampler, rng, min_robot_distance, near_robot_radius):
    away = _normalized_or(current_xy - robot_xy, np.array([1.0, 0.0], dtype=np.float64))
    base_angle = math.atan2(float(away[1]), float(away[0]))
    min_radius = min_robot_distance * 1.2
    max_radius = max(min_radius + 0.1, near_robot_radius * 0.8)

    # 黄色不沿当前径向直追车，而是换一个侧向角度靠近，表现上和红色区分开。
    for _ in range(40):
        sign = -1.0 if int(rng.integers(0, 2)) == 0 else 1.0
        angle = base_angle + sign * float(rng.uniform(math.pi / 3.0, math.pi))
        radius = float(rng.uniform(min_radius, max_radius))
        candidate = robot_xy + np.array([math.cos(angle), math.sin(angle)], dtype=np.float64) * radius
        if sampler.point_is_free(candidate):
            return candidate

    return sampler.random_free_point_near(robot_xy, near_robot_radius, rng)


def _leave_target(robot_xy, current_xy, sampler, rng, min_robot_distance, near_robot_radius):
    if np.linalg.norm(current_xy - robot_xy) < 1.0e-6:
        angle = float(rng.uniform(-math.pi, math.pi))
        direction = np.array([math.cos(angle), math.sin(angle)], dtype=np.float64)
    else:
        direction = _normalized_or(current_xy - robot_xy, np.array([1.0, 0.0], dtype=np.float64))
    leave_distance = max(near_robot_radius * 1.8, min_robot_distance * 3.0)
    return sampler.nearest_free_point(robot_xy + direction * leave_distance)


def _global_wander_target(current_xy, robot_xy, sampler, rng, min_robot_distance, near_robot_radius):
    if float(np.linalg.norm(current_xy - robot_xy)) < min_robot_distance:
        return _leave_target(
            robot_xy,
            current_xy,
            sampler,
            rng,
            min_robot_distance,
            near_robot_radius,
        )

    min_robot_gap = near_robot_radius * 1.4
    min_travel = near_robot_radius * 2.0
    best_candidate = None
    best_distance = -1.0

    # 绿色保持全图随机游走，但拒绝会穿过机器人安全圈的路线。
    for _ in range(80):
        candidate = sampler.random_free_point(rng)
        robot_distance = float(np.linalg.norm(candidate - robot_xy))
        if robot_distance <= min_robot_gap:
            continue
        if not segment_stays_clear_of_point(
            current_xy,
            candidate,
            robot_xy,
            min_robot_distance,
        ):
            continue

        travel_distance = float(np.linalg.norm(candidate - current_xy))
        if travel_distance > best_distance:
            best_distance = travel_distance
            best_candidate = candidate
        if travel_distance > min_travel:
            return candidate

    if best_candidate is not None:
        return best_candidate

    return _leave_target(
        robot_xy,
        current_xy,
        sampler,
        rng,
        min_robot_distance,
        near_robot_radius,
    )


def choose_behavior_target(
    behavior,
    current_xy,
    robot_xy,
    sampler,
    rng,
    state_mode,
    min_robot_distance,
    near_robot_radius,
    robot_yaw=0.0,
):
    current_xy = np.asarray(current_xy, dtype=np.float64)
    robot_xy = np.asarray(robot_xy, dtype=np.float64)
    min_robot_distance = max(0.1, float(min_robot_distance))
    near_robot_radius = max(min_robot_distance + 0.5, float(near_robot_radius))

    if behavior == "follower":
        direction = current_xy - robot_xy
        norm = float(np.linalg.norm(direction))
        if norm < 1.0e-3:
            direction = np.array([1.0, 0.0], dtype=np.float64)
        else:
            direction = direction / norm
        target = robot_xy + direction * min_robot_distance
        return sampler.nearest_free_point(target), "follow"

    if behavior == "wander_near_robot":
        return (
            _front_wander_target(
                current_xy,
                robot_xy,
                robot_yaw,
                sampler,
                rng,
                min_robot_distance,
                near_robot_radius,
            ),
            "wander_front",
        )

    if behavior == "approach_and_leave":
        distance = float(np.linalg.norm(current_xy - robot_xy))
        leave_until_distance = near_robot_radius * 1.35
        if state_mode == "leave" and distance < leave_until_distance:
            return (
                _leave_target(
                    robot_xy,
                    current_xy,
                    sampler,
                    rng,
                    min_robot_distance,
                    near_robot_radius,
                ),
                "leave",
            )
        if distance <= min_robot_distance * 1.45:
            return (
                _leave_target(
                    robot_xy,
                    current_xy,
                    sampler,
                    rng,
                    min_robot_distance,
                    near_robot_radius,
                ),
                "leave",
            )
        return (
            _approach_target(
                robot_xy,
                current_xy,
                sampler,
                rng,
                min_robot_distance,
                near_robot_radius,
            ),
            "approach",
        )

    return (
        _global_wander_target(
            current_xy,
            robot_xy,
            sampler,
            rng,
            min_robot_distance,
            near_robot_radius,
        ),
        "global",
    )


def build_dynamic_obstacles_xml(
    config: DynamicObstacleSceneConfig,
    map_center_x: float,
    map_center_y: float,
    map_half_x: float,
    map_half_y: float,
) -> str:
    config = clamp_scene_config(config)
    runtime_configs = build_runtime_configs(
        config,
        map_center_x,
        map_center_y,
        map_half_x,
        map_half_y,
    )
    if not runtime_configs:
        return ""

    lines = [
        "    <!-- Runtime-driven dynamic obstacles. They are not written to the initial map image. -->",
    ]
    for runtime in runtime_configs:
        body = runtime.name
        initial_pose = trajectory_pose(
            runtime.center,
            runtime.phase,
            runtime.speed,
            runtime.path_length,
            0.0,
        )
        x, y, z = initial_pose.position
        if config.shape == "box":
            geom = (
                f'<geom name="{body}_geom" type="box" '
                f'size="{config.radius:.3f} {config.radius:.3f} {0.5 * config.height:.3f}" '
            )
        else:
            geom = (
                f'<geom name="{body}_geom" type="cylinder" '
                f'size="{config.radius:.3f} {0.5 * config.height:.3f}" '
            )
        lines.extend([
            f'      <body name="{body}" pos="{x:.3f} {y:.3f} {z:.3f}">',
            f'        <freejoint name="{body}_joint"/>',
            "        "
            + geom
            + (
                f'mass="{config.mass:.3f}" rgba="{behavior_rgba(runtime.behavior)}" '
                'friction="1 0.1 0.1" condim="3" '
                'contype="1" conaffinity="1"/>'
            ),
            "      </body>",
        ])
    return "\n".join(lines)


def triangle_wave(offset: float, path_length: float) -> tuple[float, float]:
    length = max(0.1, float(path_length))
    wrapped = float(offset) % (2.0 * length)
    if wrapped <= length:
        return wrapped - 0.5 * length, 1.0
    return 1.5 * length - wrapped, -1.0


def trajectory_pose(
    center: tuple[float, float, float],
    phase: float,
    speed: float,
    path_length: float,
    sim_time: float,
) -> DynamicObstaclePose:
    distance = 0.5 * float(path_length) + float(phase) + float(speed) * float(sim_time)
    offset, direction = triangle_wave(distance, path_length)
    position = np.array(
        [float(center[0]) + offset, float(center[1]), float(center[2])],
        dtype=np.float64,
    )
    linear_velocity = np.array([float(speed) * direction, 0.0, 0.0], dtype=np.float64)
    yaw = 0.0 if direction >= 0.0 else math.pi
    return DynamicObstaclePose(
        position=position,
        yaw=yaw,
        linear_velocity=linear_velocity,
    )
