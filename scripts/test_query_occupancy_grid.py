#!/usr/bin/env python3

import importlib.util
import unittest
from pathlib import Path

from nav_msgs.msg import OccupancyGrid, Odometry


MODULE_PATH = Path(__file__).with_name("query_occupancy_grid.py")
SPEC = importlib.util.spec_from_file_location("query_occupancy_grid", MODULE_PATH)
QUERY = importlib.util.module_from_spec(SPEC)
assert SPEC.loader is not None
SPEC.loader.exec_module(QUERY)


def make_grid(width=11, height=11, resolution=1.0):
    grid = OccupancyGrid()
    grid.header.frame_id = "map"
    grid.info.width = width
    grid.info.height = height
    grid.info.resolution = resolution
    grid.info.origin.orientation.w = 1.0
    grid.data = [0] * (width * height)
    return grid


class FarthestFreeGoalTest(unittest.TestCase):
    def test_estimated_start_requires_matching_frame_time_and_finite_pose(self):
        grid = make_grid()
        grid.header.frame_id = "odom"
        grid.header.stamp.sec = 10
        odometry = Odometry()
        odometry.header.frame_id = "map"
        odometry.header.stamp.sec = 10
        with self.assertRaisesRegex(ValueError, "frames differ"):
            QUERY.planning_start_from_odometry(grid, odometry)
        odometry.header.frame_id = "odom"
        odometry.pose.pose.position.x = 1.25
        odometry.pose.pose.position.y = -0.75
        self.assertEqual(QUERY.planning_start_from_odometry(grid, odometry), (1.25, -0.75))
        odometry.header.stamp.sec = 9
        with self.assertRaisesRegex(ValueError, "stamps differ"):
            QUERY.planning_start_from_odometry(grid, odometry)
        odometry.header.stamp.sec = 10
        odometry.pose.pose.position.x = float("nan")
        with self.assertRaisesRegex(ValueError, "non-finite"):
            QUERY.planning_start_from_odometry(grid, odometry)

    def test_zero_limit_preserves_unbounded_selection(self):
        grid = make_grid()
        unlimited = QUERY.find_farthest_free(grid, 5.5, 5.5, 50, 0.0)
        explicit_legacy = QUERY.find_farthest_free(
            grid, 5.5, 5.5, 50, 0.0, max_distance=0.0
        )

        self.assertEqual(unlimited, explicit_legacy)
        self.assertEqual(unlimited, 0)

    def test_distance_limit_bounds_the_selected_cell(self):
        grid = make_grid()
        selected = QUERY.find_farthest_free(
            grid, 5.5, 5.5, 50, 0.0, max_distance=2.0
        )
        x = selected % grid.info.width
        y = selected // grid.info.width

        self.assertLessEqual((x - 5) ** 2 + (y - 5) ** 2, 4)
        self.assertEqual(max(abs(x - 5), abs(y - 5)), 2)

    def test_limit_smaller_than_one_cell_reports_no_goal(self):
        grid = make_grid()
        with self.assertRaisesRegex(ValueError, "no reachable cell beyond"):
            QUERY.find_farthest_free(
                grid, 5.5, 5.5, 50, 0.0, max_distance=0.49
            )


if __name__ == "__main__":
    unittest.main()
