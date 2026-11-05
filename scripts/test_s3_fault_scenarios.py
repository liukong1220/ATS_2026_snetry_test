#!/usr/bin/env python3
"""Fault evidence may not reuse pre-fault zeros or another domain's process."""
import unittest
from s3_fault_scenarios import process_matches, sustained_zero, planner_latency_ms, fixture_marker, obstacle_region


class FaultEvidence(unittest.TestCase):
    def test_continuity_uses_observation_time_and_rejects_pose_jumps(self):
        from s3_fault_scenarios import odom_continuity
        normal = [{"stamp": 1.0, "x": 0., "y": 0.},
                  {"stamp": 1.05, "x": 0.04, "y": 0.}]
        self.assertTrue(odom_continuity(normal)["passed"])
        normal[-1]["x"] = 0.30
        self.assertFalse(odom_continuity(normal)["passed"])
        normal[-1]["stamp"] = 1.0
        self.assertFalse(odom_continuity(normal)["passed"])

    def test_no_path_floor_uses_the_effective_planner_parameters(self):
        from s3_fault_scenarios import effective_search_floor
        p = {"footprint_length": 0.70, "footprint_width": 0.55,
             "footprint_safety_margin": 0.03, "jps_safe_distance": 0.2,
             "search_clearance_floor": -1.0}
        self.assertAlmostEqual(effective_search_floor(p, 0.1), 0.305 + 0.1 / 2**0.5)
        p["search_clearance_floor"] = 0.25
        self.assertEqual(effective_search_floor(p, 0.1), 0.25)

    def test_only_exact_owned_frontend_in_the_selected_domain_matches(self):
        command = ["/install/point_lio/lib/point_lio/pointlio_mapping", "--ros-args"]
        self.assertTrue(process_matches(command, {"ROS_DOMAIN_ID": "232"}, 232, True))
        self.assertFalse(process_matches(command, {"ROS_DOMAIN_ID": "77"}, 232, True))
        self.assertFalse(process_matches(command, {"ROS_DOMAIN_ID": "232"}, 232, False))
        self.assertFalse(process_matches(["python3", "pointlio_mapping"],
                                         {"ROS_DOMAIN_ID": "232"}, 232, True))

    def test_zero_window_requires_fresh_continuous_samples(self):
        samples = [(1.0, [0, 0, 0]), (1.5, [0, 0, 0])]
        self.assertFalse(sustained_zero(samples, 2.0, 0.4))
        samples += [(2.0, [0, 0, 0]), (2.2, [0.1, 0, 0]), (2.5, [0, 0, 0])]
        self.assertFalse(sustained_zero(samples, 2.0, 0.4))
        samples += [(2.7, [0, 0, 0]), (2.95, [0, 0, 0])]
        self.assertTrue(sustained_zero(samples, 2.0, 0.4))
        samples.append((3.0, [0.1, 0, 0]))
        self.assertFalse(sustained_zero(samples, 2.0, 0.4), "an earlier zero window hid resumed motion")


    def test_latency_requires_the_matching_post_insertion_planner_commit(self):
        marker = 1_000_000_000
        commits = [
            {"stamp_ns": 20, "received_ns": marker - 10, "committed_ns": marker + 20},
            {"stamp_ns": 21, "received_ns": marker + 10_000_000, "committed_ns": marker + 40_000_000},
            {"stamp_ns": 22, "received_ns": marker + 20_000_000, "committed_ns": marker + 90_000_000},
        ]
        self.assertEqual(planner_latency_ms(marker, 21, commits), 40.0)
        for commit in commits:
            commit["digest"] = "actual"
        self.assertEqual(planner_latency_ms(marker, 21, commits, "actual"), 40.0)
        with self.assertRaises(ValueError):
            planner_latency_ms(marker, 21, commits, "different-grid-same-stamp")
        with self.assertRaises(ValueError):
            planner_latency_ms(marker, 20, commits)
        with self.assertRaises(ValueError):
            planner_latency_ms(marker, 999, commits)
        with self.assertRaises(ValueError):
            planner_latency_ms(marker + 100_000_000, 21, commits)


    def test_physical_marker_must_match_the_post_request_position(self):
        text = 'S3 obstacle_fixture {"steady_ns": 10, "position": [2.0, 0.0, 0.3]}\n'
        text += 'S3 obstacle_fixture {"steady_ns": 21, "position": [2.0, 0.0, -10.0]}\n'
        self.assertIsNone(fixture_marker(text, 20, [2., 0., 0.3]))
        text += 'S3 obstacle_fixture {"steady_ns": 25, "position": [2.0, 0.0, 0.3]}\n'
        self.assertEqual(fixture_marker(text, 20, [2., 0., 0.3])["steady_ns"], 25)


    def test_obstacle_visibility_covers_the_complete_physical_box(self):
        from types import SimpleNamespace as N
        grid = N(info=N(resolution=0.1, width=50, height=20,
                        origin=N(position=N(x=0.0, y=-1.0))), data=[0] * 1000)
        grid.data[12 * 50 + 22] = 100  # far side at (2.25, 0.25)
        self.assertIn(100, obstacle_region(grid))
        grid.data[12 * 50 + 22] = 0
        grid.data[12 * 50 + 30] = 100  # unrelated object outside the box
        self.assertNotIn(100, obstacle_region(grid))

    def test_full_field_fixture_region_uses_odom_and_rejects_partial_unknown_coverage(self):
        from types import SimpleNamespace as N
        from s3_fixture_scene import fixture_site
        transform = {"translation": {"x": 4.5, "y": 6.25, "z": .285},
                     "rotation": {"x": 0., "y": 0., "z": 2**-.5, "w": 2**-.5}}
        site = fixture_site([4.5, 8.25, .585], transform)
        grid = N(info=N(resolution=.1, width=60, height=60,
                        origin=N(position=N(x=-1., y=-3.),
                                 orientation=N(x=0., y=0., z=0., w=1.))), data=[0] * 3600)
        grid.data[32 * 60 + 32] = 100  # odom (2.25, .25), map (4.25, 8.5)
        self.assertIn(100, obstacle_region(grid, site))
        grid.data[32 * 60 + 32] = 0
        grid.data[32 * 60 + 45] = 100
        self.assertNotIn(100, obstacle_region(grid, site))
        site["odom_position"][0] = -1.0
        self.assertIn(-1, obstacle_region(grid, site), "a clipped box was called known-free")

    def test_saved_dynamic_region_is_relative_to_the_world_box_and_raised_ground(self):
        from unittest.mock import patch
        import numpy as np
        from s3_fault_scenarios import saved_fixture_counts
        points = np.array([[4.5, 8.25, .585], [4.5, 8.25, .285], [2., 0., .3]])
        centres = np.array([[4.5, 8.25], [2., 0.], [4.5, 9.25]])
        with patch("s3_map_quality.read_ascii_pcd", return_value=points), \
             patch("s3_map_quality.load_map", return_value=({}, np.array([100, 100, 100]), centres)):
            self.assertEqual(saved_fixture_counts("unused", [4.5, 8.25, .585]),
                             {"cloud_obstacle_points": 1, "pgm_obstacle_cells": 1})


if __name__ == "__main__":
    unittest.main()
