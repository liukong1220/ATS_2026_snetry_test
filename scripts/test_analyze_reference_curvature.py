#!/usr/bin/env python3

import importlib.util
import math
import unittest
from pathlib import Path


MODULE_PATH = Path(__file__).with_name("analyze_reference_curvature.py")
SPEC = importlib.util.spec_from_file_location("analyze_reference_curvature", MODULE_PATH)
CURVATURE = importlib.util.module_from_spec(SPEC)
assert SPEC.loader is not None
SPEC.loader.exec_module(CURVATURE)


def arc(radius, sweep, count):
    return [(radius * math.cos(sweep * i / (count - 1)),
             radius * math.sin(sweep * i / (count - 1))) for i in range(count)]


class ResampleTest(unittest.TestCase):
    def test_resample_uses_fixed_arc_length_step(self):
        points = CURVATURE.resample_by_arc_length([(0.0, 0.0), (1.0, 0.0)], 0.25)
        self.assertEqual(len(points), 5)
        for index, point in enumerate(points):
            self.assertAlmostEqual(point[0], 0.25 * index)

    def test_resample_keeps_end_point_of_non_multiple_length(self):
        points = CURVATURE.resample_by_arc_length([(0.0, 0.0), (1.0, 0.0)], 0.3)
        self.assertAlmostEqual(points[-1][0], 1.0)

    def test_duplicate_points_are_removed(self):
        self.assertEqual(len(CURVATURE.dedupe([(0, 0), (0, 0), (1, 0), (1, 0)])), 2)


class MetricsTest(unittest.TestCase):
    def test_straight_line_has_zero_curvature(self):
        metrics = CURVATURE.path_metrics([(0.1 * i, 0.0) for i in range(50)])
        self.assertAlmostEqual(metrics["kmax"], 0.0)
        self.assertEqual(metrics["sign_flips"], 0)

    def test_circle_curvature_is_independent_of_input_sampling(self):
        dense = CURVATURE.path_metrics(arc(2.0, math.pi / 2, 400))
        sparse = CURVATURE.path_metrics(arc(2.0, math.pi / 2, 40))
        self.assertAlmostEqual(dense["k95"], 0.5, delta=0.02)
        self.assertAlmostEqual(sparse["k95"], 0.5, delta=0.06)

    def test_stacked_terminal_points_do_not_create_spike(self):
        # A time-sampled path whose last samples barely move (terminal yaw tail)
        # produced the kmax artefact; arc-length resampling must not.
        path = [(0.1 * i, 0.0) for i in range(30)]
        last = path[-1]
        path += [(last[0] + 1e-7 * i, 1e-7 * (i % 2)) for i in range(1, 20)]
        metrics = CURVATURE.path_metrics(path)
        self.assertLess(metrics["kmax"], 1e-3)

    def test_s_curve_counts_one_sign_flip(self):
        left = arc(1.0, math.pi / 2, 100)
        right = [(2.0 - x, 2.0 - y) for x, y in reversed(arc(1.0, math.pi / 2, 100))]
        # left ends at (0, 1); shift the mirrored arc so it continues from there.
        offset = (left[-1][0] - right[0][0], left[-1][1] - right[0][1])
        path = left + [(x + offset[0], y + offset[1]) for x, y in right[1:]]
        metrics = CURVATURE.path_metrics(path)
        self.assertEqual(metrics["sign_flips"], 1)

    def test_summary_skips_republished_duplicate_paths(self):
        path = [(0.1 * i, 0.0) for i in range(10)]
        summary = CURVATURE.summarize([(1, path), (2, path)], 0.05, 2, 0.05)
        self.assertEqual(summary["reference_count"], 1)


if __name__ == "__main__":
    unittest.main()
