#!/usr/bin/env python3
"""Independent map evaluation must distinguish missing and invented walls."""
import math
from pathlib import Path
import sys
import unittest

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))
import s3_map_quality as quality


class MapQuality(unittest.TestCase):
    def test_scene_truth_preserves_walls_and_excludes_ground_and_ramp(self):
        from s3_scene_geometry import vertical_surface_samples
        vertices = np.array([[0, 0, 0], [2, 0, 0], [2, 0, 1],
                             [0, 2, 0], [2, 2, 0.5]])
        faces = np.array([[0, 1, 2], [0, 1, 3], [1, 3, 4]])
        points = vertical_surface_samples(vertices, faces, 0.02)
        self.assertGreater(len(points), 50)
        np.testing.assert_allclose(points[:, 1], 0.0)
        self.assertLess(points[:, 0].min(), 0.01)
        self.assertGreater(points[:, 0].max(), 1.99)

    def test_rotated_map_cell_centres_use_yaml_origin(self):
        points = quality.cell_centres(2, 1, 1.0, [10, -2, math.pi / 2])
        np.testing.assert_allclose(points, [[9.5, -1.5], [9.5, -0.5]])

    def test_unknown_and_free_misses_are_reported_separately(self):
        result = quality.compare_masks(np.array([100, -1, 0, 100]),
                                       np.array([True, True, True, False]))
        self.assertEqual(result["true_positive_cells"], 1)
        self.assertEqual(result["missed_wall_cells"], 2)
        self.assertEqual(result["missed_unknown_cells"], 1)
        self.assertEqual(result["missed_free_cells"], 1)
        self.assertEqual(result["false_positive_cells"], 1)

    def test_distance_is_to_box_surface_not_its_centre(self):
        distances = quality.box_surface_distances(
            np.array([[1.0, 0.0], [0.0, 0.0], [1.1, 0.0], [2.0, 2.0]]),
            [[0.0, 0.0, 1.0, 1.0]])
        np.testing.assert_allclose(distances, [0.0, 1.0, 0.1, math.sqrt(2)])

    def test_single_point_cannot_be_called_a_registration_cloud(self):
        with self.assertRaisesRegex(ValueError, "geometric support"):
            quality.cloud_geometry(np.array([[1.0, 2.0, 3.0]]))
        # Two wall directions and the ground provide spatial support.
        result = quality.cloud_geometry(np.array([
            [0, 0, 0], [1, 0, 0], [0, 1, 0], [1, 1, 0],
            [0, 0, 1], [1, 0, 1], [0, 1, 1], [1, 1, 1]]))
        self.assertEqual(result["points"], 8)
        self.assertGreater(result["covariance_eigenvalues"][0], 0)


if __name__ == "__main__":
    unittest.main()
