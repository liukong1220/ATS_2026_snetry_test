#!/usr/bin/env python3
"""Focused regression tests for RViz semantic validation."""

import copy
import importlib.util
import unittest
from pathlib import Path


WORKSPACE = Path(__file__).resolve().parents[1]
MODULE_PATH = WORKSPACE / "scripts/validate_navigation_config.py"
SPEC = importlib.util.spec_from_file_location("validate_navigation_config", MODULE_PATH)
assert SPEC is not None and SPEC.loader is not None
VALIDATOR = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(VALIDATOR)


class NavigationRvizContractTest(unittest.TestCase):
    def load_default_rviz(self):
        return VALIDATOR.load_yaml(
            WORKSPACE / "src/ats_sentry_bringup/rviz/sentry_default_view.rviz"
        )

    def test_current_three_colour_bounds_contract_passes(self):
        VALIDATOR.assert_navigation_rviz_contract(
            self.load_default_rviz(), "odom", "test RViz"
        )

    def test_legacy_bounds_name_is_rejected(self):
        rviz = copy.deepcopy(self.load_default_rviz())
        bounds = VALIDATOR.single_display_for_topic(
            rviz, "/rog_map/bounds", "test RViz"
        )
        bounds["Name"] = "ROGMap Local Bounds"

        with self.assertRaisesRegex(AssertionError, "three ROGMap bounds"):
            VALIDATOR.assert_navigation_rviz_contract(rviz, "odom", "test RViz")

    def test_mpc_reference_display_requires_reliable_qos(self):
        rviz = copy.deepcopy(self.load_default_rviz())
        reference = VALIDATOR.single_display_for_topic(
            rviz, "/ats_swerve_mpc/reference_horizon", "test RViz"
        )
        reference["Topic"]["Reliability Policy"] = "Best Effort"

        with self.assertRaisesRegex(AssertionError, "must use Reliable"):
            VALIDATOR.assert_navigation_rviz_contract(rviz, "odom", "test RViz")

    def test_local_voxel_display_requires_producer_rgb(self):
        rviz = copy.deepcopy(self.load_default_rviz())
        local_voxel = VALIDATOR.single_display_for_topic(
            rviz, "/rog_map/viz", "test RViz"
        )
        local_voxel["Color Transformer"] = "FlatColor"

        with self.assertRaisesRegex(AssertionError, "preserve producer voxel-state colors"):
            VALIDATOR.assert_navigation_rviz_contract(rviz, "odom", "test RViz")


if __name__ == "__main__":
    unittest.main()
