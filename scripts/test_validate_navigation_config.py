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

    def test_full_navigation_configuration_contract_passes(self):
        VALIDATOR.main()

    def test_snapshot_lease_must_cover_projection_period(self):
        adapter = {"projection_rate_hz": 0.5}
        goal_manager = {
            "planning_snapshot_timeout_sec": 1.0,
            "map_ready_timeout_sec": 1.0,
        }

        with self.assertRaisesRegex(AssertionError, "publication period"):
            VALIDATOR.assert_planning_snapshot_lease_contract(
                adapter, goal_manager
            )

    def test_snapshot_lease_matches_map_heartbeat(self):
        adapter = {"projection_rate_hz": 0.5}
        goal_manager = {
            "planning_snapshot_timeout_sec": 3.0,
            "map_ready_timeout_sec": 5.0,
        }

        with self.assertRaisesRegex(AssertionError, "must match"):
            VALIDATOR.assert_planning_snapshot_lease_contract(
                adapter, goal_manager
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
            rviz, "/ats_swerve_mpc/predicted_path", "test RViz"
        )
        reference["Topic"]["Reliability Policy"] = "Best Effort"

        with self.assertRaisesRegex(AssertionError, "must use Reliable"):
            VALIDATOR.assert_navigation_rviz_contract(rviz, "odom", "test RViz")

    def test_mpc_follow_display_requires_billboards_and_elevation(self):
        rviz = copy.deepcopy(self.load_default_rviz())
        reference = VALIDATOR.single_display_for_topic(
            rviz, "/ats_swerve_mpc/predicted_path", "test RViz"
        )
        reference["Line Style"] = "Lines"

        with self.assertRaisesRegex(AssertionError, "line style must be 'Billboards'"):
            VALIDATOR.assert_navigation_rviz_contract(rviz, "odom", "test RViz")

        reference = VALIDATOR.single_display_for_topic(
            rviz, "/ats_swerve_mpc/predicted_path", "test RViz"
        )
        reference["Line Style"] = "Billboards"
        reference["Offset"]["Z"] = 0.0

        with self.assertRaisesRegex(AssertionError, "Z offset must be 0.12"):
            VALIDATOR.assert_navigation_rviz_contract(rviz, "odom", "test RViz")

    def test_local_voxel_display_requires_producer_rgb(self):
        rviz = copy.deepcopy(self.load_default_rviz())
        local_voxel = VALIDATOR.single_display_for_topic(
            rviz, "/rog_map/viz", "test RViz"
        )
        local_voxel["Color Transformer"] = "FlatColor"

        with self.assertRaisesRegex(AssertionError, "preserve producer voxel-state colors"):
            VALIDATOR.assert_navigation_rviz_contract(rviz, "odom", "test RViz")

    def test_global_fused_esdf_display_contract_passes(self):
        VALIDATOR.assert_global_fused_esdf_display(self.load_default_rviz(), "test RViz")

    def test_global_fused_esdf_rejects_volatile_qos_and_hidden_layer(self):
        rviz = copy.deepcopy(self.load_default_rviz())
        display = VALIDATOR.single_display_for_topic(
            rviz, VALIDATOR.GLOBAL_FUSED_ESDF_TOPIC, "test RViz"
        )
        display["Topic"]["Durability Policy"] = "Volatile"
        with self.assertRaisesRegex(AssertionError, "Transient Local"):
            VALIDATOR.assert_global_fused_esdf_display(rviz, "test RViz")

        rviz = copy.deepcopy(self.load_default_rviz())
        display = VALIDATOR.single_display_for_topic(
            rviz, VALIDATOR.GLOBAL_FUSED_ESDF_TOPIC, "test RViz"
        )
        display["Enabled"] = False
        with self.assertRaisesRegex(AssertionError, "must be enabled"):
            VALIDATOR.assert_global_fused_esdf_display(rviz, "test RViz")

    def test_minco_intermediate_guide_display_requires_reliable_qos(self):
        rviz = copy.deepcopy(self.load_default_rviz())
        guide = VALIDATOR.single_display_for_topic(
            rviz, "/minco/preprocessed_guide", "test RViz"
        )
        guide["Topic"]["Reliability Policy"] = "Best Effort"
        with self.assertRaisesRegex(AssertionError, "must use Reliable"):
            VALIDATOR.assert_minco_guide_displays(rviz, "test RViz")


if __name__ == "__main__":
    unittest.main()
