#!/usr/bin/env python3
"""Focused regression checks for P2 unknown-fault observer evidence gates."""

import importlib.util
import sys
import unittest
from pathlib import Path

from rclpy.qos import DurabilityPolicy, ReliabilityPolicy


def load_observer_module():
    path = Path(__file__).with_name("p2_fault_observer.py")
    spec = importlib.util.spec_from_file_location("p2_fault_observer", path)
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module


OBSERVER = load_observer_module()


def valid_report():
    pairing = {
        "paired": True,
        "ready_matches": True,
        "localization_epoch_matches": True,
        "source_generation_matches_status_rog_generation": True,
    }
    return {
        "first_all_unknown_snapshot": {"all_unknown": True, "publication_sequence": 7},
        "fault_status_identity": {"ready": False},
        "latency_sec": {"fault_to_emergency_stop_true": 0.1},
        "cmd_vel_mpc_zero_window": {"sustained_zero": True},
        "motion_ctrl_zero_window": {"sustained_zero": True},
        "fault_status_snapshot_pairing": {
            **pairing,
            "snapshot": {"all_unknown": True, "publication_sequence": 7},
        },
        "pre_fault_non_empty_reference_observed": True,
        "recovery": {
            "observed": True,
            "publication_advanced_past_fault": True,
            "recovered_status_snapshot_pairing": pairing,
            "non_empty_reference_after_recovery_without_new_goal": False,
            "no_new_goal_cmd_vel_mpc_zero_window": {
                "sustained_zero": True,
            },
            "no_new_goal_motion_ctrl_zero_window": {
                "sustained_zero": True,
            },
        },
    }


class P2FaultObserverTest(unittest.TestCase):
    def test_reference_path_qos_matches_goal_manager_publisher(self):
        qos = OBSERVER.reference_path_qos()
        self.assertEqual(qos.reliability, ReliabilityPolicy.RELIABLE)
        self.assertEqual(qos.durability, DurabilityPolicy.VOLATILE)

    def test_missing_reference_baseline_fails_gate(self):
        report = valid_report()
        report["pre_fault_non_empty_reference_observed"] = False

        verdict = OBSERVER.gate_verdict(report)

        self.assertFalse(verdict["passed"])
        self.assertFalse(verdict["checks"]["pre_fault_non_empty_reference_observed"])
        self.assertFalse(verdict["checks"]["old_reference_did_not_revive"])

    def test_complete_evidence_passes_gate(self):
        verdict = OBSERVER.gate_verdict(valid_report())

        self.assertTrue(verdict["passed"])

    def test_missing_motion_control_evidence_fails_closed(self):
        report = valid_report()
        del report["motion_ctrl_zero_window"]
        del report["recovery"]["no_new_goal_motion_ctrl_zero_window"]

        verdict = OBSERVER.gate_verdict(report)

        self.assertFalse(verdict["passed"])
        self.assertFalse(verdict["checks"]["motion_ctrl_zero_in_window"])
        self.assertFalse(
            verdict["checks"]["motion_ctrl_stays_zero_without_a_new_goal"]
        )


if __name__ == "__main__":
    unittest.main()
