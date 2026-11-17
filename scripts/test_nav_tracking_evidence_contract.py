#!/usr/bin/env python3
"""Offline counterexamples for long-run and commit-bound tracking evidence.

Imports ROS message definitions but never creates a ROS context or participant.
"""
import gzip
import json
import math
import os
import tempfile
from types import SimpleNamespace as NS
import unittest

import analyze_nav_tracking as analysis
import footprint_evaluator as fe
import nav_tracking_recorder as recorder
from test_analyze_nav_tracking import Fixture, wall_grid


def snapshot_message():
    stamp = NS(sec=10, nanosec=0)
    return NS(header=NS(stamp=stamp, frame_id="odom"), source_stamp=stamp,
              ready=True, unknown_is_obstacle=True, occupied_value_threshold=100,
              localization_epoch=1, source_generation=1, publication_sequence=1,
              info=NS(width=2, height=2, resolution=0.1,
                      origin=NS(position=NS(x=0.0, y=0.0, z=0.0),
                                orientation=NS(x=0.0, y=0.0, z=0.0, w=1.0))),
              occupancy=[0] * 4, signed_distance_m=[1.0] * 4)


def snapshot_sink(directory, budget=-1):
    records = []
    os.makedirs(os.path.join(directory, "snapshot_payload"), exist_ok=True)
    return NS(_args=NS(planning_snapshot_topic="/snapshot", max_snapshot_payloads=budget),
              _output_dir=directory, _count=lambda topic: None, _start_mono=0.0,
              _seen_snapshot_identities=set(), _snapshot_payload_by_digest={},
              _snapshot_payloads_written=0, _snapshot_payloads_truncated=False,
              _snapshots=NS(write=records.append)), records


class SnapshotEvidenceTest(unittest.TestCase):
    def test_geometry_and_distance_changes_do_not_alias_occupancy_payload(self):
        with tempfile.TemporaryDirectory() as directory:
            sink, records = snapshot_sink(directory, 10)
            msg = snapshot_message()
            recorder.NavTrackingRecorder._on_snapshot(sink, msg)
            msg.publication_sequence += 1
            msg.info.origin.position.x = 3.0
            recorder.NavTrackingRecorder._on_snapshot(sink, msg)
            msg.publication_sequence += 1
            msg.signed_distance_m = [2.0] * 4
            recorder.NavTrackingRecorder._on_snapshot(sink, msg)
            self.assertEqual(len({r["payload"] for r in records}), 3)
            store = analysis.PayloadStore(directory, records)
            self.assertEqual(len(store.digests), 3)
            for record in records:
                payload = store.payload(record["payload_key"])
                self.assertEqual(payload["identity"]["info"], record["info"])

    def test_unlimited_retention_covers_the_401st_distinct_payload(self):
        with tempfile.TemporaryDirectory() as directory:
            sink, records = snapshot_sink(directory)
            msg = snapshot_message()
            for sequence in range(1, 406):
                msg.publication_sequence = sequence
                msg.info.origin.position.x = sequence * 0.1
                recorder.NavTrackingRecorder._on_snapshot(sink, msg)
            self.assertEqual(sink._snapshot_payloads_written, 405)
            self.assertFalse(sink._snapshot_payloads_truncated)
            self.assertTrue(all(r["payload"] for r in records))
            store = analysis.PayloadStore(directory, records)
            for record in records:
                self.assertIsNotNone(store.payload(record["payload_key"]))
            self.assertLessEqual(len(store._cache), 8, "replay retained every decompressed grid")

    def test_explicit_cap_still_marks_missing_evidence(self):
        with tempfile.TemporaryDirectory() as directory:
            sink, records = snapshot_sink(directory, 1)
            msg = snapshot_message()
            recorder.NavTrackingRecorder._on_snapshot(sink, msg)
            msg.publication_sequence += 1
            msg.occupancy = [100] * 4
            recorder.NavTrackingRecorder._on_snapshot(sink, msg)
            self.assertIsNone(records[-1]["payload"])
            self.assertTrue(sink._snapshot_payloads_truncated)

    def test_unlimited_layer_retention_does_not_evict_provenance(self):
        self.assertEqual(recorder.plan_layer_retention({"/a": ["one"]}, "/a", -1),
                         (None, None))


class CommitEvidenceTest(unittest.TestCase):
    def fixture(self, directory):
        fixture = Fixture(directory, "commit")
        old = fixture.add_snapshot(wall_grid(extra_blocked=((10, 15),)), 1, 101)
        new = fixture.add_snapshot(wall_grid(), 2, 102)
        reference = fixture.add_reference([(1.0, 1.5, 0.0), (1.5, 1.5, 0.0)])
        command = fixture.events[-1]["payload"]
        command.update(map_publication_sequence=1, map_generation=7,
                       validation_map_publication_sequence=2,
                       validation_map_source_generation=102)
        from test_analyze_nav_tracking import MAP_FROM_ODOM
        command["validation_from_reference"] = dict(
            MAP_FROM_ODOM, frame_id="map", child_frame_id="odom", z=0.0,
            quaternion=[0.0, 0.0, math.sin(0.15), math.cos(0.15)])
        reference["poses_se3"] = [[x, y, 0.0, 0.0, 0.0, math.sin(yaw / 2),
                                    math.cos(yaw / 2)] for x, y, yaw in reference["poses"]]
        command["reference_poses_se3"] = reference["poses_se3"]
        return fixture, command, reference, old, new

    def evaluate(self, fixture):
        return analysis.analyze_reference_at_publish(
            fixture.samples, fixture.references,
            analysis.PayloadStore(fixture.dir, fixture.snapshots),
            analysis.LayerStore(fixture.dir), fe.FootprintParams(0.10, 0.10, 0.0),
            "/minco/reference_path", fixture.events, fixture.snapshots)[0]

    def test_final_validation_snapshot_and_tf_are_atomic_evidence(self):
        with tempfile.TemporaryDirectory() as directory:
            fixture, command, _, _, _ = self.fixture(directory)
            entry = self.evaluate(fixture)
            self.assertTrue(entry["evaluated"], entry)
            self.assertTrue(entry["verdict"]["safe"], entry)
            self.assertEqual(entry["snapshot_identity"]["publication_sequence"], 2)
            self.assertEqual(command["map_publication_sequence"], 1)
            self.assertEqual(entry["transform_evidence"], "EXECUTE-bound validation transform")

    def test_wrong_bound_frame_or_source_is_an_evidence_gap(self):
        for corrupt in ("frame", "source", "quaternion", "pose_content"):
            with self.subTest(corrupt=corrupt), tempfile.TemporaryDirectory() as directory:
                fixture, command, _, _, _ = self.fixture(directory)
                if corrupt == "frame":
                    command["validation_from_reference"]["child_frame_id"] = "map"
                elif corrupt == "source":
                    command["validation_map_source_generation"] = 103
                elif corrupt == "quaternion":
                    command["validation_from_reference"]["quaternion"] = [0, 0, 0, 0]
                else:
                    command["reference_poses_se3"] = []
                self.assertFalse(self.evaluate(fixture)["evaluated"])

    def test_se3_position_and_orientation_are_projected_after_transform(self):
        # A 60 degree pitch maps z into x. Dropping z falsely leaves the pose at x=0.
        angle = math.pi / 3
        transform = dict(x=1.0, y=2.0, z=0.0, yaw=0.0,
                         quaternion=[0.0, math.sin(angle / 2), 0.0, math.cos(angle / 2)])
        pose = [0.0, 0.0, 2.0, 0.0, 0.0, 0.0, 1.0]
        projected = analysis.transform_pose(transform, (0.0, 0.0, 0.0), pose)
        self.assertAlmostEqual(projected[0], 1.0 + math.sqrt(3))
        self.assertAlmostEqual(projected[1], 2.0)
        self.assertAlmostEqual(projected[2], 0.0)
        self.assertIsNone(analysis.transform_pose(transform, (0, 0, 0)),
                          "missing 3D pose was silently promoted into complete evidence")


if __name__ == "__main__":
    unittest.main()
