#!/usr/bin/env python3
"""Offline counterexamples for long-run and commit-bound tracking evidence.

Imports ROS message definitions but never creates a ROS context or participant.
"""
import gzip
import hashlib
import json
import math
import os
import tempfile
from types import SimpleNamespace as NS
import unittest

import analyze_nav_tracking as analysis
import footprint_evaluator as fe
import nav_tracking_recorder as recorder
import nav_tracking_payload as payload_archive
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


class ExecutionPayloadTest(unittest.TestCase):
    fixture = CommitEvidenceTest.fixture
    evaluate = CommitEvidenceTest.evaluate

    @staticmethod
    def payload_key(payload):
        encoded = json.dumps(payload, sort_keys=True, separators=(",", ":")).encode()
        return hashlib.sha256(encoded).hexdigest()

    def compact_fixture(self, directory):
        fixture, command, reference, _, _ = self.fixture(directory)
        payload = {
            "frame_id": command["reference_frame_id"],
            "stamp_identity": command["reference_stamp_identity"],
            "pose_count": command["reference_pose_count"],
            "poses": command.pop("reference_poses"),
            "poses_se3": command.pop("reference_poses_se3"),
            "pose_stamp_identities": [reference["stamp_identity"]] * reference["pose_count"],
            "pose_frame_ids": [reference["frame_id"]] * reference["pose_count"],
        }
        key = self.payload_key(payload)
        command["reference_payload_key"] = key
        definition = {"kind": "execution_reference_payload",
                      "payload": {"key": key, "reference": payload}, "mono_s": 0.0}
        fixture.events.insert(0, definition)
        return fixture, command, definition

    def test_compact_authority_reconstructs_the_same_q1_verdict(self):
        with tempfile.TemporaryDirectory() as directory:
            fixture, _, _ = self.compact_fixture(directory)
            verdict = self.evaluate(fixture)
            self.assertTrue(verdict["evaluated"], verdict)
            self.assertTrue(verdict["verdict"]["safe"], verdict)

    def test_missing_or_corrupt_payload_cannot_pass_q1(self):
        for corruption in ("missing", "hash", "frame", "count", "conflicting_definition"):
            with self.subTest(corruption=corruption), tempfile.TemporaryDirectory() as directory:
                fixture, command, definition = self.compact_fixture(directory)
                if corruption == "missing":
                    fixture.events.remove(definition)
                elif corruption == "hash":
                    definition["payload"]["reference"]["poses"][0][0] += 0.01
                elif corruption == "frame":
                    command["reference_frame_id"] = "wrong_frame"
                elif corruption == "count":
                    command["reference_pose_count"] += 1
                else:
                    import copy
                    conflicting = copy.deepcopy(definition)
                    conflicting["payload"]["reference"]["poses"][0][0] += 0.01
                    fixture.events.append(conflicting)
                self.assertFalse(self.evaluate(fixture)["evaluated"])

    def test_same_stamp_changed_content_remains_ambiguous(self):
        with tempfile.TemporaryDirectory() as directory:
            import copy
            fixture, command, definition = self.compact_fixture(directory)
            changed = copy.deepcopy(definition)
            changed["payload"]["reference"]["poses"][0][0] += 0.01
            key = self.payload_key(changed["payload"]["reference"])
            changed["payload"]["key"] = key
            other_command = dict(command, reference_payload_key=key, command_sequence=999)
            fixture.events.extend([changed, {"kind": "execution_command",
                                             "payload": other_command, "mono_s": 1.1}])
            self.assertFalse(self.evaluate(fixture)["evaluated"])

    def test_payload_resolution_rejects_forward_or_nonstring_keys(self):
        with tempfile.TemporaryDirectory() as directory:
            fixture, command, definition = self.compact_fixture(directory)
            fixture.events.remove(definition)
            fixture.events.append(definition)
            resolved = payload_archive.execution_commands(fixture.events)
            self.assertTrue(resolved[0].get("reference_payload_error"))
            command["reference_payload_key"] = []
            self.assertTrue(payload_archive.execution_commands(fixture.events)[0].get(
                "reference_payload_error"))

    def test_repeated_identical_definitions_share_the_resolved_pose_content(self):
        with tempfile.TemporaryDirectory() as directory:
            fixture, command, definition = self.compact_fixture(directory)
            fixture.events.append(definition)
            fixture.events.append({"kind": "execution_command", "payload": dict(command)})
            resolved = payload_archive.execution_commands(fixture.events)
            self.assertEqual(len(resolved), 2)
            self.assertFalse(any(c.get("reference_payload_error") for c in resolved))
            self.assertIs(resolved[0]["reference_poses"], resolved[1]["reference_poses"])

    def test_recorder_keeps_each_lease_but_writes_reference_content_once(self):
        events = []
        sink = NS(_args=NS(execution_command_topic="/execution"),
                  _count=lambda topic: None, _execution=None,
                  _execution_payload_keys=set(),
                  _event=lambda kind, payload: events.append({"kind": kind, "payload": payload}))
        stamp = NS(sec=10, nanosec=5)
        pose = NS(position=NS(x=1.0, y=2.0, z=0.3),
                  orientation=NS(x=0.0, y=0.0, z=0.0, w=1.0))
        path = NS(header=NS(stamp=stamp, frame_id="odom"),
                  poses=[NS(header=NS(stamp=stamp, frame_id="odom"), pose=pose)])
        msg = NS(header=NS(stamp=stamp), mode=1, manager_incarnation=1,
                 command_sequence=1, goal_id=1, localization_epoch=1,
                 map_generation=1, map_publication_sequence=1, failure_reason=0,
                 yaw_authority=3, reference=path)
        for sequence in range(1, 31):
            msg.command_sequence = sequence
            recorder.NavTrackingRecorder._on_execution(sink, msg)
        commands = [e["payload"] for e in events if e["kind"] == "execution_command"]
        definitions = [e["payload"] for e in events if e["kind"] == "execution_reference_payload"]
        self.assertEqual([c["command_sequence"] for c in commands], list(range(1, 31)))
        self.assertEqual(len(definitions), 1)
        self.assertTrue(all(c["reference_payload_key"] == definitions[0]["key"]
                            for c in commands))
        self.assertTrue(all("reference_poses" not in c for c in commands))
        # Reusing a stamp never authorizes dropping a changed pose or timing.
        pose.position.x += 0.01
        msg.command_sequence += 1
        recorder.NavTrackingRecorder._on_execution(sink, msg)
        path.poses[0].header.stamp = NS(sec=10, nanosec=6)
        msg.command_sequence += 1
        recorder.NavTrackingRecorder._on_execution(sink, msg)
        definitions = [e["payload"] for e in events if e["kind"] == "execution_reference_payload"]
        self.assertEqual(len(definitions), 3)
        self.assertEqual(len({d["key"] for d in definitions}), 3)


if __name__ == "__main__":
    unittest.main()
