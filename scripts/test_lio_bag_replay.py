#!/usr/bin/env python3
"""Offline tests for the frame-end association and the bit-exact comparison in scripts/lio_bag_replay.py.

    python3 -I scripts/test_lio_bag_replay.py

No ROS: only the pure functions are exercised (cloud_digest gets a duck-typed PointCloud2).  The module is loaded by path (hence -I).
"""

import importlib.util
import struct
import unittest
from pathlib import Path
from types import SimpleNamespace

SPEC = importlib.util.spec_from_file_location(
    "lio_bag_replay", Path(__file__).resolve().parent / "lio_bag_replay.py")
replay = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(replay)

POSE_A = (1.0, 2.0, 0.5, 0.0, 0.0, 0.0, 1.0)
POSE_B = (1.1, 2.0, 0.5, 0.0, 0.0, 0.0, 1.0)


def summary(frames, odom_sha="o", cloud_sha="c"):
    return {"frames": frames, "odom": {"sha256": odom_sha}, "cloud": {"sha256": cloud_sha}}


class FrameEndPoses(unittest.TestCase):
    def test_per_point_odometry_takes_the_last_message_up_to_the_cloud_stamp(self):
        odoms = [(10, POSE_A), (40, POSE_A), (50, POSE_B), (60, POSE_A), (100, POSE_B)]
        frames = replay.frame_end_poses(odoms, [(50, "f0"), (100, "f1")])
        self.assertEqual([f["odom_stamp_ns"] for f in frames], [50, 100])
        self.assertEqual([f["odom_in_frame"] for f in frames], [3, 2])
        self.assertEqual(frames[0]["pose_hex"], [v.hex() for v in POSE_B])

    def test_equal_stamps_keep_publication_order(self):
        frames = replay.frame_end_poses([(50, POSE_A), (50, POSE_B)], [(50, "f0")])
        self.assertEqual(frames[0]["pose_hex"], [v.hex() for v in POSE_B])

    def test_frame_without_odometry_has_no_pose(self):
        frames = replay.frame_end_poses([(10, POSE_A)], [(10, "f0"), (20, "f1")])
        self.assertIsNone(frames[1]["pose_hex"])
        self.assertEqual(frames[1]["odom_in_frame"], 0)

    def test_once_per_frame_odometry_matches_per_point_frame_ends(self):
        per_point = replay.frame_end_poses(
            [(30, POSE_A), (50, POSE_B), (70, POSE_A), (100, POSE_A)], [(50, "f0"), (100, "f1")])
        per_frame = replay.frame_end_poses([(50, POSE_B), (100, POSE_A)], [(50, "f0"), (100, "f1")])
        result = replay.compare_summaries(summary(per_point, odom_sha="x"), summary(per_frame))
        self.assertTrue(result["bit_exact"])
        self.assertFalse(result["odom_stream_identical"])


class CompareSummaries(unittest.TestCase):
    def frames(self, pose=POSE_A, sha="f"):
        return replay.frame_end_poses([(50, POSE_A), (100, pose)], [(50, "f0"), (100, sha)])

    def test_identical_runs_are_bit_exact(self):
        self.assertTrue(replay.compare_summaries(summary(self.frames()), summary(self.frames()))["bit_exact"])

    def test_one_ulp_in_a_pose_is_a_mismatch(self):
        nudged = (POSE_A[0] + 2.0 ** -52,) + POSE_A[1:]
        result = replay.compare_summaries(summary(self.frames()), summary(self.frames(pose=nudged)))
        self.assertFalse(result["bit_exact"])
        self.assertTrue(result["clouds_bit_exact"])
        self.assertEqual((result["pose_mismatch"], result["first_mismatch_frame"]), (1, 1))
        self.assertGreater(result["max_abs_position_m"], 0.0)

    def test_cloud_digest_and_frame_count_are_checked(self):
        result = replay.compare_summaries(summary(self.frames()), summary(self.frames(sha="g")))
        self.assertEqual(result["cloud_mismatch"], 1)
        self.assertFalse(result["bit_exact"])
        self.assertFalse(result["clouds_bit_exact"])
        shorter = summary(self.frames()[:1])
        self.assertFalse(replay.compare_summaries(summary(self.frames()), shorter)["bit_exact"])

    def test_quaternion_sign_flip_is_not_a_rotation_error(self):
        flipped = POSE_A[:3] + tuple(-v for v in POSE_A[3:])
        result = replay.compare_summaries(summary(self.frames()), summary(self.frames(pose=flipped)))
        self.assertFalse(result["bit_exact"])   # bits differ, but
        self.assertEqual(result["max_abs_quaternion"], 0.0)


def cloud(points, padding):
    """PointXYZI-like cloud: x y z intensity at offsets 0 4 8 16, 4 padding bytes at 12, 20 bytes
    per point."""
    data = b"".join(struct.pack("<3f", *p[:3]) + pad + struct.pack("<f", p[3])
                    for p, pad in zip(points, padding))
    fields = [SimpleNamespace(name=n, offset=o, datatype=7, count=1)
              for n, o in (("x", 0), ("y", 4), ("z", 8), ("intensity", 16))]
    return SimpleNamespace(header=SimpleNamespace(stamp=SimpleNamespace(sec=3, nanosec=50), frame_id="odom"),
                           height=1, width=len(points), is_dense=True, point_step=20, fields=fields,
                           data=data)


class CloudDigest(unittest.TestCase):
    POINTS = [(1.0, 2.0, 3.0, 9.0), (4.0, 5.0, 6.0, 7.0)]

    def test_padding_bytes_are_ignored(self):
        a = cloud(self.POINTS, [b"\x00" * 4, b"\x00" * 4])
        b = cloud(self.POINTS, [b"\xde\xad\xbe\xef", b"\x01" * 4])
        self.assertNotEqual(a.data, b.data)
        self.assertEqual(replay.cloud_digest(a), replay.cloud_digest(b))

    def test_field_values_and_stamp_are_covered(self):
        base = replay.cloud_digest(cloud(self.POINTS, [b"\x00" * 4] * 2))
        moved = [(1.0, 2.0, 3.0, 9.0), (4.0, 5.0, 6.000001, 7.0)]
        self.assertNotEqual(base, replay.cloud_digest(cloud(moved, [b"\x00" * 4] * 2)))
        later = cloud(self.POINTS, [b"\x00" * 4] * 2)
        later.header.stamp.nanosec = 51
        self.assertNotEqual(base, replay.cloud_digest(later))


if __name__ == "__main__":
    unittest.main()
