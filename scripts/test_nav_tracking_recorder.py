#!/usr/bin/env python3
"""Offline test for the pure logic in scripts/nav_tracking_recorder.py.

No ROS graph and no simulation.  What is checked here is exactly the part that
could silently corrupt the evidence: yaw extraction, angle wrap, the frame the
tracking error is expressed in, the nearest-index attribution, stamp identity
formatting (identity, never a subtractable number), digest stability, and the
QoS choices that decide whether a latched publisher is heard at all.
"""

import math
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

FAILURES = []


def check(condition, message):
    if condition:
        print("ok: " + message)
    else:
        FAILURES.append(message)
        print("FAIL: " + message)


def near(a, b, tol=1e-9):
    return a is not None and b is not None and abs(a - b) <= tol


def main():
    import nav_tracking_recorder as recorder
    from rclpy.qos import DurabilityPolicy
    from rclpy.qos import ReliabilityPolicy

    # --- yaw extraction must match the grid geometry the planner uses ---------
    for yaw in (0.0, 0.4, -1.3, 2.9, math.pi - 1e-6, -math.pi + 1e-6):
        quaternion = (0.0, 0.0, math.sin(0.5 * yaw), math.cos(0.5 * yaw))
        got = recorder.yaw_from_quaternion(*quaternion)
        check(near(got, yaw, 1e-12),
              "yaw_from_quaternion round-trips %.6f rad (got %.12f)" % (yaw, got))
    # A quaternion with roll/pitch present must still yield the planar yaw the
    # grid uses, not an Euler-order-dependent value.
    roll = 0.3
    yaw = 0.9
    qw = math.cos(0.5 * roll) * math.cos(0.5 * yaw)
    qx = math.sin(0.5 * roll) * math.cos(0.5 * yaw)
    qy = math.sin(0.5 * roll) * math.sin(0.5 * yaw)
    qz = math.cos(0.5 * roll) * math.sin(0.5 * yaw)
    check(near(recorder.yaw_from_quaternion(qx, qy, qz, qw), yaw, 1e-12),
          "yaw_from_quaternion ignores roll and returns the planar yaw")

    # --- angle wrap ----------------------------------------------------------
    check(near(recorder.normalize_angle(3.0 * math.pi), math.pi, 1e-9) or
          near(recorder.normalize_angle(3.0 * math.pi), -math.pi, 1e-9),
          "normalize_angle folds 3*pi onto +/-pi")
    check(near(recorder.normalize_angle(0.1 - (-0.1 + 2.0 * math.pi)), 0.2, 1e-9),
          "normalize_angle keeps a small difference across the +/-pi seam")

    # --- stamp identity is an identity, not arithmetic ------------------------
    class Stamp(object):
        def __init__(self, sec, nanosec):
            self.sec = sec
            self.nanosec = nanosec

    check(recorder.stamp_identity(Stamp(12, 5)) == "12.000000005",
          "stamp_identity zero-pads nanoseconds so it cannot be read as a float sum")
    check(recorder.stamp_identity(Stamp(0, 0)) == "0.000000000",
          "stamp_identity renders an unset stamp explicitly rather than as empty")
    check(isinstance(recorder.stamp_identity(Stamp(3, 1)), str),
          "stamp_identity returns a string, so it cannot be subtracted by accident")

    # --- digest stability ----------------------------------------------------
    first = recorder.digest_int8_array([0, 1, -1, 100, 99])
    same = recorder.digest_int8_array([0, 1, -1, 100, 99])
    unknown_vs_free = recorder.digest_int8_array([0, 1, 0, 100, 99])
    check(first == same, "digest_int8_array is stable for identical occupancy")
    check(first != unknown_vs_free,
          "digest_int8_array separates unknown (-1) from free (0)")
    check(recorder.digest_int8_array([100]) != recorder.digest_int8_array([99]),
          "digest_int8_array separates threshold-crossing values")

    # --- tracking error frame and attribution --------------------------------
    check(recorder.tracking_error((0.0, 0.0, 0.0), []) is None,
          "tracking_error reports nothing rather than zero when no reference exists")

    # Reference heading +x; the robot is 0.2 m ahead and 0.1 m to its left.
    straight = [[0.0, 0.0, 0.0], [1.0, 0.0, 0.0], [2.0, 0.0, 0.0]]
    error = recorder.tracking_error((1.2, 0.1, 0.05), straight)
    check(error["nearest_index"] == 1, "nearest reference index is the closest point")
    check(near(error["longitudinal_m"], 0.2, 1e-12),
          "longitudinal error is measured along the reference yaw")
    check(near(error["lateral_m"], 0.1, 1e-12),
          "lateral error is measured to the reference's left")
    check(near(error["yaw_error_rad"], 0.05, 1e-12), "yaw error is actual minus reference")

    # Same geometry rotated 90 deg: the decomposition must rotate with it, which is
    # what makes a lateral error comparable to the footprint half-width.
    turned = [[0.0, 0.0, 0.5 * math.pi], [0.0, 1.0, 0.5 * math.pi]]
    error = recorder.tracking_error((-0.1, 1.2, 0.5 * math.pi), turned)
    check(error["nearest_index"] == 1, "rotated case still picks the nearest point")
    check(near(error["longitudinal_m"], 0.2, 1e-12),
          "rotated longitudinal error follows the reference yaw")
    check(near(error["lateral_m"], 0.1, 1e-12),
          "rotated lateral error follows the reference's left")

    # Yaw error across the seam must not report ~2*pi.
    seam = [[0.0, 0.0, math.pi - 0.05]]
    error = recorder.tracking_error((0.0, 0.0, -math.pi + 0.05), seam)
    check(near(abs(error["yaw_error_rad"]), 0.1, 1e-9),
          "yaw error across +/-pi is the short way (%.6f rad)" % error["yaw_error_rad"])

    # A far-away actual pose still yields a finite record; a missing one does not.
    error = recorder.tracking_error((50.0, 50.0, 0.0), straight)
    check(error["nearest_index"] == 2 and error["distance_m"] > 1.0,
          "a distant pose is attributed to the last reference point with its distance")

    # --- QoS: a volatile subscriber would miss the latched lease --------------
    latched = recorder.latched_qos()
    stream = recorder.stream_qos()
    check(latched.durability == DurabilityPolicy.TRANSIENT_LOCAL,
          "latched_qos is transient_local, so the in-force lease/e-stop is received")
    check(latched.reliability == ReliabilityPolicy.RELIABLE,
          "latched_qos is reliable, matching the adapter and goal-manager publishers")
    check(stream.durability == DurabilityPolicy.VOLATILE,
          "stream_qos is volatile, matching the odometry/command publishers")
    # The rule, not the accident: a RELIABLE reader is INCOMPATIBLE with a
    # BEST_EFFORT writer and DDS reports that as silence rather than as an error.
    # /localization is published BEST_EFFORT, so a reliable subscription here
    # recorded zero poses and made every actual-vs-reference question
    # unanswerable.  BEST_EFFORT matches both writer kinds, so it is the only
    # choice that cannot silently drop a whole topic.
    check(stream.reliability == ReliabilityPolicy.BEST_EFFORT,
          "stream_qos is best_effort, so a BEST_EFFORT publisher still matches")
    check(stream.depth >= 50,
          "stream_qos keeps a deep queue so a slow tick does not drop command samples")

    # --- layer retention keeps the conflict-time content ---------------------
    # A plain chronological cap keeps the earliest layers and discards the ones in
    # force at the first conflict, which is the only moment provenance matters.
    check(recorder.plan_layer_retention({"/a": ["a1"]}, "/a", 4) == (None, None),
          "a write below the budget evicts nothing")
    check(recorder.plan_layer_retention(
              {"/a": ["a1", "a2", "a3"], "/b": ["b1"]}, "/b", 4) == ("/a", "a1"),
          "at budget the fattest topic loses its oldest non-baseline payload, "
          "so one churning layer cannot crowd out the others")
    check(recorder.plan_layer_retention(
              {"/a": ["a1"], "/b": ["b1"]}, "/a", 2) == ("/a", "a1"),
          "with no non-baseline payload left a baseline is evicted, because the "
          "newest layer content must still be written")
    check(recorder.plan_layer_retention({"/a": ["a1"]}, "/a", 0) is False,
          "a non-positive budget disables payload writing outright")
    check(recorder.plan_layer_retention(
              {"/a": ["a1", "a2"], "/b": ["b1", "b2"]}, "/a", 4) == ("/a", "a1"),
          "ties break on the topic name so two runs evict the same file")

    # --- grid metadata keeps origin yaw --------------------------------------
    class Vec(object):
        def __init__(self, x=0.0, y=0.0, z=0.0, w=1.0):
            self.x, self.y, self.z, self.w = x, y, z, w

    class Origin(object):
        def __init__(self, yaw):
            self.position = Vec(1.5, -2.5)
            self.orientation = Vec(0.0, 0.0, math.sin(0.5 * yaw), math.cos(0.5 * yaw))

    class Info(object):
        def __init__(self, yaw):
            self.width, self.height, self.resolution = 40, 30, 0.1
            self.origin = Origin(yaw)

    meta = recorder.grid_metadata(Info(0.7))
    check(near(meta["origin_yaw"], 0.7, 1e-12),
          "grid_metadata preserves origin yaw instead of dropping it")
    check(meta["width"] == 40 and meta["height"] == 30 and near(meta["resolution"], 0.1),
          "grid_metadata carries the lattice geometry the evaluator needs")
    check(near(meta["origin_x"], 1.5) and near(meta["origin_y"], -2.5),
          "grid_metadata carries the grid origin")

    if FAILURES:
        print("RESULT: nav tracking recorder test FAILED (%d)" % len(FAILURES))
        return 1
    print("RESULT: nav tracking recorder test PASSED")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
