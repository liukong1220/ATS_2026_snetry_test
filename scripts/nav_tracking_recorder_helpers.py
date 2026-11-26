#!/usr/bin/env python3
"""Read-only same-clock recorder for reference / actual / command / stop evidence.

Why this exists
---------------
The runner could already tell that a leg failed, but not *why*: it sampled only
``/localization`` position, and scraped the planner's own collision verdict out
of the launch log.  Nothing recorded the committed reference next to the pose
that was actually driven, on one clock, with the snapshot identity that was in
force at that instant.  Without that pairing the three candidate explanations
for a conflict are indistinguishable:

  1. the reference was already colliding when it was published,
  2. the reference was safe and tracking left its envelope,
  3. the same pose flipped free -> occupied because the map source changed.

This node subscribes only.  It publishes nothing, calls no service, and holds no
lease, so adding it cannot change what the navigation stack decides.

Clock discipline (monotonic steady-clock durations, ROS clock only for message stamps)
------------------------------------------------------------
* Every duration and age is computed from ``time.monotonic()`` on receipt.
  A steady-clock receive-time age is a *local transport+scheduling* measure and
  is labelled ``*_age_s_mono``; it is NOT a publisher-side latency.
* ROS stamps are recorded as identity strings only and are never subtracted
  across publishers, because the profile runs ``use_sim_time: false`` with
  several independent clocks feeding the graph.
* Identity tuples (generation, publication_sequence, command_sequence, ...) are
  always taken from a single message, never assembled from two topics.

Frames
------
``/localization`` and the committed reference are both in the planner's
``global_frame`` (``odom`` in this profile).  The fused planning grid inherits
the static map frame (``map``).  Poses are therefore stored in their own frame,
and the ``map <- odom`` transform is sampled every tick so the offline analyzer
can move poses into the grid frame explicitly instead of silently assuming the
two coincide.  A tick whose transform lookup fails records
``map_from_odom: null`` and is reported as unpaired rather than evaluated.

Artifacts (all under --output-dir)
----------------------------------
``samples.jsonl``     one record per tick: pose, twist, commands, e-stop, ages,
                      active identity, and tracking error against the reference.
``references.jsonl``  every distinct reference path, with all poses.
``snapshots.jsonl``   every distinct planning snapshot's identity + geometry.
``snapshot_payload/`` gzipped full occupancy for each distinct snapshot digest.
``layers/``           gzipped source-layer grids (static / terrain / slope) on
                      digest change, for per-cell provenance.
``events.jsonl``      discrete transitions: e-stop edges, planner status,
                      execution-command mode changes, ready edges, goal changes.
``summary.json``      counters, per-topic delivery report, truncation flags.

Nothing here evaluates footprint safety or physical contact; that is
scripts/footprint_evaluator.py (geometry) and the runner's contact gate
(rigid-body solver) respectively.
"""

import argparse
from array import array
import gzip
import hashlib
import json
import math
import os
import signal
import time

from nav_tracking_payload import reference_payload_key

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.signals import SignalHandlerOptions
from geometry_msgs.msg import Twist
from nav_msgs.msg import OccupancyGrid
from nav_msgs.msg import Odometry
from nav_msgs.msg import Path
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy
from rclpy.qos import HistoryPolicy
from rclpy.qos import QoSProfile
from rclpy.qos import ReliabilityPolicy
from std_msgs.msg import Bool
from tf2_ros import Buffer
from tf2_ros import TransformListener

from ats_navigation_interfaces.msg import ExecutionCommand
from ats_navigation_interfaces.msg import PlannerStatus
from ats_navigation_interfaces.msg import PlanningMapSnapshot
from ats_navigation_interfaces.msg import PlanningMapStatus
from ats_navigation_interfaces.msg import SwerveTelemetry

try:
    from manda_can_control.msg import MotionCtrl
except ImportError:  # pragma: no cover - the chassis bridge msg may be absent
    MotionCtrl = None


def latched_qos(depth=10):
    """Reliable + transient_local: matches the adapter, lease and e-stop topics.

    Those publishers are transient_local, so a volatile subscriber would miss the
    latched sample that is already in force when the recorder starts late.
    """
    return QoSProfile(
        depth=depth,
        history=HistoryPolicy.KEEP_LAST,
        reliability=ReliabilityPolicy.RELIABLE,
        durability=DurabilityPolicy.TRANSIENT_LOCAL,
    )


def reliable_stream_qos(depth=10):
    """Reliable + volatile for large discrete events like committed reference Paths.

    A BEST_EFFORT reader silently drops fragmented multi-KB Path messages over UDP
    when a single datagram is lost, causing missing-reference gaps on short legs.
    """
    return QoSProfile(
        depth=depth,
        history=HistoryPolicy.KEEP_LAST,
        reliability=ReliabilityPolicy.RELIABLE,
        durability=DurabilityPolicy.VOLATILE,
    )


def stream_qos(depth=1):
    """Latest state, best-effort + volatile, compatible with every writer.

    Tick samples need the latest pose/velocity, not a backlog drained after a
    slow payload callback. Reference history requests its own depth explicitly.

    A RELIABLE *reader* silently matches nothing when the writer is BEST_EFFORT,
    and DDS surfaces that as "no publisher", not as an error.  That is exactly how
    an earlier revision of this recorder subscribed to ``/localization`` --
    ``localization_fusion`` publishes BEST_EFFORT / KEEP_LAST(5) / VOLATILE --
    received zero samples, and reported ``ticks_with_pose: 0``, which left every
    actual-pose question unanswerable.

    A BEST_EFFORT reader is compatible with BEST_EFFORT *and* RELIABLE writers, so
    for read-only evidence capture the permissive side is strictly safer: dropping
    an occasional sample under load costs one tick, whereas failing to match costs
    the whole question.  The queue stays deep to make those drops unlikely, and a
    drop can never fabricate a pose -- it only reduces the paired sample count,
    which the analyzer already treats as a gap rather than as a pass.
    """
    return QoSProfile(
        depth=depth,
        history=HistoryPolicy.KEEP_LAST,
        reliability=ReliabilityPolicy.BEST_EFFORT,
        durability=DurabilityPolicy.VOLATILE,
    )


def plan_layer_retention(retained, topic, budget):
    """Decide which stored layer payload to drop so the NEWEST one always fits.

    ``retained`` maps topic -> payload names in write order.  A plain
    chronological cap keeps the *earliest* content and discards the layers that
    were in force when the run finally hit its first conflict, which is precisely
    the provenance a first-conflict sample needs.  So the budget is enforced by
    eviction instead of by refusal: the newest write is always affordable.

    Preference order for the victim: the oldest non-baseline payload of whichever
    topic holds the most (so one churning layer cannot crowd out the others, and
    each topic keeps its first payload as a baseline).  Only when no topic holds a
    second payload is a baseline itself evicted -- losing an old baseline is a
    smaller evidence loss than losing the conflict-time layer.  Ties break on the
    topic name so two runs of the same recording evict the same file.

    Returns ``(victim_topic, victim_name)`` to delete, or ``(None, None)`` when the
    write already fits.  ``budget <= 0`` disables payload writing entirely and is
    reported as ``False``.
    """
    if budget == -1:
        return (None, None)
    if budget <= 0:
        return False
    total = sum(len(names) for names in retained.values())
    if total < budget:
        return (None, None)
    ranked = sorted(
        ((-len(names), name) for name, names in retained.items() if len(names) > 1))
    if not ranked:
        ranked = sorted(
            ((-len(names), name) for name, names in retained.items() if names))
    if not ranked:
        return False
    victim_topic = ranked[0][1]
    return (victim_topic, retained[victim_topic][0])


def normalize_angle(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


def yaw_from_quaternion(qx, qy, qz, qw):
    """Same yaw extraction the planner's grid geometry uses."""
    return math.atan2(2.0 * (qw * qz + qx * qy), 1.0 - 2.0 * (qy * qy + qz * qz))


def stamp_identity(stamp):
    """A ROS stamp recorded as an identity string, never as a subtractable number."""
    return "%d.%09d" % (int(stamp.sec), int(stamp.nanosec))


def digest_bytes(payload):
    return hashlib.sha256(payload).hexdigest()[:32]


def digest_int8_array(values):
    return digest_bytes(bytes((int(v) & 0xFF) for v in values))


def path_poses(msg):
    """(x, y, yaw) for each pose of a nav_msgs/Path."""
    poses = []
    for pose in msg.poses:
        position = pose.pose.position
        orientation = pose.pose.orientation
        poses.append([
            float(position.x), float(position.y),
            yaw_from_quaternion(
                float(orientation.x), float(orientation.y),
                float(orientation.z), float(orientation.w)),
        ])
    return poses


def pose_se3(pose):
    position, orientation = pose.position, pose.orientation
    return [float(position.x), float(position.y), float(position.z),
            float(orientation.x), float(orientation.y),
            float(orientation.z), float(orientation.w)]


def transform_record(transform):
    translation, rotation = transform.transform.translation, transform.transform.rotation
    return {"stamp_identity": stamp_identity(transform.header.stamp),
            "frame_id": transform.header.frame_id, "child_frame_id": transform.child_frame_id,
            "x": float(translation.x), "y": float(translation.y), "z": float(translation.z),
            "quaternion": [float(rotation.x), float(rotation.y), float(rotation.z), float(rotation.w)],
            "yaw": yaw_from_quaternion(rotation.x, rotation.y, rotation.z, rotation.w)}


def snapshot_payload_key(identity):
    """Occupancy bytes alone cannot identify a moving grid or numeric field."""
    content = {key: identity.get(key) for key in (
        "frame_id", "info", "occupancy_digest", "distance_digest",
        "occupied_value_threshold", "unknown_is_obstacle")}
    return digest_bytes(json.dumps(content, sort_keys=True).encode("utf-8"))


def grid_metadata(info):
    orientation = info.origin.orientation
    return {
        "width": int(info.width),
        "height": int(info.height),
        "resolution": float(info.resolution),
        "origin_x": float(info.origin.position.x),
        "origin_y": float(info.origin.position.y),
        "origin_z": float(info.origin.position.z),
        "origin_yaw": yaw_from_quaternion(
            float(orientation.x), float(orientation.y),
            float(orientation.z), float(orientation.w)),
    }


def tracking_error(actual, reference_poses):
    """Nearest-reference-point tracking error, expressed in that point's frame.

    Longitudinal is along the reference yaw, lateral is to its left, so a lateral
    error is directly comparable to the footprint half-width.  The nearest point
    is reported by index so a conflict can be tied back to the same index the
    planner's own collision report uses.
    """
    if not reference_poses:
        return None
    ax, ay, ayaw = actual
    best_index = 0
    best_distance_sq = None
    for index, (rx, ry, _ryaw) in enumerate(reference_poses):
        distance_sq = (ax - rx) ** 2 + (ay - ry) ** 2
        if best_distance_sq is None or distance_sq < best_distance_sq:
            best_distance_sq = distance_sq
            best_index = index
    rx, ry, ryaw = reference_poses[best_index]
    dx = ax - rx
    dy = ay - ry
    cos_yaw = math.cos(ryaw)
    sin_yaw = math.sin(ryaw)
    return {
        "nearest_index": best_index,
        "nearest_of": len(reference_poses),
        "distance_m": math.sqrt(best_distance_sq),
        "longitudinal_m": cos_yaw * dx + sin_yaw * dy,
        "lateral_m": -sin_yaw * dx + cos_yaw * dy,
        "yaw_error_rad": normalize_angle(ayaw - ryaw),
        "reference_x": rx,
        "reference_y": ry,
        "reference_yaw": ryaw,
    }


def write_gzip_json(path, payload):
    """Preserve the complete payload without thousands of tiny compressor writes."""
    encoded = json.dumps(payload)
    with gzip.open(path, "wt", compresslevel=1) as handle:
        handle.write(encoded)


class JsonLinesWriter(object):
    """Append-only JSONL sink that flushes every record.

    Flushing per record matters: the runner may kill the whole domain when a leg
    fails, and the records written up to that moment are exactly the first-violation
    evidence that must survive.
    """

    def __init__(self, path):
        self._handle = open(path, "w")
        self.count = 0

    def write(self, record):
        self._handle.write(json.dumps(record, sort_keys=True) + "\n")
        self._handle.flush()
        self.count += 1

    def close(self):
        try:
            self._handle.close()
        except Exception:
            pass
