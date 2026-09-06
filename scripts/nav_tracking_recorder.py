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

Clock discipline (same rules as scripts/p2_fault_observer.py)
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
import gzip
import hashlib
import json
import math
import os
import time

import rclpy
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


def stream_qos(depth=50):
    """Best-effort + volatile: the only reliability that matches every writer.

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


def grid_metadata(info):
    orientation = info.origin.orientation
    return {
        "width": int(info.width),
        "height": int(info.height),
        "resolution": float(info.resolution),
        "origin_x": float(info.origin.position.x),
        "origin_y": float(info.origin.position.y),
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


class NavTrackingRecorder(Node):
    def __init__(self, args):
        super().__init__("nav_tracking_recorder")
        self._args = args
        self._start_mono = time.monotonic()
        self._output_dir = args.output_dir
        os.makedirs(os.path.join(self._output_dir, "snapshot_payload"), exist_ok=True)
        os.makedirs(os.path.join(self._output_dir, "layers"), exist_ok=True)

        self._samples = JsonLinesWriter(os.path.join(self._output_dir, "samples.jsonl"))
        self._references = JsonLinesWriter(
            os.path.join(self._output_dir, "references.jsonl"))
        self._snapshots = JsonLinesWriter(
            os.path.join(self._output_dir, "snapshots.jsonl"))
        self._events = JsonLinesWriter(os.path.join(self._output_dir, "events.jsonl"))

        # Latest-of-each state.  Every entry keeps the monotonic receive time so an
        # age is always a steady-clock local measure, never a cross-clock stamp
        # subtraction.
        self._localization = None
        self._localization_mono = None
        self._selected = None
        self._selected_mono = None
        self._autonomy_raw = None
        self._autonomy_raw_mono = None
        self._motion_control = None
        self._motion_control_mono = None
        self._telemetry = None
        self._telemetry_mono = None
        self._emergency_stop = None
        self._emergency_stop_mono = None
        self._map_ready = None
        self._map_ready_mono = None
        self._snapshot_identity = None
        self._snapshot_mono = None
        self._map_status = None
        self._execution = None
        self._execution_mono = None
        self._planner_status = None
        self._reference = None
        self._reference_mono = None

        self._seen_reference_keys = set()
        self._seen_snapshot_digests = set()
        self._seen_layer_digests = {}
        self._snapshot_payloads_written = 0
        self._snapshot_payloads_truncated = False
        self._layer_payloads_written = 0
        self._layer_payloads_truncated = False
        self._layer_payloads_evicted = 0
        # topic -> payload names still on disk, in write order (see
        # plan_layer_retention: the newest content is never the one dropped).
        self._layer_retained = {}
        self._topic_counts = {}
        self._tick = 0

        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)

        self._subscribe(args)
        period = 1.0 / max(1.0, args.rate_hz)
        self._timer = self.create_timer(period, self._on_tick)
        self.get_logger().info(
            "nav_tracking_recorder writing to %s at %.1f Hz (read-only)"
            % (self._output_dir, args.rate_hz))

    # ---- subscriptions -----------------------------------------------------

    def _count(self, topic):
        self._topic_counts[topic] = self._topic_counts.get(topic, 0) + 1

    def _subscribe(self, args):
        self.create_subscription(
            Odometry, args.localization_topic, self._on_localization, stream_qos())
        self.create_subscription(
            Twist, args.selected_topic, self._on_selected, stream_qos())
        self.create_subscription(
            Twist, args.autonomy_raw_topic, self._on_autonomy_raw, stream_qos())
        self.create_subscription(
            SwerveTelemetry, args.telemetry_topic, self._on_telemetry, stream_qos())
        self.create_subscription(
            Path, args.reference_topic, self._on_reference, stream_qos(depth=10))
        if args.candidate_reference_topic:
            self.create_subscription(
                Path, args.candidate_reference_topic, self._on_candidate_reference,
                stream_qos(depth=10))
        self.create_subscription(
            PlannerStatus, args.planner_status_topic, self._on_planner_status,
            stream_qos())
        self.create_subscription(
            ExecutionCommand, args.execution_command_topic, self._on_execution,
            latched_qos())
        self.create_subscription(
            Bool, args.emergency_stop_topic, self._on_emergency_stop, latched_qos())
        self.create_subscription(
            Bool, args.map_ready_topic, self._on_map_ready, latched_qos())
        self.create_subscription(
            PlanningMapStatus, args.map_status_topic, self._on_map_status, latched_qos())
        self.create_subscription(
            PlanningMapSnapshot, args.planning_snapshot_topic, self._on_snapshot,
            latched_qos())
        # Source layers, for per-cell provenance of a first conflict.  The static
        # map and the fused planning grid are latched; the terrain and slope grids
        # are volatile streams from terrain_analysis_ext.
        for topic, latched in (
                (args.static_map_topic, True),
                (args.planning_grid_topic, True),
                (args.traversability_grid_topic, False),
                (args.slope_grid_topic, False)):
            if not topic:
                continue
            self.create_subscription(
                OccupancyGrid, topic,
                lambda msg, name=topic: self._on_layer(name, msg),
                latched_qos() if latched else stream_qos(depth=5))
        if MotionCtrl is not None and args.motion_control_topic:
            self.create_subscription(
                MotionCtrl, args.motion_control_topic, self._on_motion_control,
                stream_qos())
        else:
            self.get_logger().warn(
                "manda_can_control/MotionCtrl unavailable; %s will be recorded as null"
                % args.motion_control_topic)

    # ---- callbacks ---------------------------------------------------------

    def _on_localization(self, msg):
        self._count(self._args.localization_topic)
        orientation = msg.pose.pose.orientation
        self._localization = {
            "frame_id": msg.header.frame_id,
            "child_frame_id": msg.child_frame_id,
            "stamp_identity": stamp_identity(msg.header.stamp),
            "x": float(msg.pose.pose.position.x),
            "y": float(msg.pose.pose.position.y),
            "z": float(msg.pose.pose.position.z),
            "yaw": yaw_from_quaternion(
                float(orientation.x), float(orientation.y),
                float(orientation.z), float(orientation.w)),
            # Odometry twist is expressed in child_frame_id, i.e. the body frame,
            # which is the same frame the MPC commands live in.
            "vx_body": float(msg.twist.twist.linear.x),
            "vy_body": float(msg.twist.twist.linear.y),
            "wz_body": float(msg.twist.twist.angular.z),
        }
        self._localization_mono = time.monotonic()

    def _on_selected(self, msg):
        self._count(self._args.selected_topic)
        self._selected = {
            "vx": float(msg.linear.x), "vy": float(msg.linear.y),
            "wz": float(msg.angular.z),
        }
        self._selected_mono = time.monotonic()

    def _on_autonomy_raw(self, msg):
        self._count(self._args.autonomy_raw_topic)
        self._autonomy_raw = {
            "vx": float(msg.linear.x), "vy": float(msg.linear.y),
            "wz": float(msg.angular.z),
        }
        self._autonomy_raw_mono = time.monotonic()

    def _on_motion_control(self, msg):
        self._count(self._args.motion_control_topic)
        self._motion_control = {
            "linear_x": float(msg.linear_x), "linear_y": float(msg.linear_y),
            "angular_z": float(msg.angular_z),
        }
        self._motion_control_mono = time.monotonic()

    def _on_telemetry(self, msg):
        self._count(self._args.telemetry_topic)
        self._telemetry = {
            "stamp_identity": stamp_identity(msg.header.stamp),
            "contact_violation_count": int(msg.contact_violation_count),
            "max_contact_force": float(msg.max_contact_force),
            "sequence": int(msg.sequence),
            "command_vx": float(msg.command_vx),
            "command_vy": float(msg.command_vy),
            "command_wz": float(msg.command_wz),
            "measured_vx": float(msg.measured_vx),
            "measured_vy": float(msg.measured_vy),
            "measured_wz": float(msg.measured_wz),
            # Saturation matters for the "did tracking leave the envelope"
            # question: a saturated command means the MPC asked for more than the
            # chassis could deliver, so the executed motion is not the planned one.
            "drive_speed_saturation_count": int(msg.drive_speed_saturation_count),
            "drive_acceleration_saturation_count":
                int(msg.drive_acceleration_saturation_count),
            "steer_rate_saturation_count": int(msg.steer_rate_saturation_count),
            "drive_speed_saturated": [bool(v) for v in msg.drive_speed_saturated],
            "steer_rate_saturated": [bool(v) for v in msg.steer_rate_saturated],
        }
        self._telemetry_mono = time.monotonic()

    def _record_reference(self, msg, source):
        """Store a distinct reference once, keyed by stamp + geometry digest.

        The committed reference is republished on every plan cycle, so keying on
        content keeps the artifact bounded while still proving whether the pose
        sequence changed between cycles.
        """
        poses = path_poses(msg)
        payload = json.dumps(poses, sort_keys=True).encode("utf-8")
        digest = digest_bytes(payload)
        key = (source, stamp_identity(msg.header.stamp), digest)
        reference = {
            "source": source,
            "frame_id": msg.header.frame_id,
            "stamp_identity": stamp_identity(msg.header.stamp),
            "digest": digest,
            "pose_count": len(poses),
            "poses": poses,
        }
        if key not in self._seen_reference_keys:
            self._seen_reference_keys.add(key)
            record = dict(reference)
            record["mono_s"] = time.monotonic() - self._start_mono
            record["reference_index"] = len(self._seen_reference_keys) - 1
            self._references.write(record)
        return reference

    def _on_reference(self, msg):
        self._count(self._args.reference_topic)
        self._reference = self._record_reference(msg, self._args.reference_topic)
        self._reference_mono = time.monotonic()

    def _on_candidate_reference(self, msg):
        self._count(self._args.candidate_reference_topic)
        # Candidates are recorded for provenance but never treated as the tracked
        # reference: only the committed one carries an execution lease.
        self._record_reference(msg, self._args.candidate_reference_topic)

    def _on_planner_status(self, msg):
        self._count(self._args.planner_status_topic)
        status = {
            "stamp_identity": stamp_identity(msg.header.stamp),
            "frame_id": msg.header.frame_id,
            "goal_id": int(msg.goal_id),
            "localization_epoch": int(msg.localization_epoch),
            "plan_request_sequence": int(msg.plan_request_sequence),
            "map_generation": int(msg.map_generation),
            "map_publication_sequence": int(msg.map_publication_sequence),
            "state": int(msg.state),
            "failure_reason": int(msg.failure_reason),
            "reference_stamp_identity": stamp_identity(msg.reference_stamp),
            "yaw_authority": int(msg.yaw_authority),
        }
        previous = self._planner_status
        self._planner_status = status
        if previous != status:
            self._event("planner_status", status)

    def _on_execution(self, msg):
        self._count(self._args.execution_command_topic)
        reference_poses = path_poses(msg.reference)
        execution = {
            "stamp_identity": stamp_identity(msg.header.stamp),
            "mode": int(msg.mode),
            "manager_incarnation": int(msg.manager_incarnation),
            "command_sequence": int(msg.command_sequence),
            "goal_id": int(msg.goal_id),
            "localization_epoch": int(msg.localization_epoch),
            "map_generation": int(msg.map_generation),
            "map_publication_sequence": int(msg.map_publication_sequence),
            "failure_reason": int(msg.failure_reason),
            "yaw_authority": int(msg.yaw_authority),
            "reference_frame_id": msg.reference.header.frame_id,
            "reference_stamp_identity": stamp_identity(msg.reference.header.stamp),
            "reference_pose_count": len(reference_poses),
        }
        previous = self._execution
        self._execution = execution
        self._execution_mono = time.monotonic()
        if previous is None or previous.get("command_sequence") != \
                execution.get("command_sequence") or \
                previous.get("mode") != execution.get("mode"):
            event = dict(execution)
            # The lease carries its own copy of the reference; keep it so a
            # "did the executed reference match the published one" question is
            # answerable from one message rather than by cross-topic guessing.
            event["reference_poses"] = reference_poses
            self._event("execution_command", event)

    def _on_emergency_stop(self, msg):
        self._count(self._args.emergency_stop_topic)
        value = bool(msg.data)
        if self._emergency_stop != value:
            self._event("emergency_stop", {"value": value,
                                           "previous": self._emergency_stop})
        self._emergency_stop = value
        self._emergency_stop_mono = time.monotonic()

    def _on_map_ready(self, msg):
        self._count(self._args.map_ready_topic)
        value = bool(msg.data)
        if self._map_ready != value:
            self._event("map_ready", {"value": value, "previous": self._map_ready})
        self._map_ready = value
        self._map_ready_mono = time.monotonic()

    def _on_map_status(self, msg):
        self._count(self._args.map_status_topic)
        self._map_status = {
            "stamp_identity": stamp_identity(msg.header.stamp),
            "frame_id": msg.header.frame_id,
            "ready": bool(msg.ready),
            "localization_epoch": int(msg.localization_epoch),
            "rog_generation": int(msg.rog_generation),
            "publication_sequence": int(msg.publication_sequence),
            "message": msg.message,
        }

    def _on_snapshot(self, msg):
        self._count(self._args.planning_snapshot_topic)
        occupancy = list(msg.occupancy)
        digest = digest_int8_array(occupancy)
        identity = {
            "stamp_identity": stamp_identity(msg.header.stamp),
            "frame_id": msg.header.frame_id,
            "source_stamp_identity": stamp_identity(msg.source_stamp),
            "ready": bool(msg.ready),
            "unknown_is_obstacle": bool(msg.unknown_is_obstacle),
            "occupied_value_threshold": int(msg.occupied_value_threshold),
            "localization_epoch": int(msg.localization_epoch),
            "source_generation": int(msg.source_generation),
            "publication_sequence": int(msg.publication_sequence),
            "occupancy_digest": digest,
            "cell_count": len(occupancy),
            "info": grid_metadata(msg.info),
        }
        self._snapshot_identity = identity
        self._snapshot_mono = time.monotonic()
        if digest in self._seen_snapshot_digests:
            return
        self._seen_snapshot_digests.add(digest)
        record = dict(identity)
        record["mono_s"] = time.monotonic() - self._start_mono
        if self._snapshot_payloads_written < self._args.max_snapshot_payloads:
            name = "snapshot_%020d_%s.json.gz" % (
                int(msg.publication_sequence), digest)
            path = os.path.join(self._output_dir, "snapshot_payload", name)
            payload = {
                "identity": identity,
                "occupancy": occupancy,
                # The RC-ESDF arrays are what distinguishes "inflated near an
                # obstacle" from "hard obstacle cell", so they travel with the
                # occupancy rather than being reconstructed later.
                "signed_distance_m": [float(v) for v in msg.signed_distance_m],
            }
            with gzip.open(path, "wt") as handle:
                json.dump(payload, handle)
            self._snapshot_payloads_written += 1
            record["payload"] = os.path.join("snapshot_payload", name)
        else:
            self._snapshot_payloads_truncated = True
            record["payload"] = None
        self._snapshots.write(record)

    def _on_layer(self, topic, msg):
        """Persist a source layer once per distinct content, for provenance.

        A first conflict has to be attributable to static / terrain / slope /
        ROG-projected occupancy, and the fused grid alone cannot say which layer
        set the cell.  Only content changes are written, so a static latched map
        costs one file.
        """
        self._count(topic)
        data = list(msg.data)
        digest = digest_int8_array(data)
        if self._seen_layer_digests.get(topic) == digest:
            return
        self._seen_layer_digests[topic] = digest
        plan = plan_layer_retention(
            self._layer_retained, topic, self._args.max_layer_payloads)
        if plan is False:
            self._layer_payloads_truncated = True
            return
        victim_topic, victim_name = plan
        if victim_name is not None:
            victim_path = os.path.join(self._output_dir, "layers", victim_name)
            try:
                os.remove(victim_path)
            except OSError:
                pass
            self._layer_retained[victim_topic].remove(victim_name)
            self._layer_payloads_evicted += 1
            self._layer_payloads_truncated = True
            self._event("layer_payload_evicted", {
                "topic": victim_topic,
                "path": os.path.join("layers", victim_name),
                "reason": "max_layer_payloads budget reached; newest content kept",
            })
        safe_topic = topic.strip("/").replace("/", "__")
        name = "%s_%s.json.gz" % (safe_topic, digest)
        path = os.path.join(self._output_dir, "layers", name)
        payload = {
            "topic": topic,
            "frame_id": msg.header.frame_id,
            "stamp_identity": stamp_identity(msg.header.stamp),
            "mono_s": time.monotonic() - self._start_mono,
            "digest": digest,
            "info": grid_metadata(msg.info),
            "data": data,
        }
        with gzip.open(path, "wt") as handle:
            json.dump(payload, handle)
        self._layer_payloads_written += 1
        self._layer_retained.setdefault(topic, []).append(name)
        self._event("layer_payload", {
            "topic": topic, "digest": digest,
            "path": os.path.join("layers", name),
            "frame_id": msg.header.frame_id,
            "info": payload["info"],
        })

    def _event(self, kind, payload):
        record = {
            "mono_s": time.monotonic() - self._start_mono,
            "kind": kind,
            "payload": payload,
        }
        self._events.write(record)

    # ---- per-tick sampling -------------------------------------------------

    def _age(self, mono_value):
        """Steady-clock age since receipt.  None when nothing has arrived yet."""
        if mono_value is None:
            return None
        return time.monotonic() - mono_value

    def _lookup_map_from_odom(self):
        """map <- odom at the latest available transform, or None.

        Recorded per tick rather than assumed identity: the profile lets
        localization_fusion own map->odom, so treating the two frames as the same
        would silently mix the grid frame with the control frame.
        """
        try:
            transform = self._tf_buffer.lookup_transform(
                self._args.grid_frame, self._args.control_frame,
                rclpy.time.Time())
        except Exception:
            return None
        translation = transform.transform.translation
        rotation = transform.transform.rotation
        return {
            "stamp_identity": stamp_identity(transform.header.stamp),
            "x": float(translation.x),
            "y": float(translation.y),
            "yaw": yaw_from_quaternion(
                float(rotation.x), float(rotation.y),
                float(rotation.z), float(rotation.w)),
        }

    def _on_tick(self):
        now_mono = time.monotonic()
        self._tick += 1
        actual = self._localization
        reference = self._reference
        error = None
        if actual is not None and reference is not None:
            if reference["frame_id"] and actual["frame_id"] and \
                    reference["frame_id"] != actual["frame_id"]:
                # Never compare across frames silently.  A mismatch is reported so
                # the analyzer can mark the tick unpaired instead of producing a
                # tracking error that mixes two frames.
                error = {"frame_unpaired": True,
                         "reference_frame": reference["frame_id"],
                         "actual_frame": actual["frame_id"]}
            else:
                error = tracking_error(
                    (actual["x"], actual["y"], actual["yaw"]), reference["poses"])
        record = {
            "tick": self._tick,
            "mono_s": now_mono - self._start_mono,
            "actual": actual,
            "actual_age_s_mono": self._age(self._localization_mono),
            "selected": self._selected,
            "selected_age_s_mono": self._age(self._selected_mono),
            "autonomy_raw": self._autonomy_raw,
            "autonomy_raw_age_s_mono": self._age(self._autonomy_raw_mono),
            "motion_control": self._motion_control,
            "motion_control_age_s_mono": self._age(self._motion_control_mono),
            "telemetry": self._telemetry,
            "telemetry_age_s_mono": self._age(self._telemetry_mono),
            "emergency_stop": self._emergency_stop,
            "emergency_stop_age_s_mono": self._age(self._emergency_stop_mono),
            "map_ready": self._map_ready,
            "map_ready_age_s_mono": self._age(self._map_ready_mono),
            "map_status": self._map_status,
            "snapshot": self._snapshot_identity,
            "snapshot_age_s_mono": self._age(self._snapshot_mono),
            "execution": self._execution,
            "execution_age_s_mono": self._age(self._execution_mono),
            "planner_status": self._planner_status,
            "reference_digest": reference["digest"] if reference else None,
            "reference_stamp_identity": reference["stamp_identity"] if reference else None,
            "reference_frame": reference["frame_id"] if reference else None,
            "reference_pose_count": reference["pose_count"] if reference else None,
            "reference_age_s_mono": self._age(self._reference_mono),
            "tracking_error": error,
            "map_from_odom": self._lookup_map_from_odom(),
        }
        self._samples.write(record)

    # ---- shutdown ----------------------------------------------------------

    def write_summary(self, reason):
        summary = {
            "reason": reason,
            "duration_s_mono": time.monotonic() - self._start_mono,
            "ticks": self._tick,
            "rate_hz": self._args.rate_hz,
            "grid_frame": self._args.grid_frame,
            "control_frame": self._args.control_frame,
            "records": {
                "samples": self._samples.count,
                "references": self._references.count,
                "snapshots": self._snapshots.count,
                "events": self._events.count,
            },
            "messages_received": dict(self._topic_counts),
            # A topic that never delivered is an evidence gap, not a pass.  It is
            # named explicitly so a report cannot silently omit it.
            "topics_without_messages": sorted(
                topic for topic in self._expected_topics()
                if self._topic_counts.get(topic, 0) == 0),
            "snapshot_payloads_written": self._snapshot_payloads_written,
            "snapshot_payloads_truncated": self._snapshot_payloads_truncated,
            "layer_payloads_written": self._layer_payloads_written,
            "layer_payloads_truncated": self._layer_payloads_truncated,
            "layer_payloads_evicted": self._layer_payloads_evicted,
            "motion_control_message_available": MotionCtrl is not None,
        }
        # Distinguish "nobody published" from "a publisher existed but never
        # reached us".  The second shape is the QoS-incompatibility signature: DDS
        # reports an unmatched reader as silence, so without this cross-check a
        # mismatched subscription is indistinguishable from an idle topic and the
        # artifact looks merely empty instead of wrong.
        publishers = self._publisher_diagnosis()
        summary["publishers"] = publishers
        summary["topics_with_publishers_but_no_messages"] = sorted(
            topic for topic, info in publishers.items()
            if info.get("count", 0) > 0 and self._topic_counts.get(topic, 0) == 0)
        path = os.path.join(self._output_dir, "summary.json")
        with open(path, "w") as handle:
            json.dump(summary, handle, indent=2, sort_keys=True)
        return summary

    def _publisher_diagnosis(self):
        """Publisher count and declared QoS per expected topic, best effort.

        Read-only introspection.  If the rmw build does not expose a field the
        entry records the failure instead of omitting the topic, so a missing
        diagnosis can never read as a clean one.
        """
        report = {}
        for topic in self._expected_topics():
            entry = {"count": 0, "endpoints": [], "messages_received":
                     self._topic_counts.get(topic, 0)}
            try:
                infos = self.get_publishers_info_by_topic(topic)
            except Exception as exc:  # noqa: BLE001 - diagnosis must not abort
                entry["error"] = "%s: %s" % (type(exc).__name__, exc)
                report[topic] = entry
                continue
            entry["count"] = len(infos)
            for info in infos:
                endpoint = {"node": None, "reliability": None, "durability": None,
                            "depth": None}
                try:
                    endpoint["node"] = "%s/%s" % (
                        info.node_namespace.rstrip("/"), info.node_name)
                    qos = info.qos_profile
                    endpoint["reliability"] = getattr(
                        qos.reliability, "name", str(qos.reliability))
                    endpoint["durability"] = getattr(
                        qos.durability, "name", str(qos.durability))
                    endpoint["depth"] = qos.depth
                except Exception as exc:  # noqa: BLE001
                    endpoint["error"] = "%s: %s" % (type(exc).__name__, exc)
                entry["endpoints"].append(endpoint)
            report[topic] = entry
        return report

    def _expected_topics(self):
        args = self._args
        topics = [
            args.localization_topic, args.selected_topic, args.autonomy_raw_topic,
            args.telemetry_topic, args.reference_topic, args.planner_status_topic,
            args.execution_command_topic, args.emergency_stop_topic,
            args.map_ready_topic, args.map_status_topic,
            args.planning_snapshot_topic, args.static_map_topic,
            args.planning_grid_topic, args.traversability_grid_topic,
            args.slope_grid_topic,
        ]
        if args.candidate_reference_topic:
            topics.append(args.candidate_reference_topic)
        if MotionCtrl is not None and args.motion_control_topic:
            topics.append(args.motion_control_topic)
        return [topic for topic in topics if topic]

    def close(self):
        for writer in (self._samples, self._references, self._snapshots, self._events):
            writer.close()


def parse_args(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output-dir", required=True)
    parser.add_argument("--rate-hz", type=float, default=50.0,
                        help="tick rate of the same-clock sample stream")
    parser.add_argument("--duration-sec", type=float, default=0.0,
                        help="stop after this many seconds; 0 runs until killed")
    parser.add_argument("--grid-frame", default="map",
                        help="frame the fused planning grid is published in")
    parser.add_argument("--control-frame", default="odom",
                        help="planner global_frame, i.e. the reference/odom frame")
    parser.add_argument("--localization-topic", default="/localization")
    parser.add_argument("--selected-topic", default="/cmd_vel/selected")
    parser.add_argument("--autonomy-raw-topic", default="/cmd_vel/autonomy_raw")
    parser.add_argument("--motion-control-topic", default="/motion_control")
    parser.add_argument("--telemetry-topic", default="/swerve/telemetry")
    parser.add_argument("--reference-topic", default="/minco/reference_path")
    parser.add_argument("--candidate-reference-topic",
                        default="/minco/reference_path_candidate")
    parser.add_argument("--planner-status-topic", default="/minco/planning_status")
    parser.add_argument("--execution-command-topic",
                        default="/planner/execution_command")
    parser.add_argument("--emergency-stop-topic", default="/planner/emergency_stop")
    parser.add_argument("--map-ready-topic", default="/rog_map_adapter/ready")
    parser.add_argument("--map-status-topic", default="/rog_map_adapter/status")
    parser.add_argument("--planning-snapshot-topic",
                        default="/rog_map_adapter/planning_snapshot")
    parser.add_argument("--static-map-topic", default="/map")
    parser.add_argument("--planning-grid-topic", default="/rc_esdf/planning_grid")
    parser.add_argument("--traversability-grid-topic", default="/traversability_grid")
    parser.add_argument("--slope-grid-topic", default="/traversability_slope_grid")
    parser.add_argument("--max-snapshot-payloads", type=int, default=400,
                        help="bound on stored full snapshots; excess is flagged")
    parser.add_argument("--max-layer-payloads", type=int, default=200)
    return parser.parse_args(argv)


def main(argv=None):
    args = parse_args(argv)
    os.makedirs(args.output_dir, exist_ok=True)
    rclpy.init()
    node = NavTrackingRecorder(args)
    reason = "signal"
    try:
        if args.duration_sec > 0.0:
            deadline = time.monotonic() + args.duration_sec
            while rclpy.ok() and time.monotonic() < deadline:
                rclpy.spin_once(node, timeout_sec=0.1)
            reason = "duration_elapsed"
        else:
            rclpy.spin(node)
            reason = "shutdown"
    except KeyboardInterrupt:
        reason = "interrupt"
    finally:
        summary = node.write_summary(reason)
        node.close()
        node.destroy_node()
        try:
            rclpy.shutdown()
        except Exception:
            pass
        print("RECORDER: reason=%s ticks=%d samples=%d references=%d snapshots=%d "
              "events=%d" % (
                  summary["reason"], summary["ticks"],
                  summary["records"]["samples"], summary["records"]["references"],
                  summary["records"]["snapshots"], summary["records"]["events"]))
        if summary["topics_without_messages"]:
            print("RECORDER: topics without messages: %s"
                  % ", ".join(summary["topics_without_messages"]))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
