#!/usr/bin/env python3
"""Evaluate one prior-map relocalization acceptance-matrix cell.

Ground truth (the constant true ``map -> odom``) is used for EVALUATION ONLY.
It is never fed back into the online candidate scoring.

Emitted gates (all must hold for ``pass``):

``pass_fault_injected``
    The poisoned /initialpose was delivered to the estimator's subscription
    (matched subscription count at publish time). An in-basin fault is absorbed
    within one registration and never reaches the authoritative map->odom, so
    the gate proves injection, not output degradation. The estimator's own log
    acknowledgement stays in the report as ``fault_acknowledged`` data only:
    the launch pins buffered logging, so its timing is not a reliable gate.
``pass_recover``
    Base-pose error reached ``RECOVER_XY_M`` / ``RECOVER_YAW_RAD`` within
    ``RECOVER_WAIT_S`` and then STAYED inside the band for ``HOLD_S``.
    A single grazing sample is not a recovery.
``pass_no_wrong_accept``
    No accepted observation carried a pose further than ``WRONG_ACCEPT_XY_M``
    from ground truth. "Accepted" must mean "close to truth".
``pass_status_live``
    ``/localization/status`` kept arriving faster than ``STATUS_GAP_S``.
``pass_obs_accepted``
    At least ``MIN_OBS_ACCEPTED`` accepted observations were published.
"""

from __future__ import annotations

import json
import math
import os
import re
import threading
import time

import rclpy
from ats_navigation_interfaces.action import NavigateToPose
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
from rclpy.action import ActionClient
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import (
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
    qos_profile_sensor_data,
)
from rclpy.time import Time
from tf2_ros import Buffer, TransformListener

try:
    from ats_navigation_interfaces.msg import LocalizationStatus, RelocalizationObservation

    HAS_OBS = True
except Exception:  # pragma: no cover - interfaces always built in this workspace
    HAS_OBS = False


TRUTH_X = float(os.environ["TRUTH_X"])
TRUTH_Y = float(os.environ["TRUTH_Y"])
TRUTH_YAW = float(os.environ["TRUTH_YAW"])
OFFSET_X = float(os.environ.get("OFFSET_X", "0"))
OFFSET_Y = float(os.environ.get("OFFSET_Y", "0"))
OFFSET_YAW = float(os.environ.get("OFFSET_YAW", "0"))
# Same poisoned map->odom the launch injects: componentwise truth + offset.
WRONG_X = TRUTH_X + OFFSET_X
WRONG_Y = TRUTH_Y + OFFSET_Y
WRONG_YAW = TRUTH_YAW + OFFSET_YAW
ENTRY = os.environ.get("ENTRY", "seeded")
RECOVER_XY = float(os.environ.get("RECOVER_XY_M", "0.80"))
RECOVER_YAW = float(os.environ.get("RECOVER_YAW_RAD", "0.35"))
HOLD_S = float(os.environ.get("HOLD_S", "3.0"))
RECOVER_WAIT_S = float(os.environ.get("RECOVER_WAIT_S", "90"))
WRONG_ACCEPT_XY = float(os.environ.get("WRONG_ACCEPT_XY_M", "1.00"))
STATUS_GAP_S = float(os.environ.get("STATUS_GAP_S", "1.0"))
MIN_OBS_ACCEPTED = int(os.environ.get("MIN_OBS_ACCEPTED", "1"))
PRE_MEASURE_S = float(os.environ.get("PRE_MEASURE_S", "12"))
NAVIGATE_BEFORE_INJECTION = os.environ.get("NAVIGATE_BEFORE_INJECTION", "false") == "true"
NAV_SCENARIO = os.environ.get("NAV_SCENARIO", "spawn")
NAV_GOAL_SEQUENCE = os.environ.get("NAV_GOAL_SEQUENCE", "")
NAV_GOAL_TIMEOUT_S = max(1.0, float(os.environ.get("NAV_GOAL_TIMEOUT_S", "120")))
NAV_GOAL_TOLERANCE_M = max(0.0, float(os.environ.get("NAV_GOAL_TOLERANCE_M", "0.75")))
NAV_STILL_LINEAR_MPS = max(0.0, float(os.environ.get("NAV_STILL_LINEAR_MPS", "0.25")))
NAV_STILL_ANGULAR_RPS = max(0.0, float(os.environ.get("NAV_STILL_ANGULAR_RPS", "0.50")))
NAV_STILL_HOLD_S = max(0.0, float(os.environ.get("NAV_STILL_HOLD_S", "0.50")))
NAV_HEALTH_GATE_READY = os.environ.get("NAV_HEALTH_GATE_READY", "true") == "true"
NAV_HEALTH_GATE_RESULT = os.environ.get("NAV_HEALTH_GATE_RESULT", "")

BASE_FRAMES = ("gimbal_yaw_odom", "base_link", "base_footprint")
RELIABLE = QoSProfile(
    depth=50, reliability=ReliabilityPolicy.RELIABLE, history=HistoryPolicy.KEEP_LAST
)
STATUS_QOS = QoSProfile(
    depth=10,
    reliability=ReliabilityPolicy.RELIABLE,
    history=HistoryPolicy.KEEP_LAST,
)


def wrap(angle: float) -> float:
    return math.atan2(math.sin(angle), math.cos(angle))


def yaw_of(q) -> float:
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def compose(ax: float, ay: float, ayaw: float, bx: float, by: float, byaw: float):
    """2D pose composition: A then B."""
    cos_a = math.cos(ayaw)
    sin_a = math.sin(ayaw)
    return (
        ax + cos_a * bx - sin_a * by,
        ay + sin_a * bx + cos_a * by,
        wrap(ayaw + byaw),
    )


def parse_goal_sequence(raw: str) -> list[tuple[float, float, float]]:
    """Parse a shell-friendly ``x,y,yaw;x,y,yaw`` map-frame goal sequence."""
    goals = []
    for index, item in enumerate(filter(None, (part.strip() for part in raw.split(";"))), 1):
        fields = [field.strip() for field in item.split(",")]
        if len(fields) != 3:
            raise ValueError(f"goal {index} must contain x,y,yaw")
        goal = tuple(float(field) for field in fields)
        if not all(math.isfinite(value) for value in goal):
            raise ValueError(f"goal {index} must be finite")
        goals.append(goal)
    if not goals:
        raise ValueError("navigation needs at least one x,y,yaw goal")
    return goals


def fault_acknowledged(wrong_base, tolerance: float = 0.15, poll_s: float = 10.0) -> bool:
    """True when the node logged receipt of the poisoned /initialpose.

    Proof that the fault reached the estimator, independent of whether the fused
    output ever degraded. Reads the launch log the cell driver already captures;
    polls because the line can trail the event by one pipe buffer.
    """
    if wrong_base is None:
        return False
    log = os.path.join(os.environ.get("OUT", "."), "launch.log")
    pattern = re.compile(r"Received initial pose: \[x: ([-0-9.]+), y: ([-0-9.]+)")
    deadline = time.monotonic() + poll_s
    while True:
        try:
            with open(log, "r", errors="ignore") as stream:
                text = stream.read()
        except OSError:
            text = ""
        for match in pattern.finditer(text):
            if (
                math.hypot(
                    float(match.group(1)) - wrong_base[0], float(match.group(2)) - wrong_base[1]
                )
                <= tolerance
            ):
                return True
        if time.monotonic() >= deadline:
            return False
        time.sleep(0.5)


class RelocProbe(Node):
    def __init__(self) -> None:
        # use_sim_time keeps published /initialpose stamps on the Gazebo clock;
        # gap measurement below still uses steady time on purpose.
        super().__init__(
            "gazebo_reloc_cell_probe",
            parameter_overrides=[rclpy.parameter.Parameter("use_sim_time", value=True)],
        )
        self.tf = Buffer()
        self.listener = TransformListener(self.tf, self)
        self.odom: tuple[float, float, float] | None = None
        self.odom_start: tuple[float, float, float] | None = None
        self.odom_travel = 0.0
        self.odom_linear_speed: float | None = None
        self.odom_angular_speed: float | None = None
        self.last_odom_receive_time: float | None = None
        self.obs_total = 0
        self.obs_accepted = 0
        self.obs_pending = 0
        self.obs_rejected = 0
        self.obs_invalid = 0
        self.obs_no_odom = 0
        self.wrong_accepts: list[dict] = []
        self.accept_errors: list[float] = []
        self.accepted_observations: list[dict] = []
        self.nonfinite_accept = 0
        self.observation_phase = "pre_fault"
        self.status_recv: list[float] = []
        self.status_states: list[int] = []
        self.saw_lost = False
        self.saw_tracking = False
        self.last_status_message = ""
        self.last_epoch = 0
        self.max_consecutive_rejections = 0
        # A 1 m fault is inside the fine basin: the node corrects it within one
        # scan, faster than the main sampling loop can observe it. Sample the
        # deviation at 50 Hz from the executor thread with buffer-only lookups,
        # which cannot block and therefore cannot starve the status callback.
        self.sampling_deviation = False
        self.initialpose_subscribers = 0
        self.max_deviation_xy = 0.0
        self.max_deviation_yaw = 0.0
        self.create_timer(0.02, self.sample_deviation)

        self.create_subscription(Odometry, "/localization", self.on_loc, qos_profile_sensor_data)
        self.create_subscription(Odometry, "/odometry", self.on_odom, qos_profile_sensor_data)
        if HAS_OBS:
            self.create_subscription(
                RelocalizationObservation,
                "/relocalization_observation",
                self.on_obs,
                RELIABLE,
            )
            self.create_subscription(
                LocalizationStatus, "/localization/status", self.on_status, STATUS_QOS
            )
        self.initialpose_pub = self.create_publisher(PoseWithCovarianceStamped, "/initialpose", 10)
        self.navigation_client = ActionClient(self, NavigateToPose, "/ats_navigate_to_pose")

    def sample_deviation(self) -> None:
        if not self.sampling_deviation:
            return
        current = self.base_error(timeout_s=0.0)
        if current is None:
            return
        self.max_deviation_xy = max(self.max_deviation_xy, current[0])
        self.max_deviation_yaw = max(self.max_deviation_yaw, current[1])

    # --- callbacks -----------------------------------------------------
    def on_loc(self, msg: Odometry) -> None:
        pass

    def on_odom(self, msg: Odometry) -> None:
        pose = (
            msg.pose.pose.position.x,
            msg.pose.pose.position.y,
            yaw_of(msg.pose.pose.orientation),
        )
        if self.odom_start is None:
            self.odom_start = pose
        else:
            self.odom_travel = max(
                self.odom_travel,
                math.hypot(pose[0] - self.odom_start[0], pose[1] - self.odom_start[1]),
            )
        self.odom = pose
        self.odom_linear_speed = math.hypot(
            float(msg.twist.twist.linear.x), float(msg.twist.twist.linear.y)
        )
        self.odom_angular_speed = abs(float(msg.twist.twist.angular.z))
        self.last_odom_receive_time = time.monotonic()

    def on_obs(self, msg) -> None:
        self.obs_total += 1
        status = int(getattr(msg, "status", -1))
        accepted = bool(getattr(msg, "accepted", False))
        if status == RelocalizationObservation.STATUS_PENDING_CONFIRMATION:
            self.obs_pending += 1
        elif status == RelocalizationObservation.STATUS_REJECTED:
            self.obs_rejected += 1
        elif status == RelocalizationObservation.STATUS_INVALID:
            self.obs_invalid += 1
        elif status == RelocalizationObservation.STATUS_NO_ODOM:
            self.obs_no_odom += 1

        if not accepted or status != RelocalizationObservation.STATUS_ACCEPTED:
            return
        self.obs_accepted += 1
        if not math.isfinite(msg.registration_error):
            # A non-finite raw error must never reach STATUS_ACCEPTED.
            self.nonfinite_accept += 1
        truth = self.truth_base_pose()
        error_xy = None
        error_yaw = None
        if truth is not None:
            error_xy = math.hypot(
                msg.pose.pose.position.x - truth[0], msg.pose.pose.position.y - truth[1]
            )
            error_yaw = abs(wrap(yaw_of(msg.pose.pose.orientation) - truth[2]))
        self.accepted_observations.append(
            {
                "phase": self.observation_phase,
                "sequence": int(msg.sequence),
                "error_xy_m": error_xy,
                "error_yaw_rad": error_yaw,
                "registration_error": float(msg.registration_error),
                "quality": float(msg.quality),
                "inliers": int(msg.inlier_count),
                "source_points": int(msg.source_points),
                "message": str(msg.message),
            }
        )
        if truth is None:
            return
        assert error_xy is not None and error_yaw is not None
        self.accept_errors.append(error_xy)
        if error_xy > WRONG_ACCEPT_XY:
            self.wrong_accepts.append(
                {
                    "sequence": int(msg.sequence),
                    "error_xy_m": error_xy,
                    "error_yaw_rad": error_yaw,
                    "registration_error": float(msg.registration_error),
                    "quality": float(msg.quality),
                    "inliers": int(msg.inlier_count),
                    "source_points": int(msg.source_points),
                    "message": str(msg.message),
                }
            )

    def on_status(self, msg) -> None:
        self.status_recv.append(time.monotonic())
        self.status_states.append(int(msg.state))
        self.last_status_message = str(msg.message)
        self.last_epoch = int(msg.epoch)
        self.max_consecutive_rejections = max(
            self.max_consecutive_rejections, int(msg.consecutive_rejections)
        )
        if int(msg.state) == LocalizationStatus.STATE_LOST:
            self.saw_lost = True
        if int(msg.state) == LocalizationStatus.STATE_TRACKING:
            self.saw_tracking = True

    # --- helpers -------------------------------------------------------
    def truth_base_pose(self):
        """Ground-truth map pose of the robot base, from truth map->odom + odom."""
        if self.odom is None:
            return None
        return compose(TRUTH_X, TRUTH_Y, TRUTH_YAW, *self.odom)

    def wrong_base_pose(self):
        """Base pose the poisoned map->odom renders, i.e. the injected fault."""
        if self.odom is None:
            return None
        return compose(WRONG_X, WRONG_Y, WRONG_YAW, *self.odom)

    def estimated_base_pose(self, timeout_s: float = 0.2):
        for frame in BASE_FRAMES:
            try:
                tf = self.tf.lookup_transform(
                    "map", frame, Time(), timeout=Duration(seconds=timeout_s)
                )
            except Exception:
                continue
            t = tf.transform.translation
            return t.x, t.y, yaw_of(tf.transform.rotation), frame
        return None

    def base_error(self, timeout_s: float = 0.2):
        estimate = self.estimated_base_pose(timeout_s)
        truth = self.truth_base_pose()
        if estimate is None or truth is None:
            return None
        return (
            math.hypot(estimate[0] - truth[0], estimate[1] - truth[1]),
            abs(wrap(estimate[2] - truth[2])),
            estimate,
        )

    def publish_initialpose(self, x: float, y: float, yaw: float) -> None:
        msg = PoseWithCovarianceStamped()
        msg.header.frame_id = "map"
        msg.pose.pose.position.x = float(x)
        msg.pose.pose.position.y = float(y)
        msg.pose.pose.orientation.z = math.sin(yaw * 0.5)
        msg.pose.pose.orientation.w = math.cos(yaw * 0.5)
        covariance = [0.0] * 36
        covariance[0] = 0.25
        covariance[7] = 0.25
        covariance[35] = 0.15
        msg.pose.covariance = covariance
        # Injection proof must not depend on log flush timing: the launch pins
        # RCUTILS_LOGGING_BUFFERED_STREAM=1, so the estimator's "Received initial
        # pose" line can trail the event by minutes. A matched subscription at
        # publish time proves the poisoned seed was delivered to the estimator.
        deadline = time.monotonic() + 5.0
        while self.initialpose_pub.get_subscription_count() < 1 and time.monotonic() < deadline:
            time.sleep(0.1)
        self.initialpose_subscribers = max(
            self.initialpose_subscribers, self.initialpose_pub.get_subscription_count()
        )
        # The executor spins in its own thread; publishing must not spin here.
        for _ in range(8):
            msg.header.stamp = self.get_clock().now().to_msg()
            self.initialpose_pub.publish(msg)
            time.sleep(0.05)

    def wait_for_future(self, future, timeout_s: float, label: str) -> tuple[bool, str]:
        deadline = time.monotonic() + timeout_s
        while time.monotonic() < deadline:
            if future.done():
                return True, ""
            time.sleep(0.05)
        return False, f"timeout waiting for {label}"

    def wait_until_still(self) -> tuple[bool, dict]:
        deadline = time.monotonic() + max(2.0, NAV_STILL_HOLD_S + 2.0)
        stable_since = None
        while time.monotonic() < deadline:
            linear = self.odom_linear_speed
            angular = self.odom_angular_speed
            fresh = self.last_odom_receive_time is not None and time.monotonic() - self.last_odom_receive_time < 1.0
            if (
                fresh
                and linear is not None
                and angular is not None
                and linear <= NAV_STILL_LINEAR_MPS
                and angular <= NAV_STILL_ANGULAR_RPS
            ):
                stable_since = stable_since or time.monotonic()
                if time.monotonic() - stable_since >= NAV_STILL_HOLD_S:
                    return True, {
                        "linear_mps": linear,
                        "angular_rps": angular,
                        "hold_s": time.monotonic() - stable_since,
                    }
            else:
                stable_since = None
            time.sleep(0.05)
        return False, {
            "linear_mps": self.odom_linear_speed,
            "angular_rps": self.odom_angular_speed,
            "hold_s": 0.0,
        }

    def navigate_before_injection(self, goals: list[tuple[float, float, float]]) -> dict:
        report = {
            "requested": True,
            "scenario": NAV_SCENARIO,
            "goal_count": len(goals),
            "goal_timeout_s": NAV_GOAL_TIMEOUT_S,
            "goal_tolerance_m": NAV_GOAL_TOLERANCE_M,
            "legs": [],
            "success": False,
            "reason": "",
        }
        if not self.navigation_client.wait_for_server(timeout_sec=30.0):
            report["reason"] = "ATS action server unavailable"
            return report
        for index, (goal_x, goal_y, goal_yaw) in enumerate(goals, 1):
            goal = NavigateToPose.Goal()
            goal.goal_pose.header.frame_id = "map"
            goal.goal_pose.pose.position.x = goal_x
            goal.goal_pose.pose.position.y = goal_y
            goal.goal_pose.pose.orientation.z = math.sin(goal_yaw * 0.5)
            goal.goal_pose.pose.orientation.w = math.cos(goal_yaw * 0.5)
            goal.timeout.sec = int(math.ceil(NAV_GOAL_TIMEOUT_S))
            send_future = self.navigation_client.send_goal_async(goal)
            done, reason = self.wait_for_future(send_future, 10.0, f"action acceptance for leg {index}")
            leg = {"index": index, "goal": {"x": goal_x, "y": goal_y, "yaw": goal_yaw}}
            if not done:
                leg["reason"] = reason
                report["legs"].append(leg)
                report["reason"] = reason
                return report
            handle = send_future.result()
            if handle is None or not handle.accepted:
                leg["reason"] = "ATS action goal rejected"
                report["legs"].append(leg)
                report["reason"] = leg["reason"]
                return report
            result_future = handle.get_result_async()
            done, reason = self.wait_for_future(result_future, NAV_GOAL_TIMEOUT_S + 15.0, f"action result for leg {index}")
            if not done:
                leg["reason"] = reason
                report["legs"].append(leg)
                report["reason"] = reason
                return report
            wrapped = result_future.result()
            result = None if wrapped is None else wrapped.result
            result_code = None if result is None else int(result.result_code)
            truth = self.truth_base_pose()
            target_error = None if truth is None else math.hypot(truth[0] - goal_x, truth[1] - goal_y)
            leg.update(
                {
                    "result_code": result_code,
                    "result_message": "" if result is None else str(result.message),
                    "truth_pose": None if truth is None else {"x": truth[0], "y": truth[1], "yaw": truth[2]},
                    "truth_target_error_m": target_error,
                }
            )
            if result_code != NavigateToPose.Result.RESULT_SUCCEEDED:
                leg["reason"] = "ATS action did not report RESULT_SUCCEEDED"
                report["legs"].append(leg)
                report["reason"] = leg["reason"]
                return report
            if target_error is None or target_error > NAV_GOAL_TOLERANCE_M:
                leg["reason"] = "ground-truth target tolerance not met"
                report["legs"].append(leg)
                report["reason"] = leg["reason"]
                return report
            report["legs"].append(leg)
        still, still_metrics = self.wait_until_still()
        report["still"] = still_metrics
        if not still:
            report["reason"] = "vehicle did not settle after navigation"
            return report
        report["success"] = True
        return report


def main() -> int:
    rclpy.init()
    node = RelocProbe()

    # The probe MUST NOT measure its own scheduling. Blocking TF lookups on the
    # sampling path starve a foreground spin_once loop, which shows up as a
    # multi-second /localization/status receive gap that the fusion node never
    # produced. Spin the callbacks in a dedicated thread and keep sampling out
    # of the executor.
    executor = rclpy.executors.SingleThreadedExecutor()
    executor.add_node(node)
    spin_thread = threading.Thread(target=executor.spin, daemon=True)
    spin_thread.start()

    # Wait until the estimate is measurable, then inject the deviation from the
    # probe instead of relying on the launch-time poisoning alone: on easy cells
    # (1 m) the node corrects the launch-time offset within ~2 s, long before the
    # topic wait releases the probe, so the fault would never be observable.
    # Injection is a WRONG /initialpose, the same poisoned-prior fault used in the
    # domain116/118 experiments.
    for _ in range(600):
        if node.base_error() is not None and node.truth_base_pose() is not None:
            break
        time.sleep(0.1)

    navigation = {
        "requested": NAVIGATE_BEFORE_INJECTION,
        "scenario": NAV_SCENARIO,
        "success": not NAVIGATE_BEFORE_INJECTION,
        "reason": "disabled" if not NAVIGATE_BEFORE_INJECTION else "",
        "legs": [],
    }
    if NAVIGATE_BEFORE_INJECTION and not NAV_HEALTH_GATE_READY:
        navigation = {
            "requested": True,
            "scenario": NAV_SCENARIO,
            "success": False,
            "reason": "localization/map health gate failed",
            "health_gate_result": NAV_HEALTH_GATE_RESULT or "unavailable",
            "legs": [],
        }
    elif NAVIGATE_BEFORE_INJECTION:
        node.observation_phase = "navigation"
        try:
            navigation = node.navigate_before_injection(parse_goal_sequence(NAV_GOAL_SEQUENCE))
        except (ValueError, RuntimeError) as exc:
            navigation = {
                "requested": True,
                "scenario": NAV_SCENARIO,
                "success": False,
                "reason": str(exc),
                "legs": [],
            }

    # A failed action must remain a failed scenario, not a spawn-point probe.
    # Take the post-fault baseline only once the target action has succeeded.
    node.observation_phase = "pre_injection"
    accepted_at_fault = node.obs_accepted
    wrong_accepts_at_fault = len(node.wrong_accepts)
    nonfinite_at_fault = node.nonfinite_accept
    wrong_base = node.wrong_base_pose() if navigation["success"] else None
    injected = False
    if wrong_base is not None:
        node.sampling_deviation = True
        node.observation_phase = "post_fault"
        node.publish_initialpose(*wrong_base)
        injected = True

    before = None
    initialpose_published = False
    best_xy = float("inf")
    best_yaw = float("inf")
    first_recover_s: float | None = None
    hold_start: float | None = None
    hold_best = 0.0
    recovered = False
    samples = 0
    last = None

    if injected:
        # Keep the worst deviation the 50 Hz sampler saw, and fall back to the
        # slow path if the fast sampler never had a transform.
        slow = None
        deadline = time.monotonic() + PRE_MEASURE_S
        while time.monotonic() < deadline and rclpy.ok():
            current = node.base_error()
            if current is not None and (slow is None or current[0] > slow[0]):
                slow = current
            time.sleep(0.05)
        for _ in range(80):
            if slow is not None:
                break
            slow = node.base_error()
            time.sleep(0.1)
        node.sampling_deviation = False
        if slow is not None:
            before = (
                (node.max_deviation_xy, node.max_deviation_yaw, slow[2])
                if node.max_deviation_xy > slow[0]
                else slow
            )

        # Seeded entry: the operator supplies the correct pose right after the
        # fault. Autonomous entry: nothing else is published; the LOST lattice
        # search must recover on its own once the force window has expired.
        truth_base = node.truth_base_pose()
        if ENTRY == "seeded" and truth_base is not None:
            node.publish_initialpose(*truth_base)
            initialpose_published = True

        start = time.monotonic()
        while time.monotonic() - start < RECOVER_WAIT_S and rclpy.ok():
            time.sleep(0.05)
            current = node.base_error()
            if current is None:
                continue
            samples += 1
            error_xy, error_yaw, estimate = current
            last = estimate
            best_xy = min(best_xy, error_xy)
            best_yaw = min(best_yaw, error_yaw)
            inside = error_xy <= RECOVER_XY and error_yaw <= RECOVER_YAW
            now = time.monotonic()
            if inside:
                if first_recover_s is None:
                    first_recover_s = now - start
                if hold_start is None:
                    hold_start = now
                hold_best = max(hold_best, now - hold_start)
                if hold_best >= HOLD_S and node.obs_accepted - accepted_at_fault >= MIN_OBS_ACCEPTED:
                    recovered = True
                    break
            else:
                hold_start = None
    else:
        # A navigation or state-readiness failure has no post-fault interval.
        # Keep the report auditable while preventing pre-fault observations from
        # being represented as recovery evidence.
        node.sampling_deviation = False

    status_gaps = [
        node.status_recv[i] - node.status_recv[i - 1] for i in range(1, len(node.status_recv))
    ]
    max_status_gap = max(status_gaps) if status_gaps else None
    post_fault_obs_accepted = node.obs_accepted - accepted_at_fault if injected else 0
    post_fault_wrong_accept_count = len(node.wrong_accepts) - wrong_accepts_at_fault if injected else 0
    post_fault_nonfinite_error_accepted = node.nonfinite_accept - nonfinite_at_fault if injected else 0

    report = {
        "cell_id": os.environ.get("CELL_ID", "cell"),
        "domain": int(os.environ.get("DOMAIN", "0")),
        "entry": ENTRY,
        "navigation": navigation,
        "offset": {
            "x": float(os.environ.get("OFFSET_X", "0")),
            "y": float(os.environ.get("OFFSET_Y", "0")),
            "yaw": float(os.environ.get("OFFSET_YAW", "0")),
        },
        "truth_map_to_odom": {"x": TRUTH_X, "y": TRUTH_Y, "yaw": TRUTH_YAW},
        "pre_error_xy_m": None if before is None else before[0],
        "pre_error_yaw_rad": None if before is None else before[1],
        "initialpose_published": initialpose_published,
        "deviation_injected": injected,
        "deviation_sampled_xy_m": node.max_deviation_xy,
        "deviation_sampled_yaw_rad": node.max_deviation_yaw,
        "odom_travel_m": node.odom_travel,
        "samples": samples,
        "last_estimated_base_pose": None
        if last is None
        else {"x": last[0], "y": last[1], "yaw": last[2], "frame": last[3]},
        "best_xy_err_m": None if math.isinf(best_xy) else best_xy,
        "best_yaw_err_rad": None if math.isinf(best_yaw) else best_yaw,
        "first_recover_s": first_recover_s,
        "hold_best_s": hold_best,
        "hold_required_s": HOLD_S,
        "obs_total": node.obs_total,
        "obs_accepted": node.obs_accepted,
        "obs_pending_confirmation": node.obs_pending,
        "obs_rejected": node.obs_rejected,
        "obs_invalid": node.obs_invalid,
        "obs_no_odom": node.obs_no_odom,
        "post_fault_obs_accepted": post_fault_obs_accepted,
        "post_fault_wrong_accept_count": post_fault_wrong_accept_count,
        "post_fault_nonfinite_error_accepted": post_fault_nonfinite_error_accepted,
        "accepted_observations": node.accepted_observations,
        "accept_error_xy_max_m": max(node.accept_errors) if node.accept_errors else None,
        "accept_error_xy_median_m": (
            sorted(node.accept_errors)[len(node.accept_errors) // 2] if node.accept_errors else None
        ),
        "wrong_accept_threshold_m": WRONG_ACCEPT_XY,
        "wrong_accept_count": len(node.wrong_accepts),
        "wrong_accepts": node.wrong_accepts[:10],
        "nonfinite_error_accepted": node.nonfinite_accept,
        "status_samples": len(node.status_recv),
        "status_max_gap_s": max_status_gap,
        "status_saw_lost": node.saw_lost,
        "status_saw_tracking": node.saw_tracking,
        "status_last_epoch": node.last_epoch,
        "status_max_consecutive_rejections": node.max_consecutive_rejections,
        "status_last_message": node.last_status_message,
        "has_obs_msg": HAS_OBS,
    }

    # An in-basin fault (1 m) is absorbed inside a single registration: the node
    # re-registers from the poisoned seed and returns the correct pose, so the
    # authoritative map->odom NEVER carries the deviation. That is the desired
    # outcome, not a missing injection, so the gate proves the estimator RECEIVED
    # the poisoned seed instead of demanding that the fused output degrade. The
    # sampled deviation stays in the report as data.
    report["fault_acknowledged"] = fault_acknowledged(wrong_base)
    report["initialpose_subscribers"] = node.initialpose_subscribers
    report["pass_navigation"] = bool(navigation["success"])
    report["pass_fault_injected"] = bool(injected and node.initialpose_subscribers >= 1)
    report["pass_recover"] = recovered
    report["pass_no_wrong_accept"] = len(node.wrong_accepts) == 0
    report["pass_no_nonfinite_accept"] = node.nonfinite_accept == 0
    report["pass_status_live"] = max_status_gap is not None and max_status_gap < STATUS_GAP_S
    report["pass_obs_accepted"] = post_fault_obs_accepted >= MIN_OBS_ACCEPTED
    report["pass"] = bool(
        report["pass_navigation"]
        and report["pass_fault_injected"]
        and report["pass_recover"]
        and report["pass_no_wrong_accept"]
        and report["pass_no_nonfinite_accept"]
        and report["pass_status_live"]
        and report["pass_obs_accepted"]
    )

    print(json.dumps(report, indent=2))
    # Shutting rclpy down under a spinning executor aborts the process and the
    # cell exit code becomes meaningless; stop the executor first.
    executor.shutdown()
    spin_thread.join(timeout=5.0)
    node.destroy_node()
    rclpy.shutdown()
    return 0 if report["pass"] else 4


if __name__ == "__main__":
    raise SystemExit(main())
