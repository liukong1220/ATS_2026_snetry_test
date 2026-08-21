#!/usr/bin/env python3
"""Structured pre-fault observer for the P2 source-unknown fault gate.

Subscribes to every topic in the fail-closed chain *before* the fault is
injected, then reports the fault -> effect latencies and the identity fields
that the shell harness must not reconstruct from text-scraped
``ros2 topic echo`` output.

Design constraints this script enforces:

* All durations use ``time.monotonic()``.  ROS stamps are recorded only as data
  identity and never subtracted to form a duration.
* Every ``PlanningMapStatus`` identity tuple (``ready``, ``rog_generation``,
  ``publication_sequence``, ``localization_epoch``) is taken from one single
  message, never assembled from separate samples.
* A ``PlanningMapStatus`` is paired with the ``PlanningMapSnapshot`` that shares
  its ``publication_sequence``; an unpaired status is reported as unpaired
  rather than silently matched to the newest snapshot.
* ``/cmd_vel_mpc`` is judged inside one fixed post-fault window.  It is the
  navigation-domain speed authority; lower-controller and wheel diagnostics do
  not participate in this algorithmic acceptance gate.
* all-unknown is decided from the structured ``PlanningMapSnapshot`` numeric
  payload: ``ready=false``, non-empty occupancy with every cell ``== -1``,
  ``signed_distance_m`` / ``gradient_x`` / ``gradient_y`` of the same length and
  all NaN, and ``source_generation`` equal to the fault numeric generation.
  ``/rc_esdf/planning_grid`` is never consulted here.

Usage::

    p2_fault_observer.py --output /tmp/observation.json \
        --fault-trigger-file /tmp/fault_injected.stamp \
        --zero-window-sec 3.0 --settle-sec 20.0

The observer starts subscribing immediately, prints ``OBSERVER_READY`` on
stdout once every subscription exists, and treats the appearance of
``--fault-trigger-file`` as the fault instant.  The harness creates that file
right after it issues the fault parameter set.
"""

from __future__ import annotations

import argparse
import json
import math
import os
import sys
import time
from dataclasses import dataclass, field

import rclpy
from ats_navigation_interfaces.msg import PlanningMapSnapshot
from ats_navigation_interfaces.msg import PlanningMapStatus
from geometry_msgs.msg import Twist
from nav_msgs.msg import Path
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Bool

DEFAULT_ZERO_THRESHOLD = 1.0e-3


def status_qos() -> QoSProfile:
    return QoSProfile(
        history=HistoryPolicy.KEEP_LAST,
        depth=50,
        reliability=ReliabilityPolicy.RELIABLE,
        durability=DurabilityPolicy.TRANSIENT_LOCAL,
    )


def stream_qos(depth: int = 200) -> QoSProfile:
    return QoSProfile(
        history=HistoryPolicy.KEEP_LAST,
        depth=depth,
        reliability=ReliabilityPolicy.RELIABLE,
        durability=DurabilityPolicy.VOLATILE,
    )


def reference_path_qos() -> QoSProfile:
    """Match Goal Manager's reliable, volatile committed-reference publisher."""
    return QoSProfile(
        history=HistoryPolicy.KEEP_LAST,
        depth=1,
        reliability=ReliabilityPolicy.RELIABLE,
        durability=DurabilityPolicy.VOLATILE,
    )


def stamp_ns(stamp) -> int:
    return int(stamp.sec) * 1_000_000_000 + int(stamp.nanosec)


@dataclass
class StatusSample:
    """One ``PlanningMapStatus``; identity fields stay together by construction."""

    monotonic: float
    ready: bool
    rog_generation: int
    publication_sequence: int
    localization_epoch: int
    stamp_ns: int
    message: str

    def identity(self) -> dict:
        return {
            "ready": self.ready,
            "rog_generation": self.rog_generation,
            "publication_sequence": self.publication_sequence,
            "localization_epoch": self.localization_epoch,
            "stamp_ns": self.stamp_ns,
            "message": self.message,
        }


@dataclass
class SnapshotSample:
    """One ``PlanningMapSnapshot`` reduced to its structural audit."""

    monotonic: float
    ready: bool
    publication_sequence: int
    source_generation: int
    localization_epoch: int
    stamp_ns: int
    source_stamp_ns: int
    width: int
    height: int
    cell_count: int
    occupancy_len: int
    unknown_cells: int
    free_cells: int
    occupied_cells: int
    signed_distance_len: int
    gradient_x_len: int
    gradient_y_len: int
    finite_numeric_cells: int

    @property
    def all_unknown(self) -> bool:
        """Structured all-unknown predicate, numeric payload only."""
        return (
            not self.ready
            and self.width > 0
            and self.height > 0
            and self.cell_count > 0
            and self.occupancy_len == self.cell_count
            and self.occupancy_len > 0
            and self.unknown_cells == self.cell_count
            and self.free_cells == 0
            and self.occupied_cells == 0
            and self.signed_distance_len == self.cell_count
            and self.gradient_x_len == self.cell_count
            and self.gradient_y_len == self.cell_count
            and self.finite_numeric_cells == 0
        )

    def audit(self) -> dict:
        payload = {
            "ready": self.ready,
            "publication_sequence": self.publication_sequence,
            "source_generation": self.source_generation,
            "localization_epoch": self.localization_epoch,
            "stamp_ns": self.stamp_ns,
            "source_stamp_ns": self.source_stamp_ns,
            "width": self.width,
            "height": self.height,
            "cell_count": self.cell_count,
            "occupancy_len": self.occupancy_len,
            "unknown_cells": self.unknown_cells,
            "free_cells": self.free_cells,
            "occupied_cells": self.occupied_cells,
            "signed_distance_len": self.signed_distance_len,
            "gradient_x_len": self.gradient_x_len,
            "gradient_y_len": self.gradient_y_len,
            "finite_numeric_cells": self.finite_numeric_cells,
            "all_unknown": self.all_unknown,
        }
        return payload


@dataclass
class VelocitySample:
    monotonic: float
    magnitude: float
    stamp_ns: int = 0


@dataclass
class Observation:
    status: list[StatusSample] = field(default_factory=list)
    snapshots: list[SnapshotSample] = field(default_factory=list)
    emergency_stop: list[tuple[float, bool]] = field(default_factory=list)
    cmd_vel: list[VelocitySample] = field(default_factory=list)
    reference_path: list[tuple[float, int, int]] = field(default_factory=list)


class FaultObserver(Node):
    def __init__(self, topics: dict) -> None:
        super().__init__("ats_p2_fault_observer")
        self.observation = Observation()
        self.subscription_count = 0

        self.create_subscription(
            PlanningMapStatus, topics["status"], self._on_status, status_qos()
        )
        self.create_subscription(
            PlanningMapSnapshot, topics["snapshot"], self._on_snapshot, status_qos()
        )
        self.create_subscription(
            Bool, topics["emergency_stop"], self._on_emergency_stop, stream_qos()
        )
        self.create_subscription(
            Twist, topics["cmd_vel"], self._on_cmd_vel, stream_qos()
        )
        self.create_subscription(
            Path,
            topics["reference_path"],
            self._on_reference_path,
            reference_path_qos(),
        )
        self.subscription_count = 5

    def _on_status(self, message: PlanningMapStatus) -> None:
        self.observation.status.append(
            StatusSample(
                monotonic=time.monotonic(),
                ready=bool(message.ready),
                rog_generation=int(message.rog_generation),
                publication_sequence=int(message.publication_sequence),
                localization_epoch=int(message.localization_epoch),
                stamp_ns=stamp_ns(message.header.stamp),
                message=str(message.message),
            )
        )

    def _on_snapshot(self, message: PlanningMapSnapshot) -> None:
        occupancy = message.occupancy
        unknown = 0
        free = 0
        occupied = 0
        for cell in occupancy:
            value = int(cell)
            if value == -1:
                unknown += 1
            elif value == 0:
                free += 1
            else:
                occupied += 1
        finite = 0
        for index in range(len(message.signed_distance_m)):
            if (
                math.isfinite(message.signed_distance_m[index])
                or (
                    index < len(message.gradient_x)
                    and math.isfinite(message.gradient_x[index])
                )
                or (
                    index < len(message.gradient_y)
                    and math.isfinite(message.gradient_y[index])
                )
            ):
                finite += 1
        self.observation.snapshots.append(
            SnapshotSample(
                monotonic=time.monotonic(),
                ready=bool(message.ready),
                publication_sequence=int(message.publication_sequence),
                source_generation=int(message.source_generation),
                localization_epoch=int(message.localization_epoch),
                stamp_ns=stamp_ns(message.header.stamp),
                source_stamp_ns=stamp_ns(message.source_stamp),
                width=int(message.info.width),
                height=int(message.info.height),
                cell_count=int(message.info.width) * int(message.info.height),
                occupancy_len=len(occupancy),
                unknown_cells=unknown,
                free_cells=free,
                occupied_cells=occupied,
                signed_distance_len=len(message.signed_distance_m),
                gradient_x_len=len(message.gradient_x),
                gradient_y_len=len(message.gradient_y),
                finite_numeric_cells=finite,
            )
        )

    def _on_emergency_stop(self, message: Bool) -> None:
        self.observation.emergency_stop.append((time.monotonic(), bool(message.data)))

    def _on_cmd_vel(self, message: Twist) -> None:
        magnitude = max(
            abs(float(message.linear.x)),
            abs(float(message.linear.y)),
            abs(float(message.angular.z)),
        )
        self.observation.cmd_vel.append(VelocitySample(time.monotonic(), magnitude))

    def _on_reference_path(self, message: Path) -> None:
        self.observation.reference_path.append(
            (time.monotonic(), len(message.poses), stamp_ns(message.header.stamp))
        )


def sustained_zero_window(
    samples: list[VelocitySample],
    window_start: float,
    window_end: float,
    threshold: float,
) -> dict:
    """Judge the navigation speed authority inside the given wall-clock window."""
    in_window = [
        sample
        for sample in samples
        if window_start <= sample.monotonic <= window_end
    ]
    peak = max((sample.magnitude for sample in in_window), default=None)
    return {
        "samples_in_window": len(in_window),
        "peak_magnitude": peak,
        "threshold": threshold,
        "sustained_zero": bool(in_window) and all(
            sample.magnitude <= threshold for sample in in_window
        ),
    }


def first_at_or_after(samples, predicate, since: float):
    """First sample satisfying ``predicate`` at or after monotonic ``since``."""
    for sample in samples:
        monotonic = sample[0] if isinstance(sample, tuple) else sample.monotonic
        if monotonic < since:
            continue
        if predicate(sample):
            return sample
    return None


def latency(sample, fault_monotonic: float):
    if sample is None:
        return None
    monotonic = sample[0] if isinstance(sample, tuple) else sample.monotonic
    return monotonic - fault_monotonic


def pair_status_with_snapshot(
    status: StatusSample, snapshots: list[SnapshotSample]
) -> dict:
    """Pair by ``publication_sequence``; report unpaired rather than guessing."""
    for snapshot in snapshots:
        if snapshot.publication_sequence == status.publication_sequence:
            return {
                "paired": True,
                "publication_sequence": status.publication_sequence,
                "localization_epoch_matches": (
                    snapshot.localization_epoch == status.localization_epoch
                ),
                "source_generation_matches_status_rog_generation": (
                    snapshot.source_generation == status.rog_generation
                ),
                "ready_matches": snapshot.ready == status.ready,
                "status": status.identity(),
                "snapshot": snapshot.audit(),
            }
    return {
        "paired": False,
        "publication_sequence": status.publication_sequence,
        "status": status.identity(),
        "snapshot": None,
    }


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument("--output", required=True)
    parser.add_argument("--fault-trigger-file", required=True)
    parser.add_argument("--recovery-trigger-file", default="")
    parser.add_argument("--ready-file", default="")
    parser.add_argument("--status-topic", default="/rog_map_adapter/status")
    parser.add_argument(
        "--snapshot-topic", default="/rog_map_adapter/planning_snapshot"
    )
    parser.add_argument("--emergency-stop-topic", default="/planner/emergency_stop")
    parser.add_argument("--cmd-vel-topic", default="/cmd_vel_mpc")
    parser.add_argument("--reference-path-topic", default="/minco/reference_path")
    parser.add_argument("--fault-wait-sec", type=float, default=180.0)
    parser.add_argument(
        "--settle-sec",
        type=float,
        default=25.0,
        help="Observation time after the fault before the zero window opens.",
    )
    parser.add_argument(
        "--zero-window-sec",
        type=float,
        default=3.0,
        help="Length of the post-fault window in which /cmd_vel_mpc must be zero.",
    )
    parser.add_argument(
        "--post-window-sec",
        type=float,
        default=0.0,
        help="Extra observation time after the zero window closes.",
    )
    parser.add_argument("--zero-threshold", type=float, default=DEFAULT_ZERO_THRESHOLD)
    parser.add_argument(
        "--require-gate",
        action="store_true",
        help="Exit non-zero unless every unknown-fault gate criterion passed.",
    )
    return parser.parse_args()


def gate_verdict(report: dict) -> dict:
    """Reduce the report to the pass/fail criteria of the unknown fault gate."""
    checks = {}
    snapshot = report.get("first_all_unknown_snapshot")
    checks["real_all_unknown_snapshot"] = bool(snapshot and snapshot["all_unknown"])
    checks["adapter_reported_not_ready"] = (
        report.get("fault_status_identity") is not None
    )
    checks["emergency_stop_asserted"] = (
        report["latency_sec"]["fault_to_emergency_stop_true"] is not None
    )
    checks["cmd_vel_mpc_zero_in_window"] = bool(
        report["cmd_vel_mpc_zero_window"]["sustained_zero"]
    )
    pairing = report.get("fault_status_snapshot_pairing")
    checks["fault_status_paired_with_snapshot"] = bool(pairing and pairing["paired"])
    checks["fault_status_ready_matches_snapshot"] = bool(
        pairing and pairing.get("ready_matches")
    )
    checks["fault_status_localization_epoch_consistent"] = bool(
        pairing and pairing.get("localization_epoch_matches")
    )
    checks["fault_snapshot_source_generation_matches_status"] = bool(
        pairing and pairing.get("source_generation_matches_status_rog_generation")
    )
    checks["fault_status_snapshot_is_the_all_unknown_snapshot"] = bool(
        pairing
        and pairing.get("snapshot")
        and pairing["snapshot"].get("all_unknown")
        and snapshot
        and pairing["snapshot"].get("publication_sequence")
        == snapshot.get("publication_sequence")
    )

    recovery = report.get("recovery")
    checks["recovery_observed"] = bool(recovery and recovery.get("observed"))
    checks["pre_fault_non_empty_reference_observed"] = bool(
        report.get("pre_fault_non_empty_reference_observed")
    )
    if recovery and recovery.get("observed"):
        checks["recovery_publication_advanced_past_fault"] = bool(
            recovery["publication_advanced_past_fault"]
        )
        recovery_pairing = recovery.get("recovered_status_snapshot_pairing")
        checks["recovered_status_paired_with_snapshot"] = bool(
            recovery_pairing and recovery_pairing["paired"]
        )
        checks["recovered_status_ready_matches_snapshot"] = bool(
            recovery_pairing and recovery_pairing.get("ready_matches")
        )
        checks["recovered_status_localization_epoch_consistent"] = bool(
            recovery_pairing and recovery_pairing.get("localization_epoch_matches")
        )
        checks["recovered_snapshot_source_generation_matches_status"] = bool(
            recovery_pairing
            and recovery_pairing.get(
                "source_generation_matches_status_rog_generation"
            )
        )
        checks["old_reference_did_not_revive"] = bool(
            report.get("pre_fault_non_empty_reference_observed")
            and not recovery["non_empty_reference_after_recovery_without_new_goal"]
        )
        checks["cmd_vel_mpc_stays_zero_without_a_new_goal"] = bool(
            recovery["no_new_goal_cmd_vel_mpc_zero_window"]["sustained_zero"]
        )
    else:
        checks["recovery_publication_advanced_past_fault"] = False
        checks["recovered_status_paired_with_snapshot"] = False
        checks["recovered_status_ready_matches_snapshot"] = False
        checks["recovered_status_localization_epoch_consistent"] = False
        checks["recovered_snapshot_source_generation_matches_status"] = False
        checks["old_reference_did_not_revive"] = False
        checks["cmd_vel_mpc_stays_zero_without_a_new_goal"] = False
    return {"checks": checks, "passed": all(checks.values())}


def spin_until(node: Node, deadline: float) -> None:
    while time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.02)


def spin_until_file(node: Node, path: str, deadline: float):
    """Spin while waiting for ``path`` to appear; return its monotonic time."""
    while time.monotonic() < deadline:
        if path and os.path.exists(path):
            return time.monotonic()
        rclpy.spin_once(node, timeout_sec=0.02)
    return None


def build_report(args: argparse.Namespace, observation: Observation,
                 fault_monotonic: float, window_start: float,
                 window_end: float) -> dict:
    pre_fault_status = [s for s in observation.status if s.monotonic < fault_monotonic]
    post_fault_status = [
        s for s in observation.status if s.monotonic >= fault_monotonic
    ]
    baseline = pre_fault_status[-1] if pre_fault_status else None

    first_all_unknown = first_at_or_after(
        observation.snapshots, lambda s: s.all_unknown, fault_monotonic
    )
    first_not_ready_status = first_at_or_after(
        observation.status, lambda s: not s.ready, fault_monotonic
    )
    first_estop = first_at_or_after(
        observation.emergency_stop, lambda s: s[1] is True, fault_monotonic
    )
    first_zero_cmd_vel = first_at_or_after(
        observation.cmd_vel,
        lambda s: s.magnitude <= args.zero_threshold,
        fault_monotonic,
    )

    fault_status = first_not_ready_status
    fault_generation = (
        first_all_unknown.source_generation if first_all_unknown is not None else None
    )

    cmd_vel_window = sustained_zero_window(
        observation.cmd_vel, window_start, window_end, args.zero_threshold
    )

    report = {
        "fault_monotonic": fault_monotonic,
        "clock": "time.monotonic for every duration; ROS stamps are identity only",
        "pre_fault_status_samples": len(pre_fault_status),
        "post_fault_status_samples": len(post_fault_status),
        "pre_fault_baseline_status": baseline.identity() if baseline else None,
        "fault_status_identity": fault_status.identity() if fault_status else None,
        "fault_status_snapshot_pairing": (
            pair_status_with_snapshot(fault_status, observation.snapshots)
            if fault_status
            else None
        ),
        "first_all_unknown_snapshot": (
            first_all_unknown.audit() if first_all_unknown else None
        ),
        "fault_numeric_generation": fault_generation,
        "latency_sec": {
            "fault_to_first_all_unknown": latency(first_all_unknown, fault_monotonic),
            "fault_to_ready_false": latency(first_not_ready_status, fault_monotonic),
            "fault_to_emergency_stop_true": latency(first_estop, fault_monotonic),
            "fault_to_first_zero_cmd_vel_mpc": latency(
                first_zero_cmd_vel, fault_monotonic
            ),
        },
        "cmd_vel_mpc_zero_window": {
            "window_start_monotonic": window_start,
            "window_end_monotonic": window_end,
            "window_length_sec": window_end - window_start,
            **cmd_vel_window,
        },
        "reference_path_after_fault": [
            {"monotonic": entry[0], "poses": entry[1], "stamp_ns": entry[2]}
            for entry in observation.reference_path
            if entry[0] >= fault_monotonic
        ],
        "pre_fault_non_empty_reference_observed": any(
            entry[0] < fault_monotonic and entry[1] > 0
            for entry in observation.reference_path
        ),
    }
    return report


def build_recovery_report(
    args: argparse.Namespace,
    observation: Observation,
    fault_status: StatusSample | None,
    recovery_monotonic: float,
    window_start: float,
    window_end: float,
) -> dict:
    """Recovery identity plus the no-revival check, all from single messages."""
    first_ready = first_at_or_after(
        observation.status, lambda s: s.ready, recovery_monotonic
    )
    publication_fault = (
        fault_status.publication_sequence if fault_status is not None else None
    )
    publication_after = (
        first_ready.publication_sequence if first_ready is not None else None
    )
    advanced = (
        publication_after is not None
        and publication_fault is not None
        and publication_after > publication_fault
    )
    cmd_vel_window = sustained_zero_window(
        observation.cmd_vel, window_start, window_end, args.zero_threshold
    )
    references = [
        {"monotonic": entry[0], "poses": entry[1], "stamp_ns": entry[2]}
        for entry in observation.reference_path
        if entry[0] >= recovery_monotonic
    ]
    return {
        "recovery_monotonic": recovery_monotonic,
        "recovery_latency_sec": latency(first_ready, recovery_monotonic),
        "publication_sequence_fault": publication_fault,
        "publication_sequence_after": publication_after,
        # The gate is strictly against the fault publication, never against the
        # pre-fault baseline.
        "publication_advanced_past_fault": advanced,
        "recovered_status_identity": (
            first_ready.identity() if first_ready is not None else None
        ),
        "recovered_status_snapshot_pairing": (
            pair_status_with_snapshot(first_ready, observation.snapshots)
            if first_ready is not None
            else None
        ),
        "no_new_goal_cmd_vel_mpc_zero_window": {
            "window_start_monotonic": window_start,
            "window_end_monotonic": window_end,
            "window_length_sec": window_end - window_start,
            **cmd_vel_window,
        },
        "reference_path_after_recovery": references,
        "non_empty_reference_after_recovery_without_new_goal": any(
            entry["poses"] > 0 for entry in references
        ),
    }


def main() -> int:
    args = parse_args()
    rclpy.init()
    node = FaultObserver(
        {
            "status": args.status_topic,
            "snapshot": args.snapshot_topic,
            "emergency_stop": args.emergency_stop_topic,
            "cmd_vel": args.cmd_vel_topic,
            "reference_path": args.reference_path_topic,
        }
    )
    try:
        if args.ready_file:
            with open(args.ready_file, "w", encoding="utf-8") as handle:
                handle.write(f"{node.subscription_count}\n")
        print("OBSERVER_READY", flush=True)

        fault_monotonic = spin_until_file(
            node, args.fault_trigger_file, time.monotonic() + args.fault_wait_sec
        )
        if fault_monotonic is None:
            print("fault trigger file never appeared", file=sys.stderr)
            return 2

        # One settle interval, then a navigation-speed zero window.
        spin_until(node, fault_monotonic + args.settle_sec)
        window_start = time.monotonic()
        window_end = window_start + max(0.1, args.zero_window_sec)
        spin_until(node, window_end)
        if args.post_window_sec > 0.0:
            spin_until(node, window_end + args.post_window_sec)

        report = build_report(
            args, node.observation, fault_monotonic, window_start, window_end
        )

        if args.recovery_trigger_file:
            fault_status = first_at_or_after(
                node.observation.status, lambda s: not s.ready, fault_monotonic
            )
            recovery_monotonic = spin_until_file(
                node,
                args.recovery_trigger_file,
                time.monotonic() + args.fault_wait_sec,
            )
            if recovery_monotonic is None:
                report["recovery"] = {
                    "observed": False,
                    "reason": "recovery trigger file never appeared",
                }
            else:
                spin_until(node, recovery_monotonic + args.settle_sec)
                recovery_window_start = time.monotonic()
                recovery_window_end = recovery_window_start + max(
                    0.1, args.zero_window_sec
                )
                spin_until(node, recovery_window_end)
                report["recovery"] = build_recovery_report(
                    args,
                    node.observation,
                    fault_status,
                    recovery_monotonic,
                    recovery_window_start,
                    recovery_window_end,
                )
                report["recovery"]["observed"] = True

        report["gate"] = gate_verdict(report)
        with open(args.output, "w", encoding="utf-8") as handle:
            json.dump(report, handle, indent=2, sort_keys=True)
        print(f"observation written to {args.output}", flush=True)
        for name, passed in sorted(report["gate"]["checks"].items()):
            print(f"gate {name}={'pass' if passed else 'FAIL'}", flush=True)
        if args.require_gate and not report["gate"]["passed"]:
            print("unknown fault gate failed", file=sys.stderr)
            return 4
        return 0
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    sys.exit(main())
