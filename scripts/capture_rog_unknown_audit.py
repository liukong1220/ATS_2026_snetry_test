#!/usr/bin/env python3
"""Capture one structurally valid non-empty ROGMap unknown audit cloud."""

import argparse
import json
import os
import sys
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import PointCloud2


class UnknownAuditCapture(Node):
    """Retain the pre-fault subscriber until the all-unknown audit cloud arrives."""

    def __init__(self, topic: str, expected_frame: str) -> None:
        super().__init__("ats_rog_unknown_audit_capture")
        self._topic = topic
        self._expected_frame = expected_frame
        self.report: dict[str, object] | None = None
        qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )
        self.create_subscription(PointCloud2, topic, self._callback, qos)

    def _callback(self, message: PointCloud2) -> None:
        if self.report is not None or message.width == 0 or message.height == 0:
            return
        stamp_ns = message.header.stamp.sec * 1_000_000_000 + message.header.stamp.nanosec
        expected_bytes = message.width * message.height * message.point_step
        if (
            message.header.frame_id != self._expected_frame
            or stamp_ns <= 0
            or message.point_step == 0
            or len(message.data) < expected_bytes
        ):
            self.get_logger().warn("ignoring malformed non-empty ROGMap unknown audit cloud")
            return
        self.report = {
            "topic": self.resolve_topic_name(self._topic),
            "frame_id": message.header.frame_id,
            "stamp_ns": stamp_ns,
            "width": message.width,
            "height": message.height,
            "point_step": message.point_step,
            "data_bytes": len(message.data),
            "subscriber_reliability": "best_effort",
            "subscriber_durability": "volatile",
        }


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument("--topic", default="/rog_map/unk")
    parser.add_argument("--expected-frame", default="odom")
    parser.add_argument("--timeout-sec", type=float, default=30.0)
    parser.add_argument("--output", required=True)
    return parser.parse_args()


def atomic_write_json(path: str, payload: dict[str, object]) -> None:
    temporary = f"{path}.{os.getpid()}.tmp"
    with open(temporary, "w", encoding="utf-8") as handle:
        json.dump(payload, handle, sort_keys=True)
        handle.write("\n")
    os.replace(temporary, path)


def main() -> int:
    args = parse_args()
    if args.timeout_sec <= 0.0:
        raise ValueError("--timeout-sec must be positive")
    rclpy.init()
    node = UnknownAuditCapture(args.topic, args.expected_frame)
    deadline = time.monotonic() + args.timeout_sec
    try:
        while rclpy.ok() and node.report is None and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.05)
        if node.report is None:
            print("timed out waiting for a valid non-empty ROGMap unknown audit cloud", file=sys.stderr)
            return 1
        atomic_write_json(args.output, node.report)
        print(json.dumps(node.report, sort_keys=True))
        return 0
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    sys.exit(main())
