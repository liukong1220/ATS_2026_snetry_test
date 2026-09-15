#!/usr/bin/env python3
"""Accumulate /registered_scan into map frame and write a prior PCD.

Gazebo GT mode already publishes /registered_scan in odom. This helper looks up
map<-cloud TF (or map<-odom + cloud frame) and dumps an ASCII/binary PCD usable
by small_gicp_relocalization as prior_pcd_file.

Example:
  ros2 run --prefix "python3" ...  # or:
  python3 scripts/dump_gazebo_prior_pcd.py \\
    --output src/ats_sentry_bringup/pcd/rmuc_2025_gazebo_prior.pcd \\
    --duration 20 --voxel 0.05
"""

from __future__ import annotations

import argparse
import sys
import time
from typing import List, Tuple

import numpy as np
import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
from tf2_ros import Buffer, TransformListener


def _quat_to_rot(x: float, y: float, z: float, w: float) -> np.ndarray:
    xx, yy, zz = x * x, y * y, z * z
    xy, xz, yz = x * y, x * z, y * z
    wx, wy, wz = w * x, w * y, w * z
    return np.array(
        [
            [1.0 - 2.0 * (yy + zz), 2.0 * (xy - wz), 2.0 * (xz + wy)],
            [2.0 * (xy + wz), 1.0 - 2.0 * (xx + zz), 2.0 * (yz - wx)],
            [2.0 * (xz - wy), 2.0 * (yz + wx), 1.0 - 2.0 * (xx + yy)],
        ],
        dtype=np.float64,
    )


def _voxel_downsample(xyz: np.ndarray, leaf: float) -> np.ndarray:
    if xyz.size == 0 or leaf <= 0.0:
        return xyz
    keys = np.floor(xyz / leaf).astype(np.int64)
    # unique rows
    _, idx = np.unique(keys, axis=0, return_index=True)
    return xyz[np.sort(idx)]


def write_pcd(path: str, xyz: np.ndarray) -> None:
    n = int(xyz.shape[0])
    header = (
        "# .PCD v0.7 - Point Cloud Data file format\n"
        "VERSION 0.7\n"
        "FIELDS x y z\n"
        "SIZE 4 4 4\n"
        "TYPE F F F\n"
        "COUNT 1 1 1\n"
        f"WIDTH {n}\n"
        "HEIGHT 1\n"
        "VIEWPOINT 0 0 0 1 0 0 0\n"
        f"POINTS {n}\n"
        "DATA binary\n"
    )
    with open(path, "wb") as f:
        f.write(header.encode("ascii"))
        f.write(xyz.astype(np.float32, copy=False).tobytes(order="C"))


class PriorPcdDumper(Node):
    def __init__(self, args: argparse.Namespace) -> None:
        super().__init__("dump_gazebo_prior_pcd")
        self._args = args
        self._tf = Buffer(cache_time=Duration(seconds=30.0))
        self._tf_listener = TransformListener(self._tf, self)
        self._chunks: List[np.ndarray] = []
        self._accepted = 0
        self._dropped = 0
        qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=5,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
        )
        self.create_subscription(PointCloud2, args.cloud_topic, self._on_cloud, qos)
        self.get_logger().info(
            f"Dumping {args.cloud_topic} -> map for {args.duration:.1f}s "
            f"(voxel={args.voxel})"
        )

    def _on_cloud(self, msg: PointCloud2) -> None:
        source = msg.header.frame_id
        if not source:
            self._dropped += 1
            return
        stamp = Time.from_msg(msg.header.stamp)
        try:
            tf = self._tf.lookup_transform(
                self._args.map_frame,
                source,
                stamp,
                timeout=Duration(seconds=self._args.tf_timeout),
            )
        except Exception:
            try:
                tf = self._tf.lookup_transform(
                    self._args.map_frame,
                    source,
                    Time(),
                    timeout=Duration(seconds=self._args.tf_timeout),
                )
            except Exception:
                self._dropped += 1
                return

        pts = point_cloud2.read_points(
            msg, field_names=("x", "y", "z"), skip_nans=True
        )
        arr = np.fromiter(
            ((p[0], p[1], p[2]) for p in pts),
            dtype=np.dtype([("x", np.float32), ("y", np.float32), ("z", np.float32)]),
        )
        if arr.size == 0:
            self._dropped += 1
            return
        xyz = np.column_stack((arr["x"], arr["y"], arr["z"])).astype(np.float64)
        t = tf.transform.translation
        q = tf.transform.rotation
        R = _quat_to_rot(q.x, q.y, q.z, q.w)
        out = (R @ xyz.T).T
        out[:, 0] += t.x
        out[:, 1] += t.y
        out[:, 2] += t.z
        # Keep a vertical band useful for GICP walls / structure.
        z0, z1 = self._args.z_min, self._args.z_max
        out = out[(out[:, 2] >= z0) & (out[:, 2] <= z1)]
        if out.size == 0:
            self._dropped += 1
            return
        self._chunks.append(out.astype(np.float32))
        self._accepted += 1

    def finish(self) -> Tuple[int, int, int]:
        if not self._chunks:
            return 0, self._accepted, self._dropped
        xyz = np.concatenate(self._chunks, axis=0)
        xyz = _voxel_downsample(xyz, self._args.voxel)
        write_pcd(self._args.output, xyz)
        return int(xyz.shape[0]), self._accepted, self._dropped


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", required=True)
    parser.add_argument("--cloud-topic", default="/registered_scan")
    parser.add_argument("--map-frame", default="map")
    parser.add_argument("--duration", type=float, default=20.0)
    parser.add_argument("--voxel", type=float, default=0.05)
    parser.add_argument("--tf-timeout", type=float, default=0.10)
    parser.add_argument("--z-min", type=float, default=-0.5)
    parser.add_argument("--z-max", type=float, default=2.5)
    args = parser.parse_args()

    rclpy.init()
    node = PriorPcdDumper(args)
    t0 = time.time()
    try:
        while rclpy.ok() and (time.time() - t0) < args.duration:
            rclpy.spin_once(node, timeout_sec=0.1)
    finally:
        n_pts, ok, drop = node.finish()
        node.get_logger().info(
            f"Wrote {args.output}: points={n_pts} clouds_ok={ok} dropped={drop}"
        )
        node.destroy_node()
        rclpy.shutdown()
    if n_pts <= 0:
        print("ERROR: no points dumped", file=sys.stderr)
        return 2
    print(f"OK points={n_pts} output={args.output}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
