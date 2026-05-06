#!/usr/bin/env python3

import sys
import time

import rclpy
from nav_msgs.msg import Path
from nav2_msgs.msg import Costmap
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy


TOPICS = [
    ("/plan", Path),
    ("/smoothed_path_visual", Path),
    ("/transformed_global_plan", Path),
    ("/global_costmap/costmap_raw", Costmap),
    ("/local_costmap/costmap_raw", Costmap),
]


class Grabber(Node):
    def __init__(self):
        super().__init__("diag_path_costs")
        self.data = {}
        self.subs = []
        transient_qos = QoSProfile(depth=1)
        transient_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        transient_qos.reliability = ReliabilityPolicy.RELIABLE
        for topic, msg_type in TOPICS:
            qos = transient_qos if msg_type is Costmap else 10
            self.subs.append(
                self.create_subscription(msg_type, topic, self._cb(topic), qos)
            )

    def _cb(self, topic):
        def inner(msg):
            if topic not in self.data:
                self.data[topic] = msg

        return inner


def world_to_cell(costmap, x, y):
    meta = costmap.metadata
    mx = int((x - meta.origin.position.x) / meta.resolution)
    my = int((y - meta.origin.position.y) / meta.resolution)
    if mx < 0 or my < 0 or mx >= meta.size_x or my >= meta.size_y:
        return None
    return mx, my


def sample_cost(path, costmap):
    meta = costmap.metadata
    cells = list(costmap.data)
    vals = []
    oob = 0

    for pose in path.poses:
        pt = world_to_cell(costmap, pose.pose.position.x, pose.pose.position.y)
        if pt is None:
            oob += 1
            continue
        mx, my = pt
        vals.append(cells[my * meta.size_x + mx])

    if not vals:
        return {"count": 0, "oob": oob}

    return {
        "count": len(vals),
        "oob": oob,
        "min": min(vals),
        "max": max(vals),
        "avg": round(sum(vals) / len(vals), 2),
        "inflated_pts": sum(1 for v in vals if v >= 1),
        "highcost_pts": sum(1 for v in vals if v >= 128),
    }


def main():
    rclpy.init()
    node = Grabber()

    start = time.time()
    while time.time() - start < 10.0 and len(node.data) < len(TOPICS):
        rclpy.spin_once(node, timeout_sec=0.2)

    required = ["/plan", "/smoothed_path_visual", "/transformed_global_plan"]
    missing = [topic for topic in required if topic not in node.data]
    if missing:
        print("MISSING_REQUIRED", missing)
        node.destroy_node()
        rclpy.shutdown()
        return 1

    available_costmaps = []
    for costmap_topic in ["/global_costmap/costmap_raw", "/local_costmap/costmap_raw"]:
        if costmap_topic in node.data:
            available_costmaps.append(costmap_topic)

    if not available_costmaps:
        print("MISSING_COSTMAPS")
        node.destroy_node()
        rclpy.shutdown()
        return 1

    for costmap_topic in available_costmaps:
        print("COSTMAP", costmap_topic)
        costmap = node.data[costmap_topic]
        for topic in ["/plan", "/smoothed_path_visual", "/transformed_global_plan"]:
            print(topic, sample_cost(node.data[topic], costmap))

    node.destroy_node()
    rclpy.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
