#!/usr/bin/env python3

from collections import deque

from geometry_msgs.msg import PoseStamped
import rclpy
from nav_msgs.msg import Odometry, Path
from rclpy.node import Node


class DebugPathRecorder(Node):
    def __init__(self) -> None:
        super().__init__("debug_path_recorder")
        self.max_len = int(self.declare_parameter("max_len", 300).value)
        self.odom_topic = self.declare_parameter("odom_topic", "odometry").value
        self.gt_topic = self.declare_parameter(
            "gt_topic", "chassis_odometry_gt"
        ).value
        self.odom_path_topic = self.declare_parameter("odom_path_topic", "odom_path").value
        self.gt_path_topic = self.declare_parameter(
            "gt_path_topic", "chassis_odometry_gt_path"
        ).value

        self.odom_poses = deque(maxlen=self.max_len)
        self.gt_poses = deque(maxlen=self.max_len)

        self.odom_path_pub = self.create_publisher(Path, self.odom_path_topic, 10)
        self.gt_path_pub = self.create_publisher(Path, self.gt_path_topic, 10)

        self.create_subscription(Odometry, self.odom_topic, self.odom_cb, 20)
        self.create_subscription(Odometry, self.gt_topic, self.gt_cb, 20)

    def odom_cb(self, msg: Odometry) -> None:
        pose_stamped = PoseStamped()
        pose_stamped.header = msg.header
        pose_stamped.pose = msg.pose.pose
        self.odom_poses.append(pose_stamped)
        self.publish_path(self.odom_path_pub, msg.header, self.odom_poses)

    def gt_cb(self, msg: Odometry) -> None:
        pose_stamped = PoseStamped()
        pose_stamped.header = msg.header
        pose_stamped.pose = msg.pose.pose
        self.gt_poses.append(pose_stamped)
        self.publish_path(self.gt_path_pub, msg.header, self.gt_poses)

    def publish_path(self, pub, header, poses) -> None:
        path = Path()
        path.header = header
        path.poses = list(poses)
        pub.publish(path)


def main() -> None:
    rclpy.init()
    node = DebugPathRecorder()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
