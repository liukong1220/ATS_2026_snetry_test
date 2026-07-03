"""Bridge Nav2 Twist commands to the MuJoCo chassis MotionCtrl interface."""

from __future__ import annotations

import rclpy
from geometry_msgs.msg import Twist
from manda_can_control.msg import MotionCtrl
from rclpy.node import Node


class TwistToMotionCtrl(Node):
    """Convert geometry_msgs/Twist into manda_can_control/MotionCtrl."""

    def __init__(self) -> None:
        super().__init__("twist_to_motion_ctrl")
        self.declare_parameter("input_topic", "cmd_vel_nav2_result")
        self.declare_parameter("output_topic", "/motion_control")
        self.declare_parameter("linear_scale", 1.0)
        self.declare_parameter("angular_scale", 1.0)
        self.declare_parameter("max_linear_x", 3.0)
        self.declare_parameter("max_linear_y", 3.0)
        self.declare_parameter("max_angular_z", 6.0)

        self.linear_scale = float(self.get_parameter("linear_scale").value)
        self.angular_scale = float(self.get_parameter("angular_scale").value)
        self.max_linear_x = abs(float(self.get_parameter("max_linear_x").value))
        self.max_linear_y = abs(float(self.get_parameter("max_linear_y").value))
        self.max_angular_z = abs(float(self.get_parameter("max_angular_z").value))

        input_topic = str(self.get_parameter("input_topic").value)
        output_topic = str(self.get_parameter("output_topic").value)
        self.publisher = self.create_publisher(MotionCtrl, output_topic, 10)
        self.subscription = self.create_subscription(
            Twist,
            input_topic,
            self._twist_callback,
            10,
        )
        self.get_logger().info(
            f"Bridging Twist '{input_topic}' -> MotionCtrl '{output_topic}'"
        )

    @staticmethod
    def _clamp(value: float, limit: float) -> float:
        if limit <= 0.0:
            return value
        return max(-limit, min(limit, value))

    def _twist_callback(self, msg: Twist) -> None:
        command = MotionCtrl()
        command.linear_x = self._clamp(
            float(msg.linear.x) * self.linear_scale,
            self.max_linear_x,
        )
        command.linear_y = self._clamp(
            float(msg.linear.y) * self.linear_scale,
            self.max_linear_y,
        )
        command.angular_z = self._clamp(
            float(msg.angular.z) * self.angular_scale,
            self.max_angular_z,
        )
        self.publisher.publish(command)


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = TwistToMotionCtrl()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
