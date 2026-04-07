
import os
import time
import unittest

from launch import LaunchDescription
from launch_ros.actions import Node
import launch_testing
import pytest
import rclpy
import sensor_msgs.msg


@pytest.mark.rostest
def generate_test_description():
    test_urdfs_dir = os.path.dirname(__file__)
    return LaunchDescription([
        Node(
            package='joint_state_publisher',
            executable='joint_state_publisher',
            arguments=[os.path.join(test_urdfs_dir, '64_joint_robot.urdf')]),
        launch_testing.actions.ReadyToTest(),
    ])


class Test64JointRobot(unittest.TestCase):
    TIMEOUT = 10

    @classmethod
    def setUpClass(cls):
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def setUp(self):
        # Create a ROS node for tests
        self.node = rclpy.create_node('test_64_joint_robot')

    def tearDown(self):
        self.node.destroy_node()

    def test_all_64_joints_published(self):
        msgs_rx = []
        sub = self.node.create_subscription(
            sensor_msgs.msg.JointState, 'joint_states', lambda msg: msgs_rx.append(msg), 1)
        try:
            end_time = time.monotonic() + self.TIMEOUT
            while time.monotonic() < end_time and not msgs_rx:
                rclpy.spin_once(self.node, timeout_sec=0.1)

            assert 64 == len(msgs_rx[0].name)
            assert 64 == len(set(msgs_rx[0].name))
            assert 64 == len(msgs_rx[0].position)
            assert 0 == len(msgs_rx[0].velocity)
            assert 0 == len(msgs_rx[0].effort)
        finally:
            self.node.destroy_subscription(sub)
