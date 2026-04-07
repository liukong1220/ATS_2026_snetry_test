

import os
import unittest

from launch import LaunchDescription
from launch_ros.actions import Node
import launch_testing
import launch_testing.asserts
import launch_testing.markers
import pytest


@pytest.mark.rostest
@launch_testing.markers.keep_alive
def generate_test_description():
    test_urdfs_dir = os.path.dirname(__file__)
    joint_state_publisher = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        arguments=[os.path.join(test_urdfs_dir, 'mimic_cycle.urdf')])
    return (
        LaunchDescription([joint_state_publisher, launch_testing.actions.ReadyToTest()]),
        {'joint_state_publisher': joint_state_publisher})


class TestMimicCycleRobot(unittest.TestCase):

    def test_termination(self, proc_info, joint_state_publisher):
        proc_info.assertWaitForShutdown(process=joint_state_publisher, timeout=10)


@launch_testing.post_shutdown_test()
class TestMimicCycleRobotAfterShutdown(unittest.TestCase):

    def test_exit(self, proc_info, proc_output):
        jsp = list(proc_info.process_names())[0]
        expected_output = 'Found an infinite recursive mimic chain'
        launch_testing.asserts.assertInStderr(proc_output,
                                              expected_output=expected_output,
                                              process=jsp)
        launch_testing.asserts.assertExitCodes(proc_info, process=jsp, allowable_exit_codes=[1])
