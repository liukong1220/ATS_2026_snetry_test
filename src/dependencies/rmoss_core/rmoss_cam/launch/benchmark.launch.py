 

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    ld = LaunchDescription()
    image_path = os.path.join(get_package_share_directory('rmoss_cam'), 'resource', 'test.jpg')
    calibration_path = 'package://rmoss_cam/resource/image_cam_calibration.yaml'
    virtual_image_cam = Node(
        package='rmoss_cam',
        executable='virtual_cam',
        name='virtual_cam1',
        parameters=[
            {'image_path': image_path,
             'camera_name': 'benchmark_cam1',
             'camera_info_url': calibration_path,
             'fps': 100,
             'autostart': True}],
        output='screen'
    )
    ld.add_action(virtual_image_cam)
    benchmark_test_node = Node(
        package='rmoss_cam',
        executable='benchmark_test',
        parameters=[
            {'image_path': image_path,
             'camera_info_url': calibration_path,
             'fps': 100,
             'autostart': True}],
        output='screen'
    )
    ld.add_action(benchmark_test_node)
    return ld
