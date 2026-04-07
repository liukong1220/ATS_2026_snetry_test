import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    image_path = os.path.join(get_package_share_directory('rmoss_cam'), 'resource', 'test.jpg')
    calibration_path = 'package://rmoss_cam/resource/image_cam_calibration.yaml'
    virtual_image_cam = Node(
        package='rmoss_cam',
        executable='virtual_cam',
        name='virtual_image_cam',
        parameters=[
            {'image_path': image_path,
             'camera_name': 'front_camera',
             'camera_info_url': calibration_path,
             'fps': 30,
             'autostart': True}],
        output='screen'
    )
    ld = LaunchDescription()
    ld.add_action(virtual_image_cam)
    return ld
