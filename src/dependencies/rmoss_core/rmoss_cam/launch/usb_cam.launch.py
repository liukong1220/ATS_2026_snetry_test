 

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    params = os.path.join(get_package_share_directory('rmoss_cam'), 'config', 'cam_params.yaml')
    return LaunchDescription([
        Node(package='rmoss_cam',
             executable='usb_cam',
             parameters=[params],
             output='screen')
    ])
