
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    params_file = LaunchConfiguration('params_file')
    declare_params_file_cmd = DeclareLaunchArgument(
        'params_file',
        default_value=os.path.join(
            get_package_share_directory('nav2_loopback_sim'), 'nav2_params.yaml'
        ),
        description='Full path to the ROS2 parameters file to use for all launched nodes',
    )

    scan_frame_id = LaunchConfiguration('scan_frame_id')
    declare_scan_frame_id_cmd = DeclareLaunchArgument(
        'scan_frame_id',
        default_value='base_scan',
    )

    loopback_sim_cmd = Node(
        package='nav2_loopback_sim',
        executable='loopback_simulator',
        name='loopback_simulator',
        output='screen',
        parameters=[params_file, {'scan_frame_id': scan_frame_id}],
    )

    ld = LaunchDescription()
    ld.add_action(declare_scan_frame_id_cmd)
    ld.add_action(declare_params_file_cmd)
    ld.add_action(loopback_sim_cmd)
    return ld
