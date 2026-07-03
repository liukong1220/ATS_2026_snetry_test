


from datetime import datetime

from launch import LaunchDescription
from launch.actions import ExecuteProcess


def generate_launch_description():
    current_time = datetime.now().strftime("%Y-%m-%d_%H-%M-%S")
    record_rosbag_cmd = ExecuteProcess(
        cmd=[
            "ros2",
            "bag",
            "record",
            "-o",
            f"rosbags/sentry_{current_time}",
            "/serial/gimbal_joint_state",
            "/livox/imu",
            "/livox/lidar",
            "--compression-mode",
            "file",
            "--compression-format",
            "zstd",
            "-d",
            "30",
        ],
        output="screen",
    )

    # Create the launch description and populate
    ld = LaunchDescription()

    # Add other nodes and processes we need
    ld.add_action(record_rosbag_cmd)

    return ld
