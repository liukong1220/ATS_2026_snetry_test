from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    params_file = LaunchConfiguration("params_file")

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "params_file",
                default_value=PathJoinSubstitution(
                    [FindPackageShare("minco_planner"), "config", "minco_planner.yaml"]
                ),
                description="Path to minco_planner parameter file.",
            ),
            Node(
                package="minco_planner",
                executable="minco_planner_node",
                name="minco_planner",
                output="screen",
                parameters=[params_file],
            ),
        ]
    )
