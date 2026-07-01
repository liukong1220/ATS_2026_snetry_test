# Copyright 2025 Lihan Chen
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import os

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription, TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, TextSubstitution


def generate_launch_description():
    pkg_simulator = get_package_share_directory("rmu_gazebo_simulator")

    gz_world_path = os.path.join(pkg_simulator, "config", "gz_world.yaml")
    with open(gz_world_path) as file:
        config = yaml.safe_load(file)
        default_world = config.get("world")

    world = LaunchConfiguration("world")
    gz_world_config = LaunchConfiguration("gz_world_path")
    robot_xmacro_file = LaunchConfiguration("robot_xmacro_file")
    auto_start_simulation = LaunchConfiguration("auto_start_simulation")

    declare_world = DeclareLaunchArgument(
        "world",
        default_value=default_world,
        description="Gazebo world name, e.g. rmuc_2025 or rmul_2025",
    )

    declare_gz_world_path = DeclareLaunchArgument(
        "gz_world_path",
        default_value=gz_world_path,
        description="Path to gz world config yaml",
    )
    declare_robot_xmacro_file = DeclareLaunchArgument(
        "robot_xmacro_file",
        default_value="",
        description="Robot SDF xmacro file path used for Gazebo spawning",
    )
    declare_auto_start_simulation = DeclareLaunchArgument(
        "auto_start_simulation",
        default_value="True",
        description=(
            "Whether to automatically unpause Gazebo after startup so the "
            "simulation matches the recommended split-flow without manual GUI interaction."
        ),
    )

    world_sdf_path = [
        TextSubstitution(text=os.path.join(pkg_simulator, "resource", "worlds", "")),
        world,
        TextSubstitution(text="_world.sdf"),
    ]
    ign_config_path = os.path.join(pkg_simulator, "resource", "ign", "gui.config")

    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_simulator, "launch", "gazebo.launch.py")
        ),
        launch_arguments={
            "world_sdf_path": world_sdf_path,
            "ign_config_path": ign_config_path,
        }.items(),
    )

    spawn_robots_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_simulator, "launch", "spawn_robots.launch.py")
        ),
        launch_arguments={
            "gz_world_path": gz_world_config,
            "world": world,
            "robot_xmacro_file": robot_xmacro_file,
        }.items(),
    )

    referee_system_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_simulator, "launch", "referee_system.launch.py")
        )
    )

    unpause_world = TimerAction(
        period=8.0,
        condition=IfCondition(auto_start_simulation),
        actions=[
            ExecuteProcess(
                cmd=[
                    "ign",
                    "service",
                    "-s",
                    "/world/default/control",
                    "--reqtype",
                    "ignition.msgs.WorldControl",
                    "--reptype",
                    "ignition.msgs.Boolean",
                    "--timeout",
                    "3000",
                    "--req",
                    "pause: false",
                ],
                output="screen",
            )
        ],
    )

    ld = LaunchDescription()

    ld.add_action(declare_world)
    ld.add_action(declare_gz_world_path)
    ld.add_action(declare_robot_xmacro_file)
    ld.add_action(declare_auto_start_simulation)
    ld.add_action(gazebo_launch)
    ld.add_action(spawn_robots_launch)
    ld.add_action(referee_system_launch)
    ld.add_action(unpause_world)

    return ld
