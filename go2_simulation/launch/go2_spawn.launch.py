# Copyright 2025 Kodo Robotics
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
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    position_x = DeclareLaunchArgument("position_x", default_value = "0.0")
    position_y = DeclareLaunchArgument("position_y", default_value = "0.0")
    orientation_yaw = DeclareLaunchArgument("orientation_yaw", default_value = "0.0")

    pkg_go2_description = get_package_share_directory("go2_description")
    pkg_go2_simulation = get_package_share_directory("go2_simulation")

    robot_description_launch = PathJoinSubstitution([pkg_go2_description, "launch", 
                                                     "robot_description.launch.py"])
    gz_bridge_yaml = os.path.join(pkg_go2_simulation, "config", "ros_gz_bridge.yaml")

    robot_state_publisher = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(robot_description_launch)
    )

    spawn_go2 = Node(
        package = "ros_gz_sim",
        executable = "create",
        arguments = [
            "-topic", "/robot_description",
            "-name", "go2",
            "-allow_renaming", "true",
            "-x", LaunchConfiguration("position_x"),
            "-y", LaunchConfiguration("position_y"),
            "-Y", LaunchConfiguration("orientation_yaw"),
            "-z", "0.4"
        ]
    )

    gz_bridge = Node(
        package = "ros_gz_bridge",
        executable = "parameter_bridge",
        arguments = [
            "--ros-args",
            "-p",
            f"config_file:={gz_bridge_yaml}"
        ],
        output = "screen"
    )

    joint_state_controller = ExecuteProcess(
        cmd = ["ros2", "control", "load_controller", "--set-state", "active",
               "joint_states_controller"],
        output = "screen"
    )
    joint_trajectory_position_controller = ExecuteProcess(
        cmd = ["ros2", "control", "load_controller", "--set-state", "active",
               "joint_group_position_controller"],
        output = "screen"
    )
    joint_trajectory_effort_controller = ExecuteProcess(
        cmd = ["ros2", "control", "load_controller", "--set-state", "active",
               "joint_group_effort_controller"],
        output = "screen"
    )
    
    ld = LaunchDescription()
    ld.add_action(position_x)
    ld.add_action(position_y)
    ld.add_action(orientation_yaw)
    # ld.add_action(robot_state_publisher)
    ld.add_action(spawn_go2)
    ld.add_action(gz_bridge)
    ld.add_action(joint_state_controller)
    # ld.add_action(joint_trajectory_position_controller)
    ld.add_action(joint_trajectory_effort_controller)
    return ld