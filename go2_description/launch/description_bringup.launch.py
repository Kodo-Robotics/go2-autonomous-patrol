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
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration
from launch.conditions import IfCondition

from launch_ros.actions import Node

def generate_launch_description():
    use_sim_time = DeclareLaunchArgument("use_sim_time", default_value = "false", 
                                         description = "Use simulation (Gazebo) clock if true")
    launch_gui = DeclareLaunchArgument("gui", default_value = "false",
                                       description = "Launch joint_state_publisher_gui if true")
    launch_rviz = DeclareLaunchArgument("rviz", default_value = "false",
                                        description = "Launch RViz")

    pkg_go2_description = get_package_share_directory("go2_description")
    robot_description_launch = PathJoinSubstitution([pkg_go2_description, "launch", 
                                                     "robot_description.launch.py"])
    robot_state_publisher = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(robot_description_launch),
        launch_arguments = {
            "use_sim_time": LaunchConfiguration("use_sim_time")
        }.items()
    )

    joint_state_publisher = Node(
        package = "joint_state_publisher",
        executable = "joint_state_publisher",
        name = "joint_state_publisher",
        output = "screen",
        parameters = [{'use_sim_time': LaunchConfiguration("use_sim_time")}]
    )

    joint_state_publisher_gui = Node(
        package = "joint_state_publisher_gui",
        executable = "joint_state_publisher_gui",
        name = "joint_state_pubisher_gui",
        output = "screen",
        parameters = [{'use_sim_time': LaunchConfiguration("use_sim_time")}],
        condition = IfCondition(LaunchConfiguration("gui"))
    )

    rviz_config_path = PathJoinSubstitution(
        [pkg_go2_description, 'rviz', 'visualize.rviz']
    )
    rviz = Node(
        package = 'rviz2',
        executable = 'rviz2',
        name = 'rviz2',
        output = 'screen',
        arguments = ['-d', rviz_config_path],
        condition = IfCondition(LaunchConfiguration("rviz")),
        parameters = [{'use_sim_time': LaunchConfiguration("use_sim_time")}]
    )

    ld = LaunchDescription()
    ld.add_action(use_sim_time)
    ld.add_action(launch_gui)
    ld.add_action(launch_rviz)
    ld.add_action(robot_state_publisher)
    ld.add_action(joint_state_publisher)
    ld.add_action(joint_state_publisher_gui)
    ld.add_action(rviz)
    return ld