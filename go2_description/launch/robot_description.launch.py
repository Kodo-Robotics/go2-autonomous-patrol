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

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, PathJoinSubstitution, LaunchConfiguration

from launch_ros.actions import Node

def generate_launch_description():
    use_sim_time = DeclareLaunchArgument("use_sim_time", default_value = "false", 
                                         choices = ["true", "false"],
                                         description = "use_sim_time")
    robot_name = DeclareLaunchArgument("robot_name", default_value = "go2patrol",
                                       description = "Robot name")
    namespace = DeclareLaunchArgument("namespace", default_value = LaunchConfiguration("robot_name"),
                                      description = "Robot namespace")
    
    pkg_go2_description = get_package_share_directory("go2_description")
    xacro_file = PathJoinSubstitution([pkg_go2_description, "urdf", "go2.urdf.xacro"])

    robot_state_publisher = Node(
        package = "robot_state_publisher",
        executable = "robot_state_publisher",
        name = "robot_state_publisher",
        output = "screen",
        parameters = [
            {"use_sim_time": LaunchConfiguration("use_sim_time")},
            {"robot_description": Command([
                'xacro', ' ', xacro_file, ' ',
                'gazebo:=ignition', ' ',
                'namespace:=', LaunchConfiguration("namespace")
            ])},
        ],
        remappings = [('/tf', 'tf'), ('/tf_static', 'tf_static')]
    )

    joint_state_publisher = Node(
        package = "joint_state_publisher",
        executable = "joint_state_publisher",
        name = "joint_state_publisher",
        output = "screen",
        parameters = [{'use_sim_time': LaunchConfiguration("use_sim_time")}],
        remappings = [('/tf', 'tf'), ('/tf_static', 'tf_static')]
    )

    ld = LaunchDescription()
    ld.add_action(use_sim_time)
    ld.add_action(robot_name)
    ld.add_action(namespace)
    ld.add_action(robot_state_publisher)
    ld.add_action(joint_state_publisher)
    return ld