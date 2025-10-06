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
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, Command
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.conditions import IfCondition
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pkg_go2_description = get_package_share_directory("go2_description")
    this_package = get_package_share_directory("go2_controller")
    joints_config = os.path.join(this_package, "config", "joints", "joints.yaml")
    gait_config = os.path.join(this_package, "config", "gait", "gait.yaml")
    links_config = os.path.join(this_package, "config", "links", "links.yaml")

    use_sim_time = DeclareLaunchArgument(name = "use_sim_time", default_value = "false",
                                         description = "Use simulation (Gazebo) clock if true")
    sim_running = DeclareLaunchArgument(name = "gazebo", default_value = "false",
                                        description = "Enable if running in Gazebo")
    publish_joint_states = DeclareLaunchArgument(name = "publish_joint_states", default_value = "true",
                                                 description = "Publish joint states")
    publish_joint_control = DeclareLaunchArgument(name = "publish_joint_control", default_value = "true",
                                                  description = "Publish joint control commands")
    publish_foot_contacts = DeclareLaunchArgument(name = "publish_foot_contacts", default_value = "true",
                                                  description = "Publish foot contacts")
    joint_controller_topic = DeclareLaunchArgument(name = "joint_controller_topic", default_value = "joint_group_effort_controller/joint_trajectory",
                                                   description = "Topic for joint controller")
    robot_description_path = DeclareLaunchArgument(
        name = "robot_description_path", 
        default_value = PathJoinSubstitution([pkg_go2_description, "xacro", "robot.xacro"]),
        description = "Path to robot description xacro file"
    )
    orientation_from_imu = DeclareLaunchArgument(name = "orientation_from_imu", default_value = "true",
                                                 description = "Use IMU for orientation in state estimation")
    base_link_frame = DeclareLaunchArgument(name = "base_link_frame", default_value = "base_link",
                                            description="Base link frame id")
    launch_rviz = DeclareLaunchArgument(name = "rviz", default_value = "false", 
                                        description="Run RViz")
    rviz_config_path = DeclareLaunchArgument(
        name = "rviz_config_path",
        default_value = PathJoinSubstitution([this_package, "rviz", "control.rviz"]),
        description = "Path to RViz config file"
    )

    robot_description_launch = PathJoinSubstitution(
        [pkg_go2_description, "launch", "robot_description.launch.py"]
    )

    robot_state_publisher = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(robot_description_launch)
    )

    quadruped_controller_node = Node(
        package = "champ_base",
        executable = "quadruped_controller_node",
        output = "screen",
        parameters=[
            {"use_sim_time": LaunchConfiguration("use_sim_time")},
            {"gazebo": LaunchConfiguration("gazebo")},
            {"publish_joint_states": LaunchConfiguration("publish_joint_states")},
            {"publish_joint_control": LaunchConfiguration("publish_joint_control")},
            {"publish_foot_contacts": LaunchConfiguration("publish_foot_contacts")},
            {"joint_controller_topic": LaunchConfiguration("joint_controller_topic")},
            {"urdf": Command(["xacro", " ", LaunchConfiguration("robot_description_path")])},
            joints_config,
            links_config,
            gait_config,
        ],
        remappings=[("/cmd_vel/smooth", "/cmd_vel")],
    )

    state_estimator_node = Node(
        package = "champ_base",
        executable = "state_estimation_node",
        output = "screen",
        parameters=[
            {"use_sim_time": LaunchConfiguration("use_sim_time")},
            {"orientation_from_imu": LaunchConfiguration("orientation_from_imu")},
            {"urdf": Command(["xacro", " ", LaunchConfiguration("robot_description_path")])},
            joints_config,
            links_config,
            gait_config,
        ],
    )

    base_to_footprint_ekf = Node(
        package = "robot_localization",
        executable = "ekf_node",
        name = "base_to_footprint_ekf",
        output = "screen",
        parameters=[
            {"base_link_frame": LaunchConfiguration("base_link_frame")},
            {"use_sim_time": LaunchConfiguration("use_sim_time")},
            os.path.join(
                get_package_share_directory("champ_base"),
                "config",
                "ekf",
                "base_to_footprint.yaml",
            ),
        ],
        remappings=[("odometry/filtered", "odom/local")],
    )

    footprint_to_odom_ekf = Node(
        package = "robot_localization",
        executable = "ekf_node",
        name = "footprint_to_odom_ekf",
        output = "screen",
        parameters=[
            {"base_link_frame": LaunchConfiguration("base_link_frame")},
            {"use_sim_time": LaunchConfiguration("use_sim_time")},
            os.path.join(
                get_package_share_directory("champ_base"),
                "config",
                "ekf",
                "footprint_to_odom.yaml",
            ),
        ],
        remappings=[("odometry/filtered", "odom")],
    )

    rviz = Node(
        package = "rviz2",
        namespace = "",
        executable = "rviz2",
        name = "rviz2",
        arguments = ["-d", LaunchConfiguration("rviz_config_path")],
        condition = IfCondition(LaunchConfiguration("rviz"))
    )

    ld =  LaunchDescription()
    ld.add_action(use_sim_time)
    ld.add_action(sim_running)
    ld.add_action(publish_joint_states)
    ld.add_action(publish_joint_control)
    ld.add_action(publish_foot_contacts)
    ld.add_action(joint_controller_topic)
    ld.add_action(robot_description_path)
    ld.add_action(orientation_from_imu)
    ld.add_action(base_link_frame)
    ld.add_action(launch_rviz)
    ld.add_action(rviz_config_path)
    ld.add_action(robot_state_publisher)
    ld.add_action(quadruped_controller_node)
    ld.add_action(state_estimator_node)
    ld.add_action(base_to_footprint_ekf)
    ld.add_action(footprint_to_odom_ekf)
    ld.add_action(rviz)
    
    return ld