#!/usr/bin/env python3
# SPDX-License-Identifier: Apache-2.0
import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration
from launch.conditions import IfCondition


def generate_launch_description():
    # --- package shares -------------------------------------------------------
    pkg_go2_config = get_package_share_directory("go2_champ_config")
    pkg_go2_description = get_package_share_directory("go2_description")
    pkg_go2_simulation = get_package_share_directory("go2_simulation")
    # -------------------------------------------------------------------------

    # --- default paths -------------------------------------------------------
    default_joints = os.path.join(pkg_go2_config, "config", "joints", "joints.yaml")
    default_gait = os.path.join(pkg_go2_config, "config", "gait", "gait.yaml")
    default_links = os.path.join(pkg_go2_config, "config", "links", "links.yaml")
    default_model_path = os.path.join(pkg_go2_description, "xacro", "robot.xacro")
    # -------------------------------------------------------------------------

    # --- LaunchConfiguration aliases -----------------------------------------
    use_sim_time = LaunchConfiguration("use_sim_time")
    rviz = LaunchConfiguration("rviz")
    robot_name = LaunchConfiguration("robot_name")
    lite = LaunchConfiguration("lite")
    # -------------------------------------------------------------------------

    # --- Declare Launch Arguments --------------------------------------------
    declare_use_sim_time = DeclareLaunchArgument(
        "use_sim_time",
        default_value="true",
        description="Use simulation (Gazebo) clock if true",
    )

    declare_rviz = DeclareLaunchArgument(
        "rviz", default_value="false", description="Launch rviz"
    )

    declare_robot_name = DeclareLaunchArgument(
        "robot_name", default_value="go2", description="Robot name"
    )

    declare_lite = DeclareLaunchArgument(
        "lite", default_value="false", description="Lite"
    )

    declare_gui = DeclareLaunchArgument("gui", default_value="true", description="Use GUI")

    declare_world_init_x = DeclareLaunchArgument("world_init_x", default_value="0.0")
    declare_world_init_y = DeclareLaunchArgument("world_init_y", default_value="0.0")
    declare_world_init_z = DeclareLaunchArgument("world_init_z", default_value="0.275")
    declare_world_init_heading = DeclareLaunchArgument(
        "world_init_heading", default_value="0.0"
    )
    # -------------------------------------------------------------------------

    # --- Include: champ_bringup.launch.py ------------------------------------
    bringup_launch = PathJoinSubstitution(
        [pkg_go2_config, "launch", "bringup.launch.py"]
    )
    bringup_include = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(bringup_launch),
        launch_arguments={
            "description_path": default_model_path,
            "joints_map_path": default_joints,
            "links_map_path": default_links,
            "gait_config_path": default_gait,
            "use_sim_time": use_sim_time,
            "robot_name": robot_name,
            "gazebo": "true",
            "lite": lite,
            "rviz": rviz,
            "joint_controller_topic": "joint_group_effort_controller/joint_trajectory",
            "hardware_connected": "false",
            "publish_foot_contacts": "false",
            "close_loop_odom": "true",
        }.items(),
    )
    # -------------------------------------------------------------------------

    # --- Include: champ_gazebo.launch.py -------------------------------------
    gazebo_launch = PathJoinSubstitution(
        [pkg_go2_simulation, "launch", "sim.launch.py"]
    )
    gazebo_include = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(gazebo_launch),
        launch_arguments={
            "use_sim_time": use_sim_time,
        }.items(),
    )
    # -------------------------------------------------------------------------

    # --- Build and return LaunchDescription ---------------------------------
    ld = LaunchDescription()

    # argument declarations
    ld.add_action(declare_use_sim_time)
    ld.add_action(declare_rviz)
    ld.add_action(declare_robot_name)
    ld.add_action(declare_lite)
    ld.add_action(declare_gui)
    ld.add_action(declare_world_init_x)
    ld.add_action(declare_world_init_y)
    ld.add_action(declare_world_init_z)
    ld.add_action(declare_world_init_heading)

    # includes
    ld.add_action(bringup_include)
    ld.add_action(gazebo_include)
    return ld