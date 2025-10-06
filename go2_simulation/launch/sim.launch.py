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
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration, PythonExpression
from launch.actions import AppendEnvironmentVariable

def generate_launch_description():
    position_x = DeclareLaunchArgument("position_x", default_value = "14.2")
    position_y = DeclareLaunchArgument("position_y", default_value = "-10.6")
    orientation_yaw = DeclareLaunchArgument("orientation_yaw", default_value = "3.142")
    use_sim_time = DeclareLaunchArgument("use_sim_time", default_value = "false", 
                                         description = "Use simulation (Gazebo) clock if true")


    pkg_go2_simulation = get_package_share_directory("go2_simulation")
    world_file = DeclareLaunchArgument(
        "world_file", 
        default_value = PathJoinSubstitution([pkg_go2_simulation, "worlds", "warehouse.sdf"])
    )
    
    gz_sim_share = get_package_share_directory("ros_gz_sim")
    gz_sim = PathJoinSubstitution([gz_sim_share, "launch", "gz_sim.launch.py"])
    gz_sim_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(gz_sim),
        launch_arguments = {
            "gz_args": PythonExpression(["'", LaunchConfiguration("world_file"), " -r'"])
        }.items()
    )

    go2_spawn = PathJoinSubstitution([pkg_go2_simulation, "launch", "go2_spawn.launch.py"])
    go2_spawn_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(go2_spawn),
        launch_arguments = {
            "position_x": LaunchConfiguration("position_x"),
            "position_y": LaunchConfiguration("position_y"),
            "orientation_yaw": LaunchConfiguration("orientation_yaw"),
            "use_sim_time": LaunchConfiguration("use_sim_time")
        }.items()
    )

    ign_resource_path = [
        AppendEnvironmentVariable(
            name = "GZ_SIM_RESOURCE_PATH",
            value = PathJoinSubstitution([pkg_go2_simulation, "worlds"])
        ),
        AppendEnvironmentVariable(
            name = "GZ_SIM_RESOURCE_PATH",
            value = PathJoinSubstitution([pkg_go2_simulation, "models"])
        )
    ]
    
    ld = LaunchDescription(ign_resource_path)
    ld.add_action(position_x)
    ld.add_action(position_y)
    ld.add_action(orientation_yaw)
    ld.add_action(world_file)
    ld.add_action(use_sim_time)
    ld.add_action(gz_sim_node)
    ld.add_action(go2_spawn_node)
    return ld