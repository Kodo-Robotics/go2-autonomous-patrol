# go2_patrol_ros2

Modular **ROS 2** framework for simulating and deploying **patrol behaviors** on the [Unitree Go2 quadruped](https://www.unitree.com/go2).  
Designed to run in simulation first (Gazebo / Isaac), then transfer seamlessly to the **real robot** using Unitree’s **Sport Mode** SDK.

## ✨ Features

- 🚶 **Patrol Missions**: Behavior Tree–based mission manager for routes, waypoints, and patrol loops.  
- 🗺️ **Navigation**: Nav2 integration (planners, costmaps, BT XML).  
- 👀 **Perception**: LiDAR + camera simulation plugins; EKF-based fusion with IMU/odom.  
- 🔌 **Driver Swap**: Pluggable **simulation driver** and **hardware driver (Sport Mode)** with identical APIs.  
- 🎥 **Operator I/O**: Optional WebRTC bridges for camera & audio.  
- 🛡️ **Safety**: E-stop service, watchdog, health reporting.  

## 📂 Repository Layout

```bash
go2_patrol_ws/src/
go2_apps_patrol/        # patrol manager (BTs, FSMs, routes)
go2_navigation/         # Nav2 configs and launch
go2_perception/         # lidar, camera, ekf, TF tree
go2_sim_driver/         # simulation backend
go2_unitree_driver/     # hardware backend (Sport Mode)
go2_io/                 # WebRTC video/audio bridges
go2_bringup/            # launch files, params, profiles
config/                 # common params, frame defs
```

## 🏗️ Architecture

```text
          +----------------+
          |   Applications |
          |  (Patrol BTs)  |
          +----------------+
                  |
                  v
          +----------------+
          |   Navigation   |
          |    (Nav2)      |
          +----------------+
                  |
        +--------------------+
        |    Robot Driver    |
        | (sim_driver OR     |
        |  unitree_driver)   |
        +--------------------+
            /          \
      Sensors           State
   (LiDAR/Camera)     (Odom/IMU)
```

* Drivers expose the same ROS topics/services whether in sim or on hardware.
* Perception stack publishes /points, /camera/image_raw, /imu.
* Apps & Nav2 always talk to /cmd_vel, /odom, /tf, and patrol services.

## 🚀 Getting Started

Prerequisites
* ROS 2 Humble or Jazzy
* Colcon build tools
* Gazebo (Fortress/Harmonic) or NVIDIA Isaac Sim (optional)
* For hardware: Unitree unitree_ros2 and SDK2

### Clone & Build

```bash
mkdir -p go2_patrol_ws/src
cd go2_patrol_ws/src
git clone https://github.com/your-org/go2_patrol_ros2.git
cd ..
rosdep install --from-paths src -y --ignore-src
colcon build
source install/setup.bash
```

## 🎮 Simulation

Launch full patrol stack in simulation:

```bash
ros2 launch go2_bringup sim_bringup.launch.py world:=warehouse.sdf
```

* Spawns Go2 in Gazebo.
* Starts perception, Nav2, patrol manager.
* RViz shows LiDAR & camera feeds.

## 🤖 Real Robot (Sport Mode)

1.	Source Unitree’s unitree_ros2:
  ```bash
  source ~/unitree_ros2/setup.sh
  ```

2.	Launch robot stack:

  ```bash
  ros2 launch go2_bringup robot_bringup.launch.py profile:=field
  ```

* /cmd_vel → Sport Mode client (vx, vy, yaw, gait).
* /sportmodestate → bridged into /odom, /imu, /health.
* Optional WebRTC bridge publishes /camera/image_raw, /mic/audio.

## 📊 Topics & Services

**Commands**
* /cmd_vel (geometry_msgs/Twist)

**State**
* /odom, /imu, /tf, /health

**Sensors**
* /points (sensor_msgs/PointCloud2)
* /camera/image_raw (sensor_msgs/Image)
* /camera/camera_info

**Services**
* /robot/stand
* /robot/sit
* /robot/estop

## 🧪 Testing Ladder

1.	Sim smoke test: Spawn robot, teleop /cmd_vel, check sensors in RViz.
2.	Nav2 sim: Run NavigateThroughPoses patrol on a map.
3.	Dry-run robot (motors off): Verify /sportmodestate, TFs, topics.
4.	Field test: Low-speed patrol route with watchdog enabled.
5.	Regression: Replay bags in CI to validate perception & Nav2 decisions.

## 📜 License

This project is licensed under the **GNU Affero General Public License v3 (AGPL-3.0)**.  
See the [LICENSE](LICENSE) file for details.

## 🙌 Acknowledgments

* Unitree Robotics for Go2 and SDK2.
* ROS 2 & Nav2 community.
* Contributors to simulation packages (Gazebo/Isaac).
