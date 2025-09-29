# Unitree Go2 Autonomous Security Robot

## Project Overview
This project enhances the Unitree Go2 quadruped robot to function as an autonomous area security and monitoring platform. Using ROS 2 Jazzy, Nav2, and onboard sensors, the robot can patrol designated areas, monitor for activity, handle charging autonomously, and be controlled via a web-based dashboard.

**Key Capabilities:**
- Autonomous patrol route execution
- Obstacle-aware navigation
- Live video streaming
- Self-managed charging
- Geofence enforcement and alerts
- Deterrence features (audio and lights)

## Features

### 1. Patrol Route Control
- **Waypoint Definition:** Operators can define patrol paths through an interactive map in the web dashboard.
- **Teach & Replay Mode:** Drive the robot manually along a route; system records the path for later autonomous repetition.
- **Autonomous Patrol Execution:** Robot follows saved routes, with support for pause, resume, and stop commands from the dashboard.
- **Dynamic Obstacle Handling:** Minor obstacles are avoided automatically; major obstructions are reported and rerouted if needed.

### 2. Automatic Charging
- **Battery Monitoring:** Robot continuously checks battery status and initiates charging when low.
- **Docking Navigation:** Navigates to a charging station with precise final alignment using fiducial markers (e.g., AprilTags).
- **Charging Feedback:** Confirms successful charging and resumes patrols or waits for operator commands after full charge.

### 3. Monitoring & Video Dashboard
- **Web Dashboard:**
  - Displays robot location, patrol paths, and geofence zones.
  - Shows robot diagnostics (battery, velocity, system health).
  - Provides controls for starting/stopping patrols, returning to dock, and triggering deterrence.
- **Live Video Feed:** Secure, low-latency stream from the front-facing camera.
- **Object Detection:** Identifies people in real-time using a pre-trained YOLO model, with bounding boxes displayed on the dashboard.

### 4. Deterrence Actions
- **Audio Alerts:** Play preloaded sounds (dog bark, siren) or relay operator voice via the robot’s speaker.
- **Lighting Controls:** Toggle built-in headlight remotely through the dashboard.

### 5. Geofence Enforcement
- **Boundary Setup:** Operators can draw polygonal safe zones on the dashboard.
- **Alert Mechanism:** Robot stops motion and sends an immediate alert if it exits the defined area.

## Technical Stack

| Layer | Technology |
|-------|------------|
| Middleware | ROS 2 Jazzy |
| Robot | Unitree Go2 EDU |
| Navigation & Mapping | Nav2, SLAM Toolbox |
| Computer Vision | OpenCV, PyTorch (YOLO) |
| Web Frontend | React / Vue |
| Web Backend | FastAPI / Node.js |
| ROS-Web Bridge | ros2-web-bridge / WebSockets |
| Docking Precision | AprilTags |

## System Architecture

```
[patrol_controller] –> /NavigateToPose –> [Nav2]
|
v
[geofence_monitor] –> /cmd_vel stop on breach
|
v
[battery_monitor] –> triggers charging routine
|
v
[docking_manager] –> Nav2 + AprilTag docking
|
v
[vision_processor] –> /detections/person –> web dashboard
|
v
[audio_controller, light_controller] –> dashboard commands
```

---

## Development Roadmap

## Development Roadmap

| Phase | Tasks | Duration | Completed |
|-------|-------|----------|-----------|
| **Phase 1** | Environment Setup: ROS 2, Gazebo simulation, robot model, SLAM configuration | 1 week | [ ] |
| **Phase 2** | Patrol & Navigation: Patrol node, waypoint management, Nav2 integration, obstacle handling | 2 weeks | [ ] |
| **Phase 3** | Autonomous Charging: Battery monitoring, docking, fiducial alignment | 1 week | [ ] |
| **Phase 4** | Video Monitoring & Dashboard: Camera streaming, web interface, object detection | 2 weeks | [ ] |
| **Phase 5** | Deterrence & Geofence: Audio/light control, boundary enforcement | 1 week | [ ] |
| **Phase 6** | Integration & Testing: End-to-end tests, debugging, demo preparation | 1–2 weeks | [ ] |
| **Phase 7** | Documentation & Deployment: Setup guides, API docs, deployment scripts | 1 week | [ ] |

**Total Duration Estimate:** ~8–10 weeks

## Getting Started
1. **Clone the Repository**
```bash
git clone https://github.com/Kodo-Robotics/go2-autonomous-patrol
cd go2-autonomous-patrol
```
2. **Install Dependencies**
```bash
rosdep install --from-paths src --ignore-src -y
colcon build
source install/setup.bash
```

3. **Launch Simulation**
```bash
ros2 launch launch/sim_launch.py
```

4. **Start Mapping & Patrol**
```bash
ros2 launch launch/mapping_launch.py
ros2 launch launch/nav_launch.py
```

5. **Access Web Dashboard**
   
Open browser at http://127.0.0.1:6080 to monitor, define routes, and control the robot.

## Demo Highlights
- Continuous autonomous patrols
- Real-time video feed and person detection
- Self-managed charging
- Geofence alerts
- Audio and lighting deterrence

## License & Contributions
- Apache 2.0 License
- Contributions welcome for improved UI, additional sensors, or new features.
