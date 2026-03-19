# Pepper Robot Simulation for ROS 2 Humble & Gazebo Classic

[![ROS 2 Humble](https://img.shields.io/badge/ROS%202-Humble-blue)](https://docs.ros.org/en/humble/)
[![Gazebo Classic 11](https://img.shields.io/badge/Gazebo-Classic%2011-orange)](http://gazebosim.org/)
[![Docker](https://img.shields.io/badge/Docker-Ready-blue)](https://www.docker.com/)
[![Platform](https://img.shields.io/badge/Platform-x86__64%20%7C%20Apple%20Silicon-lightgrey)]()

A complete, ready-to-use **ROS 2 Humble** simulation for the **SoftBank Pepper 1.0 humanoid robot** in Gazebo Classic 11. Includes full joint control (head, pelvis, arms), Nav2 autonomous navigation, realistic sensors, and browser-based GUI via noVNC — all containerized with Docker.

**No native ROS installation required.** Just Docker and a browser.

> Ported from ROS 1 Noetic to ROS 2 Humble (Ubuntu 22.04). All packages migrated to ament/colcon.

---

## Why This Package?

**This is the first and only open-source project that runs the SoftBank Pepper robot in Gazebo with ROS 2 Humble.**

As of 2026, no other project provides a working Pepper simulation in Gazebo for ROS 2. The official [ros-naoqi](https://github.com/ros-naoqi) organization has never ported their Gazebo simulation (`pepper_virtual`) or URDF package (`pepper_robot`) to ROS 2 — their ROS 2 support is limited to the real robot driver ([naoqi_driver2](https://github.com/ros-naoqi/naoqi_driver2)) and mesh files ([pepper_meshes2](https://github.com/ros-naoqi/pepper_meshes2)). This project fills that gap with a complete, ready-to-use simulation stack:

- Full URDF model with accurate meshes, sensors, and joint limits
- Two robot variants: **with arms** (15 controllable joints + visual fingers) or **without arms** (5 joints) — switchable at launch time
- Physics-based omnidirectional velocity control calibrated to real Pepper odometry
- Nav2 integration with SLAM Toolbox and AMCL out of the box
- 5 trajectory controllers: head, pelvis, left arm, right arm + joint state broadcaster
- 5 pre-built Gazebo worlds
- Works on **macOS (Apple Silicon & Intel)** and **Linux** via Docker
- **No native ROS installation required** — just Docker and a browser

---

## Quick Start

### Prerequisites

- [Docker Desktop](https://www.docker.com/products/docker-desktop/) (with BuildKit enabled)
- A modern browser (Chrome, Firefox, Safari)

### 1. Clone & launch

```bash
git clone https://github.com/tuncismail/pepper-robot-ros2-gazebo-simulation.git
cd pepper-robot-ros2-gazebo-simulation

# Launch with browser GUI (no arms, default)
docker-compose up pepper_gui
```

### 2. Open in browser

```
http://localhost:6080/vnc.html?autoconnect=true&resize=scale
```

Gazebo will load with Pepper in an empty world. First build takes ~10-15 min.

### 3. Launch with arms

```bash
# Inside the container shell, or modify docker-compose command:
ros2 launch pepper_gazebo_plugin pepper_gazebo_plugin_Y20_CPU_no_arms.launch.py arms:=true gui:=true rviz:=true
```

### Headless mode (no display)

```bash
docker-compose up pepper_headless
```

### Interactive shell

```bash
docker-compose run --rm pepper_shell
```

---

## Robot Variants

All launch files accept an `arms` argument to switch between two robot configurations at launch time:

| Variant | Launch argument | Joints | Controllers | URDF |
|---------|----------------|--------|-------------|------|
| **No arms** (default) | `arms:=false` | 5 (head + pelvis) | head, pelvis | `pepper_robot_CPU_no_arms.xacro` |
| **With arms** | `arms:=true` | 15 (head + pelvis + arms) | head, pelvis, left_arm, right_arm | `pepper_robot_CPU.xacro` |

The arms variant also includes **visual finger meshes** for rendering (fingers are not actuated).

---

## Features

### Sensors
- **RGB cameras** — front top and bottom cameras (`/camera_top/image_raw`, `/camera_bottom/image_raw`)
- **Depth camera** — ASUS Xtion-style depth sensor (`/camera_depth/depth/image_raw`, `/camera_depth/depth/points`)
- **Laser scanners** — 3 infrared laser sensors (front, left, right) merged into `/pepper/scan_merged`
- **Fake Hokuyo** — 180-degree laser for navigation (`/pepper/hokuyo_scan`)
- **Sonar** — front and back ultrasonic range sensors

### Joint Control (ros2_control)
- **Head** — HeadYaw, HeadPitch (trajectory controller)
- **Pelvis** — HipRoll, HipPitch, KneePitch (trajectory controller)
- **Left arm** — LShoulderPitch, LShoulderRoll, LElbowYaw, LElbowRoll, LWristYaw (trajectory controller, `arms:=true`)
- **Right arm** — RShoulderPitch, RShoulderRoll, RElbowYaw, RElbowRoll, RWristYaw (trajectory controller, `arms:=true`)

### Base Motion
- **Omnidirectional velocity control** — C++ Gazebo plugin reads physics state and applies `cmd_vel` commands with acceleration/jerk limits
- **Realistic odometry** — Gaussian noise calibrated to real Pepper (sigma_xy=0.02m, sigma_yaw=0.02645rad)
- **Ground truth odometry** — noise-free odometry on a separate topic for evaluation

### Navigation (Nav2)
- **SLAM Toolbox** — online SLAM for map building while navigating
- **AMCL** — localization against a pre-built map
- **DWB local planner** — dynamic window approach for local path planning
- **Pre-configured** — `nav2_params.yaml` with tuned parameters for Pepper's kinematics

### Infrastructure
- **Browser-based GUI** — Gazebo renders inside a noVNC window. No XQuartz or X11 forwarding needed.
- **RViz2 support** — launch with `rviz:=true` to visualize sensors, TF tree, and navigation data
- **Apple Silicon compatible** — runs on arm64 Macs via Docker x86 emulation
- **gazebo_ros2_control patch** — fixes URDF-in-argv crash for large robot descriptions

---

## Launch Files

All launch files are in `pepper_virtual/pepper_gazebo_plugin/launch/`.

### Simulation worlds

| Launch file | World | Example |
|------------|-------|---------|
| `pepper_gazebo_plugin_Y20_CPU_no_arms.launch.py` | Empty arena | `ros2 launch pepper_gazebo_plugin pepper_gazebo_plugin_Y20_CPU_no_arms.launch.py` |
| `pepper_gazebo_plugin_house1.launch.py` | House | `ros2 launch pepper_gazebo_plugin pepper_gazebo_plugin_house1.launch.py` |
| `pepper_gazebo_plugin_small_office.launch.py` | Small office | `ros2 launch pepper_gazebo_plugin pepper_gazebo_plugin_small_office.launch.py` |
| `pepper_gazebo_plugin_simple_office_with_people.launch.py` | Office with people | `ros2 launch pepper_gazebo_plugin pepper_gazebo_plugin_simple_office_with_people.launch.py` |
| `pepper_gazebo_plugin_nao_test.launch.py` | NAO test arena | `ros2 launch pepper_gazebo_plugin pepper_gazebo_plugin_nao_test.launch.py` |

### Launch arguments (common to all)

| Argument | Default | Description |
|----------|---------|-------------|
| `gui` | `false` | Show Gazebo gzclient window |
| `rviz` | `false` | Launch RViz2 visualization |
| `arms` | `false` | Include arms and arm controllers |

Example with all options:

```bash
ros2 launch pepper_gazebo_plugin pepper_gazebo_plugin_Y20_CPU_no_arms.launch.py \
  gui:=true rviz:=true arms:=true
```

### Navigation

```bash
# SLAM mode (default) — build a map while navigating
ros2 launch pepper_gazebo_plugin pepper_navigation.launch.py

# AMCL mode — localize against an existing map
ros2 launch pepper_gazebo_plugin pepper_navigation.launch.py \
  use_slam:=false map:=/path/to/map.yaml

# Choose a different world
ros2 launch pepper_gazebo_plugin pepper_navigation.launch.py world:=house1.world

# With arms and GUI
ros2 launch pepper_gazebo_plugin pepper_navigation.launch.py \
  gui:=true rviz:=true arms:=true
```

---

## Control Commands

### Base movement

```bash
# Move forward at 0.3 m/s
ros2 topic pub --once /pepper/cmd_vel geometry_msgs/msg/Twist \
  '{"linear": {"x": 0.3}, "angular": {"z": 0.0}}'

# Rotate in place at 0.5 rad/s
ros2 topic pub --once /pepper/cmd_vel geometry_msgs/msg/Twist \
  '{"linear": {"x": 0.0}, "angular": {"z": 0.5}}'

# Strafe left at 0.2 m/s
ros2 topic pub --once /pepper/cmd_vel geometry_msgs/msg/Twist \
  '{"linear": {"x": 0.0, "y": 0.2}, "angular": {"z": 0.0}}'

# Stop
ros2 topic pub --once /pepper/cmd_vel geometry_msgs/msg/Twist \
  '{"linear": {"x": 0.0}, "angular": {"z": 0.0}}'
```

### Head control

```bash
# Turn head left (yaw=0.5) and tilt down (pitch=0.2)
ros2 action send_goal /head_controller/follow_joint_trajectory \
  control_msgs/action/FollowJointTrajectory \
  '{trajectory: {joint_names: ["HeadYaw", "HeadPitch"],
    points: [{positions: [0.5, 0.2], time_from_start: {sec: 2}}]}}'
```

### Pelvis control

```bash
ros2 action send_goal /pelvis_controller/follow_joint_trajectory \
  control_msgs/action/FollowJointTrajectory \
  '{trajectory: {joint_names: ["HipRoll", "HipPitch", "KneePitch"],
    points: [{positions: [0.1, -0.1, 0.1], time_from_start: {sec: 2}}]}}'
```

### Arm control (requires `arms:=true`)

```bash
# Move left arm
ros2 action send_goal /left_arm_controller/follow_joint_trajectory \
  control_msgs/action/FollowJointTrajectory \
  '{trajectory: {joint_names: ["LShoulderPitch", "LShoulderRoll", "LElbowYaw", "LElbowRoll", "LWristYaw"],
    points: [{positions: [0.5, 0.5, -0.5, -1.0, 0.0], time_from_start: {sec: 2}}]}}'

# Move right arm
ros2 action send_goal /right_arm_controller/follow_joint_trajectory \
  control_msgs/action/FollowJointTrajectory \
  '{trajectory: {joint_names: ["RShoulderPitch", "RShoulderRoll", "RElbowYaw", "RElbowRoll", "RWristYaw"],
    points: [{positions: [0.5, -0.5, 0.5, 1.0, 0.0], time_from_start: {sec: 2}}]}}'
```

---

## ROS 2 Topics

### Sensor topics

| Topic | Type | Description |
|-------|------|-------------|
| `/camera_top/image_raw` | `sensor_msgs/Image` | Front RGB camera |
| `/camera_bottom/image_raw` | `sensor_msgs/Image` | Bottom RGB camera |
| `/camera_depth/depth/image_raw` | `sensor_msgs/Image` | Depth camera |
| `/camera_depth/depth/points` | `sensor_msgs/PointCloud2` | Depth point cloud |
| `/pepper/scan_front` | `sensor_msgs/LaserScan` | Front laser scan |
| `/pepper/scan_left` | `sensor_msgs/LaserScan` | Left laser scan |
| `/pepper/scan_right` | `sensor_msgs/LaserScan` | Right laser scan |
| `/pepper/scan_merged` | `sensor_msgs/LaserScan` | Merged 360-degree laser |
| `/pepper/hokuyo_scan` | `sensor_msgs/LaserScan` | Fake Hokuyo 180-degree scan |
| `/pepper/sonar_front` | `sensor_msgs/Range` | Front sonar |
| `/pepper/sonar_back` | `sensor_msgs/Range` | Back sonar |

### Control & state topics

| Topic | Type | Description |
|-------|------|-------------|
| `/pepper/cmd_vel` | `geometry_msgs/Twist` | Base velocity command |
| `/pepper/odom` | `nav_msgs/Odometry` | Odometry with noise |
| `/pepper/odom_groundtruth` | `nav_msgs/Odometry` | Ground truth odometry |
| `/joint_states` | `sensor_msgs/JointState` | All joint positions & velocities |
| `/tf` | `tf2_msgs/TFMessage` | TF transforms |
| `/robot_description` | `std_msgs/String` | URDF robot description |

### Navigation topics (with `pepper_navigation.launch.py`)

| Topic | Type | Description |
|-------|------|-------------|
| `/goal_pose` | `geometry_msgs/PoseStamped` | Navigation goal |
| `/map` | `nav_msgs/OccupancyGrid` | SLAM / map server output |
| `/local_costmap/costmap` | `nav_msgs/OccupancyGrid` | Local costmap |
| `/global_costmap/costmap` | `nav_msgs/OccupancyGrid` | Global costmap |

### Controller action servers

| Action server | Type | Available |
|---------------|------|-----------|
| `/head_controller/follow_joint_trajectory` | `FollowJointTrajectory` | Always |
| `/pelvis_controller/follow_joint_trajectory` | `FollowJointTrajectory` | Always |
| `/left_arm_controller/follow_joint_trajectory` | `FollowJointTrajectory` | `arms:=true` |
| `/right_arm_controller/follow_joint_trajectory` | `FollowJointTrajectory` | `arms:=true` |

---

## Architecture

```
pepper-robot-ros2-gazebo-simulation/
├── Dockerfile                    # ROS 2 Humble + Gazebo 11 image (amd64)
├── docker-compose.yml            # GUI / headless / shell services
├── docker-entrypoint.sh          # Xvnc -> openbox -> noVNC -> ros2 launch
├── test_e2e_headless.sh          # End-to-end test (headless, 9 checks)
├── patches/                      # Source patches (gazebo_ros2_control fix)
│
├── pepper_robot/                 # Pepper URDF description & metapackage
│   └── pepper_description/
│       └── urdf/
│           └── pepper1.0_generated_urdf/
│               ├── pepper_robot_CPU.xacro          # Full robot (with arms + fingers)
│               ├── pepper_robot_CPU_no_arms.xacro  # No arms variant
│               ├── pepperGazeboCPU.xacro           # Gazebo config (with arms)
│               └── pepperGazeboCPU_no_arms.xacro   # Gazebo config (no arms)
│
├── pepper_virtual/               # Gazebo plugin, controllers, launch files
│   ├── pepper_gazebo_plugin/
│   │   ├── launch/               # 6 launch files (all support arms:= arg)
│   │   ├── worlds/               # Gazebo world files
│   │   ├── config/               # Nav2 params, RViz2 config
│   │   └── scripts/              # laser_publisher, odom_publisher
│   └── pepper_control/
│       └── config/
│           ├── pepper_ros2_controllers.yaml            # No-arms controllers
│           └── pepper_ros2_controllers_with_arms.yaml  # With-arms controllers
│
├── pepper_meshes/                # 3D mesh files (CC-BY-NC-ND 4.0)
├── gazebo_model_velocity_plugin/ # Base velocity Gazebo plugin (C++)
├── pepper_laser_bridge/          # Laser data bridge (sim <-> real)
└── velocity_bridge/              # cmd_vel bridge
```

### GUI pipeline (inside container)

```
Xvnc :1 (port 5901)
  ├── openbox (window manager)
  └── ros2 launch Gazebo + RViz2 (DISPLAY=:1)
websockify -> noVNC (port 6080)  <-  browser
```

---

## Docker Reference

### Build manually

```bash
docker build --platform linux/amd64 -t pepper_with_gazebo:humble .
```

### Run manually

```bash
# GUI mode (browser)
docker run -d --rm --init \
  --name pepper_gui \
  -p 6080:6080 -p 5901:5901 \
  --platform linux/amd64 \
  pepper_with_gazebo:humble gui

# Then open: http://localhost:6080/vnc.html?autoconnect=true&resize=scale

# Headless
docker run --rm --init --platform linux/amd64 \
  pepper_with_gazebo:humble headless

# Interactive shell
docker run -it --rm --init --platform linux/amd64 \
  pepper_with_gazebo:humble bash
```

### Docker Compose services

| Service | Description | URL |
|---------|-------------|-----|
| `pepper_gui` | Full GUI via browser (noVNC) | http://localhost:6080/vnc.html |
| `pepper_headless` | Headless Gazebo (no display) | -- |
| `pepper_shell` | Interactive bash shell | -- |

---

## Testing

An end-to-end headless test verifies 9 checks:

```bash
docker-compose run --rm pepper_shell bash /colcon_ws/src/test_e2e_headless.sh
```

The test verifies:
- Xvfb + Gazebo start successfully
- Pepper model spawns in Gazebo
- All laser scan topics active (`/pepper/scan_front`, `scan_left`, `scan_right`)
- `/joint_states` and `/tf` publishing
- `/pepper/odom` publishing
- `robot_description` parameter loaded
- `/pepper/cmd_vel` accepts velocity commands

---

## Known Limitations

- **x86 emulation on Apple Silicon** — Gazebo runs under Docker's x86 emulation, which is slower than native. Expect ~55s startup time.
- **VNC has no password** — for local/LAN use only. Do not expose port 5901 to the internet.
- **Finger joints are visual only** — finger meshes render in the arms variant but are not actuated or controllable.
- **gazebo_ros2_control patch required** — built from source with a patch because the apt package crashes on Pepper's large URDF. See `patches/` for details.

---

## Contributing

Contributions are welcome! If you're working with Pepper in ROS 2, please open issues or pull requests. Areas where help is needed:

- Adding MoveIt 2 support for arm motion planning
- Testing on different platforms and environments
- Adding more Gazebo worlds

---

## Credits

This project builds on and extends the following upstream work:

- [ros-naoqi/pepper_virtual](https://github.com/ros-naoqi/pepper_virtual) — original Pepper Gazebo simulation (ROS 1)
- [ros-naoqi/pepper_robot](https://github.com/ros-naoqi/pepper_robot) — Pepper URDF description
- [awesomebytes/gazebo_model_velocity_plugin](https://github.com/awesomebytes/gazebo_model_velocity_plugin) — base velocity plugin
- [ros-controls/gazebo_ros2_control](https://github.com/ros-controls/gazebo_ros2_control) — ros2_control Gazebo integration

---

## License

- Project code: see [LICENSE](LICENSE)
- Pepper meshes (`pepper_meshes/`): [CC-BY-NC-ND 4.0](pepper_meshes/debian_license/) — Aldebaran Robotics / SoftBank Robotics
