# 🚁 DroneTrackAI — Real-Time Object Tracking Drone

<div align="center">

[![ROS 2 Humble](https://img.shields.io/badge/ROS2-Humble-blue?logo=ros&logoColor=white)](https://docs.ros.org/en/humble/)
[![Python 3.10+](https://img.shields.io/badge/Python-3.10+-green?logo=python&logoColor=white)](https://www.python.org/)
[![License: Apache-2.0](https://img.shields.io/badge/License-Apache_2.0-yellow.svg)](LICENSE)

[![WSL2 Compatible](https://img.shields.io/badge/Windows-WSL2%20Compatible-0078D6?logo=windows)](https://docs.microsoft.com/en-us/windows/wsl/)
[![Simulation](https://img.shields.io/badge/Mode-Simulation%20%2B%20Hardware--Ready-orange)](README.md)

**36-Hour Hackathon | Robotics & AI Track | April 2026**

*A complete ROS 2 simulation of a quadrotor drone that autonomously follows a moving target using computer vision and PID control loops.*

**Team NOS** | ✅ Successfully Running on Windows WSL2

</div>

---

## 📽️ System Architecture

```
Camera/IMU  →  Detection Node  →  Tracker Node  →  PID Controller  →  Flight Bridge  →  Gazebo
  (sensor)      (YOLOv8 sim)      (Kalman KF)     (3× PID loops)     (physics sim)     (world)
                     ↑                                                      ↓
                /detection/bbox                                        /drone/pose
                /detections (RViz)                               (feedback loop)
```

---

## ✨ Features

| Feature | Details |
|---|---|
| 🎯 **Simulated YOLOv8 Detection** | Realistic noisy bbox at 30Hz, plug-in real YOLOv8 in 4 lines |
| 📡 **Kalman Filter Tracker** | 6-DOF constant-velocity model, handles 15 frames of occlusion |
| ⚙️ **Triple PID Control** | Independent yaw, altitude, forward loops with anti-windup |
| 🚁 **Full ROS 2 Pipeline** | 5 nodes, 9 topics, live monitoring via rqt_plot |
| 🔌 **Hardware-Ready** | Swap to MAVROS for real PX4/ArduPilot in 2 code changes |
| 🎛️ **Live Tuning** | All PID gains adjustable at runtime — no restart needed |
| 🪟 **Windows Compatible** | Fully tested on Windows 11 + WSL2 + Ubuntu 22.04 |

---

## 🗂️ Repository Structure

```
drone-tracker/                          ← GitHub repository root
│
├── src/
│   └── drone_tracker/                  ← ROS 2 package
│       ├── drone_tracker/
│       │   ├── detection_node.py       ← Simulated YOLOv8 detector (30 Hz)
│       │   ├── tracker_node.py         ← Kalman filter state estimator
│       │   ├── pid_controller_node.py  ← Triple-axis PID controller
│       │   ├── flight_bridge_node.py   ← Physics sim + Gazebo bridge
│       │   ├── visualizer_node.py      ← RViz markers + HUD overlay
│       │   └── target_spawner_node.py  ← Moving target simulation
│       ├── launch/
│       │   └── simulation.launch.py    ← Main launch file
│       ├── config/
│       │   ├── params.yaml             ← All tunable PID/sim parameters
│       │   └── tracking.rviz           ← RViz layout
│       ├── worlds/
│       │   └── tracking_arena.world    ← Gazebo world with walking actor
│       ├── CMakeLists.txt
│       └── package.xml
│
├── docs/
│   ├── pid_tuning_guide.md             ← Step-by-step PID tuning
│   ├── hardware_deployment.md          ← Real drone deployment guide
│   └── windows_setup.md               ← WSL2 setup guide
│
├── .github/
│   ├── workflows/ci.yml                ← GitHub Actions CI
│   └── ISSUE_TEMPLATE/bug_report.md
│
├── .devcontainer/
│   ├── devcontainer.json               ← Codespaces config
│   └── setup.sh                        ← Auto-setup script
│
├── .gitignore
├── LICENSE                             ← Apache 2.0
└── README.md
```

---

## 🚀 Quick Start

### ▶️ Option 1 — Windows (WSL2) — Recommended

```bash
# Step 1: Install WSL2 (PowerShell as Admin, then restart)
wsl --install

# Step 2: Open Ubuntu, install ROS 2
sudo apt update && sudo apt install -y \
    ros-humble-ros-base ros-humble-cv-bridge \
    ros-humble-tf2-ros python3-colcon-common-extensions
pip3 install numpy opencv-python

# Step 3: Clone and build
git clone https://github.com/team-nos/drone-tracker.git
cd drone-tracker
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-select drone_tracker
source install/setup.bash

# Step 4: Run!
ros2 launch drone_tracker simulation.launch.py use_gazebo:=false rviz:=false
```

### ▶️ Option 2 — Ubuntu 22.04 (Native)

```bash
# Install ROS 2 Humble (if not already installed)
sudo apt install -y ros-humble-desktop ros-humble-gazebo-ros-pkgs \
                    ros-humble-cv-bridge ros-humble-tf2-ros \
                    python3-colcon-common-extensions
pip3 install numpy opencv-python
source /opt/ros/humble/setup.bash

# Clone, build, run
git clone https://github.com/team-nos/drone-tracker.git
cd drone-tracker
colcon build --symlink-install --packages-select drone_tracker
source install/setup.bash
ros2 launch drone_tracker simulation.launch.py use_gazebo:=false rviz:=false
```

### ▶️ Option 3 — Full Gazebo Simulation

```bash
ros2 launch drone_tracker simulation.launch.py use_gazebo:=true rviz:=true
```

---

## 📡 ROS 2 Topics

| Topic | Type | Description |
|---|---|---|
| `/detection/bbox` | `geometry_msgs/Point` | Normalised bbox centroid (cx, cy, area) |
| `/detections` | `visualization_msgs/MarkerArray` | RViz detection markers |
| `/track/state` | `geometry_msgs/Point` | Smoothed track state (cx, cy, area) |
| `/track/velocity` | `geometry_msgs/Point` | Estimated target velocity |
| `/drone/cmd_vel` | `geometry_msgs/Twist` | Velocity setpoint commands |
| `/drone/pose` | `geometry_msgs/PoseStamped` | Current drone world pose |
| `/target/world_position` | `geometry_msgs/Point` | Ground truth target position |
| `/pid/errors` | `geometry_msgs/Point` | (e_yaw, e_alt, e_fwd) errors |
| `/viz/markers` | `visualization_msgs/MarkerArray` | All RViz visualisation markers |

---

## 📊 Monitor Live Data

Open a **second terminal** while simulation runs:

```bash
source /opt/ros/humble/setup.bash && source install/setup.bash

# Live tracking state (cx=0.5 cy=0.5 = centered | x=-1 = LOST)
ros2 topic echo /track/state

# PID errors (x=yaw, y=altitude, z=forward)
ros2 topic echo /pid/errors

# Drone position
ros2 topic echo /drone/pose

# All active topics
ros2 topic list

# Graphical plot
ros2 run rqt_plot rqt_plot /pid/errors/x /pid/errors/y /pid/errors/z
```

---

## 🎛️ Live PID Tuning

**No restart needed — tune while simulation is running:**

```bash
# STEP 1: Tune Yaw first (horizontal centering)
ros2 param set /pid_controller yaw.kp 0.8
ros2 param set /pid_controller yaw.kd 0.15

# STEP 2: Tune Altitude (vertical centering)
ros2 param set /pid_controller alt.kp 0.6
ros2 param set /pid_controller alt.ki 0.01

# STEP 3: Tune Forward (distance keeping)
ros2 param set /pid_controller fwd.kp 1.2
ros2 param set /pid_controller fwd.kd 0.20

# Adjust following distance (bigger = fly closer)
ros2 param set /pid_controller target_area 0.06
```

---

## 🔌 Swap to Real YOLOv8

Replace `_detect()` in `detection_node.py`:

```python
from ultralytics import YOLO
self.model = YOLO('yolov8n.pt')

def _detect(self):
    frame = self.latest_frame  # from /camera/image_raw via cv_bridge
    results = self.model(frame, classes=[0], verbose=False)  # class 0 = person
    for box in results[0].boxes:
        cx   = float(box.xywhn[0][0])
        cy   = float(box.xywhn[0][1])
        area = float(box.xywhn[0][2] * box.xywhn[0][3])
        self.bbox_pub.publish(Point(x=cx, y=cy, z=area))
```

---

## 🛸 Deploy on Real Hardware (MAVROS)

Replace publisher in `flight_bridge_node.py`:

```python
from mavros_msgs.msg import TwistStamped

self.mavros_pub = self.create_publisher(
    TwistStamped, '/mavros/setpoint_velocity/cmd_vel', 10)

def _relay(self, cmd: Twist):
    ts = TwistStamped()
    ts.header.stamp = self.get_clock().now().to_msg()
    ts.header.frame_id = 'base_link'
    ts.twist = cmd
    self.mavros_pub.publish(ts)
```
> Ensure PX4/ArduPilot is in **OFFBOARD mode** before publishing setpoints.

---

## 👥 Team NOS

| Name | Role |
|---|---|---|
| Niharika Pandey | Team Lead|
| Kanishka Kumari | Member | 
| Pratham Trivedi | Member |
| Bhavya Gohel| Member |
| Man Jetani | Member |
| Aaryan Solanki | Member |

---

## 🗺️ Roadmap

| Phase | Timeline | Features |
|---|---|---|
| **Phase 2** | 1–3 months | Real YOLOv8, Multi-target (DeepSORT), bag analysis |
| **Phase 3** | 3–6 months | Real PX4 hardware, GPS fusion, deep learning re-ID |
| **Phase 4** | 6–12 months | Drone swarm, Jetson Nano edge inference |
| **Vision** | 12–24 months | Autonomous deployment for disaster response & security |

---

## 📄 License

Apache-2.0 — see [LICENSE](LICENSE)

---

<div align="center">
<strong>Team NOS</strong> — 36-Hour Hackathon | Robotics & AI | April 2026<br/>
<em>🚁 Building autonomous systems for the physical world — one ROS node at a time.</em>
</div>
