#!/bin/bash
# ==========================================
# DroneTrackAI — Team NOS
# One command to run the full simulation
# ==========================================
set -e

GREEN='\033[0;32m'; CYAN='\033[0;36m'; RED='\033[0;31m'; YELLOW='\033[1;33m'; NC='\033[0m'

echo -e "${CYAN}"
echo "  ██████╗ ██████╗  ██████╗ ███╗   ██╗███████╗"
echo "  ██╔══██╗██╔══██╗██╔═══██╗████╗  ██║██╔════╝"
echo "  ██║  ██║██████╔╝██║   ██║██╔██╗ ██║█████╗  "
echo "  ██║  ██║██╔══██╗██║   ██║██║╚██╗██║██╔══╝  "
echo "  ██████╔╝██║  ██║╚██████╔╝██║ ╚████║███████╗"
echo "  ╚═════╝ ╚═╝  ╚═╝ ╚═════╝ ╚═╝  ╚═══╝╚══════╝"
echo -e "${NC}"
echo -e "${YELLOW}  Team NOS — Real-Time Object Tracking Drone${NC}"
echo -e "${YELLOW}  36-Hour Hackathon | Robotics & AI | April 2026${NC}"
echo ""

# Check ROS
if [ ! -f "/opt/ros/humble/setup.bash" ]; then
  echo -e "${RED}ROS 2 Humble not found. Run: bash .devcontainer/setup.sh${NC}"
  exit 1
fi
source /opt/ros/humble/setup.bash

# Find workspace
WS="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
if [ ! -f "$WS/install/setup.bash" ]; then
  echo -e "${YELLOW}Building workspace...${NC}"
  cd "$WS"
  colcon build --symlink-install --packages-select drone_tracker
fi
source "$WS/install/setup.bash"

echo -e "${GREEN}✅ ROS 2 Humble ready${NC}"
echo -e "${GREEN}✅ drone_tracker package loaded${NC}"
echo ""
echo -e "${CYAN}Active nodes:${NC}"
echo "  🛰  detection_node     — YOLOv8-sim detector (30 Hz)"
echo "  📡  tracker_node       — Kalman filter state estimator"
echo "  ⚙️   pid_controller     — Triple-axis PID (yaw/alt/fwd)"
echo "  🚁  flight_bridge      — Physics + Gazebo bridge"
echo "  📊  visualizer_node    — RViz markers + HUD"
echo ""
echo -e "${CYAN}Monitor in a 2nd terminal:${NC}"
echo "  ros2 topic echo /track/state"
echo "  ros2 topic echo /pid/errors"
echo "  ros2 run rqt_plot rqt_plot /pid/errors/x /pid/errors/y /pid/errors/z"
echo ""
echo -e "Press ${RED}Ctrl+C${NC} to stop."
echo ""

ros2 launch drone_tracker simulation.launch.py use_gazebo:=false rviz:=false log_level:=info
