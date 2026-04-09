#!/bin/bash
set -e
echo "=== DroneTrackAI Team NOS — Environment Setup ==="
apt-get update -qq
apt-get install -y ros-humble-cv-bridge ros-humble-tf2-ros \
    ros-humble-launch-ros python3-colcon-common-extensions python3-pip --no-install-recommends
pip3 install --quiet numpy opencv-python
grep -q "source /opt/ros/humble/setup.bash" ~/.bashrc || \
    echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
source /opt/ros/humble/setup.bash
cd /workspaces/drone-tracker 2>/dev/null || true
colcon build --symlink-install --packages-select drone_tracker
grep -q "install/setup.bash" ~/.bashrc || \
    echo "source $(pwd)/install/setup.bash" >> ~/.bashrc
echo "=== ✅ Done! Run: bash run.sh ==="
