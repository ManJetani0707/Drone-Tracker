# Windows WSL2 Setup Guide — DroneTrackAI (Team NOS)

Complete guide to run the simulation on Windows.

---

## Step 1 — Install WSL2

Open **PowerShell as Administrator**:
```powershell
wsl --install
```
Restart your PC. After restart, Ubuntu opens — create username/password.

---

## Step 2 — Install ROS 2 Humble

In the Ubuntu terminal:
```bash
sudo apt update
sudo apt install -y software-properties-common curl

sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
  -o /usr/share/keyrings/ros-archive-keyring.gpg

echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] \
http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" \
| sudo tee /etc/apt/sources.list.d/ros2.list

sudo apt update
sudo apt install -y ros-humble-ros-base ros-humble-cv-bridge \
    ros-humble-tf2-ros python3-colcon-common-extensions
pip3 install numpy opencv-python

echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

---

## Step 3 — Clone and Build

```bash
git clone https://github.com/team-nos/drone-tracker.git
cd drone-tracker
colcon build --symlink-install --packages-select drone_tracker
source install/setup.bash
```

---

## Step 4 — Run

```bash
bash run.sh
```

---

## Troubleshooting

| Error | Fix |
|---|---|
| `ros2: command not found` | `source /opt/ros/humble/setup.bash` |
| `unknown package drone_tracker` | `colcon build` again from repo root |
| `apt update` fails (mirror error) | `sudo apt update --fix-missing` |
| Build: missing `target_spawner_node.py` | `touch src/drone_tracker/drone_tracker/target_spawner_node.py` |
| WSL install fails | Enable Virtualization in BIOS |
