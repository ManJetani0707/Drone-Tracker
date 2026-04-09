# Hardware Deployment Guide — DroneTrackAI (Team NOS)

Two code changes to go from simulation to real hardware.

---

## Change 1 — Real YOLOv8 Detection

Install:
```bash
pip3 install ultralytics
```

Replace `_detect()` in `src/drone_tracker/drone_tracker/detection_node.py`:

```python
from ultralytics import YOLO

self.model = YOLO('yolov8n.pt')

def _detect(self):
    frame = self.latest_frame  # from /camera/image_raw via cv_bridge
    results = self.model(frame, classes=[0], verbose=False)  # 0 = person
    for box in results[0].boxes:
        cx   = float(box.xywhn[0][0])
        cy   = float(box.xywhn[0][1])
        area = float(box.xywhn[0][2] * box.xywhn[0][3])
        self.bbox_pub.publish(Point(x=cx, y=cy, z=area))
```

Also subscribe to camera topic:
```python
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
self.bridge = CvBridge()
self.create_subscription(Image, '/camera/image_raw', self._img_cb, 10)

def _img_cb(self, msg):
    self.latest_frame = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
```

---

## Change 2 — MAVROS Flight Control

Install:
```bash
sudo apt install ros-humble-mavros ros-humble-mavros-extras
```

Replace publisher in `src/drone_tracker/drone_tracker/flight_bridge_node.py`:

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

> ⚠️ **Important**: PX4/ArduPilot must be in **OFFBOARD mode** before publishing setpoints.

---

## Supported Hardware

| Platform | Autopilot | Notes |
|---|---|---|
| Any quadrotor | PX4 | Set OFFBOARD mode, tune hover throttle |
| Any quadrotor | ArduPilot | Use GUIDED mode equivalent |
| DJI (hacked) | OSDK | Requires DJI ROS wrapper |

---

## PID Re-tuning for Real Hardware

Real hardware behaves differently from simulation. Start with very conservative gains:

```bash
ros2 param set /pid_controller yaw.kp 0.3   # much lower than sim
ros2 param set /pid_controller alt.kp 0.2
ros2 param set /pid_controller fwd.kp 0.4
```

Then tune upward slowly. See `docs/pid_tuning_guide.md` for the full procedure.
