#!/usr/bin/env python3
"""
detection_node.py
-----------------
Simulates a YOLOv8-style object detector in Gazebo simulation.

In a real deployment this node would:
  1. Subscribe to /camera/image_raw
  2. Run inference via ultralytics YOLOv8 or ONNX Runtime
  3. Publish bounding boxes

In simulation we use Gazebo model state (ground truth) to synthesize
realistic bounding boxes with configurable noise, mimicking real detector
output without needing a GPU or actual video feed.

Topics published:
  /detections  (visualization_msgs/MarkerArray) - for RViz
  /detection/bbox  (geometry_msgs/Point) - normalised (cx, cy, area) in [0,1]

Topics subscribed:
  /gazebo/model_states  (gazebo_msgs/ModelStates)
  /drone/pose           (geometry_msgs/PoseStamped)
"""

import math
import random

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point, PoseStamped
from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import Header, ColorRGBA


# Camera intrinsics (matches Gazebo camera plugin settings)
CAM_FOV_H = math.radians(90.0)   # horizontal FOV
CAM_FOV_V = math.radians(60.0)   # vertical FOV
IMG_W = 640
IMG_H = 480


class DetectionNode(Node):
    def __init__(self):
        super().__init__('detection_node')

        # Parameters
        self.declare_parameter('target_model', 'walking_actor')
        self.declare_parameter('detection_noise_px', 5.0)   # std dev in pixels
        self.declare_parameter('false_negative_rate', 0.05) # 5% miss rate
        self.declare_parameter('detection_hz', 30.0)

        self.target_model = self.get_parameter('target_model').value
        self.noise_std    = self.get_parameter('detection_noise_px').value
        self.fn_rate      = self.get_parameter('false_negative_rate').value

        # State
        self.drone_pose   = None
        self.target_pos   = None  # 3D world position

        # Subscriptions
        self.create_subscription(
            PoseStamped, '/drone/pose', self._drone_pose_cb, 10)
        self.create_subscription(
            Point, '/target/world_position', self._target_pos_cb, 10)

        # Publishers
        self.bbox_pub    = self.create_publisher(Point, '/detection/bbox', 10)
        self.marker_pub  = self.create_publisher(MarkerArray, '/detections', 10)

        hz = self.get_parameter('detection_hz').value
        self.create_timer(1.0 / hz, self._detect)

        self.get_logger().info('Detection node ready (simulated YOLOv8)')

    # ------------------------------------------------------------------ #
    def _drone_pose_cb(self, msg: PoseStamped):
        self.drone_pose = msg

    def _target_pos_cb(self, msg: Point):
        self.target_pos = msg

    # ------------------------------------------------------------------ #
    def _detect(self):
        if self.drone_pose is None or self.target_pos is None:
            return

        # Simulate false-negative (missed detection)
        if random.random() < self.fn_rate:
            return

        # ── Project target world position into image coordinates ──
        dx = self.target_pos.x - self.drone_pose.pose.position.x
        dy = self.target_pos.y - self.drone_pose.pose.position.y
        dz = self.target_pos.z - self.drone_pose.pose.position.z

        dist = math.sqrt(dx**2 + dy**2 + dz**2)
        if dist < 0.1:
            return

        # Yaw of drone (simplified: assume drone faces +X, extract yaw)
        q = self.drone_pose.pose.orientation
        yaw = math.atan2(
            2.0 * (q.w * q.z + q.x * q.y),
            1.0 - 2.0 * (q.y**2 + q.z**2)
        )

        # Rotate dx,dy into camera frame
        cam_x =  math.cos(yaw) * dx + math.sin(yaw) * dy
        cam_y = -math.sin(yaw) * dx + math.cos(yaw) * dy
        cam_z = dz

        # Behind camera
        if cam_x < 0.5:
            return

        # Normalised image plane coords (-1..1)
        img_u = cam_y / (cam_x * math.tan(CAM_FOV_H / 2.0))
        img_v = cam_z / (cam_x * math.tan(CAM_FOV_V / 2.0))

        # Out of FOV
        if abs(img_u) > 1.0 or abs(img_v) > 1.0:
            return

        # Convert to pixel coords (0..1 normalised, origin top-left)
        cx_norm = 0.5 - img_u * 0.5
        cy_norm = 0.5 - img_v * 0.5

        # Add Gaussian noise (normalised)
        cx_norm += random.gauss(0, self.noise_std / IMG_W)
        cy_norm += random.gauss(0, self.noise_std / IMG_H)

        # Apparent area shrinks with distance (person ~1.8m tall, 0.5m wide)
        apparent_h = (1.8 / dist) / math.tan(CAM_FOV_V / 2.0) * 0.5
        apparent_area = max(0.001, min(1.0, apparent_h * apparent_h * 0.55))

        # Publish bbox as (cx_norm, cy_norm, area_norm)
        bbox = Point(x=cx_norm, y=cy_norm, z=apparent_area)
        self.bbox_pub.publish(bbox)

        # Publish RViz marker at target world position
        self._publish_marker()

    def _publish_marker(self):
        if self.target_pos is None:
            return
        m = Marker()
        m.header = Header()
        m.header.frame_id = 'world'
        m.header.stamp = self.get_clock().now().to_msg()
        m.ns = 'detections'
        m.id = 0
        m.type = Marker.CYLINDER
        m.action = Marker.ADD
        m.pose.position.x = self.target_pos.x
        m.pose.position.y = self.target_pos.y
        m.pose.position.z = self.target_pos.z + 0.9
        m.pose.orientation.w = 1.0
        m.scale.x = 0.6
        m.scale.y = 0.6
        m.scale.z = 1.8
        m.color = ColorRGBA(r=0.2, g=0.9, b=0.3, a=0.5)
        m.lifetime.sec = 0
        m.lifetime.nanosec = 100_000_000  # 100 ms TTL
        arr = MarkerArray()
        arr.markers.append(m)
        self.marker_pub.publish(arr)


def main(args=None):
    rclpy.init(args=args)
    node = DetectionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
