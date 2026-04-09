#!/usr/bin/env python3
"""
tracker_node.py
---------------
Lightweight single-target tracker using a Kalman Filter.

In a real system you'd run DeepSORT or ByteTrack with ReID features.
Here we implement the Kalman prediction/update cycle directly so the
code is portable (no extra dependencies) and educational.

State vector: [cx, cy, area, vcx, vcy, varea]
  cx, cy   - normalised bounding box centre (0..1)
  area     - normalised bounding box area   (0..1)
  vcx, vcy - velocity of centre
  varea    - rate of area change

Topics subscribed:
  /detection/bbox  (geometry_msgs/Point)  cx_norm, cy_norm, area_norm

Topics published:
  /track/state     (geometry_msgs/Point)  smoothed cx, cy, area (or -1,-1,-1 if lost)
  /track/velocity  (geometry_msgs/Point)  estimated velocity in normalised px/s
"""

import numpy as np
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point


class KalmanTracker:
    """6-DOF constant-velocity Kalman filter for a 2D bounding box."""

    def __init__(self, dt: float = 1.0 / 30.0):
        self.dt = dt
        n, m = 6, 3  # state dim, measurement dim

        # State transition: x_{k+1} = F x_k
        self.F = np.eye(n)
        self.F[0, 3] = dt
        self.F[1, 4] = dt
        self.F[2, 5] = dt

        # Measurement matrix: z_k = H x_k
        self.H = np.zeros((m, n))
        self.H[0, 0] = 1.0
        self.H[1, 1] = 1.0
        self.H[2, 2] = 1.0

        # Process noise
        self.Q = np.diag([1e-4, 1e-4, 1e-5, 5e-4, 5e-4, 1e-5])

        # Measurement noise (tuned to match detection_noise_px ~5 px / 640)
        self.R = np.diag([1e-3, 1e-3, 5e-4])

        self.x = None   # state
        self.P = None   # covariance
        self.initialized = False

    def init(self, z: np.ndarray):
        self.x = np.zeros(6)
        self.x[:3] = z
        self.P = np.eye(6) * 0.1
        self.initialized = True

    def predict(self):
        if not self.initialized:
            return
        self.x = self.F @ self.x
        self.P = self.F @ self.P @ self.F.T + self.Q

    def update(self, z: np.ndarray):
        if not self.initialized:
            self.init(z)
            return
        S = self.H @ self.P @ self.H.T + self.R
        K = self.P @ self.H.T @ np.linalg.inv(S)
        self.x = self.x + K @ (z - self.H @ self.x)
        self.P = (np.eye(6) - K @ self.H) @ self.P

    @property
    def state(self) -> np.ndarray:
        return self.x.copy() if self.initialized else None


class TrackerNode(Node):
    def __init__(self):
        super().__init__('tracker_node')

        self.declare_parameter('max_lost_frames', 15)
        self.declare_parameter('detection_hz', 30.0)

        self.max_lost = self.get_parameter('max_lost_frames').value
        hz = self.get_parameter('detection_hz').value

        self.kf          = KalmanTracker(dt=1.0 / hz)
        self.lost_frames = 0
        self.tracking    = False

        self.create_subscription(
            Point, '/detection/bbox', self._detection_cb, 10)

        self.state_pub = self.create_publisher(Point, '/track/state',    10)
        self.vel_pub   = self.create_publisher(Point, '/track/velocity', 10)

        self.create_timer(1.0 / hz, self._predict_and_publish)

        self.get_logger().info('Tracker node ready (Kalman filter)')

    # ------------------------------------------------------------------ #
    def _detection_cb(self, msg: Point):
        z = np.array([msg.x, msg.y, msg.z])

        if not self.tracking:
            self.kf.init(z)
            self.tracking = True
            self.lost_frames = 0
            self.get_logger().info('Target acquired')
            return

        self.kf.update(z)
        self.lost_frames = 0

    def _predict_and_publish(self):
        if not self.tracking:
            self._publish_lost()
            return

        self.kf.predict()
        self.lost_frames += 1

        if self.lost_frames > self.max_lost:
            self.tracking = False
            self.lost_frames = 0
            self.get_logger().warn('Target lost')
            self._publish_lost()
            return

        s = self.kf.state
        # Clamp to valid image space
        cx   = float(np.clip(s[0], 0.0, 1.0))
        cy   = float(np.clip(s[1], 0.0, 1.0))
        area = float(np.clip(s[2], 0.001, 1.0))

        self.state_pub.publish(Point(x=cx, y=cy, z=area))
        self.vel_pub.publish(Point(x=s[3], y=s[4], z=s[5]))

    def _publish_lost(self):
        self.state_pub.publish(Point(x=-1.0, y=-1.0, z=-1.0))


def main(args=None):
    rclpy.init(args=args)
    node = TrackerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
