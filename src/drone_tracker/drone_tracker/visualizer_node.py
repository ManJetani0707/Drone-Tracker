#!/usr/bin/env python3
"""
visualizer_node.py
------------------
Publishes RViz markers for:
  - Drone body (blue arrow showing heading)
  - Target (green cylinder)
  - Track history path (yellow trail)
  - PID error vectors (thin coloured lines)
  - HUD-style text overlay showing PID errors and track status
"""

import math
import collections
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point, PoseStamped
from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import ColorRGBA, Header


class VisualizerNode(Node):
    def __init__(self):
        super().__init__('visualizer_node')

        self.declare_parameter('history_length', 200)

        self.drone_pose  = None
        self.target_pos  = None
        self.track_state = None
        self.pid_errors  = None

        max_h = self.get_parameter('history_length').value
        self.drone_path  = collections.deque(maxlen=max_h)
        self.target_path = collections.deque(maxlen=max_h)

        self.create_subscription(PoseStamped, '/drone/pose',          self._drone_cb,  10)
        self.create_subscription(Point,       '/target/world_position', self._target_cb, 10)
        self.create_subscription(Point,       '/track/state',          self._track_cb,  10)
        self.create_subscription(Point,       '/pid/errors',           self._pid_cb,    10)

        self.marker_pub = self.create_publisher(MarkerArray, '/viz/markers', 10)

        self.create_timer(0.05, self._publish)  # 20 Hz

        self.get_logger().info('Visualizer node ready')

    # ------------------------------------------------------------------ #
    def _drone_cb(self, msg):
        self.drone_pose = msg
        self.drone_path.append((msg.pose.position.x,
                                 msg.pose.position.y,
                                 msg.pose.position.z))

    def _target_cb(self, msg):
        self.target_pos = msg
        self.target_path.append((msg.x, msg.y, msg.z))

    def _track_cb(self, msg):
        self.track_state = msg

    def _pid_cb(self, msg):
        self.pid_errors = msg

    # ------------------------------------------------------------------ #
    def _publish(self):
        markers = MarkerArray()
        now = self.get_clock().now().to_msg()

        def hdr(fid='world'):
            h = Header()
            h.stamp = now
            h.frame_id = fid
            return h

        mid = 0

        # ── Drone body arrow ──
        if self.drone_pose:
            m = Marker()
            m.header = hdr()
            m.ns, m.id, m.type, m.action = 'drone', mid, Marker.ARROW, Marker.ADD
            mid += 1
            q = self.drone_pose.pose.orientation
            m.pose = self.drone_pose.pose
            m.scale.x = 1.2  # shaft length
            m.scale.y = 0.15
            m.scale.z = 0.15
            m.color = ColorRGBA(r=0.2, g=0.4, b=1.0, a=0.9)
            markers.markers.append(m)

            # Drone sphere body
            m2 = Marker()
            m2.header = hdr()
            m2.ns, m2.id, m2.type, m2.action = 'drone', mid, Marker.SPHERE, Marker.ADD
            mid += 1
            m2.pose = self.drone_pose.pose
            m2.scale.x = m2.scale.y = m2.scale.z = 0.4
            m2.color = ColorRGBA(r=0.1, g=0.2, b=0.8, a=0.8)
            markers.markers.append(m2)

        # ── Target cylinder ──
        if self.target_pos:
            m = Marker()
            m.header = hdr()
            m.ns, m.id, m.type, m.action = 'target', mid, Marker.CYLINDER, Marker.ADD
            mid += 1
            m.pose.position = Point(x=self.target_pos.x,
                                     y=self.target_pos.y,
                                     z=self.target_pos.z + 0.9)
            m.pose.orientation.w = 1.0
            m.scale.x = m.scale.y = 0.5
            m.scale.z = 1.8
            tracked = self.track_state and self.track_state.x >= 0.0
            m.color = ColorRGBA(r=0.1, g=0.9, b=0.3, a=0.7) if tracked \
                      else ColorRGBA(r=0.9, g=0.2, b=0.1, a=0.5)
            markers.markers.append(m)

        # ── Drone flight path ──
        if len(self.drone_path) > 1:
            m = Marker()
            m.header = hdr()
            m.ns, m.id, m.type, m.action = 'path', mid, Marker.LINE_STRIP, Marker.ADD
            mid += 1
            m.scale.x = 0.05
            m.color = ColorRGBA(r=0.3, g=0.6, b=1.0, a=0.6)
            for p in self.drone_path:
                m.points.append(Point(x=p[0], y=p[1], z=p[2]))
            markers.markers.append(m)

        # ── Target walk path ──
        if len(self.target_path) > 1:
            m = Marker()
            m.header = hdr()
            m.ns, m.id, m.type, m.action = 'tpath', mid, Marker.LINE_STRIP, Marker.ADD
            mid += 1
            m.scale.x = 0.04
            m.color = ColorRGBA(r=0.2, g=0.9, b=0.4, a=0.4)
            for p in self.target_path:
                m.points.append(Point(x=p[0], y=p[1], z=p[2]))
            markers.markers.append(m)

        # ── Line from drone to target ──
        if self.drone_pose and self.target_pos:
            m = Marker()
            m.header = hdr()
            m.ns, m.id, m.type, m.action = 'los', mid, Marker.LINE_LIST, Marker.ADD
            mid += 1
            m.scale.x = 0.03
            tracked = self.track_state and self.track_state.x >= 0.0
            m.color = ColorRGBA(r=1.0, g=0.9, b=0.1, a=0.6) if tracked \
                      else ColorRGBA(r=0.8, g=0.1, b=0.1, a=0.3)
            dp = self.drone_pose.pose.position
            m.points.append(Point(x=dp.x, y=dp.y, z=dp.z))
            m.points.append(Point(x=self.target_pos.x,
                                   y=self.target_pos.y,
                                   z=self.target_pos.z + 0.9))
            markers.markers.append(m)

        # ── HUD text: PID errors + track status ──
        if self.pid_errors or (self.track_state is not None):
            m = Marker()
            m.header = hdr()
            m.ns, m.id, m.type, m.action = 'hud', mid, Marker.TEXT_VIEW_FACING, Marker.ADD
            mid += 1
            dp = self.drone_pose.pose.position if self.drone_pose else Point()
            m.pose.position = Point(x=dp.x, y=dp.y, z=dp.z + 1.5)
            m.pose.orientation.w = 1.0
            m.scale.z = 0.4  # text height metres

            if self.track_state and self.track_state.x >= 0.0:
                status = 'TRACKING'
                m.color = ColorRGBA(r=0.1, g=1.0, b=0.3, a=1.0)
            else:
                status = 'LOST'
                m.color = ColorRGBA(r=1.0, g=0.2, b=0.1, a=1.0)

            if self.pid_errors:
                e = self.pid_errors
                m.text = (
                    f'{status}\n'
                    f'e_yaw={e.x:+.3f}  e_alt={e.y:+.3f}  e_fwd={e.z:+.3f}'
                )
            else:
                m.text = status

            markers.markers.append(m)

        self.marker_pub.publish(markers)


def main(args=None):
    rclpy.init(args=args)
    node = VisualizerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
