#!/usr/bin/env python3
"""
pid_controller_node.py
-----------------------
Three independent PID loops converting image-space track error
into body-frame velocity commands.

  Loop 1 — YAW     : horizontal centroid error → yaw rate (rad/s)
  Loop 2 — ALTITUDE: vertical centroid error   → climb rate (m/s)
  Loop 3 — FORWARD : apparent area error       → forward speed (m/s)
                     (small area → target far away → fly closer)

Output: geometry_msgs/Twist on /drone/cmd_vel
  linear.x  = forward velocity   (m/s, body frame)
  linear.z  = climb rate         (m/s)
  angular.z = yaw rate           (rad/s)

Topics subscribed:
  /track/state   (geometry_msgs/Point)  cx, cy, area  (normalised)
  /drone/pose    (geometry_msgs/PoseStamped)
"""

import time
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, Point, PoseStamped
from std_msgs.msg import Bool


class PID:
    """Classic PID with anti-windup clamp."""

    def __init__(self, kp, ki, kd, out_min, out_max, dt):
        self.kp, self.ki, self.kd = kp, ki, kd
        self.out_min, self.out_max = out_min, out_max
        self.dt = dt
        self._integral = 0.0
        self._prev_err  = None

    def reset(self):
        self._integral = 0.0
        self._prev_err  = None

    def compute(self, error: float) -> float:
        # Derivative
        if self._prev_err is None:
            derivative = 0.0
        else:
            derivative = (error - self._prev_err) / self.dt
        self._prev_err = error

        # Integral with anti-windup clamp
        self._integral = max(
            self.out_min / (self.ki + 1e-9),
            min(self.out_max / (self.ki + 1e-9),
                self._integral + error * self.dt)
        )

        output = self.kp * error + self.ki * self._integral + self.kd * derivative
        return max(self.out_min, min(self.out_max, output))


class PIDControllerNode(Node):
    def __init__(self):
        super().__init__('pid_controller_node')

        hz = 30.0
        dt = 1.0 / hz

        # ── Declare all PID gains as ROS parameters (tunable at runtime) ──
        self.declare_parameter('yaw.kp',    0.8)
        self.declare_parameter('yaw.ki',    0.02)
        self.declare_parameter('yaw.kd',    0.15)
        self.declare_parameter('alt.kp',    0.6)
        self.declare_parameter('alt.ki',    0.01)
        self.declare_parameter('alt.kd',    0.10)
        self.declare_parameter('fwd.kp',    1.2)
        self.declare_parameter('fwd.ki',    0.05)
        self.declare_parameter('fwd.kd',    0.20)
        self.declare_parameter('target_area',      0.04)   # desired apparent area
        self.declare_parameter('lost_timeout_sec', 1.5)
        self.declare_parameter('max_fwd_speed',    3.0)    # m/s
        self.declare_parameter('max_yaw_rate',     1.2)    # rad/s
        self.declare_parameter('max_climb_rate',   1.0)    # m/s

        p = lambda n: self.get_parameter(n).value

        self.target_area = p('target_area')
        self.lost_timeout = p('lost_timeout_sec')

        max_fwd  = p('max_fwd_speed')
        max_yaw  = p('max_yaw_rate')
        max_clmb = p('max_climb_rate')

        self.pid_yaw = PID(p('yaw.kp'), p('yaw.ki'), p('yaw.kd'), -max_yaw,  max_yaw,  dt)
        self.pid_alt = PID(p('alt.kp'), p('alt.ki'), p('alt.kd'), -max_clmb, max_clmb, dt)
        self.pid_fwd = PID(p('fwd.kp'), p('fwd.ki'), p('fwd.kd'), -0.5,      max_fwd,  dt)

        # State
        self.track_state = None
        self.last_detect = time.time()
        self.armed       = True

        # Subscriptions
        self.create_subscription(Point, '/track/state', self._track_cb, 10)
        self.create_subscription(Bool,  '/drone/arm',   self._arm_cb,   10)

        # Publishers
        self.cmd_pub  = self.create_publisher(Twist, '/drone/cmd_vel', 10)
        self.err_pub  = self.create_publisher(Point, '/pid/errors',    10)

        self.create_timer(dt, self._control_loop)

        self.get_logger().info(
            f'PID controller ready  target_area={self.target_area:.3f}'
        )

    # ------------------------------------------------------------------ #
    def _arm_cb(self, msg: Bool):
        self.armed = msg.data
        if not self.armed:
            self.pid_yaw.reset()
            self.pid_alt.reset()
            self.pid_fwd.reset()
            self.get_logger().info('Disarmed — PIDs reset')

    def _track_cb(self, msg: Point):
        self.track_state = msg
        if msg.x >= 0.0:  # valid track
            self.last_detect = time.time()

    def _control_loop(self):
        cmd = Twist()

        if not self.armed:
            self.cmd_pub.publish(cmd)  # zero command
            return

        # Watchdog: stop if track lost too long
        lost = (self.track_state is None or self.track_state.x < 0.0)
        timed_out = (time.time() - self.last_detect) > self.lost_timeout

        if lost or timed_out:
            self.cmd_pub.publish(cmd)  # hover (zero vel)
            return

        cx   = self.track_state.x   # 0 = left,  1 = right, 0.5 = centre
        cy   = self.track_state.y   # 0 = top,   1 = bottom, 0.5 = centre
        area = self.track_state.z

        # Error definitions (positive error → positive correction)
        err_yaw = 0.5 - cx          # positive → target is left → turn left (pos yaw)
        err_alt = 0.5 - cy          # positive → target is above → climb
        err_fwd = area - self.target_area  # positive → target too close → back off

        cmd.angular.z = self.pid_yaw.compute(err_yaw)
        cmd.linear.z  = self.pid_alt.compute(err_alt)
        cmd.linear.x  = self.pid_fwd.compute(err_fwd)

        self.cmd_pub.publish(cmd)
        self.err_pub.publish(Point(x=err_yaw, y=err_alt, z=err_fwd))

        self.get_logger().debug(
            f'e_yaw={err_yaw:+.3f} e_alt={err_alt:+.3f} e_fwd={err_fwd:+.3f} '
            f'→ ω={cmd.angular.z:+.2f} vz={cmd.linear.z:+.2f} vx={cmd.linear.x:+.2f}'
        )


def main(args=None):
    rclpy.init(args=args)
    node = PIDControllerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
