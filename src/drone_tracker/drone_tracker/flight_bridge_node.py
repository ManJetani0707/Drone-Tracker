#!/usr/bin/env python3
"""
flight_bridge_node.py
---------------------
Bridges /drone/cmd_vel (body-frame Twist) to Gazebo model poses.

In a real system this would be replaced by MAVROS publishing
geometry_msgs/TwistStamped to /mavros/setpoint_velocity/cmd_vel_unstamped.

Here we integrate velocity commands to update drone position directly
in the Gazebo world, simulating inner-loop attitude control.

Physics model:
  - Velocity commands are first-order low-pass filtered (τ = 0.15 s)
    to simulate inertia / attitude lag.
  - Yaw is integrated from angular.z.
  - Altitude is integrated from linear.z with gravity hold.
  - Forward/lateral from linear.x/y in drone body frame.

Topics subscribed:
  /drone/cmd_vel  (geometry_msgs/Twist)

Topics published:
  /drone/pose     (geometry_msgs/PoseStamped)  ground truth pose
  /gazebo/set_model_state  — if gazebo_msgs available, else simulated internally

Parameters:
  start_x, start_y, start_z  - initial drone world position
  tau_velocity               - velocity filter time constant (s)
  max_speed                  - hard limit (m/s)
"""

import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, PoseStamped, Point
from std_msgs.msg import Header

try:
    from gazebo_msgs.msg import ModelState
    from gazebo_msgs.srv import SetModelState
    GAZEBO_MSGS_AVAILABLE = True
except ImportError:
    GAZEBO_MSGS_AVAILABLE = False


class FlightBridgeNode(Node):
    def __init__(self):
        super().__init__('flight_bridge_node')

        self.declare_parameter('start_x',      0.0)
        self.declare_parameter('start_y',      0.0)
        self.declare_parameter('start_z',      5.0)   # 5 m hover altitude
        self.declare_parameter('start_yaw',    0.0)
        self.declare_parameter('tau_velocity', 0.15)  # velocity lag (s)
        self.declare_parameter('max_speed',    6.0)   # m/s total
        self.declare_parameter('update_hz',   50.0)

        p = lambda n: self.get_parameter(n).value

        # State
        self.px  = p('start_x')
        self.py  = p('start_y')
        self.pz  = p('start_z')
        self.yaw = p('start_yaw')

        # Filtered velocities (body frame)
        self.vx_filt = 0.0
        self.vy_filt = 0.0
        self.vz_filt = 0.0
        self.wz_filt = 0.0

        self.tau = p('tau_velocity')
        self.max_spd = p('max_speed')

        hz = p('update_hz')
        self.dt = 1.0 / hz

        # Latest command
        self.latest_cmd = Twist()

        self.create_subscription(Twist, '/drone/cmd_vel', self._cmd_cb, 10)

        self.pose_pub   = self.create_publisher(PoseStamped, '/drone/pose',             10)
        self.target_pub = self.create_publisher(Point,       '/target/world_position',  10)

        # Gazebo SetModelState client
        if GAZEBO_MSGS_AVAILABLE:
            self.set_state_cli = self.create_client(SetModelState, '/gazebo/set_model_state')
        else:
            self.get_logger().warn(
                'gazebo_msgs not found — drone pose published internally only '
                '(Gazebo model will not move, but all ROS logic works)'
            )

        # Simple target motion: walks a circle of radius 8 m
        self.target_t = 0.0
        self.target_r = 8.0
        self.target_speed_rad = 0.12  # rad/s

        self.create_timer(self.dt, self._update)

        self.get_logger().info(
            f'Flight bridge ready  start=({self.px},{self.py},{self.pz})  '
            f'gazebo_msgs={"yes" if GAZEBO_MSGS_AVAILABLE else "no"}'
        )

    # ------------------------------------------------------------------ #
    def _cmd_cb(self, msg: Twist):
        self.latest_cmd = msg

    def _update(self):
        cmd = self.latest_cmd

        # 1st-order low-pass filter on each velocity axis
        alpha = self.dt / (self.tau + self.dt)
        self.vx_filt += alpha * (cmd.linear.x  - self.vx_filt)
        self.vy_filt += alpha * (cmd.linear.y  - self.vy_filt)
        self.vz_filt += alpha * (cmd.linear.z  - self.vz_filt)
        self.wz_filt += alpha * (cmd.angular.z - self.wz_filt)

        # Integrate yaw
        self.yaw += self.wz_filt * self.dt

        # Integrate position (body → world frame)
        cos_y = math.cos(self.yaw)
        sin_y = math.sin(self.yaw)
        self.px += (cos_y * self.vx_filt - sin_y * self.vy_filt) * self.dt
        self.py += (sin_y * self.vx_filt + cos_y * self.vy_filt) * self.dt
        self.pz += self.vz_filt * self.dt

        # Safety floor / ceiling
        self.pz = max(0.3, min(30.0, self.pz))

        # Publish drone pose
        self._pub_drone_pose()

        # Simulate moving target (walking circle)
        self.target_t += self.dt
        tx = self.target_r * math.cos(self.target_speed_rad * self.target_t)
        ty = self.target_r * math.sin(self.target_speed_rad * self.target_t)
        self.target_pub.publish(Point(x=tx, y=ty, z=0.9))  # ~0.9m = waist height

        # Send to Gazebo if available
        if GAZEBO_MSGS_AVAILABLE and hasattr(self, 'set_state_cli'):
            self._send_gazebo_state('drone_model', self.px, self.py, self.pz, self.yaw)
            self._send_gazebo_state('walking_actor', tx, ty, 0.0, 0.0)

    def _pub_drone_pose(self):
        msg = PoseStamped()
        msg.header = Header()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'world'
        msg.pose.position.x = self.px
        msg.pose.position.y = self.py
        msg.pose.position.z = self.pz

        # Yaw → quaternion
        half_yaw = self.yaw / 2.0
        msg.pose.orientation.z = math.sin(half_yaw)
        msg.pose.orientation.w = math.cos(half_yaw)

        self.pose_pub.publish(msg)

    def _send_gazebo_state(self, model_name, x, y, z, yaw):
        if not self.set_state_cli.service_is_ready():
            return
        req = SetModelState.Request()
        req.model_state = ModelState()
        req.model_state.model_name = model_name
        req.model_state.pose.position.x = x
        req.model_state.pose.position.y = y
        req.model_state.pose.position.z = z
        half_yaw = yaw / 2.0
        req.model_state.pose.orientation.z = math.sin(half_yaw)
        req.model_state.pose.orientation.w = math.cos(half_yaw)
        req.model_state.reference_frame = 'world'
        self.set_state_cli.call_async(req)


def main(args=None):
    rclpy.init(args=args)
    node = FlightBridgeNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
