#!/usr/bin/python3
"""Dead-reckoning odometry from cmd_vel for Otto Starter.

Integrates linear.x and angular.z from /cmd_vel to produce odometry.
Publishes nav_msgs/Odometry on /odom, TF odom -> base_footprint,
and sensor_msgs/JointState for wheel rotation (replaces joint_state_publisher).

WARNING: With no wheel encoders, odometry drifts significantly.
This is expected for an educational demo with open-loop servos.
"""

import math
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from geometry_msgs.msg import Twist, TransformStamped, Quaternion
from nav_msgs.msg import Odometry
from sensor_msgs.msg import JointState
from tf2_ros import TransformBroadcaster

WHEEL_RADIUS = 0.0245  # m — matches URDF/firmware
WHEEL_BASE   = 0.0814  # m — matches URDF/firmware


def euler_to_quaternion(yaw):
    """Convert yaw angle to geometry_msgs Quaternion."""
    q = Quaternion()
    q.x = 0.0
    q.y = 0.0
    q.z = math.sin(yaw / 2.0)
    q.w = math.cos(yaw / 2.0)
    return q


class OdomPublisher(Node):
    def __init__(self):
        super().__init__('otto_odom_publisher')

        # Pose state
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.linear_x = 0.0
        self.angular_z = 0.0
        self.left_wheel_angle  = 0.0  # rad, accumulated
        self.right_wheel_angle = 0.0
        self.last_time = self.get_clock().now()
        self.last_cmd_time = self.get_clock().now()
        self.cmd_vel_timeout = 0.5  # seconds, matches firmware CMD_VEL_TIMEOUT_MS

        # Subscribe to cmd_vel (best effort to match firmware publisher)
        qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.cmd_vel_sub = self.create_subscription(
            Twist, 'cmd_vel', self.cmd_vel_callback, qos)

        # Publish odometry and joint states
        self.odom_pub  = self.create_publisher(Odometry,    'odom', 10)
        self.joint_pub = self.create_publisher(JointState,  'joint_states', 10)
        self.tf_broadcaster = TransformBroadcaster(self)

        # Integration timer at 50Hz
        self.timer = self.create_timer(0.02, self.timer_callback)

        self.get_logger().info('Otto odometry publisher started (dead-reckoning from cmd_vel)')

    def cmd_vel_callback(self, msg):
        self.linear_x = msg.linear.x
        self.angular_z = msg.angular.z
        self.last_cmd_time = self.get_clock().now()

    def timer_callback(self):
        now = self.get_clock().now()
        dt = (now - self.last_time).nanoseconds / 1e9
        self.last_time = now

        if dt <= 0.0 or dt > 1.0:
            return

        # Zero velocities if no cmd_vel received within timeout
        # (matches firmware servo safety stop)
        cmd_age = (now - self.last_cmd_time).nanoseconds / 1e9
        if cmd_age > self.cmd_vel_timeout:
            self.linear_x = 0.0
            self.angular_z = 0.0

        # Integrate pose and wheel angles
        self.theta += self.angular_z * dt
        self.x += self.linear_x * math.cos(self.theta) * dt
        self.y += self.linear_x * math.sin(self.theta) * dt

        v_left  = self.linear_x - self.angular_z * WHEEL_BASE / 2.0
        v_right = self.linear_x + self.angular_z * WHEEL_BASE / 2.0
        self.left_wheel_angle  += (v_left  / WHEEL_RADIUS) * dt
        self.right_wheel_angle += (v_right / WHEEL_RADIUS) * dt

        q = euler_to_quaternion(self.theta)

        # Publish TF: odom -> base_footprint
        t = TransformStamped()
        t.header.stamp = now.to_msg()
        t.header.frame_id = 'odom'
        t.child_frame_id = 'base_footprint'
        t.transform.translation.x = self.x
        t.transform.translation.y = self.y
        t.transform.translation.z = 0.0
        t.transform.rotation = q
        self.tf_broadcaster.sendTransform(t)

        # Publish Odometry message
        odom = Odometry()
        odom.header.stamp = now.to_msg()
        odom.header.frame_id = 'odom'
        odom.child_frame_id = 'base_footprint'
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.orientation = q

        # Velocity
        odom.twist.twist.linear.x = self.linear_x
        odom.twist.twist.angular.z = self.angular_z

        # High covariance — no encoder feedback, dead-reckoning only
        odom.pose.covariance[0] = 0.1   # x
        odom.pose.covariance[7] = 0.1   # y
        odom.pose.covariance[35] = 0.2  # yaw
        odom.twist.covariance[0] = 0.1
        odom.twist.covariance[35] = 0.2

        self.odom_pub.publish(odom)

        # Wheel joint states so robot_state_publisher can animate wheels
        js = JointState()
        js.header.stamp = now.to_msg()
        js.name     = ['left_wheel_joint', 'right_wheel_joint']
        js.position = [self.left_wheel_angle, self.right_wheel_angle]
        js.velocity = [v_left / WHEEL_RADIUS, v_right / WHEEL_RADIUS]
        self.joint_pub.publish(js)


def main(args=None):
    rclpy.init(args=args)
    node = OdomPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
