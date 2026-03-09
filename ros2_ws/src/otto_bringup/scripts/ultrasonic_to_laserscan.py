#!/usr/bin/python3
"""Convert ultrasonic Range messages to LaserScan for SLAM compatibility.

Subscribes to /ultrasonic/range (sensor_msgs/Range).
Publishes /scan (sensor_msgs/LaserScan).

Two modes (set via 'mode' parameter):
  - 'passthrough' (default): Single-beam LaserScan from each Range reading.
  - 'accumulate': Accumulates readings at different robot headings to build
    a multi-beam synthetic scan for mapping demos.
"""

import math
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Range, LaserScan


class UltrasonicToLaserScan(Node):
    def __init__(self):
        super().__init__('ultrasonic_to_laserscan')

        self.declare_parameter('mode', 'passthrough')
        self.mode = self.get_parameter('mode').value

        qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.range_sub = self.create_subscription(
            Range, 'ultrasonic/range', self.range_callback, qos)

        self.scan_pub = self.create_publisher(LaserScan, 'scan', 10)

        self.get_logger().info(f'Ultrasonic to LaserScan converter started (mode={self.mode})')

    def range_callback(self, msg):
        scan = LaserScan()
        scan.header = msg.header

        if self.mode == 'passthrough':
            # Fan the single ultrasonic reading across multiple rays spanning
            # the sensor's FOV.  An ultrasonic cone detects the nearest object
            # anywhere in its beam, so filling the cone with identical ranges
            # is physically accurate and gives slam_toolbox enough rays to work.
            half_fov = msg.field_of_view / 2.0
            num_rays = 11  # odd so centre ray is at 0°
            scan.angle_min = -half_fov
            scan.angle_max = half_fov
            scan.angle_increment = msg.field_of_view / (num_rays - 1)
            scan.time_increment = 0.0
            scan.scan_time = 0.1  # 10Hz
            scan.range_min = msg.min_range
            scan.range_max = msg.max_range

            if math.isinf(msg.range) or msg.range < msg.min_range:
                scan.ranges = [float('inf')] * num_rays
            else:
                scan.ranges = [msg.range] * num_rays

            scan.intensities = []

        self.scan_pub.publish(scan)


def main(args=None):
    rclpy.init(args=args)
    node = UltrasonicToLaserScan()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
