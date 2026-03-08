#!/usr/bin/env python3
"""Visualize Otto Starter sensors in RViz.

Subscribes:
  /leds              std_msgs/UInt32MultiArray  — 13 packed 0x00RRGGBB values
  /ultrasonic/range  sensor_msgs/Range          — distance reading (BEST_EFFORT)
  /line_sensors      std_msgs/Int32MultiArray   — [left_adc, right_adc] 0-4095

Publishes:
  /otto/led_markers  visualization_msgs/MarkerArray
      - 13 spheres at led_N_link frames coloured from /leds topic
      - 2 cylinders at line_sensor_{left,right}_link: red=line, green=clear
  /otto/sonar_marker visualization_msgs/Marker
      - sphere placed at measured distance along sensor X axis
"""

import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from rclpy.qos import QoSProfile, ReliabilityPolicy

from std_msgs.msg import UInt32MultiArray, Int32MultiArray
from sensor_msgs.msg import Range
from visualization_msgs.msg import Marker, MarkerArray


NUM_LEDS     = 13
SONAR_FAR    = 0.5    # m — green at this distance
SONAR_NEAR   = 0.02   # m — red at this distance
LINE_ADC_MAX = 4095   # 12-bit ADC full scale


def _rgb_from_uint32(packed: int):
    """Extract (r, g, b) 0-1 floats from 0x00RRGGBB int."""
    r = ((packed >> 16) & 0xFF) / 255.0
    g = ((packed >>  8) & 0xFF) / 255.0
    b = ((packed      ) & 0xFF) / 255.0
    return r, g, b


def _sonar_colour(distance: float):
    """Green (far) -> red (near) over SONAR_FAR -> SONAR_NEAR."""
    t = max(0.0, min(1.0, (SONAR_FAR - distance) / (SONAR_FAR - SONAR_NEAR)))
    return (t, 1.0 - t, 0.0)


def _line_colour(adc: int):
    """Green (clear/bright surface) -> red (dark surface / line detected)."""
    t = max(0.0, min(1.0, adc / LINE_ADC_MAX))
    return (1.0 - t, t, 0.0)


class OttoVisualizer(Node):

    def __init__(self):
        super().__init__('otto_visualizer')

        self._led_colours = [(1.0, 1.0, 1.0)] * NUM_LEDS  # default white
        self._sonar_dist  = SONAR_FAR
        self._line_adc    = [LINE_ADC_MAX, LINE_ADC_MAX]   # [left, right] default clear

        self._pub_leds  = self.create_publisher(MarkerArray, '/otto/led_markers', 10)
        self._pub_sonar = self.create_publisher(Marker,      '/otto/sonar_marker', 10)

        # Match firmware BEST_EFFORT publishers so ROS2 QoS negotiation succeeds
        best_effort = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)

        self.create_subscription(UInt32MultiArray, '/leds',
                                 self._leds_cb, 10)
        self.create_subscription(Range, '/ultrasonic/range',
                                 self._range_cb, best_effort)
        self.create_subscription(Int32MultiArray, '/line_sensors',
                                 self._line_cb, best_effort)

        # Publish at 10 Hz even without new data so RViz markers don't time out
        self.create_timer(0.1, self._publish)

    # -- Callbacks --------------------------------------------------------

    def _leds_cb(self, msg: UInt32MultiArray):
        for i, packed in enumerate(msg.data[:NUM_LEDS]):
            self._led_colours[i] = _rgb_from_uint32(packed)

    def _range_cb(self, msg: Range):
        self._sonar_dist = msg.range

    def _line_cb(self, msg: Int32MultiArray):
        if len(msg.data) >= 2:
            self._line_adc = [int(msg.data[0]), int(msg.data[1])]

    # -- Publish ----------------------------------------------------------

    def _publish(self):
        now      = self.get_clock().now().to_msg()
        lifetime = Duration(seconds=0.5).to_msg()

        array = MarkerArray()

        # LED spheres
        for i in range(NUM_LEDS):
            m = Marker()
            m.header.stamp    = now
            m.header.frame_id = f'led_{i}_link'
            m.ns              = 'otto_leds'
            m.id              = i
            m.type            = Marker.SPHERE
            m.action          = Marker.ADD
            m.lifetime        = lifetime
            m.scale.x = m.scale.y = m.scale.z = 0.006   # 6 mm sphere
            r, g, b = self._led_colours[i]
            m.color.r, m.color.g, m.color.b, m.color.a = r, g, b, 1.0
            m.pose.orientation.w = 1.0
            array.markers.append(m)

        # Line sensor discs (red = line detected, green = clear)
        line_frames = [
            ('line_sensor_left_link',  self._line_adc[0]),
            ('line_sensor_right_link', self._line_adc[1]),
        ]
        for i, (frame, adc) in enumerate(line_frames):
            m = Marker()
            m.header.stamp    = now
            m.header.frame_id = frame
            m.ns              = 'otto_line_sensors'
            m.id              = i
            m.type            = Marker.CYLINDER
            m.action          = Marker.ADD
            m.lifetime        = lifetime
            m.scale.x = m.scale.y = 0.010   # 10 mm diameter
            m.scale.z          = 0.002       # 2 mm thin disc
            r, g, b = _line_colour(adc)
            m.color.r, m.color.g, m.color.b, m.color.a = r, g, b, 0.9
            m.pose.orientation.w = 1.0
            array.markers.append(m)

        self._pub_leds.publish(array)

        # Sonar indicator sphere placed at measured distance along sensor X
        m = Marker()
        m.header.stamp    = now
        m.header.frame_id = 'ultrasonic_sensor_frame'
        m.ns              = 'otto_sonar'
        m.id              = 0
        m.type            = Marker.SPHERE
        m.action          = Marker.ADD
        m.lifetime        = lifetime
        d = self._sonar_dist
        if not (SONAR_NEAR <= d <= SONAR_FAR):
            d = SONAR_FAR
        m.pose.position.x    = d
        m.pose.orientation.w = 1.0
        m.scale.x = m.scale.y = m.scale.z = 0.020   # 20 mm sphere
        r, g, b = _sonar_colour(d)
        m.color.r, m.color.g, m.color.b, m.color.a = r, g, b, 0.8
        self._pub_sonar.publish(m)


def main(args=None):
    rclpy.init(args=args)
    node = OttoVisualizer()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
