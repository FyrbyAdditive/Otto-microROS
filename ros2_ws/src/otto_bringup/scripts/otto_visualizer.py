#!/usr/bin/python3
"""Visualize Otto Starter sensors in RViz.

Subscribes:
  /led_state         std_msgs/UInt8MultiArray   — [0, 1, R0,G0,B0, ..., R12,G12,B12]
                       Published by firmware whenever ring state changes (BEST_EFFORT)
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

from std_msgs.msg import UInt8MultiArray, Int32MultiArray
from sensor_msgs.msg import Range
from visualization_msgs.msg import Marker, MarkerArray


NUM_LEDS     = 13
SONAR_FAR    = 0.5    # m — green at this distance
SONAR_NEAR   = 0.02   # m — red at this distance
LINE_ADC_MAX = 500    # practical max at ~18mm sensing distance

# Firmware LED index → URDF led_N_link frame number.
# Firmware uses clock-face numbering (0=centre, 12=forward/12-o'clock,
# 3=right/3-o'clock, 6=rear, 9=left). URDF frames are sorted by angle
# from the STEP export and have a different numbering.
LED_FRAME_MAP = [6, 4, 3, 2, 1, 0, 7, 12, 11, 10, 9, 8, 5]

# Firmware LED colours are physically dim (setBrightness=50/255≈20%).
# Scale up for RViz visibility — raw values like blue=30 look near-black.
LED_VIZ_SCALE = 5.0



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

        self.create_subscription(UInt8MultiArray, '/led_state',
                                 self._leds_cb, best_effort)
        self.create_subscription(Range, '/ultrasonic/range',
                                 self._range_cb, best_effort)
        self.create_subscription(Int32MultiArray, '/line_sensors',
                                 self._line_cb, best_effort)

        # Publish at 10 Hz even without new data so RViz markers don't time out
        self.create_timer(0.1, self._publish)

    # -- Callbacks --------------------------------------------------------

    def _leds_cb(self, msg: UInt8MultiArray):
        data = msg.data
        if len(data) < 2:
            return
        mode = data[1]
        if mode == 0 and len(data) >= 5:
            # All LEDs same colour
            r, g, b = data[2] / 255.0, data[3] / 255.0, data[4] / 255.0
            self._led_colours = [(r, g, b)] * NUM_LEDS
        elif mode == 1 and len(data) > 2:
            # Individual LED colours: [R0,G0,B0, R1,G1,B1, ...]
            rgb = data[2:]
            for i in range(min(NUM_LEDS, len(rgb) // 3)):
                self._led_colours[i] = (
                    rgb[i * 3]     / 255.0,
                    rgb[i * 3 + 1] / 255.0,
                    rgb[i * 3 + 2] / 255.0,
                )
        elif mode == 2:
            self._led_colours = [(0.0, 0.0, 0.0)] * NUM_LEDS

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

        # LED spheres — firmware index i mapped to correct URDF frame
        for i in range(NUM_LEDS):
            m = Marker()
            m.header.stamp    = now
            m.header.frame_id = f'led_{LED_FRAME_MAP[i]}_link'
            m.ns              = 'otto_leds'
            m.id              = i
            m.type            = Marker.SPHERE
            m.action          = Marker.ADD
            m.lifetime        = lifetime
            m.scale.x = m.scale.y = m.scale.z = 0.010   # 10 mm sphere
            r, g, b = self._led_colours[i]
            # Boost brightness for RViz — firmware dims LEDs to ~20% physically
            r = min(1.0, r * LED_VIZ_SCALE)
            g = min(1.0, g * LED_VIZ_SCALE)
            b = min(1.0, b * LED_VIZ_SCALE)
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
            m.scale.x = m.scale.y = 0.015   # 15 mm diameter
            m.scale.z          = 0.005       # 5 mm tall — visible from side
            r, g, b = _line_colour(adc)
            m.color.r, m.color.g, m.color.b, m.color.a = r, g, b, 1.0
            # Place disc just above the floor plane (~5 mm above ground).
            # line_sensor link is 18 mm above ground; offset -0.013 m puts
            # disc centre at 5 mm above ground, clearly visible under robot.
            m.pose.position.z    = -0.013
            m.pose.position.x    = 0.0170   # disc centred at front edge of sensor board
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
