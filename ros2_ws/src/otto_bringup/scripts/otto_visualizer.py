#!/usr/bin/env python3
"""Visualize Otto Starter LED ring and ultrasonic sensor in RViz.

Subscribes:
  /otto/leds             std_msgs/UInt32MultiArray  — N packed 0x00RRGGBB values
  /otto/ultrasonic/range sensor_msgs/Range           — distance reading

Publishes:
  /otto/led_markers  visualization_msgs/MarkerArray  — sphere per LED at led_N_link
  /otto/sonar_marker visualization_msgs/Marker       — colour sphere at ultrasonic_link
"""

import rclpy
from rclpy.node import Node
from rclpy.duration import Duration

from std_msgs.msg import UInt32MultiArray
from sensor_msgs.msg import Range
from visualization_msgs.msg import Marker, MarkerArray


# Number of LED frames defined in the URDF
NUM_LEDS = 15

# Sonar colour range (metres): green (far) → red (near)
SONAR_FAR  = 0.5
SONAR_NEAR = 0.02


def _rgb_from_uint32(packed: int):
    """Extract (r, g, b) 0-1 floats from 0x00RRGGBB int."""
    r = ((packed >> 16) & 0xFF) / 255.0
    g = ((packed >>  8) & 0xFF) / 255.0
    b = ((packed      ) & 0xFF) / 255.0
    return r, g, b


def _sonar_colour(distance: float):
    """Return (r, g, b) interpolated green→red over SONAR_FAR→SONAR_NEAR."""
    t = max(0.0, min(1.0, (SONAR_FAR - distance) / (SONAR_FAR - SONAR_NEAR)))
    return (t, 1.0 - t, 0.0)


class OttoVisualizer(Node):

    def __init__(self):
        super().__init__('otto_visualizer')

        self._led_colours = [(1.0, 1.0, 1.0)] * NUM_LEDS   # default white
        self._sonar_distance = SONAR_FAR

        self._pub_leds  = self.create_publisher(MarkerArray, '/otto/led_markers', 10)
        self._pub_sonar = self.create_publisher(Marker,      '/otto/sonar_marker', 10)

        self.create_subscription(UInt32MultiArray, '/otto/leds',
                                 self._leds_cb, 10)
        self.create_subscription(Range, '/otto/ultrasonic/range',
                                 self._range_cb, 10)

        # Publish at 10 Hz even without new data so RViz doesn't time-out markers
        self.create_timer(0.1, self._publish)

    # ── Callbacks ────────────────────────────────────────────────────────

    def _leds_cb(self, msg: UInt32MultiArray):
        for i, packed in enumerate(msg.data[:NUM_LEDS]):
            self._led_colours[i] = _rgb_from_uint32(packed)

    def _range_cb(self, msg: Range):
        self._sonar_distance = msg.range

    # ── Publish ──────────────────────────────────────────────────────────

    def _publish(self):
        now = self.get_clock().now().to_msg()
        lifetime = Duration(seconds=0.5).to_msg()

        # ── LED MarkerArray ───────────────────────────────────────────
        array = MarkerArray()
        for i in range(NUM_LEDS):
            m = Marker()
            m.header.stamp    = now
            m.header.frame_id = f'led_{i}_link'
            m.ns              = 'otto_leds'
            m.id              = i
            m.type            = Marker.SPHERE
            m.action          = Marker.ADD
            m.lifetime        = lifetime
            m.scale.x = m.scale.y = m.scale.z = 0.006   # 6 mm diameter
            r, g, b = self._led_colours[i]
            m.color.r = r
            m.color.g = g
            m.color.b = b
            m.color.a = 1.0
            # Sphere centred at the frame origin
            m.pose.orientation.w = 1.0
            array.markers.append(m)
        self._pub_leds.publish(array)

        # ── Sonar Marker ──────────────────────────────────────────────
        m = Marker()
        m.header.stamp    = now
        m.header.frame_id = 'ultrasonic_sensor_frame'
        m.ns              = 'otto_sonar'
        m.id              = 0
        m.type            = Marker.SPHERE
        m.action          = Marker.ADD
        m.lifetime        = lifetime
        # Sphere placed at the measured distance along X (forward)
        d = max(SONAR_NEAR, min(SONAR_FAR, self._sonar_distance))
        m.pose.position.x = d
        m.pose.orientation.w = 1.0
        m.scale.x = m.scale.y = m.scale.z = 0.020   # 20 mm indicator sphere
        r, g, b = _sonar_colour(d)
        m.color.r = r
        m.color.g = g
        m.color.b = b
        m.color.a = 0.8
        self._pub_sonar.publish(m)


def main(args=None):
    rclpy.init(args=args)
    node = OttoVisualizer()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
