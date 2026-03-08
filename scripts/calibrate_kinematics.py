#!/usr/bin/python3
"""Kinematic calibration for Otto Starter (differential drive, no encoders).

Runs two tests to measure actual WHEEL_BASE and WHEEL_RADIUS, then prints
corrected values to paste into otto_config.h and otto_odom_publisher.py.

Usage (robot must be running and reachable via ROS2):
    python3 scripts/calibrate_kinematics.py

Tests
-----
1. SPIN TEST — calibrates WHEEL_BASE (affects turning accuracy)
   Robot spins in place for a set duration.  The sim predicts the heading
   change; you measure the actual heading change with a protractor or tape
   marks on the floor.  Ratio gives corrected WHEEL_BASE.

2. STRAIGHT-LINE TEST — calibrates WHEEL_RADIUS (affects speed/distance)
   Robot drives straight for a set duration.  You measure the actual distance
   with a tape measure.  Ratio gives corrected WHEEL_RADIUS.

Run each test independently or together; press Enter to skip a test.
"""

import sys
import time
import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

# ── Current values (from otto_config.h / otto_odom_publisher.py) ──────────────
WHEEL_RADIUS_CURRENT = 0.0245   # m
WHEEL_BASE_CURRENT   = 0.0814   # m

# ── Test parameters ────────────────────────────────────────────────────────────
SPIN_ANGULAR_VEL  = 0.8              # rad/s (slow for accuracy)
SPIN_DURATION     = math.pi / SPIN_ANGULAR_VEL   # half-turn (180°) — needs less space

STRAIGHT_VEL      = 0.10         # m/s forward speed
STRAIGHT_DURATION = 3.0          # seconds  →  0.30 m predicted


class CalibrationNode(Node):
    def __init__(self):
        super().__init__('otto_calibration')
        self._pub = self.create_publisher(Twist, '/cmd_vel', 10)

    def _send(self, linear: float, angular: float):
        msg = Twist()
        msg.linear.x  = linear
        msg.angular.z = angular
        self._pub.publish(msg)

    def _stop(self):
        self._send(0.0, 0.0)

    def run_spin_test(self):
        predicted_deg = math.degrees(SPIN_ANGULAR_VEL * SPIN_DURATION)
        print(f"\n{'='*60}")
        print("SPIN TEST — calibrates WHEEL_BASE")
        print(f"{'='*60}")
        print(f"  Robot will spin in place for {SPIN_DURATION:.1f} s "
              f"at {SPIN_ANGULAR_VEL} rad/s.")
        print(f"  Predicted rotation: {predicted_deg:.1f}°")
        print()
        print("  Before starting:")
        print("    • Place the robot on a hard flat surface.")
        print("    • Put a tape mark on the floor on BOTH sides of the robot's")
        print("      wheel axle (left side and right side), aligned with the axle.")
        print("    • After the spin, measure the angle between the two pairs of")
        print("      marks (old vs new position of each side) — average for accuracy.")
        input("  Press Enter to start spinning…")

        print(f"\n  Spinning for {SPIN_DURATION:.1f} s…")
        t0 = time.time()
        while time.time() - t0 < SPIN_DURATION:
            self._send(0.0, SPIN_ANGULAR_VEL)
            time.sleep(0.05)
        self._stop()
        print("  Stopped.")

        print()
        print("  Measure the actual heading change (degrees) from your floor marks.")
        actual_str = input("  Actual rotation (degrees, positive = CCW): ").strip()
        if not actual_str:
            print("  Skipped.")
            return

        actual_deg = float(actual_str)
        if abs(actual_deg) < 1.0:
            print("  Measurement too small — skipping.")
            return

        ratio = predicted_deg / actual_deg
        new_wheel_base = WHEEL_BASE_CURRENT * ratio
        print()
        print(f"  Predicted: {predicted_deg:.1f}°   Actual: {actual_deg:.1f}°")
        print(f"  Correction factor: {ratio:.4f}")
        print(f"  ✓  New WHEEL_BASE = {new_wheel_base:.4f} m  "
              f"(was {WHEEL_BASE_CURRENT:.4f} m)")
        print()
        print("  Update in firmware/src/otto_config.h:")
        print(f"    #define WHEEL_BASE  {new_wheel_base:.4f}  // ~{new_wheel_base*1000:.1f}mm")
        print("  Update in ros2_ws/src/otto_bringup/scripts/otto_odom_publisher.py:")
        print(f"    WHEEL_BASE = {new_wheel_base:.4f}")

    def run_straight_test(self):
        predicted_m = STRAIGHT_VEL * STRAIGHT_DURATION
        print(f"\n{'='*60}")
        print("STRAIGHT-LINE TEST — calibrates WHEEL_RADIUS")
        print(f"{'='*60}")
        print(f"  Robot will drive straight for {STRAIGHT_DURATION:.0f} s "
              f"at {STRAIGHT_VEL} m/s.")
        print(f"  Predicted distance: {predicted_m*100:.0f} cm")
        print()
        print("  Before starting:")
        print("    • Place the robot on a flat surface.")
        print("    • Mark the starting position of a wheel axle.")
        input("  Press Enter to start driving…")

        print(f"\n  Driving for {STRAIGHT_DURATION:.0f} s…")
        t0 = time.time()
        while time.time() - t0 < STRAIGHT_DURATION:
            self._send(STRAIGHT_VEL, 0.0)
            time.sleep(0.05)
        self._stop()
        print("  Stopped.")

        print()
        print("  Measure the actual distance from the start mark to the axle.")
        actual_str = input("  Actual distance (cm): ").strip()
        if not actual_str:
            print("  Skipped.")
            return

        actual_m = float(actual_str) / 100.0
        if actual_m < 0.01:
            print("  Measurement too small — skipping.")
            return

        ratio = actual_m / predicted_m
        new_wheel_radius = WHEEL_RADIUS_CURRENT * ratio
        print()
        print(f"  Predicted: {predicted_m*100:.0f} cm   Actual: {actual_m*100:.1f} cm")
        print(f"  Correction factor: {ratio:.4f}")
        print(f"  ✓  New WHEEL_RADIUS = {new_wheel_radius:.4f} m  "
              f"(was {WHEEL_RADIUS_CURRENT:.4f} m)")
        print()
        print("  Update in firmware/src/otto_config.h:")
        print(f"    #define WHEEL_DIAMETER  {new_wheel_radius*2:.4f}  "
              f"// ~{new_wheel_radius*2000:.1f}mm")
        print("  Update in ros2_ws/src/otto_bringup/scripts/otto_odom_publisher.py:")
        print(f"    WHEEL_RADIUS = {new_wheel_radius:.4f}")


def main():
    rclpy.init()
    node = CalibrationNode()

    print()
    print("Otto Kinematic Calibration")
    print("Ensure the robot is powered on and the micro-ROS agent is running.")
    print("Press Enter to skip any test.")

    try:
        node.run_spin_test()
        node.run_straight_test()
    except KeyboardInterrupt:
        node._stop()
        print("\nAborted.")
    finally:
        node._stop()
        node.destroy_node()
        rclpy.shutdown()

    print("\nDone. Rebuild firmware and colcon build after updating values.")


if __name__ == '__main__':
    main()
