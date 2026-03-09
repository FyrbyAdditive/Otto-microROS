#!/usr/bin/python3
"""Kinematic calibration for Otto Starter (differential drive, no encoders).

Watches /cmd_vel from teleop and integrates predicted motion.  You drive the
robot with teleop, then measure actual motion.  The script outputs corrected
firmware constants to paste into otto_config.h and otto_odom_publisher.py.

  STRAIGHT-LINE TEST  →  calibrates SERVO_SPEED_SCALE (servo gain, not wheel size)
  SPIN TEST           →  calibrates WHEEL_BASE

Usage:
    /usr/bin/python3 scripts/calibrate_kinematics.py

Teleop (separate terminal):
    ros2 launch otto_bringup otto_teleop.launch.py
"""

import math
import threading
import time
import rclpy
import rclpy.executors
from rclpy.node import Node
from geometry_msgs.msg import Twist

# ── Current firmware constants ────────────────────────────────────────────────
SERVO_SPEED_SCALE_CURRENT = 2500.0   # firmware/src/otto_config.h
WHEEL_BASE_CURRENT        = 0.0814   # m  — otto_config.h + otto_odom_publisher.py
CMD_VEL_TIMEOUT           = 0.5      # s  — matches firmware CMD_VEL_TIMEOUT_MS


class CalibrationNode(Node):
    def __init__(self):
        super().__init__('otto_calibration')
        self._sub = self.create_subscription(Twist, '/cmd_vel', self._cmd_vel_cb, 10)
        self._lock = threading.Lock()
        self._last_stamp     = None
        self._last_linear_x  = 0.0
        self._last_angular_z = 0.0
        self._predicted_heading  = 0.0   # rad
        self._predicted_distance = 0.0   # m
        self._draining = True            # discard messages until settled after reset

    def _reset(self):
        with self._lock:
            self._predicted_heading  = 0.0
            self._predicted_distance = 0.0
            self._last_stamp     = None
            self._last_linear_x  = 0.0
            self._last_angular_z = 0.0
            self._draining = True

    def _cmd_vel_cb(self, msg: Twist):
        now = self.get_clock().now()
        with self._lock:
            if self._draining:
                return
            if self._last_stamp is not None:
                dt = (now - self._last_stamp).nanoseconds * 1e-9
                if 0.0 < dt < 1.0:          # cap dt: ignore pauses > 1s
                    # Integrate the PREVIOUS velocity over the elapsed time (zero-order hold).
                    # msg contains the NEW velocity — integrating it would lose the last interval.
                    self._predicted_heading  += self._last_angular_z * dt
                    self._predicted_distance += self._last_linear_x  * dt
            self._last_stamp     = now
            self._last_linear_x  = msg.linear.x
            self._last_angular_z = msg.angular.z

    def snapshot(self):
        """Return accumulated totals plus any pending unintegrated velocity.

        The last stored velocity may not yet have been integrated (no subsequent
        message arrived to trigger it).  Add it now, capped at CMD_VEL_TIMEOUT —
        the firmware stops the robot that long after the last cmd_vel anyway.
        """
        now = self.get_clock().now()
        with self._lock:
            h = self._predicted_heading
            d = self._predicted_distance
            if self._last_stamp is not None and not self._draining:
                elapsed = (now - self._last_stamp).nanoseconds * 1e-9
                dt = min(elapsed, CMD_VEL_TIMEOUT)
                if dt > 0.0:
                    h += self._last_angular_z * dt
                    d += self._last_linear_x  * dt
            return h, d

    # ── Live display (runs in its own thread while user is driving) ───────────
    def _live_display(self, stop_evt: threading.Event):
        while not stop_evt.is_set():
            h, d = self.snapshot()
            print(
                f"\r  Recording...  heading: {math.degrees(h):+7.1f} deg   "
                f"distance: {d*100:6.1f} cm   (press Enter when done)",
                end='', flush=True)
            stop_evt.wait(0.2)
        print()  # newline after Enter

    def _record(self):
        """Countdown → drain → record → press Enter when done.
        Switch to the teleop window during the countdown."""
        self._reset()               # draining=True, zero state
        for n in (3, 2, 1):
            print(f"\r  Switch to teleop window now...  {n}", end='', flush=True)
            time.sleep(1.0)
        print("\r  GO!  Drive now.                       ")
        with self._lock:
            self._draining = False  # open gate; _last_stamp stays None
        stop_evt = threading.Event()
        disp = threading.Thread(target=self._live_display, args=(stop_evt,), daemon=True)
        disp.start()
        input()                     # user switches back here and presses Enter when done
        stop_evt.set()
        disp.join()
        return self.snapshot()

    # ── SPIN TEST — calibrates WHEEL_BASE ────────────────────────────────────
    def run_spin_test(self):
        print(f"\n{'='*60}")
        print("SPIN TEST — calibrates WHEEL_BASE")
        print(f"{'='*60}")
        print()
        print("  STEP 1: Mark the robot's starting heading (sticky note on the floor).")
        print("  STEP 2: When GO appears, switch to teleop and spin with 'a' or 'd'.")
        print("          Count full 360-degree rotations (3+ = more accurate).")
        print("  STEP 3: Stop the robot, switch back HERE, press Enter.")
        print("  STEP 4: Type how many full rotations you counted.")
        print()
        heading_rad, _ = self._record()

        pred_deg = math.degrees(heading_rad)
        if abs(pred_deg) < 5.0:
            print("  Less than 5 degrees recorded — did you spin?  Skipping.")
            return

        print(f"  Recorded rotation: {pred_deg:.1f} deg")
        print()
        print("  Enter actual rotation.  Two formats accepted:")
        print("    • Full rotations, e.g.  3   (= 3 × 360 deg = 1080 deg)")
        print("    • Degrees, e.g.         1080")
        actual_str = input("  Actual rotation (rotations or degrees, positive=CCW): ").strip()
        if not actual_str:
            print("  Skipped.")
            return

        val = float(actual_str)
        # If |val| <= 20, treat as full rotations
        actual_deg = val * 360.0 if abs(val) <= 20.0 else val
        if abs(actual_deg) < 1.0:
            print("  Too small — skipping.")
            return

        ratio = actual_deg / pred_deg
        new_wheel_base = WHEEL_BASE_CURRENT / ratio
        print()
        print(f"  Recorded: {pred_deg:.1f} deg   Actual: {actual_deg:.1f} deg")
        print(f"  Correction factor: {ratio:.4f}")
        print(f"  New WHEEL_BASE = {new_wheel_base:.4f} m  (was {WHEEL_BASE_CURRENT:.4f} m)")
        print()
        print("  Update in firmware/src/otto_config.h:")
        print(f"    #define WHEEL_BASE  {new_wheel_base:.4f}  // ~{new_wheel_base*1000:.1f}mm")
        print("  Update in ros2_ws/src/otto_bringup/scripts/otto_odom_publisher.py:")
        print(f"    WHEEL_BASE = {new_wheel_base:.4f}")

    # ── STRAIGHT-LINE TEST — calibrates SERVO_SPEED_SCALE ───────────────────
    def run_straight_test(self):
        print(f"\n{'='*60}")
        print("STRAIGHT-LINE TEST — calibrates SERVO_SPEED_SCALE")
        print(f"{'='*60}")
        print()
        print("  STEP 1: Put a mark on the floor at the robot's wheel axle (tape or coin).")
        print("  STEP 2: When GO appears, switch to teleop and drive straight with 'w'.")
        print("          20-30 cm is enough.  Do NOT turn.")
        print("  STEP 3: Stop the robot, switch back HERE, press Enter.")
        print("  STEP 4: Measure from the floor mark to the axle and type it in (cm).")
        print()
        _, dist_m = self._record()

        pred_m = dist_m
        if abs(pred_m) < 0.01:
            print("  Less than 1 cm recorded — did you drive forward?  Skipping.")
            return

        print(f"  Recorded distance: {pred_m*100:.1f} cm")
        print()
        actual_str = input("  Actual distance measured (cm): ").strip()
        if not actual_str:
            print("  Skipped.")
            return

        actual_m = float(actual_str) / 100.0
        if actual_m < 0.005:
            print("  Too small — skipping.")
            return

        # Robot goes less than commanded → servo gain too low → scale up.
        # new_scale = old_scale / ratio   (ratio < 1 → scale increases)
        ratio = actual_m / pred_m
        new_scale = SERVO_SPEED_SCALE_CURRENT / ratio

        # Warn if this saturates at typical teleop speed (0.2 m/s → offset > 500us)
        max_offset = 0.2 * new_scale
        if max_offset > 480:
            print(f"  Note: at 0.2 m/s the offset would be {max_offset:.0f} us "
                  f"(servo saturates at 500 us).")
            print(f"  Effective max speed ~{480.0/new_scale*100:.0f} cm/s.")

        print()
        print(f"  Recorded: {pred_m*100:.1f} cm   Actual: {actual_m*100:.1f} cm")
        print(f"  Correction factor: {ratio:.4f}")
        print(f"  New SERVO_SPEED_SCALE = {new_scale:.1f}  (was {SERVO_SPEED_SCALE_CURRENT:.1f})")
        print()
        print("  Update in firmware/src/otto_config.h:")
        print(f"    #define SERVO_SPEED_SCALE  {new_scale:.1f}")
        print("  (No change needed in otto_odom_publisher.py — wheel size is correct.)")


def main():
    rclpy.init()
    node = CalibrationNode()
    executor = rclpy.executors.SingleThreadedExecutor()
    executor.add_node(node)
    spin_thread = threading.Thread(target=executor.spin, daemon=True)
    spin_thread.start()

    print()
    print("Otto Kinematic Calibration")
    print()
    print("  BEFORE YOU START:")
    print("  - Teleop must already be open: ros2 launch otto_bringup otto_teleop.launch.py")
    print("  - You need TWO terminal windows side by side: this one and the teleop window.")
    print()
    print("  HOW EACH TEST WORKS:")
    print("  1. You choose a test and read the instructions here.")
    print("  2. A 3-second countdown starts — switch to the teleop window during it.")
    print("  3. When GO appears, drive the robot using the teleop keys.")
    print("  4. When done driving, switch BACK to this window and press Enter.")
    print("  5. Measure the actual distance or angle and type it in.")
    print("  6. This script prints the corrected value to paste into the firmware.")
    print()
    print("  TESTS:")
    print("  1. Straight-line  — drive forward, measures SERVO_SPEED_SCALE")
    print("  2. Spin           — spin in place, measures WHEEL_BASE")
    print("  3. Both")
    choice = input("Run which test? [1/2/3, default=3]: ").strip() or '3'

    try:
        if choice in ('1', '3'):
            node.run_straight_test()
        if choice in ('2', '3'):
            node.run_spin_test()
    except KeyboardInterrupt:
        print("\nAborted.")

    executor.shutdown(timeout_sec=1.0)
    node.destroy_node()
    rclpy.shutdown()
    print("\nDone.")


if __name__ == '__main__':
    main()
