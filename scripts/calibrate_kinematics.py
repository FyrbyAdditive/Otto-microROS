#!/usr/bin/python3
"""Kinematic calibration for Otto Starter (differential drive, no encoders).

Watches /cmd_vel from teleop and integrates predicted motion.  You drive the
robot with teleop, then measure actual distance.  The script outputs a
corrected SERVO_SPEED_SCALE to paste into otto_config.h.

WHEEL_BASE is a physical constant (from CAD) and is NOT calibrated here.

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
SERVO_SPEED_SCALE_CURRENT = 3623.4   # firmware/src/otto_config.h
CMD_VEL_TIMEOUT           = 0.5      # s  — matches firmware CMD_VEL_TIMEOUT_MS


class CalibrationNode(Node):
    def __init__(self):
        super().__init__('otto_calibration')
        self._sub = self.create_subscription(Twist, '/cmd_vel', self._cmd_vel_cb, 10)
        self._lock = threading.Lock()
        self._last_stamp     = None
        self._last_linear_x  = 0.0
        self._last_angular_z = 0.0
        self._predicted_distance = 0.0   # m
        self._draining = True            # discard messages until settled after reset
        self._msg_count  = 0             # messages received during current recording

    def _reset(self):
        with self._lock:
            self._predicted_distance = 0.0
            self._last_stamp     = None
            self._last_linear_x  = 0.0
            self._last_angular_z = 0.0
            self._draining = True
            self._msg_count  = 0

    def _cmd_vel_cb(self, msg: Twist):
        now = self.get_clock().now()
        with self._lock:
            if self._draining:
                return
            self._msg_count += 1
            if self._last_stamp is not None:
                dt = (now - self._last_stamp).nanoseconds * 1e-9
                if dt > 0.0:
                    # Integrate the PREVIOUS velocity, capped at CMD_VEL_TIMEOUT.
                    # The firmware stops the robot that long after each cmd_vel, so a gap
                    # longer than CMD_VEL_TIMEOUT (e.g. key presses spaced > 0.5 s apart)
                    # still only contributed CMD_VEL_TIMEOUT seconds of actual motion.
                    self._predicted_distance += self._last_linear_x * min(dt, CMD_VEL_TIMEOUT)
            self._last_stamp     = now
            self._last_linear_x  = msg.linear.x
            self._last_angular_z = msg.angular.z

    def snapshot(self):
        """Return (distance_m, msg_count) plus any pending velocity.

        The last stored velocity may not yet have been integrated (no subsequent
        message arrived to trigger it).  Add it now, capped at CMD_VEL_TIMEOUT —
        the firmware stops the robot that long after the last cmd_vel anyway.
        """
        now = self.get_clock().now()
        with self._lock:
            d = self._predicted_distance
            n = self._msg_count
            if self._last_stamp is not None and not self._draining:
                elapsed = (now - self._last_stamp).nanoseconds * 1e-9
                dt = min(elapsed, CMD_VEL_TIMEOUT)
                if dt > 0.0:
                    d += self._last_linear_x * dt
            return d, n

    # ── Live display (runs in its own thread while user is driving) ───────────
    def _live_display(self, stop_evt: threading.Event):
        while not stop_evt.is_set():
            d, n = self.snapshot()
            print(
                f"\r  Recording...  distance: {d*100:6.1f} cm   "
                f"msgs: {n:4d}   (press Enter when done)",
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

    # ── STRAIGHT-LINE TEST — calibrates SERVO_SPEED_SCALE ───────────────────
    def run_straight_test(self):
        print(f"\n{'='*60}")
        print("STRAIGHT-LINE TEST — calibrates SERVO_SPEED_SCALE")
        print(f"{'='*60}")
        print()
        print("  STEP 1: Put a mark on the floor at the robot's wheel axle (tape or coin).")
        print("  STEP 2: When GO appears, switch to teleop and HOLD 'i' to drive straight.")
        print("          20-30 cm is enough.  Do NOT turn.")
        print("  STEP 3: Release the key.  Switch back HERE and press Enter.")
        print("  STEP 4: Measure from the floor mark to the axle and type it in (cm).")
        print()
        dist_m, msg_count = self._record()
        if msg_count < 3:
            print(f"  WARNING: Only {msg_count} cmd_vel message(s) received.")
            print("  Hold the key down continuously while driving — do not tap.")
            print("  If msgs stayed at 0/1, teleop may not be publishing to /cmd_vel.")

        if abs(dist_m) < 0.01:
            print("  Less than 1 cm recorded — did you drive forward?  Skipping.")
            return

        print(f"  Recorded distance: {dist_m*100:.1f} cm")
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
        ratio = actual_m / dist_m
        new_scale = SERVO_SPEED_SCALE_CURRENT / ratio

        # Warn if this saturates at typical teleop speed (0.2 m/s → offset > 500us)
        max_offset = 0.2 * new_scale
        if max_offset > 480:
            print(f"  Note: at 0.2 m/s the offset would be {max_offset:.0f} us "
                  f"(servo saturates at 500 us).")
            print(f"  Effective max speed ~{480.0/new_scale*100:.0f} cm/s.")

        print()
        print(f"  Recorded: {dist_m*100:.1f} cm   Actual: {actual_m*100:.1f} cm")
        print(f"  Correction factor: {ratio:.4f}")
        print(f"  New SERVO_SPEED_SCALE = {new_scale:.1f}  (was {SERVO_SPEED_SCALE_CURRENT:.1f})")
        print()
        print("  Update in firmware/src/otto_config.h:")
        print(f"    #define SERVO_SPEED_SCALE  {new_scale:.1f}")
        print("  Update in ros2_ws/src/otto_bringup/scripts/otto_odom_publisher.py:")
        print(f"    SERVO_SPEED_SCALE = {new_scale:.1f}")


def main():
    rclpy.init()
    node = CalibrationNode()
    executor = rclpy.executors.SingleThreadedExecutor()
    executor.add_node(node)
    spin_thread = threading.Thread(target=executor.spin, daemon=True)
    spin_thread.start()

    print()
    print("Otto Kinematic Calibration — SERVO_SPEED_SCALE")
    print()
    print("  BEFORE YOU START:")
    print("  - Teleop must already be open: ros2 launch otto_bringup otto_teleop.launch.py")
    print("  - You need TWO terminal windows side by side: this one and the teleop window.")
    print()
    print("  HOW IT WORKS:")
    print("  1. A 3-second countdown starts — switch to the teleop window during it.")
    print("  2. When GO appears, HOLD 'i' to drive the robot straight forward ~20-30 cm.")
    print("  3. Release the key, switch BACK to this window, press Enter.")
    print("  4. Measure the actual distance and type it in.")
    print("  5. This script prints the corrected SERVO_SPEED_SCALE to paste into the firmware.")
    print()
    input("  Press Enter to begin...")

    try:
        node.run_straight_test()
    except KeyboardInterrupt:
        print("\nAborted.")

    executor.shutdown(timeout_sec=1.0)
    node.destroy_node()
    rclpy.shutdown()
    print("\nDone.")


if __name__ == '__main__':
    main()
