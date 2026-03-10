# Troubleshooting

Common issues and fixes for the Otto Starter micro-ROS project.

---

## WiFi / Connection

| Problem | Fix |
|---------|-----|
| WiFi won't connect | Verify SSID and password in `wifi_credentials.h`. The ESP32 only supports 2.4 GHz (not 5 GHz). Check serial monitor for status messages |
| Agent not reachable | Verify `AGENT_IP` in `wifi_credentials.h` matches your host IP. Ensure the agent is running (`./start.sh` or Docker). Check firewall: UDP port 8888 must be open. ESP32 and host must be on the same subnet |
| Frequent disconnections | Move closer to the WiFi AP. The firmware locks to the strongest AP at boot (BSSID lock) — power-cycle the robot to re-scan if you move it far away. Brief blips (< 6 s) are tolerated without disconnecting. Serial monitor shows `[Otto] Agent ping failed (N/3)` for transient losses — this is normal |

---

## Firmware Build

| Problem | Fix |
|---------|-----|
| First build takes > 15 min | Normal. micro_ros_platformio builds the entire micro-ROS static library. Subsequent builds are fast (~10 s) |
| `wifi_credentials.h` not found | Copy the template: `cp firmware/src/wifi_credentials.h.example firmware/src/wifi_credentials.h` and edit it |
| Build fails with micro-ROS errors | Try `pio run --target clean && pio run`. If that fails, delete `.pio/` and rebuild. Ensure `board_microros_distro` in `platformio.ini` is `jazzy` |

---

## Servos

| Problem | Fix |
|---------|-----|
| Servos don't move | Check wiring: Left = Connector 10 (GPIO 14), Right = Connector 11 (GPIO 13). Verify `cmd_vel` arrives: `ros2 topic echo /cmd_vel`. The firmware stops servos after 500 ms of no messages |
| Wrong speed or direction | Run `python3 scripts/calibrate_kinematics.py` (default scale: 3623.4). Max unsaturated speed is ~0.14 m/s. If a wheel goes backwards, the mirror inversion may need flipping |
| Servos jitter at stop | Normal for cheap continuous rotation servos near 1500 µs neutral. Adjust `SERVO_STOP_US` ±10 µs in [`otto_config.h`](../firmware/src/otto_config.h) |

---

## Sensors

| Problem | Fix |
|---------|-----|
| No ultrasonic readings | Check Connector 1 wiring (GPIO 18/19). Point at a flat surface 10–50 cm away for reliable test readings |
| IR sensors read 0 or 4095 | Check Connector 6 (left, GPIO 32) and Connector 7 (right, GPIO 33). Hold a white surface close, then dark — values should change. If stuck at 4095, check ADC attenuation |
| Battery voltage reads wrong | Calibrate `BATTERY_DIVIDER_RATIO` in [`otto_config.h`](../firmware/src/otto_config.h). Measure actual cell voltage with a multimeter, then: `actual / reported × current_ratio` |

---

## ROS2

| Problem | Fix |
|---------|-----|
| No topics visible | Ensure the agent is running and the ESP32 is connected (solid blue LED). Try: `ros2 daemon stop && ros2 daemon start` |
| RViz shows no robot model | Verify `robot_state_publisher` is running: `ros2 node list`. Check URDF: `ros2 param get /robot_state_publisher robot_description` |
| RViz shows white/untextured robot (Docker) | The `rviz` service must run alongside `bringup` or `mapping` — it does not start its own robot stack. See [docker.md](docker.md) |
