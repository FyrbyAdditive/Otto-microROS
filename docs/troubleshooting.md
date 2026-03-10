# Troubleshooting — Otto Starter micro-ROS

## WiFi / Connection Issues

**WiFi won't connect**
- Verify SSID and password in `wifi_credentials.h`
- The ESP32 only supports 2.4GHz networks (not 5GHz)
- Check serial monitor for WiFi status messages

**Agent not reachable**
- Verify `AGENT_IP` in `wifi_credentials.h` matches the host machine's IP
- Ensure the micro-ROS agent is running (check the Agent tab from `./start.sh`)
- Check firewall: UDP port 8888 must be open on the host
- Test connectivity: the ESP32 and host must be on the same subnet

**Frequent disconnections**
- Move the robot closer to the WiFi access point
- Check for 2.4GHz interference (microwaves, other devices)
- The firmware locks to the strongest AP at boot (BSSID lock) to prevent roaming — if you move the robot far from that AP, power-cycle it to re-scan
- Brief WiFi blips (< 6 seconds) are tolerated without disconnecting (consecutive ping failure counter)
- The firmware auto-reconnects on real disconnects; the blue LED blinks during reconnection
- Serial monitor shows `[Otto] Agent ping failed (N/3)` for transient losses — this is normal

## Firmware Build Issues

**First build takes forever (>15 min)**
- Normal. micro_ros_platformio builds the entire micro-ROS static library. Subsequent builds are fast.

**Build fails with "wifi_credentials.h not found"**
- Copy the template: `cp firmware/src/wifi_credentials.h.example firmware/src/wifi_credentials.h`
- Edit it with your WiFi credentials

**Build fails with micro-ROS errors**
- Try cleaning: `pio run --target clean` then `pio run`
- Delete `.pio/` and rebuild from scratch
- Ensure `board_microros_distro` in platformio.ini matches your agent version (jazzy)

## Servo Issues

**Servos don't move**
- Verify wiring: Left = Connector 10 (GPIO 14), Right = Connector 11 (GPIO 13)
- Check that cmd_vel messages are arriving: `ros2 topic echo /cmd_vel`
- The cmd_vel timeout stops servos after 500ms of no messages

**Servos spin at wrong speed or direction**
- Run `python3 scripts/calibrate_kinematics.py` for accurate calibration (default: 3623.4)
- If one wheel goes the wrong direction, the mirror inversion may need flipping
- Continuous rotation servos have no feedback — speed mapping is approximate
- At the default scale, max unsaturated speed is ~0.14 m/s

**Servos jitter at stop**
- Normal for cheap continuous rotation servos near the 1500µs neutral point
- Adjust `SERVO_STOP_US` ±10µs in `otto_config.h` to find true neutral

## Sensor Issues

**No ultrasonic readings**
- Check Connector 1 wiring (GPIO 18/19)
- Verify `SERVO_TYPE_SERIAL_BUS` is 0 in platformio.ini (serial bus mode disables ultrasonic)
- Point at a flat surface 10-50cm away for reliable test readings

**IR sensors read 0 or 4095**
- Check Connector 6 (left, GPIO 32) and Connector 7 (right, GPIO 33)
- Hold a white surface close, then a dark surface — values should change
- If stuck at 4095, check ADC attenuation setting

**Battery voltage reads wrong**
- Calibrate `BATTERY_DIVIDER_RATIO` in `otto_config.h`
- Measure actual cell voltage with a multimeter, compare to reported value
- Adjust ratio: `actual_voltage / reported_voltage * current_ratio`

## ROS2 Issues

**No topics visible**
- Ensure the agent is running and the ESP32 is connected (solid blue LED)
- Try: `ros2 daemon stop && ros2 daemon start`

**RViz shows no robot model**
- Verify robot_state_publisher is running: `ros2 node list`
- Check URDF loaded correctly: `ros2 param get /robot_state_publisher robot_description`
