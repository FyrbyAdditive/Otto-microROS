# Mapping Demo — Otto Starter Ultrasonic Mapping

This demo uses the single forward-facing RCWL-9610 ultrasonic sensor to build a basic 2D occupancy grid map using slam_toolbox.

## What to Expect

The Otto Starter has one ultrasonic beam (15-degree cone, 0.02-4m range) and no wheel encoders. Mapping quality is limited but demonstrates the full micro-ROS → ROS2 pipeline:

- Odometry is dead-reckoned from cmd_vel (drifts over time)
- The ultrasonic reading is fanned across 11 rays spanning the sensor's FOV
- Scan matching is disabled (a single beam has no angular features to match)
- slam_toolbox inserts scans at odom-estimated poses to build the occupancy grid

## Setup

### 1. Flash the firmware

Follow [flashing.md](flashing.md) to configure WiFi and flash the ESP32.

### 2. Power on Otto

Turn on the robot. The built-in blue LED will blink while searching for the micro-ROS agent.

### 3. Start the mapping stack

```bash
ros2 launch otto_bringup otto_mapping_demo.launch.py
```

This launches the micro-ROS agent, robot stack, slam_toolbox, and RViz. The agent is sourced automatically from `~/microros_agent_ws/` (built by `setup.sh`).

The blue LED on Otto should go solid, indicating successful agent connection.

### 4. Open a teleop terminal

```bash
ros2 launch otto_bringup otto_teleop.launch.py
```

### 5. Verify topics

```bash
ros2 topic list
# Should see: /ultrasonic/range, /cmd_vel, /odom, /scan, /map, etc.

ros2 topic echo /ultrasonic/range --once
# Should show a range reading
```

### 6. Map the environment

In RViz, you should see:
- The Otto robot model (coloured, with LED ring markers)
- The ultrasonic range cone (green) and scan points (orange)
- A slowly building occupancy grid map (white = free, black = occupied, grey = unknown)

**Best mapping strategy:**
1. **Rotate in place** (hold `j` or `l`) to sweep the ultrasonic sensor and fill in a ring of map data around the robot
2. **Drive forward** slowly to a new position
3. **Rotate again** to add more range data
4. Repeat — each full rotation at a new position fills in more of the map

The map builds gradually. Expect it to be rough — this is educational, not precision navigation. Odometry drift means the map will become less accurate over longer runs.

## Troubleshooting

**No range readings**: Check ultrasonic wiring on Connector 1 (GPIO 18/19). Verify with `ros2 topic echo /ultrasonic/range`.

**Robot doesn't move**: Check servo wiring (Connector 10 = left, 11 = right). Verify cmd_vel arrives: `ros2 topic echo /cmd_vel`.

**No map appearing**: Check slam_toolbox is active: `ros2 lifecycle get /slam_toolbox` should show `active [3]`. If it shows `unconfigured`, restart the mapping demo. The robot must move at least 2 cm or turn 3° before the first scan is inserted.

**Map is very noisy**: Expected with a single ultrasonic beam and no encoders. Try driving more slowly and making full 360-degree rotations at each position.

**Robot flickers between orientations**: This was caused by scan matching fighting the odometry. Scan matching is now disabled by default. If you re-enabled it, set `use_scan_matching: false` in `ros2_ws/src/otto_bringup/config/slam_toolbox.yaml`.

**Agent disconnects**: Check WiFi signal strength. The ESP32 reconnects automatically. Brief dropouts (< 6 seconds) are tolerated without losing the session.
