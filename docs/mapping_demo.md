# Mapping Demo — Otto Starter Ultrasonic Mapping

This demo uses the single forward-facing RCWL-9610 ultrasonic sensor to build a basic 2D occupancy grid map using slam_toolbox.

## What to Expect

The Otto Starter has one ultrasonic beam (15-degree cone, 0.02-4m range) and no wheel encoders. Mapping quality is limited but demonstrates the full micro-ROS → ROS2 pipeline:

- Odometry is dead-reckoned from cmd_vel (drifts significantly)
- The ultrasonic provides one range reading per scan
- slam_toolbox builds a map incrementally as the robot moves

## Setup

### 1. Flash the firmware

Follow [flashing.md](flashing.md) to configure WiFi and flash the ESP32.

### 2. Power on Otto

Turn on the robot. The built-in blue LED will blink while searching for the micro-ROS agent.

### 3. Start the mapping stack

```bash
# Option A: Docker
docker compose -f docker/docker-compose.yml up

# Option B: Native
ros2 launch otto_bringup otto_mapping_demo.launch.py
```

The blue LED on Otto should go solid, indicating successful agent connection.

### 4. Verify topics

```bash
ros2 topic list
# Should see: /ultrasonic/range, /cmd_vel, /odom, /scan, /map, etc.

ros2 topic echo /ultrasonic/range --once
# Should show a range reading
```

### 5. Open a teleop terminal

```bash
ros2 launch otto_bringup otto_teleop.launch.py
# Or:
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

### 6. Map the environment

In RViz, you should see:
- The Otto robot model
- The ultrasonic range cone (green)
- A slowly building occupancy grid (map)

Drive the robot slowly:
1. **Rotate in place** to scan the surrounding area
2. **Drive forward** to a new position
3. **Rotate again** to add more range data
4. Repeat

The map builds gradually. Expect it to be rough — this is educational, not precision navigation.

## Troubleshooting

**No range readings**: Check ultrasonic wiring on Connector 1 (GPIO 18/19). Verify with `ros2 topic echo /ultrasonic/range`.

**Robot doesn't move**: Check servo wiring (Connector 10 = left, 11 = right). Verify cmd_vel arrives: `ros2 topic echo /cmd_vel`.

**Map is very noisy**: Expected with a single ultrasonic beam and no encoders. Try driving more slowly and making full 360-degree rotations at each position.

**SLAM won't start**: Verify `/scan` topic is publishing: `ros2 topic echo /scan`. Check that the `odom → base_footprint` TF is being published: `ros2 run tf2_ros tf2_echo odom base_footprint`.

**Agent disconnects**: Check WiFi signal strength. The ESP32 reconnects automatically when the agent becomes available again.
