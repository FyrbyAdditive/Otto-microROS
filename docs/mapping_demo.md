# Mapping Demo

Build a basic 2D occupancy grid map using the single forward-facing RCWL-9610 ultrasonic sensor and slam_toolbox.

---

## What to Expect

The Otto Starter has one ultrasonic beam (15° cone, 0.02–4 m range) and no wheel encoders. Mapping quality is limited but demonstrates the full micro-ROS to ROS2 pipeline:

- Odometry is dead-reckoned from `cmd_vel` (drifts over time)
- The ultrasonic reading is fanned across 11 rays spanning the sensor's FOV
- Scan matching is disabled (a single beam has no angular features to match)
- slam_toolbox inserts scans at odom-estimated poses to build the occupancy grid

---

## Running the Demo

### Native (ROS2 installed locally)

```bash
# Terminal 1: start the agent and robot stack
./start.sh

# Terminal 2: start the mapping demo (includes slam_toolbox + RViz)
ros2 launch otto_bringup otto_mapping_demo.launch.py

# Terminal 3: start teleop
ros2 launch otto_bringup otto_teleop.launch.py
```

### Docker

```bash
# Allow X11 access for RViz
xhost +local:docker

# Terminal 1: start agent, mapping stack, and RViz
docker compose -f docker/docker-compose.yml up agent mapping rviz

# Terminal 2: start teleop
docker compose -f docker/docker-compose.yml run teleop
```

The blue LED on Otto should go solid once the agent connects.

---

## Mapping Strategy

In RViz you should see the robot model, the ultrasonic range cone (green), scan points (orange), and a slowly building occupancy grid (white = free, black = occupied, grey = unknown).

**For best results:**

1. **Rotate in place** (hold `j` or `l`) to sweep the ultrasonic sensor and fill in a ring of map data around the robot
2. **Drive forward** slowly to a new position
3. **Rotate again** to add more range data
4. Repeat — each full rotation at a new position fills in more of the map

The map builds gradually. Expect it to be rough — this is educational, not precision navigation. Odometry drift means the map becomes less accurate over longer runs.

---

## Troubleshooting

| Problem | Cause / Fix |
|---------|-------------|
| No range readings | Check ultrasonic wiring on Connector 1 (GPIO 18/19). Verify with `ros2 topic echo /ultrasonic/range` |
| Robot doesn't move | Check servo wiring (Connector 10 = left, 11 = right). Verify: `ros2 topic echo /cmd_vel` |
| No map appearing | Check slam_toolbox is active: `ros2 lifecycle get /slam_toolbox` should show `active [3]`. If unconfigured, restart the demo. The robot must move at least 2 cm or turn ~3° before the first scan is inserted |
| Map is very noisy | Expected with a single ultrasonic beam and no encoders. Drive slowly and make full 360° rotations at each position |
| Robot flickers between orientations | Scan matching was re-enabled. Set `use_scan_matching: false` in [`slam_toolbox.yaml`](../ros2_ws/src/otto_bringup/config/slam_toolbox.yaml) |
| Agent disconnects | Check WiFi signal strength. The ESP32 reconnects automatically. Brief dropouts (< 6 s) are tolerated |
