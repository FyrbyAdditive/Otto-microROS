# HP Robots Otto Starter — micro-ROS Project

micro-ROS firmware and ROS2 Jazzy support package for the HP Robots Otto Starter Kit (wheeled variant). The ESP32 runs micro-ROS over WiFi/UDP, bridging to a ROS2 host via the micro-ROS agent.

## Features

- **Firmware**: Single micro-ROS node publishing all sensor data and subscribing to actuator commands
- **Sensors**: Ultrasonic range (10Hz), IR line sensors (20Hz), battery state (1Hz)
- **Actuators**: Differential drive (cmd_vel), 19 NeoPixel LEDs, buzzer
- **Servo support**: Stock PWM servos (default) or Feetech/Waveshare serial bus servos (compile flag)
- **URDF**: Parameterized xacro supporting both wheeled and biped Otto variants
- **Mapping demo**: slam_toolbox with ultrasonic-to-laserscan conversion
- **Docker**: One-command bringup with docker compose

---

## Part 1: Flashing the Firmware

### Prerequisites

- [PlatformIO CLI](https://platformio.org/install/cli) or [VS Code + PlatformIO extension](https://platformio.org/install/ide?install=vscode)
- USB-C data cable

### 1.1 Configure WiFi

```bash
cp firmware/src/wifi_credentials.h.example firmware/src/wifi_credentials.h
```

Edit `firmware/src/wifi_credentials.h`:
```cpp
#define WIFI_SSID      "YourWiFiName"       // 2.4GHz only
#define WIFI_PASSWORD  "YourWiFiPassword"
#define AGENT_IP       "192.168.1.100"       // Your host machine's IP
#define AGENT_PORT     8888
```

Find your host IP: `ipconfig getifaddr en0` (macOS) or `hostname -I` (Linux).

### 1.2 Build and Flash

```bash
cd firmware
pio run -e otto_starter      # Build (first run: 5-15 min; subsequent: ~10 sec)
pio run -t upload             # Flash via USB-C
pio device monitor            # Verify — should print "WiFi connected, waiting for agent..."
```

The blue LED blinks while searching for the agent. See [docs/flashing.md](docs/flashing.md) for detailed troubleshooting.

---

## Part 2: Installing the ROS2 Integration

You have two options: **Docker** (easiest, recommended) or **native ROS2 Jazzy** installation.

### Option A: Docker (Recommended)

Docker handles all ROS2 dependencies automatically. You only need Docker installed.

**Install Docker:**
- **macOS**: [Docker Desktop](https://docs.docker.com/desktop/install/mac-install/)
- **Ubuntu**: `sudo apt install docker.io docker-compose-v2 && sudo usermod -aG docker $USER` (log out and back in)
- **Windows**: [Docker Desktop with WSL2](https://docs.docker.com/desktop/install/windows-install/)

**Start the full stack:**
```bash
# From the project root directory:
docker compose -f docker/docker-compose.yml up
```

This starts:
1. **micro-ROS agent** — UDP listener on port 8888, bridges ESP32 to ROS2
2. **ROS2 bringup** — Robot state publisher, odometry, scan converter

The Otto's blue LED should go solid within a few seconds, confirming connection.

**Useful Docker commands:**
```bash
# Start only the agent (lightweight, for development)
docker compose -f docker/docker-compose.yml up microros-agent

# Open an interactive ROS2 shell inside the container
docker compose -f docker/docker-compose.yml run ros2 bash

# Rebuild after changing ROS2 package code
docker compose -f docker/docker-compose.yml build ros2

# Stop everything
docker compose -f docker/docker-compose.yml down
```

### Option B: Native ROS2 Jazzy

For full control or if you already have ROS2 Jazzy installed.

**Step 1: Install ROS2 Jazzy** (if not already installed)

Follow the [official ROS2 Jazzy installation guide](https://docs.ros.org/en/jazzy/Installation.html) for your platform.

**Step 2: Install dependencies**

```bash
sudo apt install -y \
    ros-jazzy-micro-ros-agent \
    ros-jazzy-slam-toolbox \
    ros-jazzy-teleop-twist-keyboard \
    ros-jazzy-robot-state-publisher \
    ros-jazzy-joint-state-publisher \
    ros-jazzy-joint-state-publisher-gui \
    ros-jazzy-xacro \
    ros-jazzy-rviz2 \
    ros-jazzy-tf2-ros
```

**Step 3: Build the workspace**

```bash
cd ros2_ws
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install
source install/setup.bash
```

Add the source line to your shell profile to avoid repeating it:
```bash
echo "source /opt/ros/jazzy/setup.bash" >> ~/.bashrc
echo "source $(pwd)/install/setup.bash" >> ~/.bashrc
```

**Step 4: Start the micro-ROS agent**

In one terminal:
```bash
ros2 run micro_ros_agent micro_ros_agent udp4 --port 8888 -v4
```

The Otto's blue LED should go solid once connected.

**Step 5: Start the bringup**

In another terminal:
```bash
ros2 launch otto_bringup otto_microros.launch.py
```

This starts the robot state publisher (URDF → TF), dead-reckoning odometry, and ultrasonic-to-laserscan converter.

---

## Part 3: Using the Robot

### Verify Connection

```bash
# List all active topics
ros2 topic list

# Expected output:
#   /battery_state
#   /buzzer
#   /cmd_vel
#   /leds
#   /line_sensors
#   /odom
#   /scan
#   /ultrasonic/range
#   ...plus TF and parameter topics

# Check live sensor data
ros2 topic echo /ultrasonic/range --once
ros2 topic echo /battery_state --once
ros2 topic echo /line_sensors --once
```

### Drive the Robot

**Keyboard teleop:**
```bash
ros2 launch otto_bringup otto_teleop.launch.py
# Or directly:
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

Keys: `i`=forward, `,`=backward, `j`=turn left, `l`=turn right, `k`=stop.

Speed is limited to 0.1 m/s by default (safe for the small robot).

**Programmatic control:**
```bash
# Drive forward at 0.05 m/s
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.05}}"

# Rotate in place at 0.3 rad/s
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{angular: {z: 0.3}}"

# Stop
ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist "{}"
```

### Visualize in RViz

```bash
ros2 launch otto_description display.launch.py
# For the biped variant:
ros2 launch otto_description display.launch.py variant:=biped
```

Shows the robot model, TF frames, and ultrasonic range cone.

### Control LEDs

```bash
# Set all 13 eye LEDs to red (target=0 ring, mode=0 solid color, R=255 G=0 B=0)
ros2 topic pub --once /leds std_msgs/msg/UInt8MultiArray "{data: [0, 0, 255, 0, 0]}"

# Set ultrasonic LEDs to blue (target=1)
ros2 topic pub --once /leds std_msgs/msg/UInt8MultiArray "{data: [1, 0, 0, 0, 255]}"

# Turn all LEDs off (target=2 both, mode=2 off)
ros2 topic pub --once /leds std_msgs/msg/UInt8MultiArray "{data: [2, 2]}"
```

### Play Buzzer Tones

```bash
# Play 440Hz tone (A4 note)
ros2 topic pub --once /buzzer std_msgs/msg/UInt16 "{data: 440}"

# Play 880Hz tone (A5 note)
ros2 topic pub --once /buzzer std_msgs/msg/UInt16 "{data: 880}"

# Silence
ros2 topic pub --once /buzzer std_msgs/msg/UInt16 "{data: 0}"
```

### Run the Mapping Demo

```bash
ros2 launch otto_bringup otto_mapping_demo.launch.py
```

This starts everything (agent, bringup, slam_toolbox, RViz). Drive the robot around with teleop to build a map. See [docs/mapping_demo.md](docs/mapping_demo.md) for a detailed walkthrough.

---

## ROS2 Topics

| Topic | Type | Direction | Rate | Description |
|-------|------|-----------|------|-------------|
| `/cmd_vel` | geometry_msgs/Twist | Subscribe | — | Drive commands (linear.x, angular.z) |
| `/ultrasonic/range` | sensor_msgs/Range | Publish | 10Hz | Distance reading, 0.02-4m |
| `/line_sensors` | std_msgs/Int32MultiArray | Publish | 20Hz | [left_adc, right_adc], 0-4095 |
| `/battery_state` | sensor_msgs/BatteryState | Publish | 1Hz | Voltage and percentage |
| `/leds` | std_msgs/UInt8MultiArray | Subscribe | — | LED color commands |
| `/buzzer` | std_msgs/UInt16 | Subscribe | — | Tone frequency (0=off) |
| `/odom` | nav_msgs/Odometry | Publish | 50Hz | Dead-reckoned pose (host-side) |
| `/scan` | sensor_msgs/LaserScan | Publish | 10Hz | Converted from ultrasonic (host-side) |

## TF Frames

```
map (from slam_toolbox, when mapping)
 └── odom (from otto_odom_publisher)
      └── base_footprint (ground plane)
           └── base_link (body center at axle height)
                ├── head_link → ultrasonic_link
                ├── left_wheel_link (continuous joint)
                ├── right_wheel_link (continuous joint)
                ├── caster_link
                ├── ir_left_link
                ├── ir_right_link
                ├── led_ring_link
                └── imu_link
```

## Project Structure

```
├── firmware/                   # ESP32 micro-ROS firmware (PlatformIO)
│   ├── platformio.ini          # Build config (ESP32, Jazzy, 8MB flash)
│   ├── custom_meta.meta        # micro-ROS middleware sizing
│   └── src/
│       ├── main.cpp            # Entry point + micro-ROS state machine
│       ├── otto_config.h       # All GPIO pins and constants
│       ├── drive_controller.*  # Differential drive (cmd_vel → servos)
│       ├── ultrasonic_sensor.* # RCWL-9610 range publisher
│       ├── line_sensor.*       # IR ADC publisher
│       ├── battery_monitor.*   # LiPo voltage publisher
│       ├── led_controller.*    # NeoPixel subscriber
│       └── buzzer.*            # Tone subscriber
├── ros2_ws/
│   └── src/
│       ├── otto_description/   # URDF, STL meshes, RViz config, display launch
│       └── otto_bringup/       # Agent launch, odom, scan converter, SLAM config
├── docker/                     # Dockerfile.ros2, Dockerfile.firmware, compose
├── scripts/export_meshes.py    # STEP → STL mesh exporter (cadquery)
└── docs/
    ├── flashing.md             # Detailed firmware flashing guide
    ├── wiring.md               # GPIO pin reference
    ├── mapping_demo.md         # Step-by-step SLAM tutorial
    └── troubleshooting.md      # Common issues and fixes
```

## Serial Bus Servo Mode

To use Feetech/Waveshare STS/SCS serial bus servos instead of stock PWM servos, change the build flag in `platformio.ini`:

```ini
build_flags =
    -DSERVO_TYPE_SERIAL_BUS=1
```

This reconfigures GPIO 18/19 from ultrasonic sensor to servo UART. The ultrasonic sensor is disabled in this mode. See [docs/wiring.md](docs/wiring.md) for details.

## Hardware

- **MCU**: ESP32-WROOM-32E-N8 (8MB flash)
- **Servos**: 2x continuous rotation (GPIO 14 left, GPIO 13 right)
- **Ultrasonic**: RCWL-9610 on GPIO 18/19
- **IR Sensors**: Analog on GPIO 32/33
- **LEDs**: 13x WS2812B ring (GPIO 4), 6x on ultrasonic (GPIO 18)
- **Buzzer**: GPIO 25 (passive, PWM)
- **Battery**: 3.7V 1800mAh LiPo, ADC on GPIO 39

See [docs/wiring.md](docs/wiring.md) for the full connector pin reference.

## License

MIT
