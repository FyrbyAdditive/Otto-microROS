# HP Robots Otto — micro-ROS Project

![ROS2 Jazzy](https://img.shields.io/badge/ROS2-Jazzy-blue)
![PlatformIO](https://img.shields.io/badge/PlatformIO-ESP32-orange)
![License: MIT](https://img.shields.io/badge/License-MIT-green)

Control your HP Robots Otto Starter Kit with ROS2! This cute little educational robot runs micro-ROS firmware on its ESP32 and connects wirelessly to your computer so you can drive it with keyboard controls, visualise its sensors in 3D, and even build maps of the room.

## Features

- **Wireless control** — drive the robot from your keyboard over WiFi
- **3D visualisation** — see the robot, sensors, and LED ring live in RViz
- **Ultrasonic mapping** — build occupancy grid maps with slam_toolbox
- **Dead-reckoning odometry** — track position from cmd_vel with servo clamping model
- **LED ring animations** — idle status, direction-of-travel, and proximity warnings
- **Buzzer and sensors** — battery voltage, IR line sensors, ultrasonic range
- **Calibration tools** — servo speed and line sensor calibration with live feedback
- **Docker support** — run the ROS2 stack in containers without a native install

## Architecture

```
┌─────────────────────────────────────────────────────────────────┐
│  Your Computer                                                  │
│                                                                 │
│  ┌──────────────┐    ┌──────────────────────────────────────┐   │
│  │ micro-ROS    │    │  ROS2 Stack                          │   │
│  │ Agent        │◄──►│  robot_state_publisher (URDF → TF)   │   │
│  │ (UDP bridge) │    │  otto_odom_publisher (dead reckoning)│   │
│  └──────┬───────┘    │  ultrasonic_to_laserscan             │   │
│         │ UDP:8888   │  otto_visualizer (RViz markers)      │   │
│         │            │  slam_toolbox (mapping, optional)    │   │
│         │            └──────────────────────────────────────┘   │
│         │                                                       │
└─────────┼───────────────────────────────────────────────────────┘
          │ WiFi
┌─────────┴───────────────────────────────────────────────────────┐
│  Otto Robot (ESP32)                                             │
│  micro-ROS node "otto_starter"                                  │
│                                                                 │
│  Publishers: /ultrasonic/range, /line_sensors, /battery_state   │
│  Subscribers: /cmd_vel, /leds, /buzzer                          │
└─────────────────────────────────────────────────────────────────┘
```

## Quick Links

| Document | Description |
|----------|-------------|
| [docs/flashing.md](docs/flashing.md) | Firmware flashing and WiFi setup |
| [docs/building.md](docs/building.md) | Building firmware from source |
| [docs/wiring.md](docs/wiring.md) | GPIO pin reference and connector map |
| [docs/mapping_demo.md](docs/mapping_demo.md) | SLAM mapping walkthrough |
| [docs/hardware.md](docs/hardware.md) | CAD source files and mesh export |
| [docs/calibration.md](docs/calibration.md) | Servo and line sensor calibration |
| [docs/troubleshooting.md](docs/troubleshooting.md) | Common issues and fixes |
| [docs/docker.md](docs/docker.md) | Running the stack with Docker |

---

## What you need

**Hardware**
- HP Robots Otto Starter Kit (wheeled variant)
- USB-C data cable (for flashing firmware)
- 2.4 GHz WiFi network

**Computer**
- Ubuntu 24.04 with [ROS2 Jazzy](https://docs.ros.org/en/jazzy/Installation.html) installed
- Python 3 with `esptool` (`pip install esptool`) for flashing firmware

> **Alternatively**, you can run the ROS2 stack in Docker — see [docs/docker.md](docs/docker.md).

---

## Step 1: Set up the software

Run this once after cloning the project. It installs all dependencies and builds the workspace.

```bash
bash setup.sh
```

Open a new terminal when it finishes (so the shell picks up the new settings).

---

## Step 2: Flash the robot

Download the latest `firmware.bin` from the [GitHub Releases](https://github.com/FyrbyAdditive/Otto-microROS/releases) page, then flash it:

```bash
pip install esptool  # if you don't have it
esptool.py --port /dev/ttyUSB0 --baud 460800 \
  write_flash 0x10000 firmware.bin
```

See [docs/flashing.md](docs/flashing.md) for detailed instructions and troubleshooting.

---

## Step 3: Configure WiFi

On first boot, the robot enters **setup mode** — the LED ring pulses magenta and a WiFi network named **Otto-XXXX** appears.

1. Connect to the **Otto-XXXX** network from your phone or laptop
2. A captive portal opens — select your WiFi network, enter the password, and set the micro-ROS agent IP and port
3. Click **Save & Connect** — the robot reboots and connects to your WiFi

To find your computer's IP address: `hostname -I | awk '{print $1}'`

The configuration is saved to the robot's flash and persists across reboots. To re-configure, erase and re-flash.

---

## Step 4: Start the robot stack

Make sure the robot is powered on, then run:

```bash
./start.sh
```

This starts the micro-ROS agent (the WiFi bridge) and the robot stack. When the robot connects, the blue LED goes solid and the eye ring glows blue.

---

## Step 5: Drive the robot

Open a new terminal and run:

```bash
ros2 launch otto_bringup otto_teleop.launch.py
```

An `xterm` window opens with keyboard controls:

| Key | Action |
|-----|--------|
| `i` | Forward |
| `,` | Backward |
| `j` | Turn left |
| `l` | Turn right |
| `u` / `o` | Forward-left / Forward-right |
| `n` / `.` | Backward-right / Backward-left |
| `k` | Stop |

The eye ring shows the direction of travel in purple. Press `Ctrl+C` to stop teleop.

---

## Visualise in RViz

```bash
ros2 launch otto_description display.launch.py
```

Shows the 3D robot model, sensor frames, and ultrasonic range cone.

---

## Mapping demo

With the robot stack already running (`./start.sh`), launch the mapping demo in a new terminal:

```bash
ros2 launch otto_bringup otto_mapping_demo.launch.py
```

See [docs/mapping_demo.md](docs/mapping_demo.md) for a step-by-step walkthrough.

---

## ROS2 topics

| Topic | Type | Direction | Rate | Description |
|-------|------|-----------|------|-------------|
| `/cmd_vel` | geometry_msgs/Twist | Subscribe | — | Drive commands |
| `/ultrasonic/range` | sensor_msgs/Range | Publish | 10Hz | Distance, 0.02–4m |
| `/line_sensors` | std_msgs/Int32MultiArray | Publish | 20Hz | IR line sensor ADC values |
| `/battery_state` | sensor_msgs/BatteryState | Publish | 1Hz | Voltage and percentage |
| `/leds` | std_msgs/UInt8MultiArray | Subscribe | — | LED colour commands |
| `/buzzer` | std_msgs/UInt16 | Subscribe | — | Tone frequency (0=off) |
| `/odom` | nav_msgs/Odometry | Publish | 50Hz | Dead-reckoned position |
| `/scan` | sensor_msgs/LaserScan | Publish | 10Hz | Ultrasonic as laser scan |

### Control LEDs manually

```bash
# Set eye ring to red
ros2 topic pub --once /leds std_msgs/msg/UInt8MultiArray "{data: [0, 0, 255, 0, 0]}"

# Turn all LEDs off
ros2 topic pub --once /leds std_msgs/msg/UInt8MultiArray "{data: [2, 2]}"
```

### Play buzzer tones

```bash
ros2 topic pub --once /buzzer std_msgs/msg/UInt16 "{data: 440}"   # 440Hz
ros2 topic pub --once /buzzer std_msgs/msg/UInt16 "{data: 0}"     # silence
```

---

## Project structure

```
├── firmware/                   # ESP32 micro-ROS firmware (PlatformIO)
│   ├── platformio.ini
│   ├── scripts/
│   │   └── compress_html.py    # Build script: HTML → gzipped C header
│   └── src/
│       ├── main.cpp            # micro-ROS state machine
│       ├── otto_config.h       # GPIO pins and constants
│       ├── wifi_config.*       # NVS-backed WiFi/agent settings
│       ├── wifi_portal.*       # Captive portal (Soft-AP setup)
│       ├── portal_page.html    # Config page UI source
│       ├── drive_controller.*  # Differential drive
│       ├── led_controller.*    # NeoPixel LEDs
│       ├── led_ring_status.*   # Eye ring idle/direction animation
│       ├── ultrasonic_sensor.* # Range publisher
│       ├── line_sensor.*       # IR line sensor publisher
│       ├── battery_monitor.*   # Battery publisher
│       └── buzzer.*            # Tone subscriber
├── ros2_ws/
│   └── src/
│       ├── otto_description/   # URDF, meshes, RViz config
│       └── otto_bringup/       # Launch files, odom, scan converter
├── docker/                     # Docker setup (alternative to native install)
│   ├── Dockerfile
│   ├── docker-compose.yml
│   ├── entrypoint.sh
│   └── fastdds.xml
├── hardware/                   # CAD source (STEP file)
├── scripts/                    # Utility scripts
│   ├── calibrate_kinematics.py # Servo speed calibration
│   └── export_meshes.py        # STEP → STL mesh export
├── setup.sh                    # One-time dependency installer
├── start.sh                    # Start agent + robot stack
└── docs/                       # Documentation
    ├── flashing.md             # Firmware flashing and WiFi setup
    ├── building.md             # Building firmware from source
    ├── wiring.md               # GPIO pin reference
    ├── hardware.md             # CAD source files and mesh export
    ├── mapping_demo.md         # SLAM walkthrough
    ├── calibration.md          # Servo and line sensor calibration
    ├── docker.md               # Docker usage guide
    └── troubleshooting.md      # Common issues
```

---

## Serial bus servo mode

To use Feetech/Waveshare STS/SCS serial bus servos instead of the stock PWM servos, you'll need to [build the firmware from source](docs/building.md) with the `SERVO_TYPE_SERIAL_BUS=1` build flag. Pre-built releases only support the stock PWM servos.

This uses GPIO 16/17 (Connector 2) for the servo UART. The ultrasonic sensor remains available. See [docs/wiring.md](docs/wiring.md) for details.

---

## Hardware

- **MCU**: ESP32-WROOM-32E-N8 (8MB flash)
- **Servos**: 2× continuous rotation PWM (GPIO 14 left, GPIO 13 right)
- **Ultrasonic**: RCWL-9610 on GPIO 18/19
- **Line sensors**: 2× IR reflectance (GPIO 32 left, GPIO 33 right) — detect dark lines on light surfaces
- **LEDs**: 13× WS2812B eye ring (GPIO 4), 6× on ultrasonic housing (GPIO 18)
- **Buzzer**: GPIO 25 (passive, PWM)
- **Battery**: 3.7V 1800mAh LiPo, ADC on GPIO 39

---

## Troubleshooting

See [docs/troubleshooting.md](docs/troubleshooting.md).

---

## License

[MIT](LICENSE.md) — Copyright (c) 2026 Timothy Ellis, Fyrby Additive Manufacturing & Engineering

---

<p align="center">
  <br>
  <img src="docs/images/fame-logo.letterbox.png" alt="FAME logo" width="200">
</p>
