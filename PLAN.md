# HP Robots Otto Starter — micro-ROS Project Plan

## Project Goal

Build a complete micro-ROS firmware and ROS2 support package for the **HP Robots Otto Starter Kit** (wheeled variant). The ESP32 runs micro-ROS over WiFi/UDP, bridging to a ROS2 host (DGX Spark or any Linux machine running the micro-ROS agent). The project includes an accurate URDF, a Docker-based build toolchain, host-side ROS2 launch files, and a demo that maps an environment using the onboard ultrasonic sensor.

---

## 1. Hardware Specification

### 1.1 MCU

- **Module**: ESP32-WROOM-32E-N4 (Xtensa dual-core LX6 @ 240MHz, 520KB SRAM, 4MB flash)
- **Custom PCB**: HP Robots proprietary board with integrated LiPo charger (3.7V 1800mAh), 5V boost, on/off switch, USB-C (programming + charging)
- **WiFi**: 2.4GHz 802.11 b/g/n (used for micro-ROS UDP transport)
- **Bluetooth**: v4.2 + BLE (not used in this project)

### 1.2 Connectors & GPIO Pin Map

The HP Otto PCB has 11 numbered JST connectors. Each exposes one signal GPIO plus VCC and GND. The default Starter Kit wiring is:

| Connector | GPIO | Type | Default Use | Notes |
|-----------|------|------|-------------|-------|
| 1 | 18 | Digital (NeoPixel) | Ultrasonic RGB LEDs (6× WS2812B) | NeoPixel data line |
| 1 | 19 | Digital | Ultrasonic trigger/echo (shared pin) | RCWL-9610 single-wire mode |
| 2 | 16 | Digital | Spare (alt ultrasonic NeoPixel) | |
| 2 | 17 | Digital | Spare (alt ultrasonic echo) | |
| 3 | — | — | Unknown/spare | Needs verification |
| 4 | — | — | Unknown/spare | Needs verification |
| 5 | 4 | Digital (NeoPixel) | 13× RGB LED ring (eyes) | WS2812B chain |
| 6 | 32 | ADC | Left IR line sensor (analog) | ADC1_CH4 |
| 7 | 33 | ADC | Right IR line sensor (analog) | ADC1_CH5 |
| 8 | 27 | PWM | Servo: right hip/angle | |
| 9 | 15 | PWM | Servo: left hip/angle | |
| 10 | 14 | PWM | Servo: left wheel | Starter Kit uses this |
| 11 | 13 | PWM | Servo: right wheel | Starter Kit uses this |

**On-board (no connector):**

| Function | GPIO | Notes |
|----------|------|-------|
| Built-in LED | 2 | Blue status LED |
| Buzzer | 25 | Passive electromagnetic, PWM driven |
| Battery voltage | 39 | ADC (input-only pin), voltage divider |

### 1.3 Sensors

**Ultrasonic Distance Sensor (Connector 1):**
- Chip: **RCWL-9610** (HC-SR04P compatible)
- Range: 2–400cm, ~15° beam angle
- Interface: Single-wire trigger/echo on GPIO 19 (shared pin mode)
- Additional: 6× WS2812B NeoPixel LEDs integrated into sensor PCB, data on GPIO 18
- Operating voltage: 3.3V (native to ESP32, no level shifting)

**IR Line-Following Sensors (Connectors 6 & 7):**
- Type: Analog IR reflectance sensors (likely TCRT5000-style)
- Left sensor: GPIO 32 (ADC)
- Right sensor: GPIO 33 (ADC)
- Output: Analog voltage proportional to reflectance (black = low, white = high)

**Battery Voltage Monitor:**
- GPIO 39 (ADC, input-only)
- Through voltage divider on PCB
- Used for low-battery warnings

### 1.4 Actuators

**Servos (Starter Kit — wheeled variant):**
- 2× continuous rotation "Tailor" servos with metal gears
- Left wheel: Connector 10, GPIO 14
- Right wheel: Connector 11, GPIO 13  
- PWM frequency: 50Hz
- Differential drive (tank steering)
- **Note**: Connectors 8 (GPIO 27) and 9 (GPIO 15) are available for future expansion (e.g., pan/tilt head). The walking Otto variant uses all 4 servo connectors.

**Buzzer:**
- GPIO 25, passive electromagnetic type, ~80dB
- PWM-driven for tones and melodies

**RGB LEDs:**
- 13× WS2812B ring on GPIO 4 (eyes/expression)
- 6× WS2812B on GPIO 18 (ultrasonic sensor housing)

### 1.5 Physical Dimensions (from STEP CAD model)

- **Overall assembled**: ~90 × 90 × 84mm (W × D × H per product spec)
- **Wheel-to-wheel width**: ~190mm (including tires/o-rings)
- **Wheel diameter**: ~49mm (from STEP bounding box of o-ring solids)
- **Wheelbase**: ~108mm (center-to-center, from servo bracket positions)
- **Ball caster**: Located rear-center, ~12mm diameter ball
- **Body structure**: Bottom plate → Middle section → Top (with lights) → FaceUS (ultrasonic mount)
- **STEP file parts**: Document (PCB), Bracket, Bottom, O-ring (×2 tires), Toplights, FaceUS, Ballcaster, Middle, Root assembly

---

## 2. Project Structure

```
otto_starter_microros/
├── README.md                          # Quick-start guide
├── LICENSE                            # Apache-2.0 or MIT
│
├── firmware/                          # ESP32 micro-ROS firmware
│   ├── platformio.ini                 # PlatformIO build config
│   ├── colcon.meta                    # micro-ROS middleware config
│   ├── src/
│   │   ├── main.cpp                   # Entry point, WiFi + micro-ROS init
│   │   ├── otto_config.h              # All GPIO pin definitions
│   │   ├── drive_controller.cpp/h     # Differential drive (cmd_vel → servo PWM)
│   │   ├── ultrasonic_sensor.cpp/h    # HC-SR04/RCWL-9610 range publisher
│   │   ├── line_sensor.cpp/h          # IR line sensor ADC publisher
│   │   ├── battery_monitor.cpp/h      # Battery voltage publisher
│   │   ├── buzzer.cpp/h               # Buzzer subscriber (play tones)
│   │   └── led_controller.cpp/h       # NeoPixel subscriber (LED commands)
│   └── lib/
│       └── (ESP32Servo, Adafruit_NeoPixel auto-installed by PlatformIO)
│
├── ros2_ws/                           # ROS2 host-side workspace
│   └── src/
│       ├── otto_description/          # URDF + meshes
│       │   ├── package.xml
│       │   ├── CMakeLists.txt
│       │   ├── urdf/
│       │   │   └── otto_starter.urdf.xacro
│       │   ├── meshes/                # Simplified STL meshes from STEP
│       │   │   ├── body.stl
│       │   │   ├── wheel_left.stl
│       │   │   ├── wheel_right.stl
│       │   │   └── caster_ball.stl
│       │   ├── launch/
│       │   │   ├── display.launch.py  # RViz visualization
│       │   │   └── description.launch.py
│       │   └── rviz/
│       │       └── otto.rviz
│       │
│       ├── otto_bringup/              # Launch files and config
│       │   ├── package.xml
│       │   ├── CMakeLists.txt
│       │   ├── launch/
│       │   │   ├── otto_microros.launch.py    # micro-ROS agent + transforms
│       │   │   ├── otto_mapping_demo.launch.py # Mapping demo
│       │   │   └── otto_teleop.launch.py       # Keyboard/joystick control
│       │   ├── config/
│       │   │   ├── micro_ros_agent.yaml
│       │   │   ├── twist_mux.yaml
│       │   │   └── slam_toolbox.yaml
│       │   └── scripts/
│       │       ├── ultrasonic_to_laserscan.py  # Convert Range → LaserScan
│       │       └── otto_odom_publisher.py      # Dead-reckoning odometry
│       │
│       └── otto_msgs/                 # Custom message definitions (if needed)
│           ├── package.xml
│           ├── CMakeLists.txt
│           └── msg/
│               └── OttoStatus.msg     # Battery, LED state, etc.
│
├── docker/                            # Containerized build environment
│   ├── Dockerfile.firmware            # ESP32 micro-ROS build
│   ├── Dockerfile.ros2               # ROS2 host workspace build
│   └── docker-compose.yml            # One-command build + agent launch
│
└── docs/
    ├── wiring.md                      # Pin reference with photos
    ├── flashing.md                    # How to flash firmware
    ├── troubleshooting.md
    └── mapping_demo.md               # Step-by-step mapping tutorial
```

---

## 3. Build Tasks (Ordered)

### Phase 1: Foundation

#### Task 1.1 — `otto_config.h` (GPIO Pin Definitions)

Create a single header with all hardware constants:
```cpp
// Pin definitions for HP Robots Otto Starter Kit
// Custom PCB with ESP32-WROOM-32E-N4

// Servos (differential drive)
#define PIN_SERVO_LEFT    14   // Connector 10
#define PIN_SERVO_RIGHT   13   // Connector 11

// Ultrasonic sensor (RCWL-9610, single-wire mode)
#define PIN_US_TRIG_ECHO  19   // Connector 1 signal
#define PIN_US_NEOPIXEL   18   // Connector 1 RGB (6 LEDs)

// IR line sensors (analog)
#define PIN_IR_LEFT       32   // Connector 6
#define PIN_IR_RIGHT      33   // Connector 7

// On-board peripherals
#define PIN_BUZZER        25
#define PIN_LED_BUILTIN    2
#define PIN_BATTERY_ADC   39   // Input-only

// LED ring (eyes)
#define PIN_LED_RING       4   // Connector 5, 13 WS2812B LEDs
#define LED_RING_COUNT    13
#define LED_US_COUNT       6

// Servo configuration
#define SERVO_FREQ        50   // Hz
#define SERVO_STOP_US    1500  // Microseconds for stop (continuous rotation)
#define SERVO_MIN_US     1000
#define SERVO_MAX_US     2000

// Physical dimensions (meters, for ROS)
#define WHEEL_DIAMETER    0.049
#define WHEEL_BASE        0.108
#define WHEEL_RADIUS      (WHEEL_DIAMETER / 2.0)

// WiFi (user configures via menuconfig or #define)
// #define WIFI_SSID      "your_ssid"
// #define WIFI_PASSWORD  "your_password"
// #define AGENT_IP       "192.168.1.100"
// #define AGENT_PORT     8888
```

#### Task 1.2 — PlatformIO Project Setup

Create `platformio.ini` targeting ESP32 with micro-ROS Arduino or ESP-IDF component:

```ini
[env:otto_starter]
platform = espressif32
board = esp32dev
framework = arduino
monitor_speed = 115200
lib_deps =
    https://github.com/micro-ROS/micro_ros_platformio
    madhephaestus/ESP32Servo@^3.0.0
    adafruit/Adafruit NeoPixel@^1.12.0
board_microros_transport = wifi
board_microros_distro = humble
build_flags =
    -DMICRO_ROS_TRANSPORT_ARDUINO_WIFI
```

Provide a `colcon.meta` that enables the required micro-ROS features:
- Publisher: `sensor_msgs/msg/Range` (ultrasonic)
- Publisher: `sensor_msgs/msg/BatteryState` (battery)
- Publisher: `std_msgs/msg/Int32MultiArray` (line sensors)
- Subscriber: `geometry_msgs/msg/Twist` (cmd_vel)
- Subscriber: `std_msgs/msg/UInt8MultiArray` (LED commands)
- Subscriber: `std_msgs/msg/UInt16` (buzzer frequency)

#### Task 1.3 — Docker Build Environment

`Dockerfile.firmware`:
```dockerfile
FROM espressif/idf:release-v5.2
# OR for PlatformIO approach:
FROM python:3.11-slim
RUN pip install platformio
# Pre-download toolchains and micro-ROS lib
```

`docker-compose.yml`:
```yaml
services:
  build-firmware:
    build:
      context: .
      dockerfile: docker/Dockerfile.firmware
    volumes:
      - ./firmware:/workspace
    command: pio run -e otto_starter

  microros-agent:
    image: microros/micro-ros-agent:humble
    network_mode: host
    command: udp4 --port 8888 -v6
```

---

### Phase 2: Firmware Nodes

#### Task 2.1 — `main.cpp` (WiFi + micro-ROS Lifecycle)

- Connect to WiFi using credentials from config
- Initialize micro-ROS transport (WiFi UDP)
- Create node: `otto_starter`
- Initialize all publishers and subscribers
- Main loop: spin micro-ROS executor, handle timing for sensor reads
- Implement reconnection logic if agent disconnects
- Watchdog: stop servos if no cmd_vel received for >500ms

#### Task 2.2 — `drive_controller.cpp` (Differential Drive)

- Subscribe to `cmd_vel` (`geometry_msgs/msg/Twist`)
- Convert `linear.x` and `angular.z` to left/right wheel velocities using differential drive kinematics:
  ```
  v_left  = linear.x - (angular.z * WHEEL_BASE / 2)
  v_right = linear.x + (angular.z * WHEEL_BASE / 2)
  ```
- Map velocities to servo microsecond values (1000–2000µs, 1500 = stop)
- **Important**: These are continuous rotation servos with NO encoder feedback. Speed control is open-loop. The mapping from cmd_vel to servo pulse width must be empirically calibrated.
- Safety: stop if no cmd_vel for 500ms (timeout)

#### Task 2.3 — `ultrasonic_sensor.cpp` (Range Publisher)

- Publish `sensor_msgs/msg/Range` on topic `/ultrasonic/range`
- Use single-wire mode for RCWL-9610 (GPIO 19):
  ```cpp
  // Trigger: drive pin HIGH for 10µs, then read echo pulse width
  pinMode(PIN_US_TRIG_ECHO, OUTPUT);
  digitalWrite(PIN_US_TRIG_ECHO, HIGH);
  delayMicroseconds(10);
  digitalWrite(PIN_US_TRIG_ECHO, LOW);
  pinMode(PIN_US_TRIG_ECHO, INPUT);
  duration = pulseIn(PIN_US_TRIG_ECHO, HIGH, 30000); // 30ms timeout
  distance_m = (duration * 0.000343) / 2.0;
  ```
- Range message fields:
  - `radiation_type = ULTRASOUND`
  - `field_of_view = 0.26` (15° in radians)
  - `min_range = 0.02`
  - `max_range = 4.0`
  - `range = distance_m`
- Publish rate: 10Hz (100ms between readings to avoid echo interference)
- frame_id: `ultrasonic_link`

#### Task 2.4 — `line_sensor.cpp` (IR Sensor Publisher)

- Publish `std_msgs/msg/Int32MultiArray` on `/line_sensors`
- Read ADC values from GPIO 32 (left) and GPIO 33 (right)
- Array data: `[left_raw, right_raw]` (12-bit ADC, 0–4095)
- Publish rate: 20Hz
- Optionally also publish as `sensor_msgs/msg/Illuminance` for each sensor

#### Task 2.5 — `battery_monitor.cpp`

- Publish `sensor_msgs/msg/BatteryState` on `/battery_state`
- Read GPIO 39 ADC, convert through voltage divider calibration
- LiPo cell: 3.0V (empty) to 4.2V (full)
- Publish rate: 1Hz (battery changes slowly)

#### Task 2.6 — `led_controller.cpp` + `buzzer.cpp`

- LED subscriber on `/leds` — accepts color commands for the 13-LED ring and 6 ultrasonic LEDs
- Buzzer subscriber on `/buzzer` — accepts frequency (0 = off)
- These are lower priority but complete the hardware abstraction

---

### Phase 3: URDF Model

#### Task 3.1 — `otto_starter.urdf.xacro`

Create an accurate URDF for the wheeled Otto Starter. Key details:

**Links:**
- `base_footprint` — ground plane reference
- `base_link` — robot body center (at axle height)
- `body_link` — the main plastic body shell (visual + collision)
- `left_wheel_link` — left wheel + tire (continuous joint)
- `right_wheel_link` — right wheel + tire (continuous joint)
- `caster_link` — rear ball caster (fixed)
- `ultrasonic_link` — ultrasonic sensor frame (front face)
- `ir_left_link` — left IR sensor (bottom, pointing down)
- `ir_right_link` — right IR sensor (bottom, pointing down)
- `imu_link` — placeholder for future IMU expansion

**Joints:**
- `base_footprint_to_base`: fixed, z-offset = wheel_radius
- `base_to_body`: fixed
- `base_to_left_wheel`: continuous, axis Y, origin at (-wheel_base/2, 0, 0)
- `base_to_right_wheel`: continuous, axis Y, origin at (+wheel_base/2, 0, 0)
- `base_to_caster`: fixed, rear offset
- `body_to_ultrasonic`: fixed, front face
- `body_to_ir_left`: fixed, bottom front-left
- `body_to_ir_right`: fixed, bottom front-right

**Dimensions to extract from STEP file** (Claude Code should use cadquery/OCP):
- Wheel diameter: ~49mm
- Wheel width: ~5.5mm (o-ring cross-section)
- Wheelbase (axle-to-axle): ~108mm
- Body width: ~75mm
- Body depth: ~72mm
- Body height: ~62mm (bottom to top of lights)
- Ultrasonic sensor position: front face, centered, ~45mm above ground
- Caster position: rear center, ~15mm offset from body center
- IR sensors: bottom face, front, ~20mm from center-line each side

**Gazebo plugins** (for simulation):
- `libgazebo_ros_diff_drive.so` — differential drive
- `libgazebo_ros_range.so` — ultrasonic range sensor

**Mesh generation**: Export simplified STL meshes from the STEP file using cadquery, one per major part (body, wheel_left, wheel_right, caster). Keep polygon count low (<5000 faces per mesh) for RViz performance.

#### Task 3.2 — STL Mesh Export Script

Write a Python script using cadquery/OCP that:
1. Loads the STEP file
2. Identifies parts by name (Bottom, Middle, Toplights → combine as body; O-ring → wheel; Ballcaster → caster; FaceUS → face)
3. Exports each as a simplified STL with units in meters (ROS convention)
4. Places meshes in `otto_description/meshes/`

---

### Phase 4: ROS2 Host Packages

#### Task 4.1 — `otto_description` Package

Standard ROS2 description package:
- `robot_state_publisher` node reading the URDF
- Launch file for RViz visualization
- RViz config showing robot model + ultrasonic range cone

#### Task 4.2 — `otto_bringup` Package

**Launch: `otto_microros.launch.py`**
- Start micro-ROS agent (`micro_ros_agent udp4 --port 8888`)
- Start `robot_state_publisher` with URDF
- Start `ultrasonic_to_laserscan.py` node (converts Range → LaserScan for SLAM)
- Start `otto_odom_publisher.py` (dead-reckoning from cmd_vel, since no encoders)
- Start `static_transform_publisher` for any needed TF frames

**Launch: `otto_teleop.launch.py`**
- Keyboard teleop (`teleop_twist_keyboard`) remapped to `/cmd_vel`
- Optional: joystick teleop

**Launch: `otto_mapping_demo.launch.py`**
- Everything from `otto_microros.launch.py`
- Plus `slam_toolbox` in online async mode
- Configured for ultrasonic (single-beam "lidar" with rotation trick or sweep-and-map)

#### Task 4.3 — `ultrasonic_to_laserscan.py`

Since Otto has a single forward-facing ultrasonic sensor (not a spinning lidar), the mapping demo has two approaches:

**Approach A — Rotate-and-scan (recommended for demo):**
- Otto rotates in place, taking ultrasonic readings at each angle
- The node accumulates readings with the corresponding `tf` rotation
- Publishes a synthetic `LaserScan` message built from the accumulated readings
- Works with `slam_toolbox` or `cartographer` for basic 2D mapping

**Approach B — Single-beam range display:**
- Simply republish the Range as a single-ray LaserScan
- Less useful for mapping but shows the sensor working in RViz

#### Task 4.4 — `otto_odom_publisher.py`

Dead-reckoning odometry from `cmd_vel`:
- Integrates `linear.x` and `angular.z` over time
- Publishes `nav_msgs/msg/Odometry` on `/odom`
- Publishes TF: `odom` → `base_footprint`
- **WARNING**: With no wheel encoders, odometry will drift significantly. This is expected and documented. The demo is educational, not precision navigation.

---

### Phase 5: Mapping Demo

#### Task 5.1 — Demo Script / Instructions

Step-by-step demo document (`docs/mapping_demo.md`):

1. Flash firmware to Otto (via USB-C)
2. Configure WiFi credentials
3. On the host machine, run `docker compose up` (starts agent + ROS2 stack)
4. Verify topics: `ros2 topic list` shows `/ultrasonic/range`, `/cmd_vel`, etc.
5. Open RViz with the provided config
6. Use keyboard teleop to drive Otto around a room
7. Watch the ultrasonic readings appear as a range cone in RViz
8. For mapping: Otto rotates in place, scans surroundings, then drives to a new position and repeats
9. The `slam_toolbox` builds a basic 2D occupancy grid

The mapping will be crude (single ultrasonic beam, no encoders, significant drift) but it demonstrates the full micro-ROS → ROS2 pipeline.

---

## 4. Key Technical Decisions

### 4.1 micro-ROS Transport: WiFi UDP

WiFi UDP is the only viable transport since the USB-C port is the sole physical connection and is needed for charging/programming. Serial transport would require keeping the cable attached during operation.

Config: ESP32 connects to a known WiFi AP and sends micro-ROS messages to the agent's IP:port over UDP.

### 4.2 Continuous Rotation Servos — No Encoder Feedback

The Otto Starter uses cheap continuous rotation servos with no position/velocity feedback. This means:
- **No wheel odometry** — dead-reckoning from cmd_vel only
- **Open-loop speed control** — calibrate servo pulse width ↔ wheel speed empirically
- **Mapping accuracy** — will be poor, this is expected for an educational demo
- **Future upgrade path** — document how to add optical encoders or swap for encoder-equipped servos

### 4.3 Single-Wire Ultrasonic (RCWL-9610)

The RCWL-9610 uses a single GPIO for both trigger and echo (unlike classic HC-SR04 which has separate trig/echo). The pin must be switched between output (trigger) and input (echo) during each reading cycle. This is handled in the firmware driver.

### 4.4 Firmware Size Budget (4MB flash)

Estimated partition layout:
```
# 4MB flash = 4,194,304 bytes
nvs:       0x6000   (24KB)   — WiFi credentials, calibration
otadata:   0x2000   (8KB)    — OTA bookkeeping
app0:      0x1E0000 (1.9MB)  — Running firmware
app1:      0x1E0000 (1.9MB)  — OTA update slot
spiffs:    0x10000  (64KB)   — Config files (optional)
```

micro-ROS + FreeRTOS + WiFi stack + servo/sensor drivers typically fits in ~1.5–1.8MB, leaving room for OTA.

---

## 5. Unresolved Items / Assumptions

| Item | Assumption | Risk |
|------|-----------|------|
| Connectors 3 & 4 | Unused in Starter Kit, GPIO unknown | Low — not needed for base project |
| Servo calibration | 1500µs = stop, linear mapping assumed | Medium — needs empirical tuning |
| Battery ADC divider ratio | Unknown — needs measurement | Low — calibrate with multimeter |
| IR sensor model | Assumed analog TCRT5000-style | Low — ADC read works regardless |
| Ultrasonic NeoPixel behavior | Assumed standard WS2812B at 800kHz | Low — well-established protocol |
| Body dimensions from STEP | Measured via bounding boxes, ±2mm | Low — close enough for URDF |
| Connector 11 GPIO | Forum says 33 (typo), code confirms 13 | Resolved — GPIO 13 is correct |
| Connector 10 GPIO | Forum says 13, code confirms 14 | Resolved — GPIO 14 is correct |

---

## 6. Dependencies & Tools

### Firmware Build
- PlatformIO Core (CLI)
- ESP-IDF v5.2+ (auto-installed by PlatformIO)
- micro_ros_platformio library
- ESP32Servo library
- Adafruit NeoPixel library

### ROS2 Host
- ROS2 Humble or Jazzy
- micro-ros-agent (Docker or native)
- slam_toolbox
- teleop_twist_keyboard
- robot_state_publisher
- joint_state_publisher
- rviz2

### CAD/Mesh
- cadquery + OCP (Python, for STEP → STL export)
- meshlab (optional, for mesh simplification)

---

## 7. File Delivery Checklist

When the project is complete, these files must exist and be tested:

- [ ] `firmware/platformio.ini` — builds without errors
- [ ] `firmware/src/main.cpp` — compiles, connects to WiFi, creates micro-ROS node
- [ ] `firmware/src/otto_config.h` — all pin definitions
- [ ] `firmware/src/drive_controller.cpp` — differential drive from cmd_vel
- [ ] `firmware/src/ultrasonic_sensor.cpp` — publishes Range at 10Hz
- [ ] `firmware/src/line_sensor.cpp` — publishes ADC values at 20Hz
- [ ] `firmware/src/battery_monitor.cpp` — publishes BatteryState at 1Hz
- [ ] `firmware/src/led_controller.cpp` — subscribes to LED commands
- [ ] `firmware/src/buzzer.cpp` — subscribes to buzzer commands
- [ ] `ros2_ws/src/otto_description/urdf/otto_starter.urdf.xacro` — valid URDF
- [ ] `ros2_ws/src/otto_description/meshes/*.stl` — visual meshes
- [ ] `ros2_ws/src/otto_bringup/launch/otto_microros.launch.py`
- [ ] `ros2_ws/src/otto_bringup/launch/otto_mapping_demo.launch.py`
- [ ] `ros2_ws/src/otto_bringup/scripts/ultrasonic_to_laserscan.py`
- [ ] `ros2_ws/src/otto_bringup/scripts/otto_odom_publisher.py`
- [ ] `docker/Dockerfile.firmware` — builds firmware in container
- [ ] `docker/docker-compose.yml` — one-command build + agent
- [ ] `README.md` — quick start from zero to mapping demo
- [ ] `docs/mapping_demo.md` — step-by-step mapping tutorial

---

## 8. Reference Material

- **HP Robots product page**: https://hprobots.com/otto-robot/product/
- **Pin layout forum thread**: https://hprobots.com/community/start/documentaton-of-esp32-board-pin-layout/
- **Santa Otto code (confirmed pin map)**: https://www.hackster.io/527881/hp-robots-santa-otto-c3803e
- **STEP file**: `HPRobots_Ottostarter_1_.step` (provided, contains full CAD assembly)
- **micro-ROS ESP-IDF component**: https://github.com/micro-ROS/micro_ros_espidf_component
- **micro-ROS PlatformIO**: https://github.com/micro-ROS/micro_ros_platformio
- **RCWL-9610 datasheet/library**: https://github.com/Alash-electronics/AlashUltrasonic
- **OttoDIY ESP32 libraries (community)**: https://github.com/OttoDIY/OttoNinja
- **slam_toolbox**: https://github.com/SteveMacenski/slam_toolbox