# Flashing Firmware

Complete guide from a fresh clone to a running micro-ROS node on the ESP32.

---

## Prerequisites

### Hardware

- HP Robots Otto Starter Kit (ESP32-WROOM-32E-N8)
- USB-C data cable (not charge-only — if upload fails, try a different cable)
- A 2.4 GHz WiFi network (the ESP32 does not support 5 GHz)

### Software

- **PlatformIO CLI** or VS Code with the PlatformIO extension
- A USB-to-UART driver (most systems have this built in)

**Install PlatformIO CLI (if not using VS Code):**

```bash
# macOS / Linux
curl -fsSL -o get-platformio.py \
  https://raw.githubusercontent.com/platformio/platformio-core-installer/master/get-platformio.py
python3 get-platformio.py
# Add to PATH: export PATH="$HOME/.platformio/penv/bin:$PATH"

# Or via pip:
pip install platformio
```

**Install USB driver (if needed):**

| Platform | Command / Link |
|----------|---------------|
| macOS | `brew install --cask wch-ch34x-usb-serial-driver` (for CH340 chips) |
| Linux | Usually built in. If not: `sudo apt install linux-modules-extra-$(uname -r)` |
| Windows | [WCH CH341](http://www.wch.cn/downloads/CH341SER_EXE.html) or [Silicon Labs CP210x](https://www.silabs.com/developers/usb-to-uart-bridge-vcp-drivers) |

---

## Step 1 — Configure WiFi Credentials

```bash
cp firmware/src/wifi_credentials.h.example firmware/src/wifi_credentials.h
```

Open `firmware/src/wifi_credentials.h` and fill in your values:

```cpp
#define WIFI_SSID      "YourWiFiName"       // Must be 2.4 GHz
#define WIFI_PASSWORD  "YourWiFiPassword"
#define AGENT_IP       "192.168.1.100"       // IP of the machine running the micro-ROS agent
#define AGENT_PORT     8888
```

**Finding your host machine's IP:**

```bash
# macOS
ipconfig getifaddr en0

# Linux
hostname -I | awk '{print $1}'

# Windows
ipconfig | findstr "IPv4"
```

This file is `.gitignored` and will never be committed.

---

## Step 2 — Build the Firmware

```bash
cd firmware
pio run -e otto_starter
```

**First build** takes 5–15 minutes. PlatformIO will:

1. Download the ESP32 toolchain (~300 MB)
2. Download library dependencies (micro_ros_platformio, ESP32Servo, Adafruit NeoPixel)
3. Build the micro-ROS static library for Jazzy (~5–10 min)
4. Compile the Otto firmware (~10 s)

Subsequent builds only run step 4 and take ~10 seconds.

**If the build fails:**

- Delete `.pio/` and try again: `rm -rf .pio && pio run -e otto_starter`
- Ensure Python 3.8+ is installed: `python3 --version`
- Check PlatformIO is up to date: `pio upgrade`

---

## Step 3 — Connect and Flash

1. Turn **off** the Otto robot
2. Connect the USB-C cable between Otto and your computer
3. Verify the port is detected:
   ```bash
   pio device list
   # Should show something like /dev/cu.usbserial-0001 or /dev/ttyUSB0
   ```
4. Flash:
   ```bash
   pio run -t upload
   ```

**If upload fails:**

- Hold the **BOOT** button on the ESP32 while upload starts; release after "Connecting..." appears
- Try a different USB-C cable (charge-only cables won't work)
- Try a different USB port (avoid hubs)
- Check `pio device list` — if no port appears, you need a USB driver (see Prerequisites)

---

## Step 4 — Verify via Serial Monitor

```bash
pio device monitor
```

**Without the micro-ROS agent running**, you should see:

```
[Otto] WiFi connected, waiting for micro-ROS agent...
```

The blue LED blinks slowly (0.5 s on/off) while searching for the agent. This confirms WiFi is working and the firmware is running.

**With the micro-ROS agent running** (see the [README](../README.md#3-start-the-ros2-stack)):

```
[Otto] WiFi connected, waiting for micro-ROS agent...
[Otto] Agent found!
[Otto] micro-ROS entities created, connected!
```

The blue LED goes solid when connected.

---

## Step 5 — Disconnect and Run

1. Unplug the USB-C cable
2. Turn the robot **on** via the power switch
3. The firmware starts automatically from flash
4. Once WiFi connects and the agent is reachable, the blue LED goes solid

---

## Updating Firmware

After code changes:

```bash
cd firmware
pio run -t upload && pio device monitor
```

| Change | Rebuild time |
|--------|:------------:|
| Any `.cpp` or `.h` file | ~10 s |
| `otto_config.h` pin definitions or calibration values | ~10 s |
| `platformio.ini` build flags | ~10 s |
| `custom_meta.meta` (publisher/subscriber counts) | 5–15 min |
| `board_microros_distro` in `platformio.ini` | 5–15 min |

---

## Servo Calibration

After flashing, the servos may need calibration. The key constants are in [`firmware/src/otto_config.h`](../firmware/src/otto_config.h):

```cpp
#define SERVO_STOP_US       1500     // Adjust ±10 if servos creep at rest
#define SERVO_SPEED_SCALE   3623.4   // Calibrated: µs per m/s (max ~0.14 m/s unsaturated)
```

### Finding the true stop point

1. Power on with no agent running (servos should be stopped)
2. If a wheel creeps, adjust `SERVO_STOP_US` up or down by 5–10 µs
3. Rebuild and re-flash

### Calibrating speed (SERVO_SPEED_SCALE)

Use the calibration script for accurate results:

```bash
# Terminal 1: start the robot stack
./start.sh

# Terminal 2: start teleop
ros2 launch otto_bringup otto_teleop.launch.py

# Terminal 3: run calibration
python3 scripts/calibrate_kinematics.py
```

The script guides you through a straight-line test: drive forward, measure the actual distance, and it computes the corrected `SERVO_SPEED_SCALE`. Update the value in both:

- [`firmware/src/otto_config.h`](../firmware/src/otto_config.h)
- [`ros2_ws/src/otto_bringup/scripts/otto_odom_publisher.py`](../ros2_ws/src/otto_bringup/scripts/otto_odom_publisher.py)

> **Note:** The default teleop speed (0.2 m/s) saturates the servos at the current scale. For valid calibration, use a slower speed — the robot's max unsaturated speed is ~0.14 m/s.
