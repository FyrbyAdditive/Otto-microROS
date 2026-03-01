# Flashing Firmware — HP Robots Otto Starter Kit

Complete guide from a fresh clone to a running micro-ROS node on the ESP32.

## Prerequisites

### Hardware
- HP Robots Otto Starter Kit (ESP32-WROOM-32E-N8)
- USB-C data cable (not charge-only — if upload fails, try a different cable)
- A 2.4GHz WiFi network (the ESP32 does not support 5GHz)

### Software
- **PlatformIO CLI** or VS Code with the PlatformIO extension
- A USB-to-UART driver (most systems have this built in)

**Install PlatformIO CLI (if not using VS Code):**
```bash
# macOS / Linux
curl -fsSL -o get-platformio.py https://raw.githubusercontent.com/platformio/platformio-core-installer/master/get-platformio.py
python3 get-platformio.py
# Add to PATH: export PATH="$HOME/.platformio/penv/bin:$PATH"

# Or via pip:
pip install platformio
```

**Install USB driver (if needed):**
- **macOS**: `brew install --cask wch-ch34x-usb-serial-driver` (for CH340 chips)
- **Linux**: Usually built into the kernel. If not: `sudo apt install linux-modules-extra-$(uname -r)`
- **Windows**: Download from [WCH](http://www.wch.cn/downloads/CH341SER_EXE.html) or [Silicon Labs CP210x](https://www.silabs.com/developers/usb-to-uart-bridge-vcp-drivers)

## Step 1: Configure WiFi Credentials

```bash
cp firmware/src/wifi_credentials.h.example firmware/src/wifi_credentials.h
```

Open `firmware/src/wifi_credentials.h` in any editor and fill in your values:

```cpp
#define WIFI_SSID      "YourWiFiName"       // Must be 2.4GHz
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

## Step 2: Build the Firmware

```bash
cd firmware
pio run -e otto_starter
```

**First build** takes 5-15 minutes. PlatformIO will:
1. Download the ESP32 toolchain (~300MB)
2. Download library dependencies (micro_ros_platformio, ESP32Servo, Adafruit NeoPixel)
3. Build the micro-ROS static library for Jazzy (~5-10 min)
4. Compile the Otto firmware (~10 sec)

Subsequent builds only run step 4 and take ~10 seconds.

**If the build fails:**
- Delete `.pio/` and try again: `rm -rf .pio && pio run -e otto_starter`
- Ensure Python 3.8+ is installed: `python3 --version`
- Check PlatformIO is up to date: `pio upgrade`

## Step 3: Connect and Flash

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
- Try holding the **BOOT** button on the ESP32 board while the upload starts, release after "Connecting..." appears
- Try a different USB-C cable (charge-only cables won't work)
- Try a different USB port (avoid hubs)
- Check `pio device list` — if no port appears, you need a USB driver (see Prerequisites)

## Step 4: Verify via Serial Monitor

```bash
pio device monitor
```

**Without the micro-ROS agent running**, you should see:
```
[Otto] WiFi connected, waiting for micro-ROS agent...
```

The blue LED on the board will blink slowly (0.5s on/off) while searching for the agent. This confirms WiFi is working and the firmware is running.

**With the micro-ROS agent running** (see [ROS2 setup](../README.md#3-start-the-ros2-stack)), you should see:
```
[Otto] WiFi connected, waiting for micro-ROS agent...
[Otto] Agent found!
[Otto] micro-ROS entities created, connected!
```

The blue LED goes solid when connected.

## Step 5: Disconnect and Run

1. Unplug the USB-C cable
2. Turn the robot **on** via the power switch
3. The firmware starts automatically from flash
4. Once WiFi connects and the agent is reachable, the blue LED goes solid

## Updating Firmware

After code changes:
```bash
cd firmware
pio run -t upload && pio device monitor
```

**What triggers a full micro-ROS rebuild** (5-15 min):
- Changing `custom_meta.meta` (publisher/subscriber counts)
- Changing `board_microros_distro` in `platformio.ini`

**What only requires a normal rebuild** (~10 sec):
- Changing any `.cpp`, `.h` file
- Changing `otto_config.h` pin definitions or calibration values
- Changing `platformio.ini` build_flags

## Servo Calibration

After flashing, the servos may need calibration. In `firmware/src/otto_config.h`:

```cpp
#define SERVO_STOP_US       1500   // Adjust ±10 if servos creep at rest
#define SERVO_SPEED_SCALE   2500.0 // Increase for faster response to cmd_vel
```

**To find the true stop point:**
1. Power on with no agent running (servos should be stopped)
2. If a wheel creeps, adjust `SERVO_STOP_US` up or down by 5-10µs
3. Rebuild and re-flash

**To tune speed mapping:**
1. Connect to the agent
2. Send a known velocity: `ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.1}}"`
3. Measure actual speed (e.g., time how long to travel 1 meter)
4. Adjust `SERVO_SPEED_SCALE` accordingly
