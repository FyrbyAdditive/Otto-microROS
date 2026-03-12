# Building Firmware from Source

Build the Otto firmware yourself using PlatformIO. This is required if you want to modify the firmware or use serial bus servos.

---

## Install PlatformIO

```bash
# macOS / Linux
curl -fsSL -o get-platformio.py \
  https://raw.githubusercontent.com/platformio/platformio-core-installer/master/get-platformio.py
python3 get-platformio.py
# Add to PATH: export PATH="$HOME/.platformio/penv/bin:$PATH"

# Or via pip:
pip install platformio
```

---

## Build

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

## Flash

```bash
pio run -t upload
```

WiFi configuration happens on first boot via the captive portal — see [flashing.md](flashing.md#step-3--configure-wifi-first-boot).

---

## Serial Bus Servo Build

To build for Feetech/Waveshare STS/SCS serial bus servos, change the build flag in `firmware/platformio.ini`:

```ini
build_flags =
    -DSERVO_TYPE_SERIAL_BUS=1
```

This uses GPIO 16/17 (Connector 2) for the servo UART. The ultrasonic sensor remains available. See [wiring.md](wiring.md) for details.

---

## Monitor Serial Output

```bash
pio device monitor
```

---

## Rebuild Times

| Change | Rebuild time |
|--------|:------------:|
| Any `.cpp` or `.h` file | ~10 s |
| `otto_config.h` pin definitions or calibration values | ~10 s |
| `platformio.ini` build flags | ~10 s |
| `custom_meta.meta` (publisher/subscriber counts) | 5–15 min |
| `board_microros_distro` in `platformio.ini` | 5–15 min |

---

<p align="center">
  <br>
  <img src="images/fame-logo.letterbox.png" alt="FAME logo" width="200">
</p>
