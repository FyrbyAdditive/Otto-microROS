# Flashing Firmware

Get the Otto firmware onto your ESP32 and configure WiFi — no build tools needed.

---

## Prerequisites

### Hardware

- HP Robots Otto Starter Kit (ESP32-WROOM-32E-N8)
- USB-C data cable (not charge-only — if upload fails, try a different cable)
- A 2.4 GHz WiFi network (the ESP32 does not support 5 GHz)

### Software

- **esptool** — the ESP32 flash utility
- A USB-to-UART driver (most systems have this built in)

**Install esptool:**

```bash
pip install esptool
```

**Install USB driver (if needed):**

| Platform | Command / Link |
|----------|---------------|
| macOS | `brew install --cask wch-ch34x-usb-serial-driver` (for CH340 chips) |
| Linux | Usually built in. If not: `sudo apt install linux-modules-extra-$(uname -r)` |
| Windows | [WCH CH341](http://www.wch.cn/downloads/CH341SER_EXE.html) or [Silicon Labs CP210x](https://www.silabs.com/developers/usb-to-uart-bridge-vcp-drivers) |

---

## Step 1 — Download the Firmware

Download the latest `firmware.bin` from the [GitHub Releases](https://github.com/FyrbyAdditive/Otto-microROS/releases) page.

The pre-built binary supports the stock PWM continuous rotation servos. If you're using Feetech/Waveshare serial bus servos, you'll need to [build from source](building.md) with the `SERVO_TYPE_SERIAL_BUS=1` flag.

---

## Step 2 — Connect and Flash

1. Turn **off** the Otto robot
2. Connect the USB-C cable between Otto and your computer
3. Find the serial port:
   ```bash
   # Linux
   ls /dev/ttyUSB*

   # macOS
   ls /dev/cu.usbserial-*

   # Windows (Device Manager → Ports)
   ```
4. Flash the firmware:
   ```bash
   esptool.py --port /dev/ttyUSB0 --baud 460800 \
     write_flash 0x10000 firmware.bin
   ```
   Replace `/dev/ttyUSB0` with your actual port.

**If upload fails:**

- Hold the **BOOT** button on the ESP32 while the command runs; release after "Connecting..." appears
- Try a different USB-C cable (charge-only cables won't work)
- Try a different USB port (avoid hubs)
- If no serial port appears, install the USB driver (see Prerequisites)

---

## Step 3 — Configure WiFi (First Boot)

On first boot (or after erasing flash), the robot enters **setup mode**:

1. The LED ring pulses **magenta** and the buzzer plays an ascending tone
2. The robot creates a WiFi network named **Otto-XXXX** (where XXXX is unique to your board)
3. Connect to that network from your phone or laptop
4. A captive portal page opens automatically — if it doesn't, open a browser to `http://192.168.4.1`
5. Select your WiFi network from the scan results, enter the password, and fill in the micro-ROS agent IP and port
6. Click **Save & Connect** — the robot reboots and connects to your WiFi

**Finding your host machine's IP (for the agent IP field):**

```bash
# macOS
ipconfig getifaddr en0

# Linux
hostname -I | awk '{print $1}'

# Windows
ipconfig | findstr "IPv4"
```

The configuration is stored in NVS (non-volatile storage) and persists across reboots.

**To re-enter setup mode**, erase the flash and re-flash:

```bash
esptool.py --port /dev/ttyUSB0 erase_flash
esptool.py --port /dev/ttyUSB0 --baud 460800 \
  write_flash 0x10000 firmware.bin
```

---

## Step 4 — Verify

Power on the robot (unplug USB, use the power switch). You should see:

| LED state | Meaning |
|-----------|---------|
| Pulsing magenta | Setup mode — connect to Otto-XXXX WiFi |
| Pulsing cyan | Connected to WiFi, searching for micro-ROS agent |
| Solid green (briefly) | Agent found, entities created |
| Blue ring animation | Normal operation, connected to agent |

The blue onboard LED blinks while searching and goes solid when connected.

---

## Step 5 — Disconnect and Run

1. Unplug the USB-C cable
2. Turn the robot **on** via the power switch
3. The firmware starts automatically from flash
4. Once WiFi connects and the agent is reachable, the blue LED goes solid

---

## Calibration

After flashing, the servos and line sensors may need calibration. See [calibration.md](calibration.md) for step-by-step instructions covering:

- **Servo stop point** — eliminating wheel creep at rest
- **Servo speed scale** — matching commanded and actual travel distance
- **Line sensors** — adjusting the potentiometer threshold and verifying readings

---

<p align="center">
  <br>
  <img src="images/fame-logo.letterbox.png" alt="FAME logo" width="200">
</p>
