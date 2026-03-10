# Calibration

How to calibrate the Otto Starter Kit's servos and line sensors. All calibration requires the robot stack running (`./start.sh`).

---

## Servo Stop Point

The servos should be stationary when no `cmd_vel` is being published. If a wheel creeps at rest, the neutral pulse width needs adjusting.

The key constant is `SERVO_STOP_US` in [`firmware/src/otto_config.h`](../firmware/src/otto_config.h):

```cpp
#define SERVO_STOP_US  1500   // Neutral pulse width (µs)
```

### Steps

1. **Power on** the robot with no agent running (servos receive no commands)
2. **Observe** the wheels — they should be completely still
3. If a wheel creeps, **adjust** `SERVO_STOP_US` by 5–10 µs:
   - Wheel creeps forward → increase the value
   - Wheel creeps backward → decrease the value
4. **Rebuild and flash**: `cd firmware && pio run -t upload`
5. **Repeat** until both wheels are stationary at rest

---

## Servo Speed Scale

`SERVO_SPEED_SCALE` converts between `cmd_vel` linear velocity (m/s) and servo pulse offset (µs). If the robot drives further or shorter than commanded, this value needs calibrating.

An interactive calibration script automates the process.

### Prerequisites

- Robot stack running (`./start.sh`)
- Teleop open in a separate terminal: `ros2 launch otto_bringup otto_teleop.launch.py`
- A tape measure and a piece of tape or coin to mark the start position

### Running the script

```bash
python3 scripts/calibrate_kinematics.py
```

The script guides you through:

1. **Mark** the floor at the robot's wheel axle
2. **Drive straight** forward ~20–30 cm (hold `i` in the teleop window)
3. **Stop** and switch back to the calibration terminal
4. **Measure** the actual distance and type it in
5. The script prints the corrected value

### Applying the result

Update `SERVO_SPEED_SCALE` in **both** files:

| File | Line |
|------|------|
| [`firmware/src/otto_config.h`](../firmware/src/otto_config.h) | `#define SERVO_SPEED_SCALE  <new value>` |
| [`ros2_ws/src/otto_bringup/scripts/otto_odom_publisher.py`](../ros2_ws/src/otto_bringup/scripts/otto_odom_publisher.py) | `SERVO_SPEED_SCALE = <new value>` |

Then rebuild and flash the firmware: `cd firmware && pio run -t upload`

> **Note:** The default teleop speed (0.2 m/s) saturates the servos at the current scale. The robot's max unsaturated speed is ~0.14 m/s. For valid calibration, drive at a slower speed or accept that the robot will max out at full servo speed.

---

## Line Sensors

The two TCRT5000-based IR reflectance modules (Connectors 6 and 7) detect dark lines on light surfaces. Each board has:

- **Analog output (A0)** — raw IR reflectance, read by the firmware
- **Digital output (D0)** — threshold trigger, active when reflectance drops below the potentiometer setting
- **Potentiometer** — sets the D0 trigger threshold (small screw on the underside)
- **Indicator LED** — lights when D0 is triggered

### How the firmware uses the sensors

| Detail | Value |
|--------|-------|
| Output used | Analog (A0) — **not** the digital trigger |
| ADC resolution | 12-bit (0–4095) |
| Publish rate | 20 Hz on `/line_sensors` |
| High value (~3000–4095) | Reflective surface (white / light) |
| Low value (~0–1000) | Absorptive surface (dark line / no surface) |

The RViz visualizer maps these values to colour: **green** = clear surface, **red** = line detected.

### Verifying the readings

```bash
ros2 topic echo /line_sensors
```

Place the robot on a **light surface** — both values should read ~3000+. Move it over a **dark line** — the corresponding sensor should drop to ~500–1500. If the contrast between light and dark is small, clean the sensor lens with a dry cloth.

### Adjusting the potentiometer

The potentiometer controls only the **indicator LED and digital output** — it does not affect the analog readings published to ROS2. Calibrating it is optional but useful as a visual aid when positioning the sensors.

Using a small Phillips screwdriver on the underside of each sensor board:

| Direction | Effect |
|-----------|--------|
| Clockwise | Higher threshold — less sensitive (only very dark surfaces trigger) |
| Counter-clockwise | Lower threshold — more sensitive (lighter surfaces trigger) |

**To set a useful threshold:**

1. Place the sensor over the **dark line** — the indicator LED should be **on**
2. If it is off, turn the potentiometer **counter-clockwise** until it lights up
3. Move the sensor to the **light surface** — the LED should turn **off**
4. If the LED stays on for both surfaces, turn slightly **clockwise** until it just turns off on the light surface
5. Repeat for the other sensor

> **Tip:** The potentiometer is sensitive — small turns make a big difference. If you overshoot, go back the other way slowly.
