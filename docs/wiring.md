# Wiring Reference — HP Robots Otto Starter Kit

## Connector Pin Map

The HP Robots Otto PCB has 11 numbered JST connectors. Each exposes signal GPIO(s) plus VCC and GND.

| Connector | GPIO | Type | Default Use (Starter Kit) | Alt Uses |
|-----------|------|------|---------------------------|----------|
| 1 | 18, 19 | Digital | Ultrasonic sensor (RCWL-9610) | OLED display, serial bus servo UART |
| 2 | 16, 17 | Digital | *Unused* | MP3 player, encoder |
| 3 | 22, 21 | I2C | *Unused* | Matrix display, accelerometer |
| 4 | 26 | Digital | *Unused* | Tilt sensor, button |
| 5 | 4 | Digital | LED ring (13x WS2812B) | Temperature sensor |
| 6 | 32 | ADC | Left IR line sensor | Microphone |
| 7 | 33 | ADC | Right IR line sensor | Potentiometer, light sensor |
| 8 | 27 | PWM | *Unused* (biped: right hip servo) | |
| 9 | 15 | PWM | *Unused* (biped: left hip servo) | |
| 10 | 14 | PWM | Left wheel servo | |
| 11 | 13 | PWM | Right wheel servo | |

## On-Board Peripherals (No Connector)

| Function | GPIO | Notes |
|----------|------|-------|
| Built-in LED | 2 | Blue status LED |
| Buzzer | 25 | Passive electromagnetic, PWM driven |
| Battery voltage | 39 | ADC input-only, through voltage divider (2x 100k) |

## Ultrasonic Sensor (Connector 1)

The RCWL-9610 is wired to Connector 1:
- **GPIO 19**: Trigger/Echo (single-wire mode)
- **GPIO 18**: NeoPixel data for 6 WS2812B LEDs on the sensor PCB

In single-wire mode, GPIO 19 is toggled between output (trigger pulse) and input (echo measurement) for each reading.

## Servo Wiring (Starter Kit)

- **Left wheel**: Connector 10 → GPIO 14 (continuous rotation, 50Hz PWM)
- **Right wheel**: Connector 11 → GPIO 13 (continuous rotation, 50Hz PWM, mirror-mounted)
- PWM range: 1000-2000µs (1500µs = stop)

## Serial Bus Servo Mode

When `SERVO_TYPE_SERIAL_BUS=1`, GPIO 18/19 are repurposed:
- **GPIO 18**: UART RX (servo bus)
- **GPIO 19**: UART TX (servo bus)
- Baud rate: 1,000,000
- The ultrasonic sensor cannot be used in this configuration.

## Battery Monitoring

- GPIO 39 reads through a voltage divider (assumed 2:1 ratio with 2x 100kΩ resistors)
- LiPo cell: 3.0V (empty) to 4.2V (full)
- Calibrate `BATTERY_DIVIDER_RATIO` in `otto_config.h` with a multimeter
