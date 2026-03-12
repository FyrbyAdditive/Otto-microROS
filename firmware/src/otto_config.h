// otto_config.h — Hardware Configuration for HP Robots Otto Starter Kit
// ======================================================================
// Single source of truth for all GPIO pins, physical constants, and timing.
// ESP32-WROOM-32E-N8 (8MB flash) on HP Robots custom PCB.
//
// Connector reference: https://hprobots.com/community/start/documentaton-of-esp32-board-pin-layout/

#ifndef OTTO_CONFIG_H
#define OTTO_CONFIG_H

// ============================================================
// Servo type selection (set via platformio.ini build_flags)
// 0 = Stock PWM continuous rotation servos (default)
// 1 = Feetech/Waveshare serial bus servos (SCS/STS series)
// ============================================================
#ifndef SERVO_TYPE_SERIAL_BUS
#define SERVO_TYPE_SERIAL_BUS 0
#endif

// ============================================================
// GPIO Pin Definitions — Starter Kit (Wheeled Variant)
// ============================================================

// Connector 1: Ultrasonic sensor (RCWL-9610) + NeoPixel LEDs (always available)
#define PIN_US_TRIG_ECHO      19   // Connector 1 signal — trigger/echo (single-wire)
#define PIN_US_NEOPIXEL       18   // Connector 1 signal — 6x WS2812B on sensor PCB

// Connector 2: Serial bus servo UART (when SERVO_TYPE_SERIAL_BUS=1)
#define PIN_CONN2_A           16
#define PIN_CONN2_B           17

// Connector 3: Unused in Starter Kit (I2C — matrix display or accelerometer)
#define PIN_I2C_SCL           22
#define PIN_I2C_SDA           21

// Connector 4: Unused in Starter Kit (tilt sensor or button)
#define PIN_CONN4             26

// Connector 5: LED ring (13x WS2812B "eyes")
#define PIN_LED_RING           4

// Connector 6: Left IR line sensor (analog)
#define PIN_IR_LEFT           32   // ADC1_CH4

// Connector 7: Right IR line sensor (analog)
#define PIN_IR_RIGHT          33   // ADC1_CH5

// Connector 8: Servo angle right (biped variant hip)
#define PIN_SERVO_ANGLE_R     27   // Unused in Starter Kit

// Connector 9: Servo angle left (biped variant hip)
#define PIN_SERVO_ANGLE_L     15   // Unused in Starter Kit

// Connector 10: Left wheel servo (continuous rotation)
#define PIN_SERVO_LEFT        14

// Connector 11: Right wheel servo (continuous rotation)
#define PIN_SERVO_RIGHT       13

// On-board peripherals (no connector)
#define PIN_LED_BUILTIN        2   // Blue status LED
#define PIN_BUZZER            25   // Passive electromagnetic buzzer, PWM driven
#define PIN_BATTERY_ADC       39   // Input-only, voltage divider on PCB

// ============================================================
// Serial Bus Servo Configuration (when SERVO_TYPE_SERIAL_BUS=1)
// Uses Connector 2 pins as UART2 — ultrasonic/US LEDs remain available
// ============================================================
#if SERVO_TYPE_SERIAL_BUS
#define SERVO_BUS_RXD         16   // Connector 2, UART2 RX
#define SERVO_BUS_TXD         17   // Connector 2, UART2 TX
#define SERVO_BUS_BAUD   1000000   // 1 Mbaud (Feetech default)
#define SERVO_BUS_ID_LEFT      1   // Bus servo ID for left wheel
#define SERVO_BUS_ID_RIGHT     2   // Bus servo ID for right wheel
#endif

// ============================================================
// LED Configuration
// ============================================================
#define LED_RING_COUNT        13   // Total WS2812B LEDs in eye module (Connector 5)
#define LED_CENTER_INDEX       0   // Center LED (inside the ring)
#define LED_RING_OUTER_COUNT  12   // LEDs around the ring perimeter (clock layout)
// Ring LEDs are 1-12, arranged like a clock: 12=forward, 3=right, 6=back, 9=left
#define LED_US_COUNT           6   // WS2812B LEDs on ultrasonic housing (Connector 1)

// ============================================================
// PWM Servo Configuration (when SERVO_TYPE_SERIAL_BUS=0)
// ============================================================
#define SERVO_FREQ            50   // Hz (standard servo PWM)
#define SERVO_STOP_US       1500   // Microseconds for stop (continuous rotation neutral)
#define SERVO_MIN_US        1000   // Full speed one direction
#define SERVO_MAX_US        2000   // Full speed other direction

// Per-servo stop calibration offsets (microseconds added to SERVO_STOP_US).
// Cheap continuous rotation servos rarely have dead-center at exactly 1500us.
// Adjust until each wheel stops completely: positive = shift stop point up.
#define SERVO_LEFT_TRIM        0
#define SERVO_RIGHT_TRIM       0

// Speed scale: maps m/s velocity to servo microsecond offset from center.
// Calibrated empirically using scripts/calibrate_kinematics.py.
// Max unsaturated speed = SERVO_MAX_OFFSET / SERVO_SPEED_SCALE ≈ 0.138 m/s.
#define SERVO_SPEED_SCALE   3623.4

// Dead band: minimum microsecond offset from neutral before the servo actually
// moves. Offsets smaller than this get clamped to zero (neutral) to prevent
// the servo from briefly spinning the wrong direction when crossing its
// internal dead zone. Cheap "Tailor" servos can have 40-60us dead bands.
// Set this to the widest dead band of your two servos (err on the high side).
#define SERVO_DEAD_BAND_US    60

// ============================================================
// Physical Dimensions (meters, for ROS)
// ============================================================
#define WHEEL_DIAMETER      0.049  // ~49mm from STEP CAD model
#define WHEEL_BASE          0.0814 // ~81.4mm axle-to-axle from STEP CAD
#define WHEEL_RADIUS        (WHEEL_DIAMETER / 2.0)
#define WHEEL_WIDTH         0.0055 // ~5.5mm tire (o-ring)

// Body dimensions (approximate, from STEP)
#define BODY_WIDTH          0.075  // 75mm
#define BODY_DEPTH          0.072  // 72mm
#define BODY_HEIGHT         0.062  // 62mm (bottom plate to top of lights)

// ============================================================
// Sensor Timing (milliseconds)
// ============================================================
#define US_PUBLISH_PERIOD_MS      100   // 10 Hz ultrasonic
#define US_PULSE_TIMEOUT_US     11700   // 11.7ms = 2m max range (reduces executor block time)
#define LINE_PUBLISH_PERIOD_MS     50   // 20 Hz line sensors
#define BATTERY_PUBLISH_PERIOD_MS 1000  // 1 Hz battery

// ============================================================
// Safety
// ============================================================
#define CMD_VEL_TIMEOUT_MS    500  // Stop servos if no cmd_vel received

// ============================================================
// ADC Configuration
// ============================================================
#define ADC_RESOLUTION        12   // 12-bit (0-4095)
#define ADC_ATTEN             ADC_11db  // Full 0-3.3V range

// Battery voltage divider ratio (measured_voltage = raw_voltage * ratio)
// Default assumes 2:1 divider (two 100k resistors). Calibrate with multimeter.
#define BATTERY_DIVIDER_RATIO  2.0
#define BATTERY_FULL_V         4.2  // LiPo fully charged
#define BATTERY_EMPTY_V        3.0  // LiPo cutoff

// ============================================================
// micro-ROS Agent Ping
// ============================================================
// Agent ping tuning.
// Each ping check blocks the executor for TIMEOUT * ATTEMPTS ms — keep this
// well below CMD_VEL_TIMEOUT_MS (500ms) so the safety stop never fires due to
// a stalled ping.  Resilience against transient packet loss comes from
// AGENT_PING_FAIL_MAX: the agent must fail that many consecutive checks
// (spaced ~2 s apart) before entities are torn down (~6 s total tolerance).
#define AGENT_PING_TIMEOUT_MS  200  // ms per attempt — max executor freeze: 200ms
#define AGENT_PING_ATTEMPTS      1  // single attempt per check; retry via fail counter
#define AGENT_PING_FAIL_MAX      3  // consecutive failures before disconnect (~6 s)
#define AGENT_RECONNECT_MS     500  // Delay between reconnection attempts

// ============================================================
// Captive Portal / Soft-AP Configuration
// ============================================================
#define AP_SSID_PREFIX        "Otto-"   // AP name becomes "Otto-XXXX" (last 4 hex of MAC)
#define AP_CHANNEL            1         // WiFi channel for the config AP
#define WIFI_CONNECT_RETRIES  3         // STA connect attempts before falling back to portal

#endif // OTTO_CONFIG_H
