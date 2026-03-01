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

// Connector 1: Ultrasonic sensor (RCWL-9610) + NeoPixel LEDs
// When SERVO_TYPE_SERIAL_BUS=1, these become UART RX/TX for servo bus
#define PIN_US_TRIG_ECHO      19   // Connector 1 signal — trigger/echo (single-wire)
#define PIN_US_NEOPIXEL       18   // Connector 1 signal — 6x WS2812B on sensor PCB

// Connector 2: Unused in Starter Kit (MP3 player or encoder)
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
// Uses Connector 1 pins as UART — ultrasonic sensor unavailable
// ============================================================
#if SERVO_TYPE_SERIAL_BUS
#define SERVO_BUS_RXD         18   // Connector 1, repurposed as UART RX
#define SERVO_BUS_TXD         19   // Connector 1, repurposed as UART TX
#define SERVO_BUS_BAUD   1000000   // 1 Mbaud (Feetech default)
#define SERVO_BUS_ID_LEFT      1   // Bus servo ID for left wheel
#define SERVO_BUS_ID_RIGHT     2   // Bus servo ID for right wheel
#endif

// ============================================================
// LED Configuration
// ============================================================
#define LED_RING_COUNT        13   // WS2812B LEDs in the eye ring (Connector 5)
#define LED_US_COUNT           6   // WS2812B LEDs on ultrasonic housing (Connector 1)

// ============================================================
// PWM Servo Configuration (when SERVO_TYPE_SERIAL_BUS=0)
// ============================================================
#define SERVO_FREQ            50   // Hz (standard servo PWM)
#define SERVO_STOP_US       1500   // Microseconds for stop (continuous rotation neutral)
#define SERVO_MIN_US        1000   // Full speed one direction
#define SERVO_MAX_US        2000   // Full speed other direction

// Speed scale: maps m/s velocity to servo microsecond offset from center.
// With max cmd_vel ~0.2 m/s mapping to 500us offset: SPEED_SCALE = 500 / 0.2 = 2500
// Adjust empirically after testing with your servos.
#define SERVO_SPEED_SCALE   2500.0

// ============================================================
// Physical Dimensions (meters, for ROS)
// ============================================================
#define WHEEL_DIAMETER      0.049  // ~49mm from STEP CAD model
#define WHEEL_BASE          0.108  // ~108mm axle-to-axle from STEP
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
#define AGENT_PING_TIMEOUT_MS  100
#define AGENT_PING_ATTEMPTS      1
#define AGENT_RECONNECT_MS     500  // Delay between reconnection attempts

#endif // OTTO_CONFIG_H
