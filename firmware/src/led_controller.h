// led_controller.h — WS2812B NeoPixel subscriber + state publisher
//
// /leds command topic (UInt8MultiArray) — ROS → robot:
//   Byte 0: target   0=ring, 1=ultrasonic LEDs, 2=both
//   Byte 1: mode     0=solid colour, 1=per-LED colour, 2=off, 3=release to firmware
//   Bytes 2-3: uint16 override duration ms (big-endian)
//              0x0000 or 0xFFFF = hold indefinitely until next command or release
//              1-65534 = return to firmware animation after N ms
//   Bytes 4+: colour payload
//              mode 0: R G B
//              mode 1: R G B R G B … (one triplet per LED, up to LED_RING_COUNT)
//              mode 2: (none)
//              mode 3: (none — duration bytes also ignored)
//
// /led_state publish topic (UInt8MultiArray) — robot → ROS:
//   Always mode-1 format: [0, 1, R0,G0,B0, R1,G1,B1, …, R12,G12,B12]
//   Published whenever the ring changes (firmware animation or ROS command).

#ifndef LED_CONTROLLER_H
#define LED_CONTROLLER_H

#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

void led_init();
void led_create_entities(rcl_node_t *node, rclc_support_t *support, rcl_allocator_t *allocator);
void led_add_to_executor(rclc_executor_t *executor);
void led_destroy_entities(rcl_node_t *node);
void led_off();

// Status indication (usable before micro-ROS is active)
void led_status_color(uint8_t r, uint8_t g, uint8_t b);
void led_status_off();

// Proximity indicator on ultrasonic LEDs (red=close, orange=mid, green=far)
void led_proximity(float distance_m);

// Direct pixel access for ring status animations
void led_ring_set_pixel(uint16_t index, uint8_t r, uint8_t g, uint8_t b);
void led_ring_show();

// Returns true while a timed ROS override is active (firmware animation suppressed)
bool led_ring_overridden();

#endif // LED_CONTROLLER_H
