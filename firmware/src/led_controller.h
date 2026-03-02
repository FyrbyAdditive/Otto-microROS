// led_controller.h — WS2812B NeoPixel LED Subscriber

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

#endif // LED_CONTROLLER_H
