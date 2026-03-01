// led_controller.h — WS2812B NeoPixel LED Subscriber

#ifndef LED_CONTROLLER_H
#define LED_CONTROLLER_H

#include <rcl/rcl.h>
#include <rclc/rclc.h>

void led_init();
void led_create_entities(rcl_node_t *node, rclc_support_t *support, rcl_allocator_t *allocator);
void led_add_to_executor(rclc_executor_t *executor);
void led_destroy_entities(rcl_node_t *node);
void led_off();

#endif // LED_CONTROLLER_H
