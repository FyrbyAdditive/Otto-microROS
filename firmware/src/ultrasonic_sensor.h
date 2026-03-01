// ultrasonic_sensor.h — RCWL-9610 Ultrasonic Range Publisher
// Disabled when SERVO_TYPE_SERIAL_BUS=1 (GPIO 18/19 used for servo UART).

#ifndef ULTRASONIC_SENSOR_H
#define ULTRASONIC_SENSOR_H

#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

void ultrasonic_init();
void ultrasonic_create_entities(rcl_node_t *node, rclc_support_t *support, rcl_allocator_t *allocator);
void ultrasonic_add_to_executor(rclc_executor_t *executor);
void ultrasonic_destroy_entities(rcl_node_t *node);

// Standalone reading (works without micro-ROS, updates proximity LEDs)
void ultrasonic_standalone_read();

#endif // ULTRASONIC_SENSOR_H
