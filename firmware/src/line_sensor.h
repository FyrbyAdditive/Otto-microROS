// line_sensor.h — IR Line Sensor ADC Publisher

#ifndef LINE_SENSOR_H
#define LINE_SENSOR_H

#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

void line_sensor_init();
void line_sensor_create_entities(rcl_node_t *node, rclc_support_t *support, rcl_allocator_t *allocator);
void line_sensor_add_to_executor(rclc_executor_t *executor);
void line_sensor_destroy_entities(rcl_node_t *node);

#endif // LINE_SENSOR_H
