// battery_monitor.h — LiPo Battery State Publisher

#ifndef BATTERY_MONITOR_H
#define BATTERY_MONITOR_H

#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

void battery_init();
void battery_create_entities(rcl_node_t *node, rclc_support_t *support, rcl_allocator_t *allocator);
void battery_add_to_executor(rclc_executor_t *executor);
void battery_destroy_entities(rcl_node_t *node);

#endif // BATTERY_MONITOR_H
