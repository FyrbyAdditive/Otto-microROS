// drive_controller.h — Differential Drive Controller for Otto Starter
// Supports both PWM continuous rotation servos and serial bus servos.

#ifndef DRIVE_CONTROLLER_H
#define DRIVE_CONTROLLER_H

#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

void drive_init();
void drive_create_entities(rcl_node_t *node, rclc_support_t *support, rcl_allocator_t *allocator);
void drive_add_to_executor(rclc_executor_t *executor);
void drive_destroy_entities(rcl_node_t *node);
void drive_stop();
void drive_check_timeout();

#endif // DRIVE_CONTROLLER_H
