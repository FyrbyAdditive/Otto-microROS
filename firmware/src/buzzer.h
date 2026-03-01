// buzzer.h — Passive Buzzer Subscriber (PWM tone generation)

#ifndef BUZZER_H
#define BUZZER_H

#include <rcl/rcl.h>
#include <rclc/rclc.h>

void buzzer_init();
void buzzer_create_entities(rcl_node_t *node, rclc_support_t *support, rcl_allocator_t *allocator);
void buzzer_add_to_executor(rclc_executor_t *executor);
void buzzer_destroy_entities(rcl_node_t *node);
void buzzer_off();

// Status indication (usable before micro-ROS is active)
void buzzer_tone(uint16_t freq_hz, uint16_t duration_ms);
void buzzer_beep();

#endif // BUZZER_H
