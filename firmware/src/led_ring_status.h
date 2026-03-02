// led_ring_status.h — Ring LED status animation
// Shows idle/moving state on the 13-LED eye module.

#ifndef LED_RING_STATUS_H
#define LED_RING_STATUS_H

// Call once when entering AGENT_CONNECTED state
void led_ring_status_start();

// Call from cmd_vel callback and drive_stop with current velocities
void led_ring_status_set_velocity(double linear_x, double angular_z);

// Call each loop iteration in AGENT_CONNECTED state
void led_ring_status_update();

#endif // LED_RING_STATUS_H
