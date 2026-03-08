// led_ring_status.cpp — Ring LED status animation
// Idle: dim blue ring, purple center.
// Moving: purple indicator on ring at travel direction, dim blue elsewhere.
//
// LED layout (clock face):
//   LED 0  = center
//   LED 12 = 12 o'clock (forward)
//   LED 3  = 3 o'clock  (right)
//   LED 6  = 6 o'clock  (backward)
//   LED 9  = 9 o'clock  (left)

#include "led_ring_status.h"
#include "led_controller.h"
#include "otto_config.h"

// Colors
static const uint8_t BLUE_R = 0, BLUE_G = 0, BLUE_B = 30;
static const uint8_t PURP_R = 60, PURP_G = 0, PURP_B = 60;

static double vel_linear = 0.0;
static double vel_angular = 0.0;
static bool dirty = true;  // Force initial draw

void led_ring_status_start() {
    vel_linear = 0.0;
    vel_angular = 0.0;
    dirty = true;
}

void led_ring_status_set_velocity(double linear_x, double angular_z) {
    if (linear_x != vel_linear || angular_z != vel_angular) {
        vel_linear = linear_x;
        vel_angular = angular_z;
        dirty = true;
    }
}

// Map velocity signs to ring LED index (1-12).
// Uses sign-based lookup for clean 8-direction mapping to clock positions.
//   fwd/turn:  (+,0)=12  (+,+)=11  (0,+)=9  (-,+)=7  (-,0)=6  (-,-)=5  (0,-)=3  (+,-)=1
static int direction_to_led(double linear, double angular) {
    int fwd  = (linear > 0.001) ? 1 : (linear < -0.001) ? -1 : 0;
    int turn = (angular > 0.001) ? 1 : (angular < -0.001) ? -1 : 0;

    // 3x3 lookup table indexed by [fwd+1][turn+1]
    //              turn: -1   0   +1
    // fwd  +1:           1   12   11     (forward-right, forward, forward-left)
    // fwd   0:           3    0    9     (right, stopped, left)
    // fwd  -1:           5    6    7     (backward-right, backward, backward-left)
    static const int led_map[3][3] = {
        { 5, 6, 7 },   // fwd = -1
        { 3, 0, 9 },   // fwd =  0  (0 means stopped — won't be used)
        { 1, 12, 11 }, // fwd = +1
    };
    return led_map[fwd + 1][turn + 1];
}

void led_ring_status_update() {
    // While a timed ROS command is active, suppress firmware animation.
    // Track transitions so the ring redraws immediately when override expires.
    static bool was_overridden = false;
    bool overridden = led_ring_overridden();
    if (overridden) {
        was_overridden = true;
        return;
    }
    if (was_overridden) {
        was_overridden = false;
        dirty = true;  // force redraw now that override expired
    }

    if (!dirty) return;
    dirty = false;

    bool moving = (vel_linear > 0.001 || vel_linear < -0.001 ||
                   vel_angular > 0.001 || vel_angular < -0.001);

    if (!moving) {
        // Idle: purple center, dim blue ring
        led_ring_set_pixel(LED_CENTER_INDEX, PURP_R, PURP_G, PURP_B);
        for (int i = 1; i <= LED_RING_OUTER_COUNT; i++) {
            led_ring_set_pixel(i, BLUE_R, BLUE_G, BLUE_B);
        }
    } else {
        // Moving: dim blue center, purple indicator on ring
        led_ring_set_pixel(LED_CENTER_INDEX, BLUE_R, BLUE_G, BLUE_B);

        int target = direction_to_led(vel_linear, vel_angular);

        for (int i = 1; i <= LED_RING_OUTER_COUNT; i++) {
            // Light 3 adjacent LEDs: target and its two neighbors
            int d = abs(i - target);
            if (d > 6) d = 12 - d;  // Wrap-around distance on ring
            if (d <= 1) {
                led_ring_set_pixel(i, PURP_R, PURP_G, PURP_B);
            } else {
                led_ring_set_pixel(i, BLUE_R, BLUE_G, BLUE_B);
            }
        }
    }

    led_ring_show();
}
