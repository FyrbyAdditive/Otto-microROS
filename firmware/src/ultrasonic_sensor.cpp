// ultrasonic_sensor.cpp — RCWL-9610 Ultrasonic Range Publisher
// Publishes sensor_msgs/Range on /ultrasonic/range at 10Hz.
// Uses GPIO 19 for trigger/echo (single-wire mode).
// Disabled entirely when SERVO_TYPE_SERIAL_BUS=1 (GPIO 18/19 used for servo UART).

#include "ultrasonic_sensor.h"
#include "otto_config.h"
#include <Arduino.h>
#include <sensor_msgs/msg/range.h>

#if !SERVO_TYPE_SERIAL_BUS

static rcl_publisher_t range_pub;
static rcl_timer_t range_timer;
static sensor_msgs__msg__Range range_msg;

// Static frame_id string (avoids heap allocation)
static char us_frame_id[] = "ultrasonic_link";

static void range_timer_callback(rcl_timer_t *timer, int64_t last_call_time) {
    (void)last_call_time;
    if (timer == NULL) return;

    // Trigger pulse: drive pin HIGH for 10us
    pinMode(PIN_US_TRIG_ECHO, OUTPUT);
    digitalWrite(PIN_US_TRIG_ECHO, HIGH);
    delayMicroseconds(10);
    digitalWrite(PIN_US_TRIG_ECHO, LOW);

    // Read echo pulse width
    pinMode(PIN_US_TRIG_ECHO, INPUT);
    unsigned long duration = pulseIn(PIN_US_TRIG_ECHO, HIGH, 30000);  // 30ms timeout (~5m)

    float distance_m;
    if (duration == 0) {
        distance_m = INFINITY;  // No echo received (ROS convention)
    } else {
        distance_m = (duration * 0.000343f) / 2.0f;
        if (distance_m < 0.02f || distance_m > 4.0f) {
            distance_m = INFINITY;
        }
    }

    // Fill range message
    range_msg.radiation_type = sensor_msgs__msg__Range__ULTRASOUND;
    range_msg.field_of_view = 0.2618f;  // 15 degrees in radians
    range_msg.min_range = 0.02f;
    range_msg.max_range = 4.0f;
    range_msg.range = distance_m;

    // Timestamp (if time sync is available, otherwise zero)
    int64_t nanos = rmw_uros_epoch_nanos();
    range_msg.header.stamp.sec = (int32_t)(nanos / 1000000000LL);
    range_msg.header.stamp.nanosec = (uint32_t)(nanos % 1000000000LL);

    rcl_publish(&range_pub, &range_msg, NULL);
}

void ultrasonic_init() {
    // No persistent GPIO setup needed — pin mode switches each reading cycle
}

void ultrasonic_create_entities(rcl_node_t *node, rclc_support_t *support, rcl_allocator_t *allocator) {
    rclc_publisher_init_best_effort(
        &range_pub, node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Range),
        "ultrasonic/range");

    rclc_timer_init_default(
        &range_timer, support,
        RCL_MS_TO_NS(US_PUBLISH_PERIOD_MS),
        range_timer_callback);

    // Set static frame_id
    range_msg.header.frame_id.data = us_frame_id;
    range_msg.header.frame_id.size = strlen(us_frame_id);
    range_msg.header.frame_id.capacity = sizeof(us_frame_id);
}

void ultrasonic_add_to_executor(rclc_executor_t *executor) {
    rclc_executor_add_timer(executor, &range_timer);
}

void ultrasonic_destroy_entities(rcl_node_t *node) {
    rcl_publisher_fini(&range_pub, node);
    rcl_timer_fini(&range_timer);
}

#else
// Stubs when ultrasonic is disabled (serial bus servo mode)
void ultrasonic_init() {}
void ultrasonic_create_entities(rcl_node_t *node, rclc_support_t *support, rcl_allocator_t *allocator) {
    (void)node; (void)support; (void)allocator;
}
void ultrasonic_add_to_executor(rclc_executor_t *executor) { (void)executor; }
void ultrasonic_destroy_entities(rcl_node_t *node) { (void)node; }
#endif
