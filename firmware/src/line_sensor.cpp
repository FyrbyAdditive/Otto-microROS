// line_sensor.cpp — IR Line Sensor ADC Publisher
// Publishes std_msgs/Int32MultiArray on /line_sensors at 20Hz.
// Left: GPIO 32 (Connector 6), Right: GPIO 33 (Connector 7)

#include "line_sensor.h"
#include "otto_config.h"
#include <Arduino.h>
#include <std_msgs/msg/int32_multi_array.h>

static rcl_publisher_t line_pub;
static rcl_timer_t line_timer;
static std_msgs__msg__Int32MultiArray line_msg;

// Static data array (2 elements: left, right)
static int32_t line_data[2];

static void line_timer_callback(rcl_timer_t *timer, int64_t last_call_time) {
    (void)last_call_time;
    if (timer == NULL) return;

    line_data[0] = analogRead(PIN_IR_LEFT);
    line_data[1] = analogRead(PIN_IR_RIGHT);

    rcl_publish(&line_pub, &line_msg, NULL);
}

void line_sensor_init() {
    analogReadResolution(ADC_RESOLUTION);
    // Note: analogSetAttenuation is global on ESP32 Arduino core
    analogSetAttenuation(ADC_11db);
}

void line_sensor_create_entities(rcl_node_t *node, rclc_support_t *support, rcl_allocator_t *allocator) {
    rclc_publisher_init_best_effort(
        &line_pub, node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32MultiArray),
        "line_sensors");

    rclc_timer_init_default(
        &line_timer, support,
        RCL_MS_TO_NS(LINE_PUBLISH_PERIOD_MS),
        line_timer_callback);

    // Point message data to static array
    line_msg.data.data = line_data;
    line_msg.data.size = 2;
    line_msg.data.capacity = 2;
}

void line_sensor_add_to_executor(rclc_executor_t *executor) {
    rclc_executor_add_timer(executor, &line_timer);
}

void line_sensor_destroy_entities(rcl_node_t *node) {
    rcl_publisher_fini(&line_pub, node);
    rcl_timer_fini(&line_timer);
}
