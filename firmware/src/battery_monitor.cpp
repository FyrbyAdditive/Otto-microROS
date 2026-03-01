// battery_monitor.cpp — LiPo Battery State Publisher
// Publishes sensor_msgs/BatteryState on /battery_state at 1Hz.
// GPIO 39 ADC through voltage divider on PCB.

#include "battery_monitor.h"
#include "otto_config.h"
#include <Arduino.h>
#include <sensor_msgs/msg/battery_state.h>
#include <math.h>

static rcl_publisher_t battery_pub;
static rcl_timer_t battery_timer;
static sensor_msgs__msg__BatteryState battery_msg;

static char battery_frame_id[] = "base_link";

static void battery_timer_callback(rcl_timer_t *timer, int64_t last_call_time) {
    (void)last_call_time;
    if (timer == NULL) return;

    int raw = analogRead(PIN_BATTERY_ADC);
    float raw_voltage = raw * (3.3f / 4095.0f);
    float cell_voltage = raw_voltage * BATTERY_DIVIDER_RATIO;

    // Linear percentage: 3.0V = 0%, 4.2V = 100%
    float pct = (cell_voltage - BATTERY_EMPTY_V) / (BATTERY_FULL_V - BATTERY_EMPTY_V);
    pct = constrain(pct, 0.0f, 1.0f);

    battery_msg.voltage = cell_voltage;
    battery_msg.percentage = pct;
    battery_msg.power_supply_status = sensor_msgs__msg__BatteryState__POWER_SUPPLY_STATUS_DISCHARGING;
    battery_msg.power_supply_technology = sensor_msgs__msg__BatteryState__POWER_SUPPLY_TECHNOLOGY_LIPO;
    battery_msg.present = true;

    // Fields we cannot measure — set to NaN per ROS convention
    battery_msg.current = NAN;
    battery_msg.charge = NAN;
    battery_msg.capacity = NAN;
    battery_msg.design_capacity = NAN;
    battery_msg.temperature = NAN;

    int64_t nanos = rmw_uros_epoch_nanos();
    battery_msg.header.stamp.sec = (int32_t)(nanos / 1000000000LL);
    battery_msg.header.stamp.nanosec = (uint32_t)(nanos % 1000000000LL);

    rcl_publish(&battery_pub, &battery_msg, NULL);
}

void battery_init() {
    // GPIO 39 is input-only, no setup needed beyond ADC config
    analogReadResolution(ADC_RESOLUTION);
}

void battery_create_entities(rcl_node_t *node, rclc_support_t *support, rcl_allocator_t *allocator) {
    rclc_publisher_init_best_effort(
        &battery_pub, node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, BatteryState),
        "battery_state");

    rclc_timer_init_default(
        &battery_timer, support,
        RCL_MS_TO_NS(BATTERY_PUBLISH_PERIOD_MS),
        battery_timer_callback);

    // Set static frame_id
    battery_msg.header.frame_id.data = battery_frame_id;
    battery_msg.header.frame_id.size = strlen(battery_frame_id);
    battery_msg.header.frame_id.capacity = sizeof(battery_frame_id);
}

void battery_add_to_executor(rclc_executor_t *executor) {
    rclc_executor_add_timer(executor, &battery_timer);
}

void battery_destroy_entities(rcl_node_t *node) {
    rcl_publisher_fini(&battery_pub, node);
    rcl_timer_fini(&battery_timer);
}
