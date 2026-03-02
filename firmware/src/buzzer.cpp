// buzzer.cpp — Passive Buzzer Subscriber
// Subscribes to /buzzer (std_msgs/UInt16).
// Value 0 = silence, any other value = frequency in Hz.
// Uses ESP32 LEDC peripheral for tone generation.
// Note: ESP32 Arduino Core 2.0.x uses the legacy LEDC API
//       (ledcSetup/ledcAttachPin/ledcWrite with channel numbers).

#include "buzzer.h"
#include "otto_config.h"
#include <Arduino.h>
#include <std_msgs/msg/u_int16.h>

// Use a high LEDC channel to avoid collision with ESP32Servo, which
// allocates channels starting from 0.  Channels 8-15 belong to the
// low-speed timer group and are never touched by ESP32Servo.
#define BUZZER_LEDC_CHANNEL  15
#define BUZZER_LEDC_BITS     8  // 8-bit resolution

static rcl_subscription_t buzzer_sub;
static std_msgs__msg__UInt16 buzzer_msg;
static bool buzzer_attached = false;

static void buzzer_attach(uint32_t freq) {
    ledcSetup(BUZZER_LEDC_CHANNEL, freq, BUZZER_LEDC_BITS);
    ledcAttachPin(PIN_BUZZER, BUZZER_LEDC_CHANNEL);
    buzzer_attached = true;
}

static void buzzer_callback(const void *msgin) {
    const std_msgs__msg__UInt16 *msg = (const std_msgs__msg__UInt16 *)msgin;

    if (msg->data == 0) {
        buzzer_off();
    } else {
        buzzer_attach(msg->data);
        ledcWrite(BUZZER_LEDC_CHANNEL, 128);  // 50% duty cycle for max volume
    }
}

void buzzer_init() {
    // No init needed — LEDC channel attached on first use
}

void buzzer_create_entities(rcl_node_t *node, rclc_support_t *support, rcl_allocator_t *allocator) {
    rclc_subscription_init_best_effort(
        &buzzer_sub, node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, UInt16),
        "buzzer");
}

void buzzer_add_to_executor(rclc_executor_t *executor) {
    rclc_executor_add_subscription(executor, &buzzer_sub, &buzzer_msg, &buzzer_callback, ON_NEW_DATA);
}

void buzzer_destroy_entities(rcl_node_t *node) {
    rcl_subscription_fini(&buzzer_sub, node);
}

void buzzer_off() {
    if (buzzer_attached) {
        ledcWrite(BUZZER_LEDC_CHANNEL, 0);
    }
}

void buzzer_tone(uint16_t freq_hz, uint16_t duration_ms) {
    if (freq_hz == 0) {
        buzzer_off();
        return;
    }
    buzzer_attach(freq_hz);
    ledcWrite(BUZZER_LEDC_CHANNEL, 128);
    delay(duration_ms);
    ledcWrite(BUZZER_LEDC_CHANNEL, 0);
}

void buzzer_beep() {
    buzzer_tone(1000, 50);
}
