// led_controller.cpp — WS2812B NeoPixel LED Subscriber
// Subscribes to /leds (std_msgs/UInt8MultiArray).
// 13-LED ring on GPIO 4 (always available).
// 6 ultrasonic LEDs on GPIO 18 (only when SERVO_TYPE_SERIAL_BUS=0).
//
// Message format:
//   Byte 0: target (0=ring, 1=ultrasonic LEDs, 2=both)
//   Byte 1: mode (0=set all same color, 1=individual LEDs, 2=off)
//   Bytes 2-4: R, G, B (for mode 0)
//   Bytes 2+: R,G,B triplets per LED (for mode 1)

#include "led_controller.h"
#include "otto_config.h"
#include <Arduino.h>
#include <Adafruit_NeoPixel.h>
#include <std_msgs/msg/u_int8_multi_array.h>

static rcl_subscription_t led_sub;
static std_msgs__msg__UInt8MultiArray led_msg;

// Static buffer for incoming LED data (max: 1 + 1 + 13*3 = 41 bytes)
static uint8_t led_data_buf[64];

static Adafruit_NeoPixel ring(LED_RING_COUNT, PIN_LED_RING, NEO_GRB + NEO_KHZ800);

#if !SERVO_TYPE_SERIAL_BUS
static Adafruit_NeoPixel us_leds(LED_US_COUNT, PIN_US_NEOPIXEL, NEO_GRB + NEO_KHZ800);
#endif

static void set_strip_color(Adafruit_NeoPixel &strip, uint8_t r, uint8_t g, uint8_t b) {
    for (uint16_t i = 0; i < strip.numPixels(); i++) {
        strip.setPixelColor(i, strip.Color(r, g, b));
    }
    strip.show();
}

static void set_strip_individual(Adafruit_NeoPixel &strip, const uint8_t *rgb_data, size_t data_len) {
    uint16_t n = strip.numPixels();
    for (uint16_t i = 0; i < n && (i * 3 + 2) < data_len; i++) {
        strip.setPixelColor(i, strip.Color(rgb_data[i * 3], rgb_data[i * 3 + 1], rgb_data[i * 3 + 2]));
    }
    strip.show();
}

static void led_callback(const void *msgin) {
    const std_msgs__msg__UInt8MultiArray *msg = (const std_msgs__msg__UInt8MultiArray *)msgin;

    if (msg->data.size < 2) return;

    uint8_t target = msg->data.data[0];
    uint8_t mode = msg->data.data[1];

    bool do_ring = (target == 0 || target == 2);
    bool do_us = (target == 1 || target == 2);

    if (mode == 0 && msg->data.size >= 5) {
        // Set all same color
        uint8_t r = msg->data.data[2];
        uint8_t g = msg->data.data[3];
        uint8_t b = msg->data.data[4];
        if (do_ring) set_strip_color(ring, r, g, b);
#if !SERVO_TYPE_SERIAL_BUS
        if (do_us) set_strip_color(us_leds, r, g, b);
#endif
    } else if (mode == 1 && msg->data.size > 2) {
        // Individual LED colors
        const uint8_t *rgb = &msg->data.data[2];
        size_t rgb_len = msg->data.size - 2;
        if (do_ring) set_strip_individual(ring, rgb, rgb_len);
#if !SERVO_TYPE_SERIAL_BUS
        if (do_us) set_strip_individual(us_leds, rgb, rgb_len);
#endif
    } else if (mode == 2) {
        // Off
        if (do_ring) set_strip_color(ring, 0, 0, 0);
#if !SERVO_TYPE_SERIAL_BUS
        if (do_us) set_strip_color(us_leds, 0, 0, 0);
#endif
    }
}

void led_init() {
    ring.begin();
    ring.setBrightness(50);
    ring.show();
#if !SERVO_TYPE_SERIAL_BUS
    us_leds.begin();
    us_leds.setBrightness(50);
    us_leds.show();
#endif
}

void led_create_entities(rcl_node_t *node, rclc_support_t *support, rcl_allocator_t *allocator) {
    rclc_subscription_init_best_effort(
        &led_sub, node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, UInt8MultiArray),
        "leds");

    // Point message data to static buffer
    led_msg.data.data = led_data_buf;
    led_msg.data.size = 0;
    led_msg.data.capacity = sizeof(led_data_buf);
}

void led_add_to_executor(rclc_executor_t *executor) {
    rclc_executor_add_subscription(executor, &led_sub, &led_msg, &led_callback, ON_NEW_DATA);
}

void led_destroy_entities(rcl_node_t *node) {
    rcl_subscription_fini(&led_sub, node);
}

void led_off() {
    set_strip_color(ring, 0, 0, 0);
#if !SERVO_TYPE_SERIAL_BUS
    set_strip_color(us_leds, 0, 0, 0);
#endif
}
