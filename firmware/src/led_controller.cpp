// led_controller.cpp — WS2812B NeoPixel subscriber + state publisher
// See led_controller.h for the full /leds command and /led_state publish protocol.
//
// 13-LED ring on GPIO 4 (always available).
// 6 ultrasonic LEDs on GPIO 18 (always available — servo bus uses Connector 2).

#include "led_controller.h"
#include "otto_config.h"
#include <Arduino.h>
#include <Adafruit_NeoPixel.h>
#include <std_msgs/msg/u_int8_multi_array.h>
#include <math.h>

// ── Subscriber ────────────────────────────────────────────────────────────────
static rcl_subscription_t             led_sub;
static std_msgs__msg__UInt8MultiArray led_msg;
static uint8_t                        led_data_buf[64];  // max: 2 + 2 + 13*3 = 43

// ── Publisher ─────────────────────────────────────────────────────────────────
static rcl_publisher_t                led_state_pub;
static bool                           led_pub_ready = false;
// Format: [target=0, mode=1, R0,G0,B0, ..., R12,G12,B12] = 2 + 13*3 = 41 bytes
static uint8_t                        led_state_buf[2 + LED_RING_COUNT * 3];
static std_msgs__msg__UInt8MultiArray led_state_msg;

// ── Override state ────────────────────────────────────────────────────────────
static uint32_t led_override_until_ms   = 0;
static bool     led_override_indefinite = false;

// ── Hardware ──────────────────────────────────────────────────────────────────
static Adafruit_NeoPixel ring(LED_RING_COUNT, PIN_LED_RING, NEO_GRB + NEO_KHZ800);
static Adafruit_NeoPixel us_leds(LED_US_COUNT, PIN_US_NEOPIXEL, NEO_GRB + NEO_KHZ800);

// ── Helpers ───────────────────────────────────────────────────────────────────
static void set_strip_color(Adafruit_NeoPixel &strip, uint8_t r, uint8_t g, uint8_t b) {
    for (uint16_t i = 0; i < strip.numPixels(); i++) {
        strip.setPixelColor(i, strip.Color(r, g, b));
    }
    strip.show();
}

static void set_strip_individual(Adafruit_NeoPixel &strip,
                                 const uint8_t *rgb_data, size_t data_len) {
    uint16_t n = strip.numPixels();
    for (uint16_t i = 0; i < n && (i * 3 + 2) < data_len; i++) {
        strip.setPixelColor(i, strip.Color(rgb_data[i * 3],
                                           rgb_data[i * 3 + 1],
                                           rgb_data[i * 3 + 2]));
    }
    strip.show();
}

static void led_state_publish() {
    if (!led_pub_ready) return;
    led_state_buf[0] = 0;  // target: ring
    led_state_buf[1] = 1;  // mode: individual
    for (uint16_t i = 0; i < LED_RING_COUNT; i++) {
        uint32_t c = ring.getPixelColor(i);
        led_state_buf[2 + i * 3]     = (c >> 16) & 0xFF;  // R
        led_state_buf[2 + i * 3 + 1] = (c >>  8) & 0xFF;  // G
        led_state_buf[2 + i * 3 + 2] =  c        & 0xFF;  // B
    }
    led_state_msg.data.size = sizeof(led_state_buf);
    rcl_publish(&led_state_pub, &led_state_msg, NULL);
}

// ── Override query ────────────────────────────────────────────────────────────
bool led_ring_overridden() {
    if (led_override_indefinite) return true;
    if (led_override_until_ms == 0) return false;
    if ((int32_t)(millis() - led_override_until_ms) >= 0) {
        led_override_until_ms = 0;  // expired — clear
        return false;
    }
    return true;
}

// ── Subscriber callback ───────────────────────────────────────────────────────
static void led_callback(const void *msgin) {
    const std_msgs__msg__UInt8MultiArray *msg =
        (const std_msgs__msg__UInt8MultiArray *)msgin;

    if (msg->data.size < 2) return;

    uint8_t target = msg->data.data[0];
    uint8_t mode   = msg->data.data[1];

    bool do_ring = (target == 0 || target == 2);
    bool do_us   = (target == 1 || target == 2);

    // Mode 3: release — immediately return control to firmware animation
    if (mode == 3) {
        led_override_indefinite = false;
        led_override_until_ms   = 0;
        return;
    }

    // Bytes 2-3: uint16 override duration ms (big-endian)
    // 0x0000 or 0xFFFF = indefinite; 1-65534 = timed
    uint16_t dur_ms = 0;  // default: indefinite
    if (msg->data.size >= 4) {
        dur_ms = ((uint16_t)msg->data.data[2] << 8) | msg->data.data[3];
    }
    if (dur_ms == 0 || dur_ms == 0xFFFF) {
        led_override_indefinite = true;
        led_override_until_ms   = 0;
    } else {
        led_override_indefinite = false;
        led_override_until_ms   = millis() + dur_ms;
    }

    // Colour payload starts at byte 4 (or byte 2 for legacy 3/5-byte messages)
    const uint8_t *payload     = msg->data.data + 4;
    size_t         payload_len = (msg->data.size >= 4) ? msg->data.size - 4 : 0;

    if (mode == 0 && payload_len >= 3) {
        if (do_ring) set_strip_color(ring, payload[0], payload[1], payload[2]);
        if (do_us) set_strip_color(us_leds, payload[0], payload[1], payload[2]);
    } else if (mode == 1 && payload_len > 0) {
        if (do_ring) set_strip_individual(ring, payload, payload_len);
        if (do_us) set_strip_individual(us_leds, payload, payload_len);
    } else if (mode == 2) {
        if (do_ring) set_strip_color(ring, 0, 0, 0);
        if (do_us) set_strip_color(us_leds, 0, 0, 0);
    }

    if (do_ring) led_state_publish();
}

// ── Public API ────────────────────────────────────────────────────────────────
void led_init() {
    ring.begin();
    ring.setBrightness(50);
    ring.show();
    us_leds.begin();
    us_leds.setBrightness(50);
    us_leds.show();
}

void led_create_entities(rcl_node_t *node, rclc_support_t *support,
                         rcl_allocator_t *allocator) {
    rclc_subscription_init_best_effort(
        &led_sub, node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, UInt8MultiArray),
        "leds");
    led_msg.data.data     = led_data_buf;
    led_msg.data.size     = 0;
    led_msg.data.capacity = sizeof(led_data_buf);

    rclc_publisher_init_best_effort(
        &led_state_pub, node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, UInt8MultiArray),
        "led_state");
    led_state_msg.data.data     = led_state_buf;
    led_state_msg.data.capacity = sizeof(led_state_buf);
    led_state_msg.data.size     = sizeof(led_state_buf);
    led_pub_ready = true;
}

void led_add_to_executor(rclc_executor_t *executor) {
    rclc_executor_add_subscription(executor, &led_sub, &led_msg,
                                   &led_callback, ON_NEW_DATA);
}

void led_destroy_entities(rcl_node_t *node) {
    led_pub_ready           = false;
    led_override_indefinite = false;
    led_override_until_ms   = 0;
    rcl_subscription_fini(&led_sub, node);
    rcl_publisher_fini(&led_state_pub, node);
}

void led_off() {
    set_strip_color(ring, 0, 0, 0);
    set_strip_color(us_leds, 0, 0, 0);
}

void led_status_color(uint8_t r, uint8_t g, uint8_t b) {
    set_strip_color(ring, r, g, b);
    // Ultrasonic LEDs reserved for proximity indicator
}

void led_status_off() {
    led_off();
}

void led_ring_set_pixel(uint16_t index, uint8_t r, uint8_t g, uint8_t b) {
    ring.setPixelColor(index, ring.Color(r, g, b));
}

void led_ring_show() {
    ring.show();
    led_state_publish();
}

void led_proximity(float distance_m) {
    if (distance_m <= 0.0f || isinf(distance_m) || isnan(distance_m)) {
        set_strip_color(us_leds, 0, 0, 0);
        return;
    }
    if (distance_m < 0.02f) distance_m = 0.02f;
    if (distance_m > 1.5f)  distance_m = 1.5f;

    uint8_t r, g, b = 0;
    if (distance_m < 0.3f) {
        float t = (distance_m - 0.02f) / (0.3f - 0.02f);
        r = 255;
        g = (uint8_t)(80.0f * t);
    } else {
        float t = (distance_m - 0.3f) / (1.0f - 0.3f);
        if (t > 1.0f) t = 1.0f;
        r = (uint8_t)(255.0f * (1.0f - t));
        g = (uint8_t)(80.0f + 175.0f * t);
    }
    set_strip_color(us_leds, r, g, b);
}
