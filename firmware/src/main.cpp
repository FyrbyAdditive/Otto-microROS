// main.cpp — HP Robots Otto Starter Kit micro-ROS Firmware
// =========================================================
// ESP32-WROOM-32E-N8 running micro-ROS over WiFi/UDP.
// Single node "otto_starter" with publishers for sensors and
// subscribers for actuators, managed through one rclc_executor.
//
// State machine handles agent connection lifecycle:
//   WAITING_AGENT → AGENT_AVAILABLE → AGENT_CONNECTED → AGENT_DISCONNECTED
//
// Safety: servos stop on agent disconnect or cmd_vel timeout.

#include <Arduino.h>
#include <WiFi.h>
#include <micro_ros_platformio.h>

#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <rmw_microxrcedds_c/config.h>
#include <micro_ros_utilities/type_utilities.h>

#include "otto_config.h"
#include "drive_controller.h"
#include "ultrasonic_sensor.h"
#include "line_sensor.h"
#include "battery_monitor.h"
#include "led_controller.h"
#include "led_ring_status.h"
#include "buzzer.h"

// WiFi credentials (user must copy wifi_credentials.h.example)
#if __has_include("wifi_credentials.h")
#include "wifi_credentials.h"
#else
#error "Missing wifi_credentials.h — copy wifi_credentials.h.example to wifi_credentials.h and edit it."
#endif

// Agent connection state machine
enum AgentState {
    WAITING_AGENT,
    AGENT_AVAILABLE,
    AGENT_CONNECTED,
    AGENT_DISCONNECTED
};

static AgentState state = WAITING_AGENT;

// micro-ROS core handles
static rcl_allocator_t allocator;
static rclc_support_t support;
static rcl_node_t node;
static rclc_executor_t executor;

// Executor handle count:
//   3 subscriptions (cmd_vel, leds, buzzer)
//   3 timers (ultrasonic, line_sensors, battery) — ultrasonic may be 0 if serial bus servo mode
#if SERVO_TYPE_SERIAL_BUS
#define EXECUTOR_HANDLE_COUNT 5  // 3 subs + 2 timers (no ultrasonic timer)
#else
#define EXECUTOR_HANDLE_COUNT 6  // 3 subs + 3 timers
#endif

// Error handling macros
#define RCCHECK(fn) { rcl_ret_t rc = (fn); if (rc != RCL_RET_OK) { return false; } }
#define RCSOFTCHECK(fn) { rcl_ret_t rc = (fn); (void)rc; }

static bool create_entities() {
    allocator = rcl_get_default_allocator();

    RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));
    RCCHECK(rclc_node_init_default(&node, "otto_starter", "", &support));

    // Create all publisher/subscriber entities
    drive_create_entities(&node, &support, &allocator);
    ultrasonic_create_entities(&node, &support, &allocator);
    line_sensor_create_entities(&node, &support, &allocator);
    battery_create_entities(&node, &support, &allocator);
    led_create_entities(&node, &support, &allocator);
    buzzer_create_entities(&node, &support, &allocator);

    // Initialize executor and add all handles
    RCCHECK(rclc_executor_init(&executor, &support.context, EXECUTOR_HANDLE_COUNT, &allocator));

    drive_add_to_executor(&executor);
    ultrasonic_add_to_executor(&executor);
    line_sensor_add_to_executor(&executor);
    battery_add_to_executor(&executor);
    led_add_to_executor(&executor);
    buzzer_add_to_executor(&executor);

    // Sync time with the agent
    RCSOFTCHECK(rmw_uros_sync_session(1000));

    return true;
}

static void destroy_entities() {
    // Safety: stop all actuators immediately
    drive_stop();
    led_off();
    buzzer_off();

    RCSOFTCHECK(rclc_executor_fini(&executor));

    drive_destroy_entities(&node);
    ultrasonic_destroy_entities(&node);
    line_sensor_destroy_entities(&node);
    battery_destroy_entities(&node);
    led_destroy_entities(&node);
    buzzer_destroy_entities(&node);

    RCSOFTCHECK(rcl_node_fini(&node));
    RCSOFTCHECK(rclc_support_fini(&support));
}

void setup() {
    Serial.begin(115200);

    // Status LED
    pinMode(PIN_LED_BUILTIN, OUTPUT);
    digitalWrite(PIN_LED_BUILTIN, LOW);

    // Initialize all hardware modules
    drive_init();
    ultrasonic_init();
    line_sensor_init();
    battery_init();
    led_init();
    buzzer_init();

    // Stage 1: Power on — orange LEDs + startup beep
    Serial.println("[Otto] Booting...");
    led_status_color(255, 80, 0);  // Orange
    buzzer_beep();

    // Stage 2: Connecting WiFi — yellow LEDs
    Serial.println("[Otto] Connecting to WiFi...");
    led_status_color(255, 200, 0);  // Yellow

    // Apply static IP BEFORE WiFi.begin() — ESP32 requires this order.
    // WiFi.config() after WiFi.begin() can break the network stack.
#ifdef STATIC_IP
    {
        IPAddress static_ip, gateway, subnet;
        static_ip.fromString(STATIC_IP);
        gateway.fromString(STATIC_GATEWAY);
        subnet.fromString(STATIC_SUBNET);
        WiFi.config(static_ip, gateway, subnet);
        Serial.print("[Otto] Using static IP: ");
        Serial.println(STATIC_IP);
    }
#endif

    // Connect WiFi and register micro-ROS UDP transport
    IPAddress agent_ip;
    agent_ip.fromString(AGENT_IP);
    char ssid[] = WIFI_SSID;
    char pass[] = WIFI_PASSWORD;
    set_microros_wifi_transports(ssid, pass, agent_ip, AGENT_PORT);

    Serial.print("[Otto] WiFi IP: ");
    Serial.println(WiFi.localIP());

    // Stage 3: WiFi connected — cyan LEDs + beep
    Serial.println("[Otto] WiFi connected, waiting for micro-ROS agent...");
    led_status_color(0, 200, 255);  // Cyan
    buzzer_beep();

    state = WAITING_AGENT;
}

void loop() {
    static unsigned long last_state_change = 0;

    switch (state) {
        case WAITING_AGENT:
            // Blink built-in LED + pulse cyan ring while searching for agent
            digitalWrite(PIN_LED_BUILTIN, (millis() / 500) % 2);
            {
                // Pulse cyan brightness using a triangle wave
                uint8_t brightness = (millis() / 4) % 256;
                if (brightness > 127) brightness = 255 - brightness;
                brightness = brightness * 2;  // 0-254 range
                led_status_color(0, (brightness * 200) / 255, brightness);
            }

            // Proximity LEDs work even without agent
            ultrasonic_standalone_read();

            if (millis() - last_state_change > AGENT_RECONNECT_MS) {
                if (rmw_uros_ping_agent(AGENT_PING_TIMEOUT_MS, AGENT_PING_ATTEMPTS) == RMW_RET_OK) {
                    Serial.println("[Otto] Agent found!");
                    state = AGENT_AVAILABLE;
                    last_state_change = millis();
                }
            }
            break;

        case AGENT_AVAILABLE:
            if (create_entities()) {
                Serial.println("[Otto] micro-ROS entities created, connected!");
                state = AGENT_CONNECTED;
                last_state_change = millis();
                digitalWrite(PIN_LED_BUILTIN, HIGH);  // Solid on

                // Connected — green LEDs + ascending two-tone jingle
                led_status_color(0, 255, 0);  // Green
                buzzer_tone(880, 80);
                delay(30);
                buzzer_tone(1320, 120);

                // Start ring status animation (idle: purple center, dim blue ring)
                led_ring_status_start();
                led_ring_status_update();
            } else {
                Serial.println("[Otto] Failed to create entities, retrying...");
                state = WAITING_AGENT;
                last_state_change = millis();
            }
            break;

        case AGENT_CONNECTED:
            // Spin the executor (processes all callbacks)
            RCSOFTCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(10)));

            // Check cmd_vel timeout (safety stop)
            drive_check_timeout();

            // Update ring LED status animation
            led_ring_status_update();

            // Periodic agent ping to detect disconnection
            if (millis() - last_state_change > 2000) {
                if (rmw_uros_ping_agent(AGENT_PING_TIMEOUT_MS, AGENT_PING_ATTEMPTS) != RMW_RET_OK) {
                    Serial.println("[Otto] Agent lost, disconnecting...");
                    state = AGENT_DISCONNECTED;
                    last_state_change = millis();
                }
                last_state_change = millis();
            }
            break;

        case AGENT_DISCONNECTED:
            // Disconnected — red flash + warning tone
            led_status_color(255, 0, 0);  // Red
            buzzer_tone(440, 150);
            delay(50);
            buzzer_tone(330, 200);

            destroy_entities();
            Serial.println("[Otto] Entities destroyed, waiting for agent...");
            state = WAITING_AGENT;
            last_state_change = millis();
            digitalWrite(PIN_LED_BUILTIN, LOW);
            // Cyan will resume on next loop iteration (WAITING_AGENT pulse)
            break;
    }
}
