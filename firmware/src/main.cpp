// main.cpp — HP Robots Otto Starter Kit micro-ROS Firmware
// =========================================================
// ESP32-WROOM-32E-N8 running micro-ROS over WiFi/UDP.
// Single node "otto_starter" with publishers for sensors and
// subscribers for actuators, managed through one rclc_executor.
//
// State machine handles configuration and agent connection lifecycle:
//   CONFIG_PORTAL → WAITING_AGENT → AGENT_AVAILABLE → AGENT_CONNECTED → AGENT_DISCONNECTED
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
#include "wifi_config.h"
#include "wifi_portal.h"

// Agent connection state machine
enum AgentState {
    CONFIG_PORTAL,      // Captive portal for WiFi/agent setup
    WAITING_AGENT,
    AGENT_AVAILABLE,
    AGENT_CONNECTED,
    AGENT_DISCONNECTED
};

static AgentState state = WAITING_AGENT;
static int ping_fail_count = 0;  // consecutive ping failures; reset on success or reconnect

// Runtime WiFi config loaded from NVS
static WifiConfig g_cfg = {};

// Scan for all APs with our SSID, pick the one with the strongest signal, and
// connect with the BSSID locked so the ESP32 cannot roam mid-session.
// Uses credentials from g_cfg (loaded from NVS).
// Returns true on success, false if SSID not found or association timed out.
static bool wifi_scan_and_connect() {
    Serial.println("[Otto] Scanning for best AP...");
    int n = WiFi.scanNetworks();
    uint8_t best_bssid[6] = {};
    int best_channel = 0;
    int best_rssi = -200;
    for (int i = 0; i < n; i++) {
        if (WiFi.SSID(i) == g_cfg.ssid && WiFi.RSSI(i) > best_rssi) {
            best_rssi = WiFi.RSSI(i);
            best_channel = WiFi.channel(i);
            memcpy(best_bssid, WiFi.BSSID(i), 6);
        }
    }
    WiFi.scanDelete();

    if (best_rssi == -200) {
        Serial.printf("[Otto] SSID '%s' not found in scan\n", g_cfg.ssid);
        return false;
    }

    Serial.printf("[Otto] Best AP: %02X:%02X:%02X:%02X:%02X:%02X ch%d RSSI%d\n",
        best_bssid[0], best_bssid[1], best_bssid[2],
        best_bssid[3], best_bssid[4], best_bssid[5],
        best_channel, best_rssi);

    WiFi.begin(g_cfg.ssid, g_cfg.pass, best_channel, best_bssid);
    unsigned long t = millis();
    while (WiFi.status() != WL_CONNECTED && millis() - t < 15000) delay(100);

    if (WiFi.status() != WL_CONNECTED) {
        Serial.println("[Otto] WiFi association timed out");
        return false;
    }
    return true;
}

// Reconnect after a real WiFi drop — scan so we pick the best AP again,
// then connect with BSSID locked.  Only called from WAITING_AGENT.
static void wifi_reconnect() {
    Serial.println("[Otto] WiFi dropped — reconnecting...");
    led_status_color(255, 200, 0);  // Yellow
    if (wifi_scan_and_connect()) {
        Serial.print("[Otto] WiFi reconnected: ");
        Serial.println(WiFi.localIP());
    } else {
        Serial.println("[Otto] WiFi reconnect failed, will retry...");
    }
}

// micro-ROS core handles
static rcl_allocator_t allocator;
static rclc_support_t support;
static rcl_node_t node;
static rclc_executor_t executor;

// Executor handle count: 3 subscriptions (cmd_vel, leds, buzzer) + 3 timers (ultrasonic, line_sensors, battery)
#define EXECUTOR_HANDLE_COUNT 6

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

    // Log reset reason — helps diagnose brownout/watchdog resets
    {
        esp_reset_reason_t reason = esp_reset_reason();
        const char* rs;
        switch (reason) {
            case ESP_RST_POWERON:   rs = "POWER_ON";      break;
            case ESP_RST_SW:        rs = "SOFTWARE";       break;
            case ESP_RST_PANIC:     rs = "PANIC/CRASH";    break;
            case ESP_RST_INT_WDT:   rs = "INT_WATCHDOG";  break;
            case ESP_RST_TASK_WDT:  rs = "TASK_WATCHDOG"; break;
            case ESP_RST_WDT:       rs = "WATCHDOG";       break;
            case ESP_RST_DEEPSLEEP: rs = "DEEP_SLEEP";     break;
            case ESP_RST_BROWNOUT:  rs = "BROWNOUT *** CHECK BATTERY/POWER ***"; break;
            default:                rs = "UNKNOWN";        break;
        }
        Serial.printf("[Otto] Reset reason: %s\n", rs);
    }

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

    // Stage 2: Load config from NVS
    if (!wifi_config_load(g_cfg)) {
        // No saved config — enter captive portal
        Serial.println("[Otto] No WiFi config found, starting setup portal...");
        led_status_color(170, 0, 255);  // Magenta
        buzzer_tone(660, 100);
        delay(50);
        buzzer_tone(880, 100);
        portal_start();
        state = CONFIG_PORTAL;
        return;
    }

    // Stage 3: Connecting WiFi — yellow LEDs
    Serial.printf("[Otto] Connecting to '%s'...\n", g_cfg.ssid);
    led_status_color(255, 200, 0);  // Yellow

    // WiFi settings applied before first scan/connect so they take effect immediately.
    WiFi.mode(WIFI_STA);
    WiFi.setTxPower(WIFI_POWER_13dBm);
    WiFi.setSleep(false);
    WiFi.setAutoReconnect(false);

    // Try to connect; if all retries fail, fall back to portal
    bool connected = false;
    for (int attempt = 0; attempt < WIFI_CONNECT_RETRIES; attempt++) {
        Serial.printf("[Otto] WiFi attempt %d/%d\n", attempt + 1, WIFI_CONNECT_RETRIES);
        if (wifi_scan_and_connect()) {
            connected = true;
            break;
        }
        delay(1000);
    }

    if (!connected) {
        Serial.println("[Otto] WiFi connection failed, starting setup portal...");
        led_status_color(170, 0, 255);  // Magenta
        buzzer_tone(440, 150);
        delay(50);
        buzzer_tone(330, 150);
        portal_start();
        state = CONFIG_PORTAL;
        return;
    }

    // Register micro-ROS UDP transport using config from NVS
    {
        IPAddress agent_ip;
        agent_ip.fromString(g_cfg.agent_ip);
        static struct micro_ros_agent_locator locator;
        locator.address = agent_ip;
        locator.port    = g_cfg.agent_port;
        rmw_uros_set_custom_transport(
            false, (void *) &locator,
            platformio_transport_open,
            platformio_transport_close,
            platformio_transport_write,
            platformio_transport_read);
    }

    Serial.print("[Otto] WiFi IP: ");
    Serial.println(WiFi.localIP());
    Serial.printf("[Otto] Agent: %s:%d\n", g_cfg.agent_ip, g_cfg.agent_port);

    // Stage 4: WiFi connected — cyan LEDs + beep
    Serial.println("[Otto] WiFi connected, waiting for micro-ROS agent...");
    led_status_color(0, 200, 255);  // Cyan
    buzzer_beep();

    state = WAITING_AGENT;
}

void loop() {
    static unsigned long last_state_change = 0;

    switch (state) {
        case CONFIG_PORTAL:
            portal_loop();
            // Pulse magenta on ring while in portal mode
            {
                uint8_t brightness = (millis() / 4) % 256;
                if (brightness > 127) brightness = 255 - brightness;
                brightness = brightness * 2;
                led_status_color((brightness * 170) / 255, 0, brightness);
            }
            digitalWrite(PIN_LED_BUILTIN, (millis() / 300) % 2);
            break;

        case WAITING_AGENT:
            // If WiFi itself dropped, reconnect by scanning for the best AP
            if (WiFi.status() != WL_CONNECTED) {
                wifi_reconnect();
                last_state_change = millis();
                break;
            }

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
                ping_fail_count = 0;
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

            // Periodic agent ping to detect disconnection.
            // Runs every 2 s; blocks at most AGENT_PING_TIMEOUT_MS (200ms) — safely
            // below CMD_VEL_TIMEOUT_MS (500ms).  Requires AGENT_PING_FAIL_MAX
            // consecutive failures before tearing down entities (~6 s tolerance).
            if (millis() - last_state_change > 2000) {
                last_state_change = millis();
                if (rmw_uros_ping_agent(AGENT_PING_TIMEOUT_MS, AGENT_PING_ATTEMPTS) == RMW_RET_OK) {
                    ping_fail_count = 0;
                } else {
                    ping_fail_count++;
                    Serial.printf("[Otto] Agent ping failed (%d/%d)\n", ping_fail_count, AGENT_PING_FAIL_MAX);
                    if (ping_fail_count >= AGENT_PING_FAIL_MAX) {
                        Serial.println("[Otto] Agent lost, disconnecting...");
                        ping_fail_count = 0;
                        state = AGENT_DISCONNECTED;
                    }
                }
            }
            break;

        case AGENT_DISCONNECTED:
            destroy_entities();
            Serial.println("[Otto] Entities destroyed, waiting for agent...");
            // Disconnected — red flash + warning tone (after destroy so no DDS delay)
            led_status_color(255, 0, 0);  // Red
            buzzer_tone(440, 150);
            delay(50);
            buzzer_tone(330, 200);
            state = WAITING_AGENT;
            last_state_change = millis();
            digitalWrite(PIN_LED_BUILTIN, LOW);
            // Cyan will resume on next loop iteration (WAITING_AGENT pulse)
            break;
    }
}
