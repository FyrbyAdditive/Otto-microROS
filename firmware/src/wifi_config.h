// wifi_config.h — NVS-backed WiFi & Agent Configuration
// ======================================================
// Persists WiFi credentials and micro-ROS agent settings in NVS
// so they survive reboots. Populated via the captive portal.

#ifndef WIFI_CONFIG_H
#define WIFI_CONFIG_H

#include <stdint.h>
#include <stdbool.h>

struct WifiConfig {
    char ssid[33];       // Max 32-char SSID + null
    char pass[65];       // Max 64-char WPA password + null
    char agent_ip[16];   // Dot-decimal IPv4 + null
    uint16_t agent_port;
};

// Load config from NVS into `cfg`. Returns true if valid config was found.
bool wifi_config_load(WifiConfig &cfg);

// Save config to NVS. Returns true on success.
bool wifi_config_save(const WifiConfig &cfg);

// Check if NVS contains a previously saved config.
bool wifi_config_has_saved();

// Clear saved config from NVS.
void wifi_config_clear();

#endif // WIFI_CONFIG_H
