// wifi_config.cpp — NVS-backed WiFi & Agent Configuration

#include "wifi_config.h"
#include <Preferences.h>
#include <Arduino.h>

static const char* NVS_NAMESPACE = "otto";
static const char* KEY_SSID      = "ssid";
static const char* KEY_PASS      = "pass";
static const char* KEY_AGENT_IP  = "agent_ip";
static const char* KEY_AGENT_PORT = "agent_port";

bool wifi_config_load(WifiConfig &cfg) {
    Preferences prefs;
    if (!prefs.begin(NVS_NAMESPACE, true)) return false;  // read-only

    if (!prefs.isKey(KEY_SSID)) {
        prefs.end();
        return false;
    }

    String ssid = prefs.getString(KEY_SSID, "");
    String pass = prefs.getString(KEY_PASS, "");
    String ip   = prefs.getString(KEY_AGENT_IP, "192.168.1.100");
    uint16_t port = prefs.getUShort(KEY_AGENT_PORT, 8888);
    prefs.end();

    if (ssid.length() == 0) return false;

    strncpy(cfg.ssid, ssid.c_str(), sizeof(cfg.ssid) - 1);
    cfg.ssid[sizeof(cfg.ssid) - 1] = '\0';
    strncpy(cfg.pass, pass.c_str(), sizeof(cfg.pass) - 1);
    cfg.pass[sizeof(cfg.pass) - 1] = '\0';
    strncpy(cfg.agent_ip, ip.c_str(), sizeof(cfg.agent_ip) - 1);
    cfg.agent_ip[sizeof(cfg.agent_ip) - 1] = '\0';
    cfg.agent_port = port;

    return true;
}

bool wifi_config_save(const WifiConfig &cfg) {
    Preferences prefs;
    if (!prefs.begin(NVS_NAMESPACE, false)) return false;  // read-write

    prefs.putString(KEY_SSID, cfg.ssid);
    prefs.putString(KEY_PASS, cfg.pass);
    prefs.putString(KEY_AGENT_IP, cfg.agent_ip);
    prefs.putUShort(KEY_AGENT_PORT, cfg.agent_port);
    prefs.end();

    Serial.printf("[Otto] Config saved: SSID='%s' Agent=%s:%d\n",
                  cfg.ssid, cfg.agent_ip, cfg.agent_port);
    return true;
}

bool wifi_config_has_saved() {
    Preferences prefs;
    if (!prefs.begin(NVS_NAMESPACE, true)) return false;
    bool has = prefs.isKey(KEY_SSID) && prefs.getString(KEY_SSID, "").length() > 0;
    prefs.end();
    return has;
}

void wifi_config_clear() {
    Preferences prefs;
    if (!prefs.begin(NVS_NAMESPACE, false)) return;
    prefs.clear();
    prefs.end();
    Serial.println("[Otto] Config cleared from NVS");
}

