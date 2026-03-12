// wifi_portal.cpp — Soft-AP Captive Portal for WiFi Configuration

#include "wifi_portal.h"
#include "wifi_config.h"
#include "otto_config.h"
#include "portal_html.h"

#include <Arduino.h>
#include <WiFi.h>
#include <DNSServer.h>
#include <WebServer.h>

static DNSServer dns;
static WebServer server(80);
static const IPAddress AP_IP(192, 168, 4, 1);
static const IPAddress AP_SUBNET(255, 255, 255, 0);

// Build AP SSID from MAC: "Otto-XXXX"
static String ap_ssid() {
    uint8_t mac[6];
    WiFi.macAddress(mac);
    char buf[16];
    snprintf(buf, sizeof(buf), "%s%02X%02X", AP_SSID_PREFIX, mac[4], mac[5]);
    return String(buf);
}

// Serve the gzipped HTML page
static void handle_root() {
    server.sendHeader("Content-Encoding", "gzip");
    server.send_P(200, "text/html", (const char*)PORTAL_HTML_GZ, PORTAL_HTML_GZ_LEN);
}

// Return JSON array of scanned WiFi networks
static void handle_scan() {
    // Brief STA scan while in AP mode (AP+STA)
    WiFi.mode(WIFI_AP_STA);
    int n = WiFi.scanNetworks();

    // Deduplicate SSIDs, keeping strongest RSSI
    struct Net { String ssid; int rssi; };
    Net nets[32];
    int count = 0;
    for (int i = 0; i < n && count < 32; i++) {
        String s = WiFi.SSID(i);
        if (s.length() == 0) continue;
        bool found = false;
        for (int j = 0; j < count; j++) {
            if (nets[j].ssid == s) {
                if (WiFi.RSSI(i) > nets[j].rssi) nets[j].rssi = WiFi.RSSI(i);
                found = true;
                break;
            }
        }
        if (!found) {
            nets[count].ssid = s;
            nets[count].rssi = WiFi.RSSI(i);
            count++;
        }
    }
    WiFi.scanDelete();
    WiFi.mode(WIFI_AP);  // back to AP-only

    // Sort by signal strength (strongest first)
    for (int i = 0; i < count - 1; i++)
        for (int j = i + 1; j < count; j++)
            if (nets[j].rssi > nets[i].rssi) {
                Net tmp = nets[i]; nets[i] = nets[j]; nets[j] = tmp;
            }

    // Build JSON
    String json = "[";
    for (int i = 0; i < count; i++) {
        if (i > 0) json += ",";
        json += "{\"s\":\"";
        // Escape quotes in SSID
        String escaped = nets[i].ssid;
        escaped.replace("\\", "\\\\");
        escaped.replace("\"", "\\\"");
        json += escaped;
        json += "\",\"r\":";
        json += String(nets[i].rssi);
        json += "}";
    }
    json += "]";

    server.sendHeader("Cache-Control", "no-cache");
    server.send(200, "application/json", json);
}

// Save config from POST and reboot
static void handle_save() {
    if (!server.hasArg("s") || !server.hasArg("i") || !server.hasArg("o")) {
        server.send(400, "text/plain", "Missing fields");
        return;
    }

    WifiConfig cfg = {};
    String ssid = server.arg("s");
    String pass = server.arg("p");
    String ip   = server.arg("i");
    int port    = server.arg("o").toInt();

    if (ssid.length() == 0 || ssid.length() > 32) {
        server.send(400, "text/plain", "Invalid SSID");
        return;
    }
    if (port < 1 || port > 65535) {
        server.send(400, "text/plain", "Invalid port");
        return;
    }

    strncpy(cfg.ssid, ssid.c_str(), sizeof(cfg.ssid) - 1);
    strncpy(cfg.pass, pass.c_str(), sizeof(cfg.pass) - 1);
    strncpy(cfg.agent_ip, ip.c_str(), sizeof(cfg.agent_ip) - 1);
    cfg.agent_port = (uint16_t)port;

    wifi_config_save(cfg);

    server.send(200, "text/plain", "OK");

    // Give the response time to send, then reboot
    delay(500);
    ESP.restart();
}

// Captive portal: redirect all unknown requests to the config page
static void handle_redirect() {
    server.sendHeader("Location", String("http://") + AP_IP.toString(), true);
    server.send(302, "text/plain", "");
}

void portal_start() {
    String ssid = ap_ssid();
    Serial.printf("[Otto] Starting config portal: %s\n", ssid.c_str());

    WiFi.mode(WIFI_AP);
    WiFi.softAPConfig(AP_IP, AP_IP, AP_SUBNET);
    WiFi.softAP(ssid.c_str(), NULL, AP_CHANNEL);

    // DNS: resolve all domains to our AP IP (captive portal redirect)
    dns.start(53, "*", AP_IP);

    // HTTP routes
    server.on("/", HTTP_GET, handle_root);
    server.on("/scan", HTTP_GET, handle_scan);
    server.on("/save", HTTP_POST, handle_save);

    // Captive portal detection endpoints (Android, iOS, Windows)
    server.on("/generate_204", handle_redirect);     // Android
    server.on("/hotspot-detect.html", handle_redirect); // iOS
    server.on("/connecttest.txt", handle_redirect);  // Windows
    server.on("/redirect", handle_redirect);         // Windows
    server.on("/ncsi.txt", handle_redirect);         // Windows

    // Everything else → redirect to portal
    server.onNotFound(handle_redirect);

    server.begin();
    Serial.printf("[Otto] Portal ready at http://%s\n", AP_IP.toString().c_str());
}

void portal_loop() {
    dns.processNextRequest();
    server.handleClient();
}

void portal_stop() {
    server.stop();
    dns.stop();
    WiFi.softAPdisconnect(true);
    WiFi.mode(WIFI_STA);
    Serial.println("[Otto] Portal stopped");
}
