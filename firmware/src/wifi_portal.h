// wifi_portal.h — Soft-AP Captive Portal for WiFi Configuration
// ==============================================================
// Creates a WiFi access point with a captive portal web interface.
// Users connect to the AP from any device, get redirected to the
// config page, enter WiFi credentials + agent IP/port, and save.

#ifndef WIFI_PORTAL_H
#define WIFI_PORTAL_H

// Start the captive portal: creates AP, DNS server, and HTTP server.
// AP SSID is "Otto-XXXX" where XXXX is derived from the MAC address.
void portal_start();

// Must be called from loop() while portal is active.
// Handles DNS requests and HTTP clients.
void portal_loop();

// Stop the portal: tears down AP, DNS, and HTTP server.
void portal_stop();

#endif // WIFI_PORTAL_H
