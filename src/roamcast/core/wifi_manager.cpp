#include "wifi_manager.h"
#include <WiFi.h>

void rc_wifi_init(const char* ssid, const char* password) {
    WiFi.mode(WIFI_STA);
    WiFi.begin(ssid, password);

    Serial.print("[RoamCast] Connecting to WiFi");
    int attempts = 0;
    while (WiFi.status() != WL_CONNECTED && attempts < 60) {
        delay(500);
        Serial.print(".");
        attempts++;
    }

    if (WiFi.status() == WL_CONNECTED) {
        Serial.println();
        Serial.printf("[RoamCast] WiFi connected - IP: %s\n", WiFi.localIP().toString().c_str());
        Serial.printf("[RoamCast] MAC: %s\n", WiFi.macAddress().c_str());
        Serial.printf("[RoamCast] RSSI: %d dBm\n", WiFi.RSSI());
        configTime(0, 0, "pool.ntp.org", "time.nist.gov");
    } else {
        Serial.println("\n[RoamCast] WiFi connection failed!");
    }
}

bool rc_wifi_is_connected() { return WiFi.status() == WL_CONNECTED; }
int rc_wifi_get_rssi() { return WiFi.RSSI(); }
String rc_wifi_get_ip() { return WiFi.localIP().toString(); }
String rc_wifi_get_mac() { return WiFi.macAddress(); }
