#include "wifi_manager.h"
#include "../RoamCastLog.h"
#include <WiFi.h>

void rc_wifi_init(const char* ssid, const char* password) {
    WiFi.mode(WIFI_STA);
    WiFi.begin(ssid, password);

    int attempts = 0;
    while (WiFi.status() != WL_CONNECTED && attempts < 60) {
        delay(500);
        attempts++;
    }

    if (WiFi.status() == WL_CONNECTED) {
        configTime(0, 0, "pool.ntp.org", "time.nist.gov");
    }
}

bool rc_wifi_is_connected() { return WiFi.status() == WL_CONNECTED; }
int rc_wifi_get_rssi() { return WiFi.RSSI(); }
String rc_wifi_get_ip() { return WiFi.localIP().toString(); }
String rc_wifi_get_mac() { return WiFi.macAddress(); }
