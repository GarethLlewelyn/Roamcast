#pragma once

#include <Arduino.h>

void rc_wifi_init(const char* ssid, const char* password);
bool rc_wifi_is_connected();
int rc_wifi_get_rssi();
String rc_wifi_get_ip();
String rc_wifi_get_mac();
