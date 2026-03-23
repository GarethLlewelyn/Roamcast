#include "mdns_discovery.h"
#include "runtime_config.h"

#include <ESPmDNS.h>

static bool _initialized = false;

void rc_mdns_discovery_init() {
    if (_initialized) return;

    if (!MDNS.begin("satellite-device")) {
        Serial.println("[RoamCast mDNS] Failed to start responder");
        return;
    }
    _initialized = true;
    Serial.println("[RoamCast mDNS] Responder started");
}

bool rc_mdns_discovery_find_hub(uint32_t timeout_ms) {
    if (!_initialized) return false;

    Serial.println("[RoamCast mDNS] Searching for _audiocombine._tcp ...");

    unsigned long start = millis();
    while (millis() - start < timeout_ms) {
        int n = MDNS.queryService("audiocombine", "tcp");
        if (n > 0) {
            IPAddress ip = MDNS.IP(0);
            uint16_t port = MDNS.port(0);

            char ip_str[46];
            snprintf(ip_str, sizeof(ip_str), "%s", ip.toString().c_str());

            rc_set_server_ip(ip_str);
            rc_set_api_port(port);

            // Try to read mqtt_port from TXT records
            String mqtt_txt = MDNS.txt(0, "mqtt_port");
            if (mqtt_txt.length() > 0) {
                rc_set_mqtt_port((uint16_t)mqtt_txt.toInt());
            }

            Serial.printf("[RoamCast mDNS] Hub found at %s (API:%d, MQTT:%d)\n",
                          rc_get_server_ip(), rc_get_api_port(), rc_get_mqtt_port());
            return true;
        }
        delay(1000);
    }

    Serial.println("[RoamCast mDNS] No hub found within timeout");
    return false;
}
