#include "health_reporter.h"
#include "../RoamCastInternal.h"
#include "mqtt_client.h"
#include "wifi_manager.h"

// Defined in RoamCast main orchestrator or user sketch
extern unsigned long rc_loop_get_max_us();
extern void rc_loop_reset_max_us();

// Audio capture stats — provided by audio_capture module
extern bool rc_audio_capture_is_streaming();
extern uint32_t rc_audio_capture_get_dma_underruns();
extern uint32_t rc_audio_capture_get_udp_send_failures();

static const char* _device_id = nullptr;
static unsigned long _last_heartbeat_ms = 0;
static uint32_t _heartbeat_interval_ms = 30000;

void rc_health_reporter_init(const char* device_id, uint32_t heartbeat_interval_ms) {
    _device_id = device_id;
    _heartbeat_interval_ms = heartbeat_interval_ms;
    _last_heartbeat_ms = millis();

    // Publish initial online status
    rc_mqtt_publish_status("online", rc_audio_capture_is_streaming());
    Serial.println("[RoamCast] Health reporter initialized");
}

void rc_health_reporter_loop() {
    unsigned long now = millis();
    if (now - _last_heartbeat_ms < _heartbeat_interval_ms) {
        return;
    }
    _last_heartbeat_ms = now;

    if (!rc_mqtt_is_connected()) {
        return;
    }

    // Publish status heartbeat
    rc_mqtt_publish_status("online", rc_audio_capture_is_streaming());

    // Publish detailed health metrics
    rc_mqtt_publish_health(
        rc_wifi_get_rssi(),
        ESP.getFreeHeap(),
        temperatureRead(),
        millis() / 1000,
        rc_audio_capture_get_dma_underruns(),
        rc_audio_capture_get_udp_send_failures(),
        rc_mqtt_get_publish_failures(),
        ESP.getMinFreeHeap(),
        rc_loop_get_max_us()
    );

    // Reset loop max after reporting so next interval captures fresh peak
    rc_loop_reset_max_us();
}
