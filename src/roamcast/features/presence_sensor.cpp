#include "presence_sensor.h"
#include "module_scanner.h"
#include "../core/mqtt_client.h"

#include <ArduinoJson.h>
#include <time.h>

#ifdef ROAMCAST_FEATURE_PRESENCE

#include <Wire.h>
#include <M5_STHS34PF80.h>

// --- Config (stored from init parameters) ---
static int _cfg_sda_pin = 2;
static int _cfg_scl_pin = 1;
static uint8_t _cfg_i2c_addr = 0x5A;
static uint16_t _cfg_presence_threshold = 200;
static uint16_t _cfg_motion_threshold = 200;
static uint32_t _cfg_read_interval_ms = 500;
static uint32_t _cfg_keepalive_ms = 10000;

// TMOS PIR sensor instance
static M5_STHS34PF80 _tmos;

// State
static bool _available = false;
static PresenceData _current = {false, false, 0, 0, 0.0f, 0};
static PresenceData _last_published = {false, false, 0, 0, 0.0f, 0};
static unsigned long _last_read_ms = 0;
static unsigned long _last_publish_ms = 0;

static String get_presence_timestamp() {
    struct tm timeinfo;
    if (getLocalTime(&timeinfo, 100)) {
        char buf[30];
        strftime(buf, sizeof(buf), "%Y-%m-%dT%H:%M:%SZ", &timeinfo);
        return String(buf);
    }
    return String("1970-01-01T00:00:00Z");
}

static bool _init_sensor() {
    // Initialize STHS34PF80 via M5Stack library
    // Wire is already initialized by module_scanner_init()
    if (!_tmos.begin(&Wire, _cfg_i2c_addr, _cfg_sda_pin, _cfg_scl_pin)) {
        Serial.println("Presence sensor: STHS34PF80 begin() failed");
        return false;
    }

    // Configure sensor
    _tmos.setPresenceThreshold(_cfg_presence_threshold);
    _tmos.setMotionThreshold(_cfg_motion_threshold);

    Serial.println("Presence sensor: STHS34PF80 configured");
    Serial.printf("  Presence threshold: %d\n", _cfg_presence_threshold);
    Serial.printf("  Motion threshold: %d\n", _cfg_motion_threshold);

    return true;
}

static void _read_sensor() {
    // Check if new data is available
    sths34pf80_tmos_drdy_status_t drdy;
    _tmos.getDataReady(&drdy);
    if (!drdy.drdy) {
        return;
    }

    // Read status flags (presence, motion, ambient shock)
    sths34pf80_tmos_func_status_t status;
    _tmos.getStatus(&status);

    // Read raw presence and motion values
    int16_t presence_val = 0;
    int16_t motion_val = 0;
    float temp = 0.0f;

    _tmos.getPresenceValue(&presence_val);
    _tmos.getMotionValue(&motion_val);
    _tmos.getTemperatureData(&temp);

    _current.occupied = status.pres_flag;
    _current.motion = status.mot_flag;
    _current.presence_val = presence_val;
    _current.motion_val = motion_val;
    _current.temperature = temp;
    _current.timestamp_ms = millis();
}

static bool _should_publish() {
    unsigned long now = millis();

    // Always publish on state change
    if (_current.occupied != _last_published.occupied) return true;
    if (_current.motion != _last_published.motion) return true;

    // Publish if presence value changed significantly (>50 units)
    int pres_delta = abs((int)_current.presence_val - (int)_last_published.presence_val);
    if (pres_delta > 50) return true;

    // Keepalive publish
    if (now - _last_publish_ms >= _cfg_keepalive_ms) return true;

    return false;
}

static void _publish_presence() {
    JsonDocument doc;
    doc["occupied"] = _current.occupied;
    doc["motion"] = _current.motion;
    doc["presence_val"] = _current.presence_val;
    doc["motion_val"] = _current.motion_val;
    doc["temperature"] = serialized(String(_current.temperature, 1));
    doc["timestamp"] = get_presence_timestamp();

    char buffer[256];
    serializeJson(doc, buffer, sizeof(buffer));
    rc_mqtt_publish_presence(buffer);

    _last_published = _current;
    _last_publish_ms = millis();
}

// --- Public API ---

void presence_sensor_init(int sda_pin, int scl_pin, uint8_t i2c_addr,
                          uint16_t presence_threshold, uint16_t motion_threshold,
                          uint32_t read_interval_ms, uint32_t keepalive_ms) {
    _cfg_sda_pin = sda_pin;
    _cfg_scl_pin = scl_pin;
    _cfg_i2c_addr = i2c_addr;
    _cfg_presence_threshold = presence_threshold;
    _cfg_motion_threshold = motion_threshold;
    _cfg_read_interval_ms = read_interval_ms;
    _cfg_keepalive_ms = keepalive_ms;

    _available = module_scanner_has_module("presence_tmos");

    if (_available) {
        if (_init_sensor()) {
            Serial.printf("Presence sensor: initialized (TMOS PIR at 0x%02X)\n", _cfg_i2c_addr);
        } else {
            _available = false;
            Serial.println("Presence sensor: TMOS PIR detected on I2C but driver init failed");
        }
    } else {
        Serial.println("Presence sensor: no TMOS PIR module detected");
    }
}

void presence_sensor_loop() {
    unsigned long now = millis();

    // Check for hot-plug if not yet available
    if (!_available) {
        if (module_scanner_has_module("presence_tmos")) {
            if (_init_sensor()) {
                _available = true;
                Serial.println("Presence sensor: TMOS PIR hot-plugged and initialized");
            }
        }
        return;
    }

    // Rate-limit reads
    if (now - _last_read_ms < _cfg_read_interval_ms) return;
    _last_read_ms = now;

    _read_sensor();

    if (_should_publish() && rc_mqtt_is_connected()) {
        _publish_presence();
    }
}

bool presence_sensor_available() {
    return _available;
}

PresenceData presence_sensor_get() {
    return _current;
}

#else // !ROAMCAST_FEATURE_PRESENCE

void presence_sensor_init(int, int, uint8_t, uint16_t, uint16_t, uint32_t, uint32_t) {}
void presence_sensor_loop() {}
bool presence_sensor_available() { return false; }
PresenceData presence_sensor_get() {
    PresenceData empty = {false, false, 0, 0, 0.0f, 0};
    return empty;
}

#endif // ROAMCAST_FEATURE_PRESENCE
