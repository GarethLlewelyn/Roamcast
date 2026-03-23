#include "module_scanner.h"
#include "../core/mqtt_client.h"

#include <ArduinoJson.h>
#include <time.h>

#ifdef ROAMCAST_FEATURE_MODULES

#include <Wire.h>

// Known module I2C address registry
struct KnownModule {
    uint8_t address;
    const char* type;
    const char* name;
    const char* capability;
};

static const KnownModule KNOWN_MODULES[] = {
    {0x5A, "presence_tmos", "M5Stack TMOS PIR (STHS34PF80)", "presence"},
    {0x51, "env_sensor",    "M5Stack ENV III",                "sensors"},
    {0x38, "touch",         "M5Stack Touch",                  "buttons"},
    {0x62, "light_sensor",  "M5Stack Light",                  "sensors"},
};
static const int KNOWN_MODULE_COUNT = sizeof(KNOWN_MODULES) / sizeof(KNOWN_MODULES[0]);

// --- Config (stored from init parameters) ---
static int _cfg_sda_pin = -1;
static int _cfg_scl_pin = -1;
static uint32_t _cfg_scan_interval_ms = 60000;

// State
static DetectedModule _detected[8];
static uint8_t _detected_count = 0;
static unsigned long _last_scan_ms = 0;
static bool _force_scan = false;
static bool _initial_publish_done = false;

static String get_scan_timestamp() {
    struct tm timeinfo;
    if (getLocalTime(&timeinfo, 100)) {
        char buf[30];
        strftime(buf, sizeof(buf), "%Y-%m-%dT%H:%M:%SZ", &timeinfo);
        return String(buf);
    }
    return String("1970-01-01T00:00:00Z");
}

static bool _do_scan() {
    DetectedModule new_detected[8];
    uint8_t new_count = 0;
    bool changed = false;

    for (int i = 0; i < KNOWN_MODULE_COUNT && new_count < 8; i++) {
        Wire.beginTransmission(KNOWN_MODULES[i].address);
        uint8_t err = Wire.endTransmission();

        if (err == 0) {
            new_detected[new_count].i2c_address = KNOWN_MODULES[i].address;
            new_detected[new_count].type = KNOWN_MODULES[i].type;
            new_detected[new_count].name = KNOWN_MODULES[i].name;
            new_detected[new_count].capability = KNOWN_MODULES[i].capability;
            new_detected[new_count].active = true;
            new_count++;
            Serial.printf("  I2C 0x%02X: %s\n", KNOWN_MODULES[i].address, KNOWN_MODULES[i].name);
        }
    }

    // Check if anything changed
    if (new_count != _detected_count) {
        changed = true;
    } else {
        for (uint8_t i = 0; i < new_count; i++) {
            if (new_detected[i].i2c_address != _detected[i].i2c_address) {
                changed = true;
                break;
            }
        }
    }

    // Update state
    memcpy(_detected, new_detected, sizeof(DetectedModule) * new_count);
    _detected_count = new_count;

    return changed;
}

static void _publish_modules() {
    JsonDocument doc;
    JsonArray modules = doc["modules"].to<JsonArray>();

    for (uint8_t i = 0; i < _detected_count; i++) {
        JsonObject mod = modules.add<JsonObject>();
        mod["type"] = _detected[i].type;
        mod["name"] = _detected[i].name;
        char addr_str[7];
        snprintf(addr_str, sizeof(addr_str), "0x%02X", _detected[i].i2c_address);
        mod["i2c_address"] = addr_str;
        mod["status"] = _detected[i].active ? "active" : "inactive";
    }
    doc["scan_time"] = get_scan_timestamp();

    char buffer[512];
    serializeJson(doc, buffer, sizeof(buffer));
    rc_mqtt_publish_modules(buffer);

    Serial.printf("Published modules: %d detected\n", _detected_count);
}

// --- Public API ---

void module_scanner_init(int sda_pin, int scl_pin, uint32_t scan_interval_ms) {
    _cfg_sda_pin = sda_pin;
    _cfg_scl_pin = scl_pin;
    _cfg_scan_interval_ms = scan_interval_ms;

    // Explicitly initialize Wire on the configured pins
    Wire.begin(_cfg_sda_pin, _cfg_scl_pin);
    Serial.printf("Module scanner: Wire initialized on SDA=%d, SCL=%d\n",
                  _cfg_sda_pin, _cfg_scl_pin);

    // Run the initial scan
    Serial.println("Module scanner: initial I2C scan...");
    _do_scan();
    Serial.printf("Module scanner: %d modules detected\n", _detected_count);
    _last_scan_ms = millis();
}

void module_scanner_loop() {
    unsigned long now = millis();

    // Publish initial modules after MQTT is connected (deferred from init)
    if (!_initial_publish_done && rc_mqtt_is_connected()) {
        _publish_modules();
        _initial_publish_done = true;
    }

    bool should_scan = _force_scan ||
                       (now - _last_scan_ms >= _cfg_scan_interval_ms);

    if (!should_scan) return;

    _force_scan = false;
    _last_scan_ms = now;

    bool changed = _do_scan();
    if (changed && rc_mqtt_is_connected()) {
        _publish_modules();
    }
}

void module_scanner_force_scan() {
    _force_scan = true;
    Serial.println("Module scanner: force scan requested");
}

uint8_t module_scanner_get_count() {
    return _detected_count;
}

const DetectedModule* module_scanner_get_modules() {
    return _detected;
}

bool module_scanner_has_module(const char* type) {
    for (uint8_t i = 0; i < _detected_count; i++) {
        if (strcmp(_detected[i].type, type) == 0 && _detected[i].active) {
            return true;
        }
    }
    return false;
}

#else // !ROAMCAST_FEATURE_MODULES

void module_scanner_init(int, int, uint32_t) {}
void module_scanner_loop() {}
void module_scanner_force_scan() {}
uint8_t module_scanner_get_count() { return 0; }
const DetectedModule* module_scanner_get_modules() { return nullptr; }
bool module_scanner_has_module(const char*) { return false; }

#endif // ROAMCAST_FEATURE_MODULES
