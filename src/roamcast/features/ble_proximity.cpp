#include "ble_proximity.h"
#include "../core/mqtt_client.h"

#include <ArduinoJson.h>
#include <time.h>
#include <math.h>

#ifdef ROAMCAST_FEATURE_BLE

#include <NimBLEDevice.h>

#define IBEACON_COMPANY_ID   0x004C  // Apple
#define IBEACON_TYPE         0x02
#define IBEACON_DATA_LEN     0x15    // 21 bytes after type+length

struct IBeaconData {
    uint8_t uuid[16];
    uint16_t major;
    uint16_t minor;
    int8_t tx_power;
    bool valid;
};

static uint16_t _cfg_publish_interval_ms = 500;
static uint32_t _cfg_stale_timeout_ms = 10000;
static uint16_t _cfg_scan_interval_ms = 500;
static uint16_t _cfg_scan_window_ms = 200;
static float _cfg_rssi_smoothing_alpha = 0.3f;

static bool _available = false;
static BleTarget _targets[BLE_MAX_TARGETS] = {};
static int _target_count = 0;

static BleProximityResult _results[BLE_MAX_RESULTS] = {};
static int _result_count = 0;

static unsigned long _last_publish_ms = 0;
static NimBLEScan* _scan = nullptr;

static String _get_timestamp() {
    struct tm timeinfo;
    if (getLocalTime(&timeinfo, 100)) {
        char buf[30];
        strftime(buf, sizeof(buf), "%Y-%m-%dT%H:%M:%SZ", &timeinfo);
        return String(buf);
    }
    return String("1970-01-01T00:00:00Z");
}

static bool _strcasestr(const char* haystack, const char* needle) {
    if (!haystack || !needle || needle[0] == '\0') return false;
    size_t hay_len = strlen(haystack);
    size_t nee_len = strlen(needle);
    if (nee_len > hay_len) return false;

    for (size_t i = 0; i <= hay_len - nee_len; i++) {
        bool match = true;
        for (size_t j = 0; j < nee_len; j++) {
            if (tolower((unsigned char)haystack[i + j]) != tolower((unsigned char)needle[j])) {
                match = false;
                break;
            }
        }
        if (match) return true;
    }
    return false;
}

static uint8_t _hex_nibble(char c) {
    if (c >= '0' && c <= '9') return c - '0';
    if (c >= 'a' && c <= 'f') return c - 'a' + 10;
    if (c >= 'A' && c <= 'F') return c - 'A' + 10;
    return 0xFF;
}

static bool _parse_uuid_string(const char* str, uint8_t* out) {
    int idx = 0;
    for (int i = 0; str[i] && idx < 16; i++) {
        if (str[i] == '-') continue;
        uint8_t high = _hex_nibble(str[i]);
        if (!str[i + 1]) return false;
        uint8_t low = _hex_nibble(str[i + 1]);
        if (high > 15 || low > 15) return false;
        out[idx++] = (high << 4) | low;
        i++; // skip low nibble (loop will advance past high)
    }
    return idx == 16;
}

static void _uuid_to_str(const uint8_t* uuid, char* out, size_t out_len) {
    snprintf(out, out_len,
        "%02x%02x%02x%02x-%02x%02x-%02x%02x-%02x%02x-%02x%02x%02x%02x%02x%02x",
        uuid[0], uuid[1], uuid[2], uuid[3],
        uuid[4], uuid[5], uuid[6], uuid[7],
        uuid[8], uuid[9], uuid[10], uuid[11],
        uuid[12], uuid[13], uuid[14], uuid[15]);
}

static IBeaconData _parse_ibeacon(const std::string& mfr_data) {
    IBeaconData result = {};
    result.valid = false;

    // Need at least 23 bytes: 2 (type+length) + 16 (UUID) + 2 (major) + 2 (minor) + 1 (tx power)
    // With company ID prefix: 25 bytes
    if (mfr_data.length() < 23) return result;

    const uint8_t* data = (const uint8_t*)mfr_data.data();
    int offset = 0;

    // NimBLE getManufacturerData() includes company ID in some versions
    uint16_t company_id = data[0] | (data[1] << 8);  // little-endian
    if (company_id == IBEACON_COMPANY_ID) {
        offset = 2;
    }

    // Need enough bytes remaining
    if ((int)mfr_data.length() < offset + 23) return result;

    // Check iBeacon type (0x02) and length (0x15)
    if (data[offset] != IBEACON_TYPE || data[offset + 1] != IBEACON_DATA_LEN) return result;
    offset += 2;

    // Extract UUID (16 bytes, big-endian)
    memcpy(result.uuid, data + offset, 16);
    // Major (2 bytes, big-endian)
    result.major = (data[offset + 16] << 8) | data[offset + 17];
    // Minor (2 bytes, big-endian)
    result.minor = (data[offset + 18] << 8) | data[offset + 19];
    // TX Power (1 byte, signed)
    result.tx_power = (int8_t)data[offset + 20];
    result.valid = true;
    return result;
}

static int _find_or_create_result(const char* target_id) {
    // Find existing
    for (int i = 0; i < _result_count; i++) {
        if (strcmp(_results[i].target_id, target_id) == 0) return i;
    }
    // Create new if space
    if (_result_count < BLE_MAX_RESULTS) {
        int idx = _result_count++;
        memset(&_results[idx], 0, sizeof(BleProximityResult));
        strncpy(_results[idx].target_id, target_id, sizeof(_results[idx].target_id) - 1);
        _results[idx].smoothed_rssi = -100.0f; // Initial value (very far)
        _results[idx].sample_count = 0;
        return idx;
    }
    return -1; // No space
}

static int8_t _median(int8_t* arr, int count) {
    // Simple insertion sort + pick middle (count is small, max 8)
    int8_t sorted[8];
    for (int i = 0; i < count; i++) sorted[i] = arr[i];
    for (int i = 1; i < count; i++) {
        int8_t key = sorted[i];
        int j = i - 1;
        while (j >= 0 && sorted[j] > key) {
            sorted[j + 1] = sorted[j];
            j--;
        }
        sorted[j + 1] = key;
    }
    return sorted[count / 2];
}

static void _update_rssi(int idx, int8_t rssi) {
    _results[idx].rssi = rssi;
    _results[idx].last_seen_ms = millis();
    _results[idx].present = true;

    // Accumulate sample for median filtering at publish time
    if (_results[idx].sample_count < 8) {
        _results[idx].raw_samples[_results[idx].sample_count++] = rssi;
    }
}

static void _check_device(const char* device_name, int8_t rssi) {
    if (!device_name || device_name[0] == '\0') return;

    for (int t = 0; t < _target_count; t++) {
        if (!_targets[t].active || _targets[t].match_type != 0) continue;
        if (_targets[t].name_pattern[0] == '\0') continue;

        if (_strcasestr(device_name, _targets[t].name_pattern)) {
            int idx = _find_or_create_result(device_name);
            if (idx < 0) return;

            _update_rssi(idx, rssi);
            return; // Only match first target pattern
        }
    }
}

static void _check_ibeacon(const IBeaconData& beacon, int8_t rssi) {
    for (int t = 0; t < _target_count; t++) {
        if (!_targets[t].active || _targets[t].match_type != 1) continue;

        // Compare UUID
        if (memcmp(beacon.uuid, _targets[t].ibeacon_uuid, 16) != 0) continue;

        // Check major (0xFFFF = wildcard)
        if (_targets[t].ibeacon_major != 0xFFFF && _targets[t].ibeacon_major != beacon.major) continue;

        // Check minor (0xFFFF = wildcard)
        if (_targets[t].ibeacon_minor != 0xFFFF && _targets[t].ibeacon_minor != beacon.minor) continue;

        // Build target_id string based on configured filter specificity
        char target_id[48];
        char uuid_str[37];
        _uuid_to_str(beacon.uuid, uuid_str, sizeof(uuid_str));

        if (_targets[t].ibeacon_major != 0xFFFF && _targets[t].ibeacon_minor != 0xFFFF) {
            snprintf(target_id, sizeof(target_id), "%s/%u/%u", uuid_str, beacon.major, beacon.minor);
        } else if (_targets[t].ibeacon_major != 0xFFFF) {
            snprintf(target_id, sizeof(target_id), "%s/%u", uuid_str, beacon.major);
        } else {
            strncpy(target_id, uuid_str, sizeof(target_id) - 1);
            target_id[sizeof(target_id) - 1] = '\0';
        }

        int idx = _find_or_create_result(target_id);
        if (idx < 0) return;

        _results[idx].tx_power = beacon.tx_power;
        _update_rssi(idx, rssi);
        return; // Only match first iBeacon target
    }
}

class BleProximityScanCallbacks : public NimBLEAdvertisedDeviceCallbacks {
    void onResult(NimBLEAdvertisedDevice* device) override {
        int8_t rssi = device->getRSSI();

        // Check name-based targets
        if (device->haveName()) {
            std::string name = device->getName();
            if (!name.empty()) {
                _check_device(name.c_str(), rssi);
            }
        }

        if (device->haveManufacturerData()) {
            std::string mfr = device->getManufacturerData();
            IBeaconData beacon = _parse_ibeacon(mfr);
            if (beacon.valid) {
                _check_ibeacon(beacon, rssi);
            }
        }
    }
};

static BleProximityScanCallbacks _scan_callbacks;

static void _update_stale() {
    unsigned long now = millis();
    for (int i = 0; i < _result_count; i++) {
        if (_results[i].present && (now - _results[i].last_seen_ms > _cfg_stale_timeout_ms)) {
            _results[i].present = false;
        }
    }
}

static void _publish_ble_proximity() {
    JsonDocument doc;
    JsonArray targets = doc["targets"].to<JsonArray>();

    for (int i = 0; i < _result_count; i++) {
        if (!_results[i].present) continue;

        // Apply median filter on accumulated samples, then EMA smooth
        if (_results[i].sample_count > 0) {
            int8_t median_rssi = _median(_results[i].raw_samples, _results[i].sample_count);
            if (_results[i].smoothed_rssi <= -100.0f) {
                _results[i].smoothed_rssi = (float)median_rssi;
            } else {
                _results[i].smoothed_rssi =
                    _cfg_rssi_smoothing_alpha * median_rssi +
                    (1.0f - _cfg_rssi_smoothing_alpha) * _results[i].smoothed_rssi;
            }
            _results[i].sample_count = 0; // Reset buffer for next window
        }

        JsonObject t = targets.add<JsonObject>();
        t["id"] = _results[i].target_id;
        t["rssi"] = _results[i].rssi;
        t["smoothed_rssi"] = (int)roundf(_results[i].smoothed_rssi);
        if (_results[i].tx_power != 0) {
            t["tx_power"] = _results[i].tx_power;
        }
    }

    doc["timestamp"] = _get_timestamp();

    char buffer[512];
    serializeJson(doc, buffer, sizeof(buffer));
    rc_mqtt_publish_ble_proximity(buffer);

    _last_publish_ms = millis();
}

void ble_proximity_init(uint16_t publish_interval_ms, uint32_t stale_timeout_ms,
                        uint16_t scan_interval_ms, uint16_t scan_window_ms,
                        float rssi_smoothing_alpha) {
    _cfg_publish_interval_ms = publish_interval_ms;
    _cfg_stale_timeout_ms = stale_timeout_ms;
    _cfg_scan_interval_ms = scan_interval_ms;
    _cfg_scan_window_ms = scan_window_ms;
    _cfg_rssi_smoothing_alpha = rssi_smoothing_alpha;

    NimBLEDevice::init("");

    _scan = NimBLEDevice::getScan();
    _scan->setAdvertisedDeviceCallbacks(&_scan_callbacks, false);
    _scan->setActiveScan(true);
    _scan->setInterval((uint16_t)(_cfg_scan_interval_ms * 1000 / 625));
    _scan->setWindow((uint16_t)(_cfg_scan_window_ms * 1000 / 625));
    _scan->setDuplicateFilter(false);
    _scan->setMaxResults(0);

    _scan->start(0, nullptr, false);

    _available = true;
}

void ble_proximity_loop() {
    if (!_available) return;
    if (_target_count == 0) return;

    _update_stale();

    unsigned long now = millis();
    if (now - _last_publish_ms >= _cfg_publish_interval_ms) {
        if (rc_mqtt_is_connected()) {
            _publish_ble_proximity();
        }
    }
}

bool ble_proximity_available() {
    return _available;
}

void ble_proximity_set_targets(const char* targets_json) {
    JsonDocument doc;
    DeserializationError err = deserializeJson(doc, targets_json);
    if (err) {
        Serial.printf("BLE proximity: targets JSON parse error: %s\n", err.c_str());
        return;
    }

    _target_count = 0;
    memset(_targets, 0, sizeof(_targets));
    JsonArray arr = doc["targets"].as<JsonArray>();
    for (JsonObject obj : arr) {
        if (_target_count >= BLE_MAX_TARGETS) break;

        const char* match_type = obj["match_type"] | "name";

        if (strcmp(match_type, "ibeacon") == 0) {
            const char* uuid_str = obj["ibeacon_uuid"] | "";
            if (strlen(uuid_str) < 32) continue;

            if (!_parse_uuid_string(uuid_str, _targets[_target_count].ibeacon_uuid)) {
                Serial.printf("BLE proximity: invalid UUID: %s\n", uuid_str);
                continue;
            }
            _targets[_target_count].match_type = 1;
            _targets[_target_count].ibeacon_major = obj["ibeacon_major"] | 0xFFFF;
            _targets[_target_count].ibeacon_minor = obj["ibeacon_minor"] | 0xFFFF;
            _targets[_target_count].name_pattern[0] = '\0';
            _targets[_target_count].active = true;
            _target_count++;
        } else {
            const char* name = obj["name_pattern"] | "";
            if (strlen(name) > 0) {
                strncpy(_targets[_target_count].name_pattern, name,
                        sizeof(_targets[_target_count].name_pattern) - 1);
                _targets[_target_count].match_type = 0;
                _targets[_target_count].active = true;
                _target_count++;
            }
        }
    }

    _result_count = 0;

    if (_available && _scan && !_scan->isScanning()) {
        _scan->start(0, false);
    }
}

int ble_proximity_get_result_count() {
    return _result_count;
}

BleProximityResult ble_proximity_get_result(int index) {
    if (index >= 0 && index < _result_count) return _results[index];
    BleProximityResult empty = {};
    return empty;
}

#else // !ROAMCAST_FEATURE_BLE

void ble_proximity_init(uint16_t, uint32_t, uint16_t, uint16_t, float) {}
void ble_proximity_loop() {}
bool ble_proximity_available() { return false; }
void ble_proximity_set_targets(const char*) {}
int ble_proximity_get_result_count() { return 0; }
BleProximityResult ble_proximity_get_result(int) {
    BleProximityResult empty = {};
    return empty;
}

#endif // ROAMCAST_FEATURE_BLE
