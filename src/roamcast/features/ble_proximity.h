#pragma once

#include <Arduino.h>

#define BLE_MAX_TARGETS     4
#define BLE_MAX_RESULTS     8

struct BleTarget {
    char name_pattern[32];
    uint8_t match_type;        // 0 = name, 1 = ibeacon
    uint8_t ibeacon_uuid[16];
    uint16_t ibeacon_major;    // 0xFFFF = wildcard
    uint16_t ibeacon_minor;    // 0xFFFF = wildcard
    bool active;
};

struct BleProximityResult {
    char target_id[48];
    int8_t rssi;
    float smoothed_rssi;
    int8_t tx_power;
    uint32_t last_seen_ms;
    bool present;
    int8_t raw_samples[8];
    int sample_count;
};

void ble_proximity_init(uint16_t publish_interval_ms, uint32_t stale_timeout_ms,
                        uint16_t scan_interval_ms, uint16_t scan_window_ms,
                        float rssi_smoothing_alpha);
void ble_proximity_loop();
bool ble_proximity_available();
void ble_proximity_set_targets(const char* targets_json);
int ble_proximity_get_result_count();
BleProximityResult ble_proximity_get_result(int index);
