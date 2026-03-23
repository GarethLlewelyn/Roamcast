#pragma once

#include <Arduino.h>

struct DetectedModule {
    uint8_t i2c_address;
    const char* type;
    const char* name;
    const char* capability;
    bool active;
};

void module_scanner_init(int sda_pin, int scl_pin, uint32_t scan_interval_ms);
void module_scanner_loop();
void module_scanner_force_scan();
uint8_t module_scanner_get_count();
const DetectedModule* module_scanner_get_modules();
bool module_scanner_has_module(const char* type);
