#pragma once

#include <Arduino.h>

struct PresenceData {
    bool occupied;          // Presence flag from STHS34PF80
    bool motion;            // Motion flag from STHS34PF80
    int16_t presence_val;   // Raw presence value (signed)
    int16_t motion_val;     // Raw motion value (signed)
    float temperature;      // Object temperature in C
    unsigned long timestamp_ms;
};

void presence_sensor_init(int sda_pin, int scl_pin, uint8_t i2c_addr,
                          uint16_t presence_threshold, uint16_t motion_threshold,
                          uint32_t read_interval_ms, uint32_t keepalive_ms);
void presence_sensor_loop();
bool presence_sensor_available();
PresenceData presence_sensor_get();
