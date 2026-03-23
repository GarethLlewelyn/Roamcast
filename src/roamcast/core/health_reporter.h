#pragma once

#include <Arduino.h>

void rc_health_reporter_init(const char* device_id, uint32_t heartbeat_interval_ms);
void rc_health_reporter_loop();
