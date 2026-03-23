#pragma once

#include <Arduino.h>

void rc_mdns_discovery_init();
bool rc_mdns_discovery_find_hub(uint32_t timeout_ms);
