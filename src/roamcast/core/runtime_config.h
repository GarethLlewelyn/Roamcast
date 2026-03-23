#pragma once

#include <Arduino.h>

// Initialize with default values (called from RoamCast::begin())
void rc_runtime_config_init(const char* server_ip, uint16_t api_port, uint16_t mqtt_port);

const char* rc_get_server_ip();
uint16_t rc_get_api_port();
uint16_t rc_get_mqtt_port();

void rc_set_server_ip(const char* ip);
void rc_set_api_port(uint16_t port);
void rc_set_mqtt_port(uint16_t port);
