#pragma once

#include <Arduino.h>

struct RcProvisionedConfig {
    char wifi_ssid[64];
    char wifi_password[64];
    char hub_ip[46];
    uint16_t hub_api_port;
    char hub_username[64];
    char hub_password[128];
    char mqtt_username[64];
    char mqtt_password[128];
    bool valid;
};

void rc_provisioning_init(const char* default_ssid, const char* default_pass,
                          const char* default_hub_ip, uint16_t default_api_port,
                          const char* default_auth_user, const char* default_auth_pass,
                          const char* default_mqtt_user, const char* default_mqtt_pass);
bool rc_provisioning_is_provisioned();
RcProvisionedConfig rc_provisioning_get_config();
bool rc_provisioning_start_portal(const char* ap_name);
void rc_provisioning_factory_reset();
