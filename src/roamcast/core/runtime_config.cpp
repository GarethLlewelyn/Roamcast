#include "runtime_config.h"

static char _server_ip[46];
static uint16_t _api_port;
static uint16_t _mqtt_port;

void rc_runtime_config_init(const char* server_ip, uint16_t api_port, uint16_t mqtt_port) {
    if (server_ip) {
        strncpy(_server_ip, server_ip, sizeof(_server_ip) - 1);
        _server_ip[sizeof(_server_ip) - 1] = '\0';
    } else {
        _server_ip[0] = '\0';
    }
    _api_port = api_port;
    _mqtt_port = mqtt_port;
}

const char* rc_get_server_ip()  { return _server_ip; }
uint16_t rc_get_api_port()      { return _api_port; }
uint16_t rc_get_mqtt_port()     { return _mqtt_port; }

void rc_set_server_ip(const char* ip) {
    strncpy(_server_ip, ip, sizeof(_server_ip) - 1);
    _server_ip[sizeof(_server_ip) - 1] = '\0';
}

void rc_set_api_port(uint16_t port) { _api_port = port; }
void rc_set_mqtt_port(uint16_t port) { _mqtt_port = port; }
