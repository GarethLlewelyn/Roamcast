#pragma once

#include <Arduino.h>

typedef void (*rc_mqtt_command_callback_t)(const char* command, const char* params_json);

void rc_mqtt_init(const char* device_id, const char* mqtt_user, const char* mqtt_pass,
                  uint32_t reconnect_delay_ms);
void rc_mqtt_loop();
bool rc_mqtt_is_connected();
void rc_mqtt_publish_discovery(const char* firmware_version, const char* hardware_model,
                               bool has_speaker, bool has_led,
                               bool has_csi, bool has_ble, bool is_full_duplex,
                               uint16_t udp_audio_port);
void rc_mqtt_publish_capabilities(bool has_speaker, bool has_led,
                                   bool has_csi, bool has_ble, bool is_full_duplex);
void rc_mqtt_publish_status(const char* state, bool audio_streaming);
void rc_mqtt_publish_health(int wifi_rssi, uint32_t free_heap, float cpu_temp, uint32_t uptime_s,
                            uint32_t dma_underruns, uint32_t udp_send_failures,
                            uint32_t mqtt_publish_failures, uint32_t heap_min_free,
                            unsigned long loop_max_us);
void rc_mqtt_publish_audio_level(float rms, float peak, bool is_speech);
void rc_mqtt_publish_modules(const char* modules_json);
void rc_mqtt_publish_presence(const char* presence_json);
void rc_mqtt_publish_csi_motion(const char* csi_json);
void rc_mqtt_publish_ble_proximity(const char* ble_json);
void rc_mqtt_set_command_callback(rc_mqtt_command_callback_t cb);
uint32_t rc_mqtt_get_publish_failures();
