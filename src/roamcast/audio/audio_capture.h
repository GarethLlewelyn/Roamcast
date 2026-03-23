#pragma once

#include <Arduino.h>

void rc_audio_capture_init(const char* device_id, uint16_t udp_audio_port,
                           float dc_block_alpha, uint16_t gate_threshold,
                           uint8_t gate_hold_frames, uint16_t audio_level_report_ms);
void rc_audio_capture_start();
void rc_audio_capture_stop();
void rc_audio_capture_loop();
bool rc_audio_capture_is_streaming();
float rc_audio_capture_get_rms();
float rc_audio_capture_get_peak();
uint32_t rc_audio_capture_get_dma_underruns();
uint32_t rc_audio_capture_get_udp_send_failures();
