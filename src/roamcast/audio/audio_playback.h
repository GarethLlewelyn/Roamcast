#pragma once

#include <Arduino.h>

void rc_audio_playback_init();
void rc_audio_playback_loop();

void rc_audio_playback_set_volume(uint8_t volume);
uint8_t rc_audio_playback_get_volume();

void rc_audio_playback_play_tone(uint16_t freq_hz, uint16_t duration_ms);
bool rc_audio_playback_is_playing();

void rc_audio_playback_tts_start();
bool rc_audio_playback_tts_active();

void rc_audio_playback_music_start();
void rc_audio_playback_music_stop();
void rc_audio_playback_music_flush();
