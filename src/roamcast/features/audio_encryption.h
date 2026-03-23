#pragma once

#include <Arduino.h>

// AES-128-CTR audio encryption for UDP frames.
// Key and nonce prefix are received via MQTT set_audio_key command.
// Nonce = prefix (12 bytes) || sequence_number (4 bytes).

void audio_encryption_init();
bool audio_encryption_is_enabled();

// Store a new session key received from the hub.
void audio_encryption_set_key(const uint8_t* key, size_t key_len,
                               const uint8_t* nonce_prefix, size_t prefix_len);

// Encrypt a PCM frame in-place. Returns true on success.
bool audio_encryption_encrypt(uint8_t* data, size_t len, uint32_t sequence);

// Decrypt a PCM frame in-place. Returns true on success.
bool audio_encryption_decrypt(uint8_t* data, size_t len, uint32_t sequence);

// Clear stored key (device disconnect / factory reset).
void audio_encryption_clear_key();
