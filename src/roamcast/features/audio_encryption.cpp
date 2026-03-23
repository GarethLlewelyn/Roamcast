#include "audio_encryption.h"

#ifdef ROAMCAST_FEATURE_ENCRYPTION

#include <mbedtls/aes.h>
#include <string.h>

// Session key storage
static uint8_t _aes_key[16];
static uint8_t _nonce_prefix[12];
static bool _key_set = false;

// mbedtls AES context (hardware-accelerated on ESP32-S3)
static mbedtls_aes_context _aes_ctx;

void audio_encryption_init() {
    mbedtls_aes_init(&_aes_ctx);
    _key_set = false;
    Serial.println("Audio encryption module initialized");
}

bool audio_encryption_is_enabled() {
    return _key_set;
}

void audio_encryption_set_key(const uint8_t* key, size_t key_len,
                               const uint8_t* nonce_prefix, size_t prefix_len) {
    if (key_len != 16 || prefix_len != 12) {
        Serial.printf("Invalid key/prefix length: key=%d, prefix=%d\n", key_len, prefix_len);
        return;
    }

    memcpy(_aes_key, key, 16);
    memcpy(_nonce_prefix, nonce_prefix, 12);

    // Set the key for encryption (AES-CTR uses encrypt direction for both enc/dec)
    int ret = mbedtls_aes_setkey_enc(&_aes_ctx, _aes_key, 128);
    if (ret != 0) {
        Serial.printf("mbedtls_aes_setkey_enc failed: %d\n", ret);
        _key_set = false;
        return;
    }

    _key_set = true;
    Serial.println("Audio encryption key set");
}

static void build_nonce(uint8_t nonce[16], uint32_t sequence) {
    memcpy(nonce, _nonce_prefix, 12);
    nonce[12] = (uint8_t)(sequence & 0xFF);
    nonce[13] = (uint8_t)((sequence >> 8) & 0xFF);
    nonce[14] = (uint8_t)((sequence >> 16) & 0xFF);
    nonce[15] = (uint8_t)((sequence >> 24) & 0xFF);
}

bool audio_encryption_encrypt(uint8_t* data, size_t len, uint32_t sequence) {
    if (!_key_set) return false;

    uint8_t nonce[16];
    build_nonce(nonce, sequence);

    // AES-CTR: stream_block and nc_off are working state for partial blocks
    uint8_t stream_block[16];
    size_t nc_off = 0;

    int ret = mbedtls_aes_crypt_ctr(&_aes_ctx, len, &nc_off, nonce, stream_block, data, data);
    if (ret != 0) {
        Serial.printf("AES encrypt failed: %d\n", ret);
        return false;
    }
    return true;
}

bool audio_encryption_decrypt(uint8_t* data, size_t len, uint32_t sequence) {
    // AES-CTR decrypt is identical to encrypt
    return audio_encryption_encrypt(data, len, sequence);
}

void audio_encryption_clear_key() {
    memset(_aes_key, 0, sizeof(_aes_key));
    memset(_nonce_prefix, 0, sizeof(_nonce_prefix));
    _key_set = false;
    mbedtls_aes_free(&_aes_ctx);
    mbedtls_aes_init(&_aes_ctx);
    Serial.println("Audio encryption key cleared");
}

#else // !ROAMCAST_FEATURE_ENCRYPTION

void audio_encryption_init() {}
bool audio_encryption_is_enabled() { return false; }
void audio_encryption_set_key(const uint8_t*, size_t, const uint8_t*, size_t) {}
bool audio_encryption_encrypt(uint8_t*, size_t, uint32_t) { return false; }
bool audio_encryption_decrypt(uint8_t*, size_t, uint32_t) { return false; }
void audio_encryption_clear_key() {}

#endif // ROAMCAST_FEATURE_ENCRYPTION
