#include "csi_motion.h"
#include "../core/mqtt_client.h"

#include <ArduinoJson.h>
#include <math.h>
#include <time.h>

#ifdef ROAMCAST_FEATURE_CSI

#include <esp_wifi.h>

// Max subcarriers for internal history buffer (matches CSI_MAX_REPORTED_SUBCARRIERS)
#define CSI_MAX_SUBCARRIERS 64
// Max history depth (capped at init time)
#define CSI_MAX_HISTORY_DEPTH 32

// --- Config (stored from init parameters) ---
static uint16_t _cfg_publish_interval_ms = 250;
static uint32_t _cfg_keepalive_ms = 5000;
static float _cfg_publish_threshold = 0.05f;
static float _cfg_smoothing_alpha = 0.3f;
static uint8_t _cfg_history_depth = 10;
static float _cfg_motion_max_variance = 500.0f;

// --- Static state ---
static bool _available = false;
static CsiMotionData _current = {};
static CsiMotionData _last_published = {};
static unsigned long _last_publish_ms = 0;

// Circular buffer of subcarrier amplitude snapshots
static float _amplitude_history[CSI_MAX_HISTORY_DEPTH][CSI_MAX_SUBCARRIERS];
static int _history_index = 0;
static int _history_count = 0;
static int _last_subcarrier_count = 0;

// Smoothed motion intensity
static float _smoothed_motion = 0.0f;

// Packet counter since last publish
static uint32_t _packet_count = 0;

// ISR -> loop communication
static volatile bool _new_data_flag = false;

// Latest frame rx_ctrl + mac (written in callback, read in loop)
static volatile int8_t _last_rssi = 0;
static volatile int8_t _last_noise_floor = 0;
static volatile uint8_t _last_channel = 0;
static volatile uint8_t _last_secondary_channel = 0;
static volatile uint8_t _last_bandwidth = 0;
static volatile uint8_t _last_sig_mode = 0;
static volatile uint8_t _last_rate = 0;
static volatile uint8_t _last_stbc = 0;
static volatile uint32_t _last_hw_timestamp = 0;
static volatile uint8_t _last_mac[6] = {0};

// Latest per-subcarrier phase data (written in callback)
static float _last_phases[CSI_MAX_SUBCARRIERS];
static float _last_amplitudes[CSI_MAX_SUBCARRIERS];
static int _last_amp_count = 0;

// --- Timestamp helper ---
static String _get_timestamp() {
    struct tm timeinfo;
    if (getLocalTime(&timeinfo, 100)) {
        char buf[30];
        strftime(buf, sizeof(buf), "%Y-%m-%dT%H:%M:%SZ", &timeinfo);
        return String(buf);
    }
    return String("1970-01-01T00:00:00Z");
}

// --- CSI callback (runs in WiFi task context -- keep fast) ---
static void _csi_rx_callback(void* ctx, wifi_csi_info_t* info) {
    if (!info || !info->buf || info->len == 0) return;

    int8_t* buf = info->buf;
    int len = info->len;

    // CSI buffer contains interleaved [imaginary, real] pairs per subcarrier
    // Skip first 4 bytes if first_word_invalid (ESP32-S3 hardware quirk)
    int start_byte = info->first_word_invalid ? 4 : 0;
    int num_subcarriers = (len - start_byte) / 2;
    if (num_subcarriers > CSI_MAX_SUBCARRIERS) num_subcarriers = CSI_MAX_SUBCARRIERS;
    if (num_subcarriers <= 0) return;

    // Compute amplitude and phase for each subcarrier
    int idx = _history_index;
    for (int i = 0; i < num_subcarriers; i++) {
        int offset = start_byte + i * 2;
        float imag = (float)buf[offset];
        float real = (float)buf[offset + 1];
        float amp = sqrtf(imag * imag + real * real);
        _amplitude_history[idx][i] = amp;
        _last_amplitudes[i] = amp;
        _last_phases[i] = atan2f(imag, real);
    }
    // Zero remaining slots
    for (int i = num_subcarriers; i < CSI_MAX_SUBCARRIERS; i++) {
        _amplitude_history[idx][i] = 0.0f;
        _last_amplitudes[i] = 0.0f;
        _last_phases[i] = 0.0f;
    }
    _last_amp_count = num_subcarriers;

    // Extract rx_ctrl metadata
    _last_rssi = info->rx_ctrl.rssi;
    _last_noise_floor = info->rx_ctrl.noise_floor;
    _last_channel = info->rx_ctrl.channel;
    _last_secondary_channel = info->rx_ctrl.secondary_channel;
    _last_bandwidth = info->rx_ctrl.cwb;
    _last_sig_mode = info->rx_ctrl.sig_mode;
    _last_rate = info->rx_ctrl.rate;
    _last_stbc = info->rx_ctrl.stbc;
    _last_hw_timestamp = info->rx_ctrl.timestamp;

    // Extract source MAC
    memcpy((void*)_last_mac, info->mac, 6);

    _last_subcarrier_count = num_subcarriers;
    _history_index = (_history_index + 1) % _cfg_history_depth;
    if (_history_count < _cfg_history_depth) _history_count++;
    _packet_count++;
    _new_data_flag = true;
}

// --- Motion intensity calculation ---
static void _compute_motion_intensity() {
    if (_history_count < 2 || _last_subcarrier_count <= 0) {
        _current.motion_intensity = 0.0f;
        _current.raw_variance = 0.0f;
        return;
    }

    int sc_count = _last_subcarrier_count;
    float total_variance = 0.0f;
    float total_amplitude = 0.0f;

    // For each subcarrier, compute variance across the history window
    for (int sc = 0; sc < sc_count; sc++) {
        // Compute mean
        float sum = 0.0f;
        for (int h = 0; h < _history_count; h++) {
            sum += _amplitude_history[h][sc];
        }
        float mean = sum / _history_count;
        total_amplitude += mean;

        // Compute variance
        float var_sum = 0.0f;
        for (int h = 0; h < _history_count; h++) {
            float diff = _amplitude_history[h][sc] - mean;
            var_sum += diff * diff;
        }
        total_variance += var_sum / _history_count;
    }

    float mean_variance = total_variance / sc_count;
    float mean_amplitude = total_amplitude / sc_count;

    // EMA smoothing
    _smoothed_motion = _cfg_smoothing_alpha * mean_variance
                     + (1.0f - _cfg_smoothing_alpha) * _smoothed_motion;

    // Normalize to 0.0 - 1.0
    float normalized = _smoothed_motion / _cfg_motion_max_variance;
    if (normalized > 1.0f) normalized = 1.0f;
    if (normalized < 0.0f) normalized = 0.0f;

    _current.motion_intensity = normalized;
    _current.raw_variance = mean_variance;
    _current.mean_amplitude = mean_amplitude;
    _current.timestamp_ms = millis();

    // Copy enriched metadata from latest frame
    _current.rssi = _last_rssi;
    _current.noise_floor = _last_noise_floor;
    _current.snr = _last_rssi - _last_noise_floor;
    _current.channel = _last_channel;
    _current.secondary_channel = _last_secondary_channel;
    _current.bandwidth = _last_bandwidth;
    _current.sig_mode = _last_sig_mode;
    _current.rate = _last_rate;
    _current.stbc = _last_stbc;
    _current.hw_timestamp_us = _last_hw_timestamp;

    // Format source MAC
    snprintf(_current.source_mac, sizeof(_current.source_mac),
             "%02X:%02X:%02X:%02X:%02X:%02X",
             _last_mac[0], _last_mac[1], _last_mac[2],
             _last_mac[3], _last_mac[4], _last_mac[5]);

    // Copy per-subcarrier data
    _current.subcarrier_count = _last_amp_count;
    memcpy(_current.amplitudes, _last_amplitudes, sizeof(float) * _last_amp_count);
    memcpy(_current.phases, _last_phases, sizeof(float) * _last_amp_count);
}

// --- Publish decision ---
static bool _should_publish() {
    unsigned long now = millis();

    // Rate limit
    if (now - _last_publish_ms < _cfg_publish_interval_ms) return false;

    // Publish on significant change
    float delta = fabsf(_current.motion_intensity - _last_published.motion_intensity);
    if (delta >= _cfg_publish_threshold) return true;

    // Keepalive
    if (now - _last_publish_ms >= _cfg_keepalive_ms) return true;

    return false;
}

// --- MQTT publish (enriched payload) ---
static void _publish_csi_motion() {
    JsonDocument doc;

    // Core motion data
    doc["motion_intensity"] = serialized(String(_current.motion_intensity, 4));
    doc["raw_variance"] = serialized(String(_current.raw_variance, 2));
    doc["packet_count"] = _current.packet_count;
    doc["mean_amplitude"] = serialized(String(_current.mean_amplitude, 2));

    // RF metadata
    doc["rssi"] = _current.rssi;
    doc["noise_floor"] = _current.noise_floor;
    doc["snr"] = _current.snr;
    doc["channel"] = _current.channel;
    doc["secondary_channel"] = _current.secondary_channel;
    doc["bandwidth"] = _current.bandwidth;
    doc["sig_mode"] = _current.sig_mode;
    doc["rate"] = _current.rate;
    doc["stbc"] = _current.stbc;
    doc["hw_timestamp_us"] = _current.hw_timestamp_us;
    doc["source_mac"] = _current.source_mac;

    // Per-subcarrier arrays
    doc["subcarrier_count"] = _current.subcarrier_count;
    JsonArray amp_arr = doc["amplitudes"].to<JsonArray>();
    JsonArray phase_arr = doc["phases"].to<JsonArray>();
    for (int i = 0; i < _current.subcarrier_count; i++) {
        amp_arr.add(serialized(String(_current.amplitudes[i], 1)));
        phase_arr.add(serialized(String(_current.phases[i], 3)));
    }

    doc["timestamp"] = _get_timestamp();

    // Larger buffer for enriched payload (~2KB with 64 subcarriers)
    char buffer[2048];
    serializeJson(doc, buffer, sizeof(buffer));
    rc_mqtt_publish_csi_motion(buffer);

    _last_published = _current;
    _last_publish_ms = millis();
}

// --- Public API ---

void csi_motion_init(uint16_t publish_interval_ms, uint32_t keepalive_ms,
                     float publish_threshold, float smoothing_alpha,
                     uint8_t history_depth, float motion_max_variance) {
    _cfg_publish_interval_ms = publish_interval_ms;
    _cfg_keepalive_ms = keepalive_ms;
    _cfg_publish_threshold = publish_threshold;
    _cfg_smoothing_alpha = smoothing_alpha;
    _cfg_history_depth = (history_depth > CSI_MAX_HISTORY_DEPTH) ? CSI_MAX_HISTORY_DEPTH : history_depth;
    _cfg_motion_max_variance = motion_max_variance;

    wifi_csi_config_t csi_config = {
        .lltf_en = true,
        .htltf_en = true,
        .stbc_htltf2_en = true,
        .ltf_merge_en = true,
        .channel_filter_en = false,
        .manu_scale = false,
        .shift = 0,
    };

    esp_err_t err;

    err = esp_wifi_set_csi_config(&csi_config);
    if (err != ESP_OK) {
        Serial.printf("CSI motion: set_csi_config failed (0x%x)\n", err);
        return;
    }

    err = esp_wifi_set_csi_rx_cb(_csi_rx_callback, NULL);
    if (err != ESP_OK) {
        Serial.printf("CSI motion: set_csi_rx_cb failed (0x%x)\n", err);
        return;
    }

    err = esp_wifi_set_csi(true);
    if (err != ESP_OK) {
        Serial.printf("CSI motion: esp_wifi_set_csi(true) failed (0x%x)\n", err);
        return;
    }

    _available = true;
    Serial.println("CSI motion: initialized (enriched mode)");
    Serial.printf("  History depth: %d, Max subcarriers: %d\n", _cfg_history_depth, CSI_MAX_SUBCARRIERS);
    Serial.printf("  Smoothing alpha: %.2f, Max variance: %.1f\n", _cfg_smoothing_alpha, _cfg_motion_max_variance);
    Serial.println("  Extracting: RSSI, noise_floor, SNR, channel, bandwidth, sig_mode, rate, STBC, MAC, phase");
}

void csi_motion_loop() {
    if (!_available) return;
    if (!_new_data_flag) return;
    _new_data_flag = false;

    _compute_motion_intensity();
    _current.packet_count = _packet_count;

    if (_should_publish() && rc_mqtt_is_connected()) {
        _publish_csi_motion();
    }
}

bool csi_motion_available() {
    return _available;
}

CsiMotionData csi_motion_get() {
    return _current;
}

#else // !ROAMCAST_FEATURE_CSI

void csi_motion_init(uint16_t, uint32_t, float, float, uint8_t, float) {}
void csi_motion_loop() {}
bool csi_motion_available() { return false; }
CsiMotionData csi_motion_get() {
    CsiMotionData empty = {};
    return empty;
}

#endif // ROAMCAST_FEATURE_CSI
