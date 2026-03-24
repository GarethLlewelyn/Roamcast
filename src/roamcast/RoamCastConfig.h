#pragma once

#include <Arduino.h>

// --- Audio Input Configuration ---
struct RoamCastAudioInputConfig {
    uint32_t sample_rate;       // Default: 16000
    uint8_t magnification;      // Default: 16 (M5Stack-specific, ignored by generic)
    uint8_t noise_filter;       // Default: 8 (M5Stack-specific, ignored by generic)
    uint8_t dma_buf_count;      // Default: 2
    uint16_t dma_buf_len;       // Default: 256
};

// --- Audio Output Configuration ---
struct RoamCastAudioOutputConfig {
    uint32_t sample_rate;       // Default: 16000
    uint16_t dma_buf_len;       // Default: 256
    uint8_t dma_buf_count;      // Default: 8
};

// --- Audio Input Callbacks (REQUIRED) ---
struct AudioInputCallbacks {
    bool (*init)(const RoamCastAudioInputConfig* cfg);
    bool (*begin)();
    void (*end)();
    // Start async recording into buffer. Returns true if started.
    bool (*record)(int16_t* buffer, size_t samples, uint32_t sample_rate);
    // Returns true when the recording started by record() is complete.
    bool (*isRecordingDone)();
    // Can the mic stay active during speaker playback?
    bool (*isFullDuplex)();
};

// --- Audio Output Callbacks (OPTIONAL — NULL = mic-only device) ---
struct AudioOutputCallbacks {
    bool (*init)(const RoamCastAudioOutputConfig* cfg);
    bool (*begin)();
    void (*end)();
    // Queue PCM samples for playback on a specific channel.
    bool (*playRaw)(const int16_t* data, size_t samples, uint32_t sample_rate,
                    bool stereo, int repeat, int channel, bool stop_current);
    bool (*isPlaying)();
    void (*stop)();
    void (*setVolume)(uint8_t vol);  // 0-255
};

// --- Status Indicator Callbacks (OPTIONAL — NULL = no LED) ---
struct StatusIndicatorCallbacks {
    void (*init)();
    void (*setColor)(uint8_t r, uint8_t g, uint8_t b);
    void (*off)();
};

// --- Button Callbacks (OPTIONAL — NULL = no button) ---
struct ButtonCallbacks {
    void (*init)();
    bool (*wasClicked)();
    bool (*pressedFor)(uint32_t ms);
};

// --- Feature Flags ---
struct RoamCastFeatures {
    bool ble_enabled;           // Requires ROAMCAST_FEATURE_BLE build flag
    bool csi_enabled;           // Requires ROAMCAST_FEATURE_CSI build flag
    bool encryption_enabled;    // Requires ROAMCAST_FEATURE_ENCRYPTION build flag
    bool modules_enabled;       // Requires ROAMCAST_FEATURE_MODULES build flag
    bool presence_enabled;      // Requires ROAMCAST_FEATURE_PRESENCE build flag
    bool mdns_enabled;          // mDNS hub auto-discovery
};

// --- Board-level hooks ---
typedef void (*BoardInitFn)();
typedef void (*BoardLoopFn)();

// --- Main Configuration Struct ---
struct RoamCastConfig {
    // Device identity
    const char* hardware_model;         // e.g., "atom_echo_s3r", "my_custom_board"
    const char* firmware_version;       // e.g., "1.0.0"
    const char* device_id_prefix;       // e.g., "echo_s3r", "roamcast". NULL = "roamcast"

    // Network defaults (overridden by provisioning/mDNS at runtime)
    const char* wifi_ssid;              // NULL = must provision via captive portal
    const char* wifi_password;          // NULL = open network
    const char* server_ip;              // NULL = must discover via mDNS or provisioning
    uint16_t mqtt_port;                 // Default: 1883
    uint16_t api_port;                  // Default: 8100
    uint16_t udp_audio_port;            // Default: 5100
    uint16_t tts_udp_port;             // Default: 5101

    // Authentication (overridden by provisioning at runtime)
    const char* auth_username;          // NULL = skip auth
    const char* auth_password;
    const char* mqtt_username;          // NULL = anonymous MQTT
    const char* mqtt_password;

    // Audio DSP settings
    float dc_block_alpha;               // Default: 0.995f
    uint16_t gate_threshold;            // Default: 150
    uint8_t gate_hold_frames;           // Default: 5

    // Hardware callbacks
    AudioInputCallbacks* audio_input;   // REQUIRED — must not be NULL
    AudioOutputCallbacks* audio_output; // NULL = mic-only device (no speaker)
    StatusIndicatorCallbacks* status_indicator;  // NULL = no LED
    ButtonCallbacks* button;            // NULL = no button

    // Board lifecycle hooks
    BoardInitFn board_init;             // Called at start of begin(). NULL = skip.
    BoardLoopFn board_loop;             // Called at start of loop(). NULL = skip.

    // Feature flags
    RoamCastFeatures features;

    // Provisioning settings
    const char* portal_ap_name;         // Default: "RoamCast-Setup"
    uint32_t factory_reset_hold_ms;     // Default: 5000

    // I2C pins (for module scanner, -1 = board default)
    int i2c_sda_pin;
    int i2c_scl_pin;

    // Timing
    uint32_t heartbeat_interval_ms;     // Default: 30000
    uint32_t audio_level_report_ms;     // Default: 500
    uint32_t mqtt_reconnect_delay_ms;   // Default: 5000

    // BLE settings (only used if ROAMCAST_FEATURE_BLE)
    uint16_t ble_publish_interval_ms;   // Default: 500
    uint32_t ble_stale_timeout_ms;      // Default: 10000
    uint16_t ble_scan_interval_ms;      // Default: 500
    uint16_t ble_scan_window_ms;        // Default: 200
    float ble_rssi_smoothing_alpha;     // Default: 0.3f

    // CSI settings (only used if ROAMCAST_FEATURE_CSI)
    uint16_t csi_publish_interval_ms;   // Default: 250
    uint32_t csi_keepalive_ms;          // Default: 5000
    float csi_publish_threshold;        // Default: 0.05f
    float csi_smoothing_alpha;          // Default: 0.3f
    uint8_t csi_history_depth;          // Default: 10
    float csi_motion_max_variance;      // Default: 500.0f

    // mDNS discovery
    uint32_t mdns_discovery_timeout_ms; // Default: 10000

    // Debug logging (0 = essential only, 1 = verbose debug)
    uint8_t debug_level;                // Default: 0
};

// Helper to create a config with sensible defaults
inline RoamCastConfig roamcast_default_config() {
    RoamCastConfig cfg = {};
    cfg.hardware_model = "generic_esp32s3";
    cfg.firmware_version = "0.0.0";
    cfg.device_id_prefix = nullptr;
    cfg.wifi_ssid = nullptr;
    cfg.wifi_password = nullptr;
    cfg.server_ip = nullptr;
    cfg.mqtt_port = 1883;
    cfg.api_port = 8100;
    cfg.udp_audio_port = 5100;
    cfg.tts_udp_port = 5101;
    cfg.auth_username = nullptr;
    cfg.auth_password = nullptr;
    cfg.mqtt_username = nullptr;
    cfg.mqtt_password = nullptr;
    cfg.dc_block_alpha = 0.995f;
    cfg.gate_threshold = 150;
    cfg.gate_hold_frames = 5;
    cfg.audio_input = nullptr;
    cfg.audio_output = nullptr;
    cfg.status_indicator = nullptr;
    cfg.button = nullptr;
    cfg.board_init = nullptr;
    cfg.board_loop = nullptr;
    cfg.features = { false, false, false, false, false, true };
    cfg.portal_ap_name = "RoamCast-Setup";
    cfg.factory_reset_hold_ms = 5000;
    cfg.i2c_sda_pin = -1;
    cfg.i2c_scl_pin = -1;
    cfg.heartbeat_interval_ms = 30000;
    cfg.audio_level_report_ms = 500;
    cfg.mqtt_reconnect_delay_ms = 5000;
    cfg.ble_publish_interval_ms = 500;
    cfg.ble_stale_timeout_ms = 10000;
    cfg.ble_scan_interval_ms = 500;
    cfg.ble_scan_window_ms = 200;
    cfg.ble_rssi_smoothing_alpha = 0.3f;
    cfg.csi_publish_interval_ms = 250;
    cfg.csi_keepalive_ms = 5000;
    cfg.csi_publish_threshold = 0.05f;
    cfg.csi_smoothing_alpha = 0.3f;
    cfg.csi_history_depth = 10;
    cfg.csi_motion_max_variance = 500.0f;
    cfg.mdns_discovery_timeout_ms = 10000;
    cfg.debug_level = 0;
    return cfg;
}
