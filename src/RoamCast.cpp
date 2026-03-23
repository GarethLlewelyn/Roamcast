#include "RoamCast.h"

#include "roamcast/RoamCastConfig.h"
#include "roamcast/RoamCastInternal.h"
#include "roamcast/core/runtime_config.h"
#include "roamcast/core/wifi_manager.h"
#include "roamcast/core/auth_client.h"
#include "roamcast/core/mqtt_client.h"
#include "roamcast/core/health_reporter.h"
#include "roamcast/core/provisioning.h"
#include "roamcast/core/mdns_discovery.h"
#include "roamcast/audio/audio_capture.h"
#include "roamcast/audio/audio_playback.h"
#include "roamcast/led/led_controller.h"

#ifdef ROAMCAST_FEATURE_ENCRYPTION
#include "roamcast/features/audio_encryption.h"
#include <mbedtls/base64.h>
#endif

#ifdef ROAMCAST_FEATURE_BLE
#include "roamcast/features/ble_proximity.h"
#endif

#ifdef ROAMCAST_FEATURE_CSI
#include "roamcast/features/csi_motion.h"
#endif

#ifdef ROAMCAST_FEATURE_MODULES
#include "roamcast/features/module_scanner.h"
#endif

#ifdef ROAMCAST_FEATURE_PRESENCE
#include "roamcast/features/presence_sensor.h"
#endif

#include <ArduinoJson.h>

// ---------------------------------------------------------------------------
// Singleton pointer for static callback routing
// ---------------------------------------------------------------------------
static RoamCast* _instance = nullptr;

// ---------------------------------------------------------------------------
// Extern functions that health_reporter.cpp calls to get loop timing stats
// ---------------------------------------------------------------------------
static unsigned long _rc_loop_max_us = 0;

extern "C" {
    unsigned long rc_loop_get_max_us() {
        return _rc_loop_max_us;
    }
    void rc_loop_reset_max_us() {
        _rc_loop_max_us = 0;
    }
}

// ---------------------------------------------------------------------------
// RoamCast implementation
// ---------------------------------------------------------------------------

RoamCast::RoamCast()
    : _cfg{}
    , _device_id{0}
    , _setup_complete(false)
    , _loop_max_us(0)
    , _user_cmd_cb(nullptr)
{
    _instance = this;
}

// ---------------------------------------------------------------------------
// begin()
// ---------------------------------------------------------------------------
void RoamCast::begin(RoamCastConfig cfg) {
    _cfg = cfg;
    _setup_complete = false;

    // 1. Board-level init hook
    if (_cfg.board_init) {
        _cfg.board_init();
    }

    // 2. Derive device_id from MAC: "{prefix}_{4 MAC bytes hex}"
    uint8_t mac[6];
    esp_efuse_mac_get_default(mac);
    const char* prefix = _cfg.device_id_prefix ? _cfg.device_id_prefix : "roamcast";
    snprintf(_device_id, sizeof(_device_id), "%s_%02X%02X%02X%02X",
             prefix, mac[2], mac[3], mac[4], mac[5]);
    Serial.printf("Device ID: %s\n", _device_id);

    // 3. Store config in internal shared state for sub-modules
    roamcast::internal::setCallbacks(
        _cfg.audio_input, _cfg.audio_output,
        _cfg.status_indicator, _cfg.button);
    roamcast::internal::setConfig(&_cfg);

    // 4. Initialize runtime config with defaults from config struct
    rc_runtime_config_init(
        _cfg.server_ip,
        _cfg.api_port,
        _cfg.mqtt_port);

    // 5. Initialize provisioning (loads NVS or uses config defaults)
    rc_provisioning_init(
        _cfg.wifi_ssid, _cfg.wifi_password,
        _cfg.server_ip, _cfg.api_port,
        _cfg.auth_username, _cfg.auth_password,
        _cfg.mqtt_username, _cfg.mqtt_password);

    // 6. If not provisioned AND no compile-time WiFi: start captive portal (blocks)
    if (!rc_provisioning_is_provisioned() && _cfg.wifi_ssid == nullptr) {
        Serial.println("No WiFi credentials configured — starting provisioning portal");
        const char* ap_name = _cfg.portal_ap_name ? _cfg.portal_ap_name : "RoamCast-Setup";
        rc_provisioning_start_portal(ap_name);  // Blocks until user submits, then reboots
        return;  // Should never reach here after reboot
    }

    // 7. Override runtime config from provisioned values if available
    RcProvisionedConfig prov = rc_provisioning_get_config();
    if (prov.valid && strlen(prov.hub_ip) > 0) {
        rc_set_server_ip(prov.hub_ip);
        rc_set_api_port(prov.hub_api_port);
    }

    // Determine effective credentials (provisioned overrides config)
    const char* eff_wifi_ssid = (prov.valid && strlen(prov.wifi_ssid) > 0)
                                 ? prov.wifi_ssid : _cfg.wifi_ssid;
    const char* eff_wifi_pass = (prov.valid && strlen(prov.wifi_password) > 0)
                                 ? prov.wifi_password : _cfg.wifi_password;
    const char* eff_auth_user = (prov.valid && strlen(prov.hub_username) > 0)
                                 ? prov.hub_username : _cfg.auth_username;
    const char* eff_auth_pass = (prov.valid && strlen(prov.hub_password) > 0)
                                 ? prov.hub_password : _cfg.auth_password;
    const char* eff_mqtt_user = (prov.valid && strlen(prov.mqtt_username) > 0)
                                 ? prov.mqtt_username : _cfg.mqtt_username;
    const char* eff_mqtt_pass = (prov.valid && strlen(prov.mqtt_password) > 0)
                                 ? prov.mqtt_password : _cfg.mqtt_password;

    // 8. Audio encryption init
#ifdef ROAMCAST_FEATURE_ENCRYPTION
    if (_cfg.features.encryption_enabled) {
        audio_encryption_init();
    }
#endif

    // 9. LED init + show connecting state
    rc_led_init();
    rc_led_set(RC_LED_ORANGE_SOLID);

    // 10. Connect to WiFi (blocking)
    rc_wifi_init(eff_wifi_ssid, eff_wifi_pass);
    if (!rc_wifi_is_connected()) {
        Serial.println("WiFi failed - entering error state");
        rc_led_set(RC_LED_RED_SOLID);
        return;
    }

    // 11. WiFi connected
    rc_led_set(RC_LED_BLUE_SOLID);

    // 12. mDNS discovery if enabled
    if (_cfg.features.mdns_enabled) {
        bool should_discover = (rc_get_server_ip() == nullptr || strlen(rc_get_server_ip()) == 0);
        if (!should_discover) {
            Serial.printf("Hub configured at %s, trying mDNS for updates...\n", rc_get_server_ip());
        }
        rc_mdns_discovery_init();
        if (rc_mdns_discovery_find_hub(_cfg.mdns_discovery_timeout_ms)) {
            Serial.printf("Using mDNS-discovered hub: %s:%d\n", rc_get_server_ip(), rc_get_api_port());
        } else if (should_discover) {
            Serial.println("mDNS discovery failed and no hub IP configured!");
        } else {
            Serial.printf("mDNS discovery failed, using configured: %s\n", rc_get_server_ip());
        }
    }

    // 13. Module scanner init (before MQTT so capabilities are known)
#ifdef ROAMCAST_FEATURE_MODULES
    if (_cfg.features.modules_enabled) {
        module_scanner_init(_cfg.i2c_sda_pin, _cfg.i2c_scl_pin, 60000);
    }
#endif

    // 14. Authenticate with hub (probe + JWT login)
    rc_auth_client_init(_device_id);
    rc_auth_client_startup(eff_auth_user, eff_auth_pass);

    // 15. Connect to MQTT broker
    rc_mqtt_init(_device_id, eff_mqtt_user, eff_mqtt_pass, _cfg.mqtt_reconnect_delay_ms);

    // 16. Set command callback
    rc_mqtt_set_command_callback(RoamCast::_static_command_handler);

    // 17. Publish discovery and capabilities
    bool has_speaker = roamcast::internal::hasSpeaker();
    bool has_led = (_cfg.status_indicator != nullptr);
    bool full_duplex = roamcast::internal::isFullDuplex();
    bool has_csi = false;
    bool has_ble = false;

#ifdef ROAMCAST_FEATURE_CSI
    has_csi = _cfg.features.csi_enabled;
#endif
#ifdef ROAMCAST_FEATURE_BLE
    has_ble = _cfg.features.ble_enabled;
#endif

    rc_mqtt_publish_discovery(
        _cfg.firmware_version, _cfg.hardware_model,
        has_speaker, has_led, has_csi, has_ble, full_duplex,
        _cfg.udp_audio_port);
    rc_mqtt_publish_capabilities(
        has_speaker, has_led, has_csi, has_ble, full_duplex);

    // 18. Presence sensor init
#ifdef ROAMCAST_FEATURE_PRESENCE
    if (_cfg.features.presence_enabled) {
        presence_sensor_init();
    }
#endif

    // 19. CSI motion init
#ifdef ROAMCAST_FEATURE_CSI
    if (_cfg.features.csi_enabled) {
        csi_motion_init(
            _cfg.csi_publish_interval_ms,
            _cfg.csi_keepalive_ms,
            _cfg.csi_publish_threshold,
            _cfg.csi_smoothing_alpha,
            _cfg.csi_history_depth,
            _cfg.csi_motion_max_variance);
    }
#endif

    // 20. BLE proximity init
#ifdef ROAMCAST_FEATURE_BLE
    if (_cfg.features.ble_enabled) {
        ble_proximity_init(
            _cfg.ble_publish_interval_ms,
            _cfg.ble_stale_timeout_ms,
            _cfg.ble_scan_interval_ms,
            _cfg.ble_scan_window_ms,
            _cfg.ble_rssi_smoothing_alpha);
    }
#endif

    // 21. Audio capture init
    rc_audio_capture_init(
        _device_id,
        _cfg.udp_audio_port,
        _cfg.dc_block_alpha,
        _cfg.gate_threshold,
        _cfg.gate_hold_frames,
        _cfg.audio_level_report_ms);

    // 22. Audio playback init
    rc_audio_playback_init();

    // 23. Start streaming immediately
    rc_audio_capture_start();

    // 24. Show streaming LED
    rc_led_set(RC_LED_BLUE_PULSE);

    // 25. Health reporter init
    rc_health_reporter_init(_device_id, _cfg.heartbeat_interval_ms);

    _setup_complete = true;
    Serial.println("\nRoamCast setup complete - streaming audio\n");
}

// ---------------------------------------------------------------------------
// loop()
// ---------------------------------------------------------------------------
void RoamCast::loop() {
    if (!_setup_complete) return;

    unsigned long loop_start = micros();

    // 1. Board-level loop hook
    if (_cfg.board_loop) {
        _cfg.board_loop();
    }

    // 2. Check WiFi: if disconnected, set red LED, delay, return
    if (!rc_wifi_is_connected()) {
        rc_led_set(RC_LED_RED_SOLID);
        delay(100);
        return;
    }

    // 3. Audio playback loop (always runs — handles tone, TTS, music state machine)
    rc_audio_playback_loop();

    // 4. Audio capture loop (only when not playing — mic/speaker conflict on half-duplex)
    if (!rc_audio_playback_is_playing()) {
        rc_audio_capture_loop();
    }

    // 5. MQTT loop
    rc_mqtt_loop();

    // 6. Auth client loop (periodic token refresh)
    const char* eff_auth_user = _cfg.auth_username;
    const char* eff_auth_pass = _cfg.auth_password;
    RcProvisionedConfig prov = rc_provisioning_get_config();
    if (prov.valid && strlen(prov.hub_username) > 0) {
        eff_auth_user = prov.hub_username;
        eff_auth_pass = prov.hub_password;
    }
    rc_auth_client_loop(eff_auth_user, eff_auth_pass);

    // 7. Health reporter loop
    rc_health_reporter_loop();

    // 8. Module scanner loop
#ifdef ROAMCAST_FEATURE_MODULES
    if (_cfg.features.modules_enabled) {
        module_scanner_loop();
    }
#endif

    // 9. Presence sensor loop
#ifdef ROAMCAST_FEATURE_PRESENCE
    if (_cfg.features.presence_enabled) {
        presence_sensor_loop();
    }
#endif

    // 10. CSI motion loop
#ifdef ROAMCAST_FEATURE_CSI
    if (_cfg.features.csi_enabled) {
        csi_motion_loop();
    }
#endif

    // 11. BLE proximity loop
#ifdef ROAMCAST_FEATURE_BLE
    if (_cfg.features.ble_enabled) {
        ble_proximity_loop();
    }
#endif

    // 12. LED animation loop
    rc_led_loop();

    // 13. Button handling (if callbacks provided)
    if (_cfg.button) {
        if (_cfg.button->wasClicked && _cfg.button->wasClicked()) {
            if (rc_audio_capture_is_streaming()) {
                rc_audio_capture_stop();
                rc_led_set(RC_LED_BLUE_SOLID);
                Serial.println("Streaming paused (button)");
            } else {
                rc_audio_capture_start();
                rc_led_set(RC_LED_BLUE_PULSE);
                Serial.println("Streaming resumed (button)");
            }
        }
        if (_cfg.button->pressedFor && _cfg.button->pressedFor(_cfg.factory_reset_hold_ms)) {
            Serial.println("Button held — factory reset!");
            rc_led_set(RC_LED_RED_SOLID);
            rc_provisioning_factory_reset();
        }
    }

    // 14. Track worst-case loop duration for health diagnostics
    unsigned long loop_dur = micros() - loop_start;
    if (loop_dur > _loop_max_us) _loop_max_us = loop_dur;
    _rc_loop_max_us = _loop_max_us;
}

// ---------------------------------------------------------------------------
// Status queries
// ---------------------------------------------------------------------------

const char* RoamCast::getDeviceId() const {
    return _device_id;
}

bool RoamCast::isWifiConnected() const {
    return rc_wifi_is_connected();
}

bool RoamCast::isMqttConnected() const {
    return rc_mqtt_is_connected();
}

bool RoamCast::isStreaming() const {
    return rc_audio_capture_is_streaming();
}

bool RoamCast::isPlaying() const {
    return rc_audio_playback_is_playing();
}

// ---------------------------------------------------------------------------
// Manual audio control
// ---------------------------------------------------------------------------

void RoamCast::startStreaming() {
    if (!rc_audio_capture_is_streaming()) {
        rc_audio_capture_start();
        rc_led_set(RC_LED_BLUE_PULSE);
    }
}

void RoamCast::stopStreaming() {
    if (rc_audio_capture_is_streaming()) {
        rc_audio_capture_stop();
        rc_led_set(RC_LED_BLUE_SOLID);
    }
}

// ---------------------------------------------------------------------------
// User command callback
// ---------------------------------------------------------------------------

void RoamCast::setCommandCallback(UserCommandCallback cb) {
    _user_cmd_cb = cb;
}

// ---------------------------------------------------------------------------
// Static command handler — routes from C callback to instance method
// ---------------------------------------------------------------------------

void RoamCast::_static_command_handler(const char* command, const char* params_json) {
    if (_instance) {
        _instance->_on_command(command, params_json);
    }
}

// ---------------------------------------------------------------------------
// Command dispatch
// ---------------------------------------------------------------------------

void RoamCast::_on_command(const char* command, const char* params_json) {
    if (strcmp(command, "set_led") == 0) {
        rc_led_set(RC_LED_GREEN_SOLID);
    }
    else if (strcmp(command, "start_audio") == 0) {
        startStreaming();
    }
    else if (strcmp(command, "stop_audio") == 0) {
        stopStreaming();
    }
    else if (strcmp(command, "set_volume") == 0) {
        JsonDocument doc;
        deserializeJson(doc, params_json);
        int vol = doc["volume"] | 128;
        rc_audio_playback_set_volume((uint8_t)vol);
    }
    else if (strcmp(command, "play_tone") == 0) {
        JsonDocument doc;
        deserializeJson(doc, params_json);
        uint16_t freq = doc["frequency"] | 1000;
        uint16_t dur = doc["duration"] | 200;
        rc_audio_playback_play_tone(freq, dur);
    }
    else if (strcmp(command, "tts_start") == 0) {
        rc_audio_playback_tts_start();
    }
    else if (strcmp(command, "music_start") == 0) {
        rc_audio_playback_music_start();
    }
    else if (strcmp(command, "music_stop") == 0) {
        rc_audio_playback_music_stop();
    }
    else if (strcmp(command, "music_flush") == 0) {
        rc_audio_playback_music_flush();
    }
#ifdef ROAMCAST_FEATURE_MODULES
    else if (strcmp(command, "scan_modules") == 0) {
        module_scanner_force_scan();
    }
#endif
#ifdef ROAMCAST_FEATURE_ENCRYPTION
    else if (strcmp(command, "set_audio_key") == 0) {
        JsonDocument doc;
        deserializeJson(doc, params_json);
        const char* key_b64 = doc["key"] | "";
        const char* prefix_b64 = doc["nonce_prefix"] | "";

        uint8_t key[16], prefix[12];
        size_t key_len = 0, prefix_len = 0;
        mbedtls_base64_decode(key, sizeof(key), &key_len,
                               (const uint8_t*)key_b64, strlen(key_b64));
        mbedtls_base64_decode(prefix, sizeof(prefix), &prefix_len,
                               (const uint8_t*)prefix_b64, strlen(prefix_b64));
        audio_encryption_set_key(key, key_len, prefix, prefix_len);
    }
#endif
#ifdef ROAMCAST_FEATURE_BLE
    else if (strcmp(command, "set_ble_targets") == 0) {
        ble_proximity_set_targets(params_json);
    }
#endif
    else if (strcmp(command, "factory_reset") == 0) {
        rc_provisioning_factory_reset();
    }
    else {
        // Unhandled built-in command — forward to user callback
        if (_user_cmd_cb) {
            _user_cmd_cb(command, params_json);
        }
    }
}
