#include "RoamCast.h"

#include "roamcast/RoamCastConfig.h"
#include "roamcast/RoamCastInternal.h"
#include "roamcast/RoamCastLog.h"
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

#ifdef ROAMCAST_FEATURE_MODULES
#include "roamcast/features/module_scanner.h"
#endif

#ifdef ROAMCAST_FEATURE_PRESENCE
#include "roamcast/features/presence_sensor.h"
#endif

#include <ArduinoJson.h>

static RoamCast* _instance = nullptr;

static unsigned long _rc_loop_max_us = 0;

extern "C" {
    unsigned long rc_loop_get_max_us() {
        return _rc_loop_max_us;
    }
    void rc_loop_reset_max_us() {
        _rc_loop_max_us = 0;
    }
}

RoamCast::RoamCast()
    : _cfg{}
    , _device_id{0}
    , _setup_complete(false)
    , _loop_max_us(0)
    , _user_cmd_cb(nullptr)
{
    _instance = this;
}

void RoamCast::begin(RoamCastConfig cfg) {
    _cfg = cfg;
    _setup_complete = false;

    if (_cfg.board_init) {
        _cfg.board_init();
    }

    delay(100);

    roamcast::log::setDebugLevel(_cfg.debug_level);

    RC_LOG("Starting (debug_level=%d, heap=%u)", _cfg.debug_level, ESP.getFreeHeap());

    uint8_t mac[6];
    esp_efuse_mac_get_default(mac);
    const char* prefix = _cfg.device_id_prefix ? _cfg.device_id_prefix : "roamcast";
    snprintf(_device_id, sizeof(_device_id), "%s_%02X%02X%02X%02X",
             prefix, mac[2], mac[3], mac[4], mac[5]);
    RC_LOG("Device ID: %s", _device_id);

    RC_DBG("callbacks in=%p out=%p led=%p btn=%p",
            _cfg.audio_input, _cfg.audio_output, _cfg.status_indicator, _cfg.button);
    roamcast::internal::setCallbacks(
        _cfg.audio_input, _cfg.audio_output,
        _cfg.status_indicator, _cfg.button);
    roamcast::internal::setConfig(&_cfg);

    RC_DBG("runtime server=%s api=%d mqtt=%d",
            _cfg.server_ip ? _cfg.server_ip : "NULL", _cfg.api_port, _cfg.mqtt_port);
    rc_runtime_config_init(
        _cfg.server_ip,
        _cfg.api_port,
        _cfg.mqtt_port);

    rc_provisioning_init(
        _cfg.wifi_ssid, _cfg.wifi_password,
        _cfg.server_ip, _cfg.api_port,
        _cfg.auth_username, _cfg.auth_password,
        _cfg.mqtt_username, _cfg.mqtt_password);
    RC_DBG("provisioned=%d", rc_provisioning_is_provisioned());

    if (!rc_provisioning_is_provisioned() && _cfg.wifi_ssid == nullptr) {
        RC_LOG("No WiFi credentials — starting provisioning portal");
        const char* ap_name = _cfg.portal_ap_name ? _cfg.portal_ap_name : "RoamCast-Setup";
        rc_provisioning_start_portal(ap_name);
        return;
    }

    RcProvisionedConfig prov = rc_provisioning_get_config();
    RC_DBG("provision valid=%d hub_ip=%s", prov.valid, prov.hub_ip);
    if (prov.valid && strlen(prov.hub_ip) > 0) {
        rc_set_server_ip(prov.hub_ip);
        rc_set_api_port(prov.hub_api_port);
    }

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

#ifdef ROAMCAST_FEATURE_ENCRYPTION
    if (_cfg.features.encryption_enabled) {
        audio_encryption_init();
    }
#endif

    rc_led_init();
    rc_led_set(RC_LED_ORANGE_SOLID);

    RC_LOG("Connecting to WiFi '%s'...", eff_wifi_ssid ? eff_wifi_ssid : "NULL");
    rc_wifi_init(eff_wifi_ssid, eff_wifi_pass);
    if (!rc_wifi_is_connected()) {
        RC_LOG("WiFi FAILED — setup incomplete");
        rc_led_set(RC_LED_RED_SOLID);
        return;
    }
    RC_LOG("WiFi connected — IP: %s, RSSI: %d dBm", rc_wifi_get_ip().c_str(), rc_wifi_get_rssi());

    rc_led_set(RC_LED_BLUE_SOLID);

    if (_cfg.features.mdns_enabled) {
        rc_mdns_discovery_init();
        if (rc_mdns_discovery_find_hub(_cfg.mdns_discovery_timeout_ms)) {
            RC_LOG("mDNS: hub at %s (API:%d, MQTT:%d)", rc_get_server_ip(), rc_get_api_port(), rc_get_mqtt_port());
        } else if (rc_get_server_ip() == nullptr || strlen(rc_get_server_ip()) == 0) {
            RC_LOG("mDNS: discovery failed, no hub IP configured!");
        } else {
            RC_LOG("mDNS: discovery failed, using configured %s", rc_get_server_ip());
        }
    }

#ifdef ROAMCAST_FEATURE_MODULES
    if (_cfg.features.modules_enabled) {
        module_scanner_init(_cfg.i2c_sda_pin, _cfg.i2c_scl_pin, 60000);
    }
#endif

    RC_LOG("Authenticating with hub at %s:%d...", rc_get_server_ip(), rc_get_api_port());
    rc_auth_client_init(_device_id);
    rc_auth_client_startup(eff_auth_user, eff_auth_pass);
    RC_DBG("auth state=%d", rc_auth_client_get_state());

    RC_LOG("Connecting to MQTT at %s:%d...", rc_get_server_ip(), rc_get_mqtt_port());
    rc_mqtt_init(_device_id, eff_mqtt_user, eff_mqtt_pass, _cfg.mqtt_reconnect_delay_ms);
    RC_LOG("MQTT connected: %s", rc_mqtt_is_connected() ? "yes" : "no");

    rc_mqtt_set_command_callback(RoamCast::_static_command_handler);

    bool has_speaker = roamcast::internal::hasSpeaker();
    bool has_led = (_cfg.status_indicator != nullptr);
    bool full_duplex = roamcast::internal::isFullDuplex();
    bool has_ble = false;

#ifdef ROAMCAST_FEATURE_BLE
    has_ble = _cfg.features.ble_enabled;
#endif

    RC_DBG("discovery spk=%d led=%d duplex=%d ble=%d",
            has_speaker, has_led, full_duplex, has_ble);
    rc_mqtt_publish_discovery(
        _cfg.firmware_version, _cfg.hardware_model,
        has_speaker, has_led, has_ble, full_duplex,
        _cfg.udp_audio_port);
    rc_mqtt_publish_capabilities(
        has_speaker, has_led, has_ble, full_duplex);

#ifdef ROAMCAST_FEATURE_PRESENCE
    if (_cfg.features.presence_enabled) {
        presence_sensor_init();
    }
#endif

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

    RC_DBG("audio_capture port=%d", _cfg.udp_audio_port);
    rc_audio_capture_init(
        _device_id,
        _cfg.udp_audio_port,
        _cfg.dc_block_alpha,
        _cfg.gate_threshold,
        _cfg.gate_hold_frames,
        _cfg.audio_level_report_ms);

    rc_audio_playback_init();

    rc_audio_capture_start();

    rc_led_set(RC_LED_BLUE_PULSE);

    rc_health_reporter_init(_device_id, _cfg.heartbeat_interval_ms);

    _setup_complete = true;
    RC_LOG("Setup complete — streaming audio (heap=%u)", ESP.getFreeHeap());
}

void RoamCast::loop() {
    if (!_setup_complete) return;

    unsigned long loop_start = micros();

    if (_cfg.board_loop) {
        _cfg.board_loop();
    }

    if (!rc_wifi_is_connected()) {
        rc_led_set(RC_LED_RED_SOLID);
        delay(100);
        return;
    }

    rc_audio_playback_loop();

    if (!rc_audio_playback_is_playing()) {
        rc_audio_capture_loop();
    }

    rc_mqtt_loop();

    const char* eff_auth_user = _cfg.auth_username;
    const char* eff_auth_pass = _cfg.auth_password;
    RcProvisionedConfig prov = rc_provisioning_get_config();
    if (prov.valid && strlen(prov.hub_username) > 0) {
        eff_auth_user = prov.hub_username;
        eff_auth_pass = prov.hub_password;
    }
    rc_auth_client_loop(eff_auth_user, eff_auth_pass);

    rc_health_reporter_loop();

#ifdef ROAMCAST_FEATURE_MODULES
    if (_cfg.features.modules_enabled) {
        module_scanner_loop();
    }
#endif

#ifdef ROAMCAST_FEATURE_PRESENCE
    if (_cfg.features.presence_enabled) {
        presence_sensor_loop();
    }
#endif

#ifdef ROAMCAST_FEATURE_BLE
    if (_cfg.features.ble_enabled) {
        ble_proximity_loop();
    }
#endif

    rc_led_loop();

    if (_cfg.button) {
        if (_cfg.button->wasClicked && _cfg.button->wasClicked()) {
            if (rc_audio_capture_is_streaming()) {
                rc_audio_capture_stop();
                rc_led_set(RC_LED_BLUE_SOLID);
                RC_LOG("Streaming paused (button)");
            } else {
                rc_audio_capture_start();
                rc_led_set(RC_LED_BLUE_PULSE);
                RC_LOG("Streaming resumed (button)");
            }
        }
        if (_cfg.button->pressedFor && _cfg.button->pressedFor(_cfg.factory_reset_hold_ms)) {
            RC_LOG("Button held — factory reset!");
            rc_led_set(RC_LED_RED_SOLID);
            rc_provisioning_factory_reset();
        }
    }

    unsigned long loop_dur = micros() - loop_start;
    if (loop_dur > _loop_max_us) _loop_max_us = loop_dur;
    _rc_loop_max_us = _loop_max_us;
}

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

void RoamCast::setCommandCallback(UserCommandCallback cb) {
    _user_cmd_cb = cb;
}

void RoamCast::_static_command_handler(const char* command, const char* params_json) {
    if (_instance) {
        _instance->_on_command(command, params_json);
    }
}

void RoamCast::_on_command(const char* command, const char* params_json) {
    RC_LOG("Command: %s", command);
    RC_DBG("params: %s", params_json ? params_json : "null");

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
        if (_user_cmd_cb) {
            _user_cmd_cb(command, params_json);
        }
    }
}
