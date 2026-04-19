#include "mqtt_client.h"
#include "auth_client.h"
#include "runtime_config.h"
#include "wifi_manager.h"
#include "../RoamCastLog.h"

#ifdef ROAMCAST_FEATURE_MODULES
#include "../features/module_scanner.h"
#endif

#include <WiFi.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>
#include <time.h>

static WiFiClient wifi_client;
static PubSubClient client(wifi_client);

static const char* _device_id = nullptr;
static rc_mqtt_command_callback_t _command_callback = nullptr;
static unsigned long last_reconnect_attempt = 0;
static uint32_t _mqtt_publish_failures = 0;
static uint32_t _reconnect_delay_ms = 5000;

// Stored MQTT credentials
static char _mqtt_user[64];
static char _mqtt_pass[128];
static bool _has_mqtt_creds = false;

// Stored discovery parameters
static char _firmware_version[32];
static char _hardware_model[32];
static bool _has_speaker = false;
static bool _has_led = false;
static bool _has_ble = false;
static bool _is_full_duplex = false;
static uint16_t _udp_audio_port = 5100;

// Topic buffers
static char topic_discovery[80];
static char topic_status[80];
static char topic_health[80];
static char topic_audio_level[80];
static char topic_command[80];
static char topic_capabilities[80];
static char topic_modules[80];
static char topic_presence[80];
static char topic_ble_proximity[80];

static String get_timestamp() {
    struct tm timeinfo;
    if (getLocalTime(&timeinfo, 100)) {
        char buf[30];
        strftime(buf, sizeof(buf), "%Y-%m-%dT%H:%M:%SZ", &timeinfo);
        return String(buf);
    }
    return String("1970-01-01T00:00:00Z");
}

static void build_capabilities_array(JsonArray& caps) {
    caps.add("audio_in");
    if (_has_speaker) caps.add("audio_out");
    if (_has_led) caps.add("led");
#ifdef ROAMCAST_FEATURE_MODULES
    if (rc_module_scanner_has_module("presence_tmos")) caps.add("presence");
    if (rc_module_scanner_has_module("env_sensor")) caps.add("sensors");
    if (rc_module_scanner_has_module("touch")) caps.add("buttons");
    if (rc_module_scanner_has_module("light_sensor")) caps.add("sensors");
#endif
    if (_has_ble) caps.add("ble_proximity");
    if (_is_full_duplex) caps.add("full_duplex");
}

static void mqtt_callback(char* topic, byte* payload, unsigned int length) {
    JsonDocument doc;
    DeserializationError err = deserializeJson(doc, payload, length);
    if (err) {
        RC_LOG("MQTT: JSON parse error: %s", err.c_str());
        return;
    }

    const char* command = doc["command"] | "";
    // Command receipt is always logged (essential)
    RC_LOG("MQTT command: %s", command);

    if (strcmp(command, "restart") == 0) {
        RC_LOG("Restarting device...");
        delay(500);
        ESP.restart();
    }

    if (_command_callback) {
        char params[256];
        if (doc["params"].is<JsonObject>()) {
            serializeJson(doc["params"], params, sizeof(params));
        } else {
            serializeJson(doc, params, sizeof(params));
        }
        _command_callback(command, params);
    }
}

static const char* get_mqtt_user_or_null() {
    return _has_mqtt_creds && strlen(_mqtt_user) > 0 ? _mqtt_user : NULL;
}

static const char* get_mqtt_pass_or_null() {
    return _has_mqtt_creds && strlen(_mqtt_pass) > 0 ? _mqtt_pass : NULL;
}

static void do_connect() {
    char lwt_payload[128];
    JsonDocument lwt_doc;
    lwt_doc["state"] = "offline";
    lwt_doc["timestamp"] = get_timestamp();
    serializeJson(lwt_doc, lwt_payload, sizeof(lwt_payload));

    RC_DBG("MQTT: Connecting to %s:%d...", rc_get_server_ip(), rc_get_mqtt_port());

    if (client.connect(_device_id, get_mqtt_user_or_null(), get_mqtt_pass_or_null(),
                       topic_status, 1, true, lwt_payload)) {
        RC_LOG("MQTT connected to %s:%d", rc_get_server_ip(), rc_get_mqtt_port());
        client.subscribe(topic_command);

        rc_mqtt_publish_discovery(_firmware_version, _hardware_model,
                                  _has_speaker, _has_led,
                                  _has_ble, _is_full_duplex,
                                  _udp_audio_port);
        rc_mqtt_publish_capabilities(_has_speaker, _has_led,
                                     _has_ble, _is_full_duplex);
    } else {
        RC_LOG("MQTT connection failed (rc=%d)", client.state());
    }
}

void rc_mqtt_init(const char* device_id, const char* mqtt_user, const char* mqtt_pass,
                  uint32_t reconnect_delay_ms) {
    _device_id = device_id;
    _reconnect_delay_ms = reconnect_delay_ms;

    if (mqtt_user && strlen(mqtt_user) > 0) {
        strncpy(_mqtt_user, mqtt_user, sizeof(_mqtt_user) - 1);
        _mqtt_user[sizeof(_mqtt_user) - 1] = '\0';
        if (mqtt_pass) {
            strncpy(_mqtt_pass, mqtt_pass, sizeof(_mqtt_pass) - 1);
            _mqtt_pass[sizeof(_mqtt_pass) - 1] = '\0';
        } else {
            _mqtt_pass[0] = '\0';
        }
        _has_mqtt_creds = true;
    } else {
        _mqtt_user[0] = '\0';
        _mqtt_pass[0] = '\0';
        _has_mqtt_creds = false;
    }

    snprintf(topic_discovery, sizeof(topic_discovery),
             "satellite/devices/discovery");
    snprintf(topic_status, sizeof(topic_status),
             "satellite/devices/%s/status", device_id);
    snprintf(topic_health, sizeof(topic_health),
             "satellite/devices/%s/health", device_id);
    snprintf(topic_audio_level, sizeof(topic_audio_level),
             "satellite/devices/%s/audio/level", device_id);
    snprintf(topic_command, sizeof(topic_command),
             "satellite/devices/%s/command/exec", device_id);
    snprintf(topic_capabilities, sizeof(topic_capabilities),
             "satellite/devices/%s/capabilities", device_id);
    snprintf(topic_modules, sizeof(topic_modules),
             "satellite/devices/%s/modules", device_id);
    snprintf(topic_presence, sizeof(topic_presence),
             "satellite/devices/%s/presence/data", device_id);
    snprintf(topic_ble_proximity, sizeof(topic_ble_proximity),
             "satellite/devices/%s/ble/proximity", device_id);

    client.setServer(rc_get_server_ip(), rc_get_mqtt_port());
    client.setCallback(mqtt_callback);
    client.setBufferSize(2560);

    do_connect();
}

void rc_mqtt_loop() {
    if (!client.connected()) {
        unsigned long now = millis();
        if (now - last_reconnect_attempt >= _reconnect_delay_ms) {
            last_reconnect_attempt = now;
            RC_LOG("MQTT reconnecting to %s:%d...", rc_get_server_ip(), rc_get_mqtt_port());
            do_connect();
        }
        return;
    }
    client.loop();
}

bool rc_mqtt_is_connected() {
    return client.connected();
}

void rc_mqtt_publish_discovery(const char* firmware_version, const char* hardware_model,
                               bool has_speaker, bool has_led,
                               bool has_ble, bool is_full_duplex,
                               uint16_t udp_audio_port) {
    strncpy(_firmware_version, firmware_version ? firmware_version : "0.0.0",
            sizeof(_firmware_version) - 1);
    _firmware_version[sizeof(_firmware_version) - 1] = '\0';
    strncpy(_hardware_model, hardware_model ? hardware_model : "unknown",
            sizeof(_hardware_model) - 1);
    _hardware_model[sizeof(_hardware_model) - 1] = '\0';
    _has_speaker = has_speaker;
    _has_led = has_led;
    _has_ble = has_ble;
    _is_full_duplex = is_full_duplex;
    _udp_audio_port = udp_audio_port;

    JsonDocument doc;
    doc["device_id"] = _device_id;
    doc["mac_address"] = rc_wifi_get_mac();
    doc["ip_address"] = rc_wifi_get_ip();
    doc["firmware_version"] = _firmware_version;
    doc["hardware_model"] = _hardware_model;

    JsonArray caps = doc["capabilities"].to<JsonArray>();
    build_capabilities_array(caps);

    doc["udp_audio_port"] = _udp_audio_port;
    doc["authenticated"] = rc_auth_client_is_authenticated();
    doc["auth_method"] = "jwt";
    doc["timestamp"] = get_timestamp();

    char buffer[512];
    serializeJson(doc, buffer, sizeof(buffer));
    client.publish(topic_discovery, buffer);
    RC_DBG("MQTT: Published discovery for %s", _device_id);
}

void rc_mqtt_publish_capabilities(bool has_speaker, bool has_led,
                                   bool has_ble, bool is_full_duplex) {
    _has_speaker = has_speaker;
    _has_led = has_led;
    _has_ble = has_ble;
    _is_full_duplex = is_full_duplex;

    JsonDocument cap_doc;
    JsonArray caps = cap_doc["capabilities"].to<JsonArray>();
    build_capabilities_array(caps);
    cap_doc["timestamp"] = get_timestamp();

    char cap_buf[256];
    serializeJson(cap_doc, cap_buf, sizeof(cap_buf));
    client.publish(topic_capabilities, cap_buf, true);
}

void rc_mqtt_publish_status(const char* state, bool audio_streaming) {
    JsonDocument doc;
    doc["state"] = state;
    doc["uptime_seconds"] = millis() / 1000;
    doc["wifi_rssi"] = rc_wifi_get_rssi();
    doc["free_heap"] = ESP.getFreeHeap();
    doc["cpu_temp"] = temperatureRead();
    doc["audio_streaming"] = audio_streaming;
    doc["timestamp"] = get_timestamp();

    char buffer[256];
    serializeJson(doc, buffer, sizeof(buffer));
    if (!client.publish(topic_status, buffer, true)) _mqtt_publish_failures++;
}

void rc_mqtt_publish_health(int wifi_rssi, uint32_t free_heap, float cpu_temp, uint32_t uptime_s,
                            uint32_t dma_underruns, uint32_t udp_send_failures,
                            uint32_t mqtt_publish_failures, uint32_t heap_min_free,
                            unsigned long loop_max_us) {
    JsonDocument doc;
    doc["wifi_rssi"] = wifi_rssi;
    doc["free_heap"] = free_heap;
    doc["cpu_temp"] = cpu_temp;
    doc["uptime_seconds"] = uptime_s;
    doc["dma_underruns"] = dma_underruns;
    doc["udp_send_failures"] = udp_send_failures;
    doc["mqtt_publish_failures"] = mqtt_publish_failures;
    doc["heap_min_free"] = heap_min_free;
    doc["loop_max_us"] = loop_max_us;
    doc["timestamp"] = get_timestamp();

    char buffer[384];
    serializeJson(doc, buffer, sizeof(buffer));
    if (!client.publish(topic_health, buffer)) _mqtt_publish_failures++;
}

void rc_mqtt_publish_audio_level(float rms, float peak, bool is_speech) {
    JsonDocument doc;
    doc["rms"] = serialized(String(rms, 6));
    doc["peak"] = serialized(String(peak, 6));
    doc["is_speech"] = is_speech;
    doc["timestamp"] = get_timestamp();

    char buffer[192];
    serializeJson(doc, buffer, sizeof(buffer));
    if (!client.publish(topic_audio_level, buffer)) _mqtt_publish_failures++;
}

void rc_mqtt_publish_modules(const char* modules_json) {
    client.publish(topic_modules, modules_json, true);
}

void rc_mqtt_publish_presence(const char* presence_json) {
    client.publish(topic_presence, presence_json);
}

void rc_mqtt_publish_ble_proximity(const char* ble_json) {
    client.publish(topic_ble_proximity, ble_json);
}

void rc_mqtt_set_command_callback(rc_mqtt_command_callback_t cb) {
    _command_callback = cb;
}

uint32_t rc_mqtt_get_publish_failures() {
    return _mqtt_publish_failures;
}
