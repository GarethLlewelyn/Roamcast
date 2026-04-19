#include "auth_client.h"
#include "runtime_config.h"
#include "../RoamCastLog.h"

#include <HTTPClient.h>
#include <WiFiClient.h>
#include <ArduinoJson.h>
#include <mbedtls/base64.h>

static const char* _device_id = nullptr;
static RcAuthState _state = RC_AUTH_NOT_STARTED;
static char        _jwt_token[2048];
static unsigned long _token_expiry_epoch = 0;
static unsigned long _last_refresh_check = 0;

static const unsigned long REFRESH_CHECK_INTERVAL_MS = 60000;
static const unsigned long REFRESH_BEFORE_EXPIRY_S   = 3600;
static const int           LOGIN_RETRY_DELAY_MS      = 2000;
static const int           HTTP_TIMEOUT_MS            = 5000;

static String build_api_url(const char* path) {
    return String("http://") + rc_get_server_ip() + ":" + String(rc_get_api_port()) + "/api/" + path;
}

static bool probe_hub() {
    HTTPClient http;
    WiFiClient wifi;

    String ping_url = build_api_url("auth/ping");
    http.begin(wifi, ping_url);
    http.setTimeout(HTTP_TIMEOUT_MS);
    int code = http.GET();
    http.end();

    if (code <= 0) {
        RC_LOG("Auth: Hub unreachable at %s:%d (HTTP %d)", rc_get_server_ip(), rc_get_api_port(), code);
        return false;
    }

    RC_LOG("Auth: Hub reachable (HTTP %d)", code);
    return true;
}

static bool check_auth_required() {
    HTTPClient http;
    WiFiClient wifi;

    String devices_url = build_api_url("devices");
    http.begin(wifi, devices_url);
    http.setTimeout(HTTP_TIMEOUT_MS);
    int code = http.GET();
    http.end();

    if (code == 401 || code == 403) {
        RC_DBG("Auth: Hub requires authentication");
        return true;
    }

    RC_DBG("Auth: Hub does not require auth (HTTP %d)", code);
    return false;
}

static bool extract_token_expiry(const char* jwt) {
    const char* first_dot = strchr(jwt, '.');
    if (!first_dot) return false;

    const char* payload_start = first_dot + 1;
    const char* second_dot = strchr(payload_start, '.');
    if (!second_dot) return false;

    size_t b64_len = second_dot - payload_start;
    if (b64_len > 2048) return false;

    char b64[2048];
    memcpy(b64, payload_start, b64_len);
    b64[b64_len] = '\0';

    for (size_t i = 0; i < b64_len; i++) {
        if (b64[i] == '-') b64[i] = '+';
        else if (b64[i] == '_') b64[i] = '/';
    }

    while (b64_len % 4 != 0) {
        b64[b64_len++] = '=';
        b64[b64_len] = '\0';
    }

    unsigned char decoded[2048];
    size_t decoded_len = 0;
    int ret = mbedtls_base64_decode(decoded, sizeof(decoded) - 1, &decoded_len,
                                     (const unsigned char*)b64, b64_len);
    if (ret != 0) {
        RC_DBG("Auth: Base64 decode failed (%d)", ret);
        return false;
    }
    decoded[decoded_len] = '\0';

    JsonDocument doc;
    DeserializationError err = deserializeJson(doc, (char*)decoded, decoded_len);
    if (err) {
        RC_DBG("Auth: JWT payload parse error: %s", err.c_str());
        return false;
    }

    if (doc["exp"].is<unsigned long>()) {
        _token_expiry_epoch = doc["exp"].as<unsigned long>();
        RC_DBG("Auth: Token expires at epoch %lu", _token_expiry_epoch);
        return true;
    }

    return false;
}

static bool do_login(const char* username, const char* password) {
    HTTPClient http;
    WiFiClient wifi;

    String login_url = build_api_url("auth/login");
    http.begin(wifi, login_url);
    http.setTimeout(HTTP_TIMEOUT_MS);
    http.addHeader("Content-Type", "application/json");

    JsonDocument req;
    req["username"] = username;
    req["password"] = password;
    char body[256];
    serializeJson(req, body, sizeof(body));

    int code = http.POST(body);

    if (code == 200) {
        String response = http.getString();
        http.end();

        JsonDocument resp;
        DeserializationError err = deserializeJson(resp, response);
        if (err) {
            RC_LOG("Auth: Login response parse error: %s", err.c_str());
            return false;
        }

        const char* token = resp["access_token"] | (const char*)nullptr;
        if (!token) {
            RC_LOG("Auth: No access_token in response");
            return false;
        }

        strncpy(_jwt_token, token, sizeof(_jwt_token) - 1);
        _jwt_token[sizeof(_jwt_token) - 1] = '\0';

        if (!extract_token_expiry(_jwt_token)) {
            struct timeval tv;
            gettimeofday(&tv, NULL);
            _token_expiry_epoch = tv.tv_sec + (6 * 24 * 3600);
        }

        RC_LOG("Auth: Login successful");
        return true;
    }

    String err_body = http.getString();
    http.end();
    RC_LOG("Auth: Login failed (HTTP %d)", code);
    return false;
}

void rc_auth_client_init(const char* device_id) {
    _device_id = device_id;
    _state = RC_AUTH_NOT_STARTED;
    _jwt_token[0] = '\0';
    _token_expiry_epoch = 0;
    _last_refresh_check = 0;
}

void rc_auth_client_startup(const char* username, const char* password) {
    if (username == nullptr || strlen(username) == 0) {
        RC_LOG("Auth: No credentials — skipping");
        _state = RC_AUTH_NOT_REQUIRED;
        return;
    }

    _state = RC_AUTH_PROBING;

    if (!probe_hub()) {
        RC_LOG("Auth: Hub unreachable — degraded mode");
        _state = RC_AUTH_DEGRADED;
        return;
    }

    if (!check_auth_required()) {
        RC_LOG("Auth: Hub is open — no auth needed");
        _state = RC_AUTH_NOT_REQUIRED;
        return;
    }

    if (do_login(username, password)) {
        _state = RC_AUTH_AUTHENTICATED;
        return;
    }

    RC_DBG("Auth: Retrying login...");
    delay(LOGIN_RETRY_DELAY_MS);

    if (do_login(username, password)) {
        _state = RC_AUTH_AUTHENTICATED;
        return;
    }

    RC_LOG("Auth: Login failed — degraded mode");
    _state = RC_AUTH_DEGRADED;
}

void rc_auth_client_loop(const char* username, const char* password) {
    if (_state != RC_AUTH_AUTHENTICATED) return;

    unsigned long now = millis();
    if (now - _last_refresh_check < REFRESH_CHECK_INTERVAL_MS) return;
    _last_refresh_check = now;

    struct timeval tv;
    gettimeofday(&tv, NULL);
    unsigned long current_epoch = tv.tv_sec;

    if (_token_expiry_epoch == 0) return;

    if (current_epoch + REFRESH_BEFORE_EXPIRY_S >= _token_expiry_epoch) {
        RC_LOG("Auth: Token expiring soon — refreshing...");

        if (username != nullptr && strlen(username) > 0 && do_login(username, password)) {
            RC_LOG("Auth: Token refreshed");
        } else {
            RC_LOG("Auth: Token refresh failed — degraded mode");
            _state = RC_AUTH_DEGRADED;
        }
    }
}

bool rc_auth_client_is_authenticated() {
    return _state == RC_AUTH_AUTHENTICATED;
}

bool rc_auth_client_is_degraded() {
    return _state == RC_AUTH_DEGRADED;
}

RcAuthState rc_auth_client_get_state() {
    return _state;
}

const char* rc_auth_client_get_token() {
    if (_state == RC_AUTH_AUTHENTICATED && _jwt_token[0] != '\0') {
        return _jwt_token;
    }
    return nullptr;
}
