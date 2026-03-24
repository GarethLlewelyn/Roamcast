#include "provisioning.h"
#include "runtime_config.h"
#include "../RoamCastLog.h"

#include <Preferences.h>
#include <WiFi.h>
#include <WebServer.h>
#include <DNSServer.h>

static Preferences _prefs;
static RcProvisionedConfig _config;
static bool _provisioned = false;

// Default values passed in via init (replaces compile-time config.h defines)
static char _default_ssid[64];
static char _default_pass[64];
static char _default_hub_ip[46];
static uint16_t _default_api_port;
static char _default_auth_user[64];
static char _default_auth_pass[128];
static char _default_mqtt_user[64];
static char _default_mqtt_pass[128];

// Captive portal HTML form
static const char PORTAL_HTML[] PROGMEM = R"rawliteral(
<!DOCTYPE html><html><head>
<meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1">
<title>Satellite Device Setup</title>
<style>
body{font-family:system-ui;max-width:420px;margin:40px auto;padding:0 20px;background:#18181b;color:#e4e4e7}
h1{font-size:1.4em;color:#a1a1aa}
input,select{width:100%;padding:10px;margin:6px 0 16px;border:1px solid #3f3f46;border-radius:6px;background:#27272a;color:#e4e4e7;font-size:14px;box-sizing:border-box}
label{font-size:13px;color:#a1a1aa}
button{width:100%;padding:12px;background:#10b981;color:white;border:none;border-radius:6px;font-size:16px;cursor:pointer;margin-top:10px}
button:hover{background:#059669}
.note{font-size:12px;color:#71717a;margin-top:20px}
</style></head><body>
<h1>Satellite Device Setup</h1>
<form method="POST" action="/save">
<label>WiFi SSID</label><input name="ssid" required>
<label>WiFi Password</label><input name="wpass" type="password">
<label>Hub IP Address (leave blank for auto-discovery)</label><input name="hip" placeholder="Auto-discover via mDNS">
<label>Hub API Port</label><input name="hport" value="8100" type="number">
<label>Hub Username</label><input name="huser" value="admin">
<label>Hub Password</label><input name="hpass" type="password">
<label>MQTT Username (optional)</label><input name="muser" placeholder="device">
<label>MQTT Password (optional)</label><input name="mpass" type="password">
<button type="submit">Save & Connect</button>
</form>
<p class="note">Device will reboot and connect to the configured network.</p>
</body></html>
)rawliteral";

static const char SAVE_HTML[] PROGMEM = R"rawliteral(
<!DOCTYPE html><html><head>
<meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1">
<title>Saved</title>
<style>body{font-family:system-ui;max-width:420px;margin:40px auto;padding:0 20px;background:#18181b;color:#e4e4e7;text-align:center}h1{color:#10b981}</style>
</head><body>
<h1>Configuration Saved</h1>
<p>Device will reboot in 3 seconds...</p>
</body></html>
)rawliteral";

static void safe_copy(char* dst, const char* src, size_t dst_size) {
    if (src) {
        strncpy(dst, src, dst_size - 1);
        dst[dst_size - 1] = '\0';
    } else {
        dst[0] = '\0';
    }
}

void rc_provisioning_init(const char* default_ssid, const char* default_pass,
                          const char* default_hub_ip, uint16_t default_api_port,
                          const char* default_auth_user, const char* default_auth_pass,
                          const char* default_mqtt_user, const char* default_mqtt_pass) {
    // Store defaults for fallback
    safe_copy(_default_ssid, default_ssid, sizeof(_default_ssid));
    safe_copy(_default_pass, default_pass, sizeof(_default_pass));
    safe_copy(_default_hub_ip, default_hub_ip, sizeof(_default_hub_ip));
    _default_api_port = default_api_port;
    safe_copy(_default_auth_user, default_auth_user, sizeof(_default_auth_user));
    safe_copy(_default_auth_pass, default_auth_pass, sizeof(_default_auth_pass));
    safe_copy(_default_mqtt_user, default_mqtt_user, sizeof(_default_mqtt_user));
    safe_copy(_default_mqtt_pass, default_mqtt_pass, sizeof(_default_mqtt_pass));

    _prefs.begin("sat_prov", true); // read-only

    _provisioned = _prefs.getBool("valid", false);

    if (_provisioned) {
        _prefs.getString("ssid", _config.wifi_ssid, sizeof(_config.wifi_ssid));
        _prefs.getString("wpass", _config.wifi_password, sizeof(_config.wifi_password));
        _prefs.getString("hip", _config.hub_ip, sizeof(_config.hub_ip));
        _config.hub_api_port = _prefs.getUShort("hport", 8100);
        _prefs.getString("huser", _config.hub_username, sizeof(_config.hub_username));
        _prefs.getString("hpass", _config.hub_password, sizeof(_config.hub_password));
        _prefs.getString("muser", _config.mqtt_username, sizeof(_config.mqtt_username));
        _prefs.getString("mpass", _config.mqtt_password, sizeof(_config.mqtt_password));
        _config.valid = true;
        RC_DBG("Provisioning: loaded NVS credentials");
    } else {
        // Fall back to defaults passed in via init parameters
        safe_copy(_config.wifi_ssid, _default_ssid, sizeof(_config.wifi_ssid));
        safe_copy(_config.wifi_password, _default_pass, sizeof(_config.wifi_password));
        safe_copy(_config.hub_ip, _default_hub_ip, sizeof(_config.hub_ip));
        _config.hub_api_port = _default_api_port;
        safe_copy(_config.hub_username, _default_auth_user, sizeof(_config.hub_username));
        safe_copy(_config.hub_password, _default_auth_pass, sizeof(_config.hub_password));
        safe_copy(_config.mqtt_username, _default_mqtt_user, sizeof(_config.mqtt_username));
        safe_copy(_config.mqtt_password, _default_mqtt_pass, sizeof(_config.mqtt_password));
        _config.valid = (strlen(_default_ssid) > 0);
        RC_DBG("Provisioning: using provided defaults");
    }

    _prefs.end();
}

bool rc_provisioning_is_provisioned() {
    return _provisioned;
}

RcProvisionedConfig rc_provisioning_get_config() {
    return _config;
}

bool rc_provisioning_start_portal(const char* ap_name) {
    RC_LOG("Starting provisioning captive portal...");

    // Start AP
    WiFi.mode(WIFI_AP);
    WiFi.softAP(ap_name ? ap_name : "RoamCast-Setup");
    delay(500);

    IPAddress apIP = WiFi.softAPIP();
    RC_LOG("Portal AP IP: %s", apIP.toString().c_str());

    // DNS server for captive portal (redirect all to AP IP)
    DNSServer dnsServer;
    dnsServer.start(53, "*", apIP);

    // Web server
    WebServer server(80);
    volatile bool saved = false;

    server.on("/", HTTP_GET, [&server]() {
        server.send(200, "text/html", PORTAL_HTML);
    });

    server.on("/save", HTTP_POST, [&server, &saved]() {
        Preferences prefs;
        prefs.begin("sat_prov", false); // read-write

        prefs.putString("ssid", server.arg("ssid"));
        prefs.putString("wpass", server.arg("wpass"));
        prefs.putString("hip", server.arg("hip"));
        prefs.putUShort("hport", server.arg("hport").toInt());
        prefs.putString("huser", server.arg("huser"));
        prefs.putString("hpass", server.arg("hpass"));
        prefs.putString("muser", server.arg("muser"));
        prefs.putString("mpass", server.arg("mpass"));
        prefs.putBool("valid", true);
        prefs.end();

        server.send(200, "text/html", SAVE_HTML);
        saved = true;
        RC_LOG("Provisioning: credentials saved");
    });

    // Catch-all for captive portal detection
    server.onNotFound([&server]() {
        server.sendHeader("Location", "/");
        server.send(302);
    });

    server.begin();

    // Block until user submits
    while (!saved) {
        dnsServer.processNextRequest();
        server.handleClient();
        delay(10);
    }

    // Give the response time to be sent
    delay(2000);

    server.stop();
    dnsServer.stop();
    WiFi.softAPdisconnect(true);

    RC_LOG("Provisioning complete — rebooting...");
    delay(1000);
    ESP.restart();

    return true; // Never reached
}

void rc_provisioning_factory_reset() {
    RC_LOG("Factory reset: clearing NVS...");
    Preferences prefs;
    prefs.begin("sat_prov", false);
    prefs.clear();
    prefs.end();
    RC_LOG("NVS cleared — rebooting into provisioning mode...");
    delay(1000);
    ESP.restart();
}
