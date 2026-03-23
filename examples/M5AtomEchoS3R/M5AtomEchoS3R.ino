// RoamCast — M5Stack Atom EchoS3R Example
// Full-featured satellite device using M5Unified.

#include <RoamCast.h>
#include <boards/M5AtomEchoS3R.h>

RoamCast device;

void setup() {
    auto cfg = M5AtomEchoS3R::config();

    // Override network defaults (or leave NULL to use captive portal provisioning)
    // cfg.wifi_ssid = "MyNetwork";
    // cfg.wifi_password = "MyPassword";
    // cfg.server_ip = "192.168.1.100";
    // cfg.auth_username = "admin";
    // cfg.auth_password = "changeme";

    device.begin(cfg);
}

void loop() {
    device.loop();
}
