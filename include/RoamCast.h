#pragma once

#include <Arduino.h>
#include "roamcast/RoamCastConfig.h"

class RoamCast {
public:
    RoamCast();

    // Initialize the device. Call once in setup().
    // If wifi_ssid is NULL and device is not provisioned, starts captive portal.
    void begin(RoamCastConfig cfg);

    // Main loop. Call every iteration in loop().
    void loop();

    // Get the device ID (derived from MAC address).
    const char* getDeviceId() const;

    // Check connection status.
    bool isWifiConnected() const;
    bool isMqttConnected() const;

    // Check audio status.
    bool isStreaming() const;
    bool isPlaying() const;

    // Manual audio control.
    void startStreaming();
    void stopStreaming();

    // Set a custom command callback for user-defined MQTT commands.
    // Called for any command not handled by the built-in handler.
    typedef void (*UserCommandCallback)(const char* command, const char* params_json);
    void setCommandCallback(UserCommandCallback cb);

private:
    RoamCastConfig _cfg;
    char _device_id[32];
    bool _setup_complete;
    unsigned long _loop_max_us;
    UserCommandCallback _user_cmd_cb;

    void _on_command(const char* command, const char* params_json);
    static void _static_command_handler(const char* command, const char* params_json);
};
