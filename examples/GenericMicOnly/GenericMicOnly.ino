// RoamCast — Generic ESP32-S3 Mic-Only Example
// Minimal satellite device: just a microphone, no speaker.
// Works with INMP441, SPH0645, or similar I2S MEMS microphones.

#include <RoamCast.h>
#include <boards/GenericESP32S3.h>

RoamCast device;

void setup() {
    GenericESP32S3Pins pins = {
        .mic_pin_data = 3,       // I2S SD (data in from mic)
        .mic_pin_clock = 4,      // I2S SCK (bit clock)
        .mic_pin_ws = 5,         // I2S WS (word select / LRCLK)
        .spk_pin_data = -1,      // No speaker
        .spk_pin_clock = -1,
        .spk_pin_ws = -1,
        .led_pin = 21,           // WS2812B NeoPixel on GPIO 21 (-1 to disable)
        .button_pin = 0,         // Boot button on GPIO 0 (-1 to disable)
        .full_duplex = false     // N/A for mic-only
    };

    auto cfg = GenericESP32S3::config(pins);
    cfg.hardware_model = "my_mic_board";

    // Leave wifi_ssid NULL to use captive portal on first boot
    // Or set credentials here:
    // cfg.wifi_ssid = "MyNetwork";
    // cfg.wifi_password = "MyPassword";

    device.begin(cfg);
}

void loop() {
    device.loop();
}
