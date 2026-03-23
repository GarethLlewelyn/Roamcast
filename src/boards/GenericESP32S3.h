#pragma once

#include "../roamcast/RoamCastConfig.h"

struct GenericESP32S3Pins {
    int mic_pin_data;       // I2S data in (SD)
    int mic_pin_clock;      // I2S bit clock (SCK)
    int mic_pin_ws;         // I2S word select (WS/LRCLK)
    int spk_pin_data;       // I2S data out (-1 = no speaker)
    int spk_pin_clock;      // I2S bit clock for speaker (-1 = no speaker)
    int spk_pin_ws;         // I2S word select for speaker (-1 = no speaker)
    int led_pin;            // WS2812B or GPIO LED (-1 = no LED)
    int button_pin;         // GPIO button (-1 = no button)
    bool full_duplex;       // true if mic and speaker use separate I2S peripherals
};

namespace GenericESP32S3 {
    // Returns a RoamCastConfig for a generic ESP32-S3 board.
    // Uses ESP-IDF I2S driver directly. No M5Unified dependency.
    RoamCastConfig config(GenericESP32S3Pins pins);
}
