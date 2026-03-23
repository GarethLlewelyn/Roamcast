// RoamCast — Generic ESP32-S3 Full-Duplex Example
// Separate I2S peripherals for mic and speaker.
// Mic stays active during TTS/music playback (no switching delay).

#include <RoamCast.h>
#include <boards/GenericESP32S3.h>

RoamCast device;

void setup() {
    GenericESP32S3Pins pins = {
        .mic_pin_data = 3,
        .mic_pin_clock = 4,
        .mic_pin_ws = 5,
        .spk_pin_data = 6,      // I2S data out to DAC/amp
        .spk_pin_clock = 7,     // I2S bit clock for speaker
        .spk_pin_ws = 8,        // I2S word select for speaker
        .led_pin = 21,
        .button_pin = 0,
        .full_duplex = true      // Separate I2S buses — mic and speaker coexist
    };

    auto cfg = GenericESP32S3::config(pins);
    cfg.hardware_model = "my_fullduplex_board";

    device.begin(cfg);
}

void loop() {
    device.loop();
}
