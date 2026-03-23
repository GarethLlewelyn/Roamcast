// RoamCast — Custom Callbacks Example
// Shows how to write your own audio callbacks for non-standard hardware.

#include <RoamCast.h>

// --- Custom mic implementation ---
static bool my_mic_init(const RoamCastAudioInputConfig* cfg) {
    // Initialize your custom mic hardware here
    Serial.printf("Custom mic init: %d Hz\n", cfg->sample_rate);
    return true;
}

static bool my_mic_begin() {
    // Start your mic
    return true;
}

static void my_mic_end() {
    // Stop your mic
}

static bool my_mic_record(int16_t* buffer, size_t samples, uint32_t sample_rate) {
    // Fill buffer with audio samples from your hardware.
    // This can be synchronous (blocking) or start an async DMA transfer.
    memset(buffer, 0, samples * sizeof(int16_t)); // Silence placeholder
    return true;
}

static bool my_mic_is_recording_done() {
    // Return true when the recording started by record() is complete.
    return true;
}

static bool my_mic_is_full_duplex() {
    return false;
}

static AudioInputCallbacks my_mic_cbs = {
    my_mic_init, my_mic_begin, my_mic_end,
    my_mic_record, my_mic_is_recording_done, my_mic_is_full_duplex
};

RoamCast device;

void setup() {
    RoamCastConfig cfg = roamcast_default_config();
    cfg.hardware_model = "custom_board";
    cfg.audio_input = &my_mic_cbs;
    // cfg.audio_output = &my_speaker_cbs;  // Optional
    // cfg.status_indicator = &my_led_cbs;  // Optional

    device.begin(cfg);
}

void loop() {
    device.loop();
}
