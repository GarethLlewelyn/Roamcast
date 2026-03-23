#pragma once

#include <Arduino.h>
#include <math.h>

// Pure DSP functions extracted from audio_capture for reuse and testability.
// No hardware dependencies — operates on PCM buffers.

// DC blocker state
struct DcBlockerState {
    float prev_in;
    float prev_out;
};

// Voice gate state
struct VoiceGateState {
    uint8_t hold_counter;
    bool open;
    bool was_open;
};

// Remove DC offset from MEMS microphone signal.
// Single-pole high-pass filter: y[n] = x[n] - x[n-1] + alpha * y[n-1]
static inline void dc_block_inplace(int16_t* buffer, size_t len, float alpha,
                                     DcBlockerState& state) {
    for (size_t i = 0; i < len; i++) {
        float in = (float)buffer[i];
        float out = in - state.prev_in + alpha * state.prev_out;
        state.prev_in = in;
        state.prev_out = out;
        int32_t val = (int32_t)out;
        if (val > 32767) val = 32767;
        if (val < -32768) val = -32768;
        buffer[i] = (int16_t)val;
    }
}

// Calculate RMS and peak levels from a PCM buffer.
static inline void calculate_levels(const int16_t* buffer, size_t length,
                                     float& rms_out, float& peak_out) {
    int64_t sum_squares = 0;
    int16_t max_sample = 0;

    for (size_t i = 0; i < length; i++) {
        int32_t sample = buffer[i];
        sum_squares += sample * sample;
        int16_t abs_sample = abs(sample);
        if (abs_sample > max_sample) {
            max_sample = abs_sample;
        }
    }

    rms_out = sqrtf((float)sum_squares / length) / 32768.0f;
    peak_out = (float)max_sample / 32768.0f;
}

// Update voice gate state using simple peak threshold.
// Hold timer keeps gate open for hold_frames after speech to avoid choppy cutoffs.
static inline void voice_gate_update(const int16_t* buffer, size_t len,
                                      uint16_t threshold, uint8_t hold_frames,
                                      VoiceGateState& state) {
    bool frame_has_speech = false;

    int16_t frame_peak = 0;
    for (size_t i = 0; i < len; i++) {
        int16_t s = abs(buffer[i]);
        if (s > frame_peak) frame_peak = s;
    }
    frame_has_speech = (frame_peak > threshold);

    state.was_open = state.open;

    if (frame_has_speech) {
        state.open = true;
        state.hold_counter = hold_frames;
    } else if (state.hold_counter > 0) {
        state.hold_counter--;
    } else {
        state.open = false;
    }
}
