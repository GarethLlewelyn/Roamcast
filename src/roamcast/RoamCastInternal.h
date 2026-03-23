#pragma once

#include "RoamCastConfig.h"

// Internal shared state — set once during begin(), read by all modules.
// These are library-internal; users should not include this header.

namespace roamcast {
namespace internal {

// Set by RoamCast::begin()
void setCallbacks(AudioInputCallbacks* input, AudioOutputCallbacks* output,
                  StatusIndicatorCallbacks* indicator, ButtonCallbacks* btn);
void setConfig(const RoamCastConfig* cfg);

// Accessed by audio_capture, audio_playback, led_controller
AudioInputCallbacks* getAudioInput();
AudioOutputCallbacks* getAudioOutput();
StatusIndicatorCallbacks* getStatusIndicator();
ButtonCallbacks* getButton();
const RoamCastConfig* getConfig();

// Convenience queries
bool hasSpeaker();
bool isFullDuplex();

} // namespace internal
} // namespace roamcast
