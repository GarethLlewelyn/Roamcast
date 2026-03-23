#include "RoamCastInternal.h"

namespace roamcast {
namespace internal {

static AudioInputCallbacks* _audio_input = nullptr;
static AudioOutputCallbacks* _audio_output = nullptr;
static StatusIndicatorCallbacks* _status_indicator = nullptr;
static ButtonCallbacks* _button = nullptr;
static const RoamCastConfig* _config = nullptr;

void setCallbacks(AudioInputCallbacks* input, AudioOutputCallbacks* output,
                  StatusIndicatorCallbacks* indicator, ButtonCallbacks* btn) {
    _audio_input = input;
    _audio_output = output;
    _status_indicator = indicator;
    _button = btn;
}

void setConfig(const RoamCastConfig* cfg) {
    _config = cfg;
}

AudioInputCallbacks* getAudioInput() { return _audio_input; }
AudioOutputCallbacks* getAudioOutput() { return _audio_output; }
StatusIndicatorCallbacks* getStatusIndicator() { return _status_indicator; }
ButtonCallbacks* getButton() { return _button; }
const RoamCastConfig* getConfig() { return _config; }

bool hasSpeaker() { return _audio_output != nullptr; }
bool isFullDuplex() {
    return _audio_input && _audio_input->isFullDuplex && _audio_input->isFullDuplex();
}

} // namespace internal
} // namespace roamcast
