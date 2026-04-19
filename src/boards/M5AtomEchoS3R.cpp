#include "M5AtomEchoS3R.h"
#include <M5Unified.h>

// M5 Atom Echo S3R board preset (M5Unified). Mic and speaker share one I2S bus (half-duplex).

static RoamCastAudioOutputConfig _spk_cfg = { 16000, 256, 8 };

// Track whether mic/speaker I2S is currently active
static bool _speaker_active = false;
static bool _mic_active = false;

static bool mic_init(const RoamCastAudioInputConfig* cfg) {
    auto mic_cfg = M5.Mic.config();
    mic_cfg.sample_rate = cfg->sample_rate;
    mic_cfg.magnification = cfg->magnification;
    mic_cfg.noise_filter_level = cfg->noise_filter;
    mic_cfg.dma_buf_count = cfg->dma_buf_count;
    mic_cfg.dma_buf_len = cfg->dma_buf_len;
    M5.Mic.config(mic_cfg);
    return true;
}

static bool mic_begin() {
    M5.Speaker.end();
    _speaker_active = false;
    _mic_active = M5.Mic.begin();
    return _mic_active;
}

static void mic_end() {
    M5.Mic.end();
    _mic_active = false;
}

static bool mic_record(int16_t* buffer, size_t samples, uint32_t sample_rate) {
    return M5.Mic.record(buffer, samples, sample_rate);
}

static bool mic_is_recording_done() {
    return M5.Mic.isRecording() == 0;
}

static bool mic_is_full_duplex() {
    return false;
}

static AudioInputCallbacks _mic_cbs = {
    mic_init,
    mic_begin,
    mic_end,
    mic_record,
    mic_is_recording_done,
    mic_is_full_duplex
};

static bool spk_init(const RoamCastAudioOutputConfig* cfg) {
    _spk_cfg = *cfg;
    return true;
}

static bool spk_begin() {
    M5.Mic.end();
    _mic_active = false;

    auto spk_cfg = M5.Speaker.config();
    if (_spk_cfg.sample_rate > 0) spk_cfg.sample_rate = _spk_cfg.sample_rate;
    if (_spk_cfg.dma_buf_len > 0) spk_cfg.dma_buf_len = _spk_cfg.dma_buf_len;
    if (_spk_cfg.dma_buf_count > 0) spk_cfg.dma_buf_count = _spk_cfg.dma_buf_count;
    M5.Speaker.config(spk_cfg);
    _speaker_active = M5.Speaker.begin();
    return _speaker_active;
}

static void spk_end() {
    if (_speaker_active) {
        M5.Speaker.end();
        _speaker_active = false;
    }
}

static bool spk_play_raw(const int16_t* data, size_t samples, uint32_t sample_rate,
                          bool stereo, int repeat, int channel, bool stop_current) {
    return M5.Speaker.playRaw(data, samples, sample_rate, stereo, repeat, channel, stop_current);
}

static bool spk_is_playing() {
    return M5.Speaker.isPlaying();
}

static void spk_stop() {
    M5.Speaker.stop();
}

static void spk_set_volume(uint8_t vol) {
    M5.Speaker.setVolume(vol);
}

static AudioOutputCallbacks _spk_cbs = {
    spk_init,
    spk_begin,
    spk_end,
    spk_play_raw,
    spk_is_playing,
    spk_stop,
    spk_set_volume
};

static void led_init_fn() {}

static void led_set_color(uint8_t r, uint8_t g, uint8_t b) {
    M5.Led.setColor(0, r, g, b);
}

static void led_off() {
    M5.Led.setColor(0, 0, 0, 0);
}

static StatusIndicatorCallbacks _led_cbs = {
    led_init_fn,
    led_set_color,
    led_off
};

static void btn_init_fn() {}

static bool btn_was_clicked() {
    return M5.BtnA.wasClicked();
}

static bool btn_pressed_for(uint32_t ms) {
    return M5.BtnA.pressedFor(ms);
}

static ButtonCallbacks _btn_cbs = {
    btn_init_fn,
    btn_was_clicked,
    btn_pressed_for
};

static void board_init() {
    auto cfg = M5.config();
    cfg.internal_mic = true;
    cfg.internal_spk = true;
    cfg.serial_baudrate = 115200;
    M5.begin(cfg);

    M5.Speaker.end();
    _speaker_active = false;
}

static void board_loop() {
    M5.update();
}

RoamCastConfig M5AtomEchoS3R::config() {
    RoamCastConfig cfg = roamcast_default_config();

    cfg.hardware_model   = "atom_echo_s3r";
    cfg.device_id_prefix = "echo_s3r";
    cfg.firmware_version = "1.4.0";

    cfg.audio_input      = &_mic_cbs;
    cfg.audio_output     = &_spk_cbs;
    cfg.status_indicator = &_led_cbs;
    cfg.button           = &_btn_cbs;

    cfg.board_init       = board_init;
    cfg.board_loop       = board_loop;

    cfg.features.ble_enabled        = true;
    cfg.features.encryption_enabled = true;
    cfg.features.modules_enabled    = true;
    cfg.features.presence_enabled   = true;
    cfg.features.mdns_enabled       = true;

    cfg.i2c_sda_pin = 2;
    cfg.i2c_scl_pin = 1;

    return cfg;
}
