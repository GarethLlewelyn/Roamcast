#include "GenericESP32S3.h"
#include <driver/i2s.h>

// --- Static pin config ---
static GenericESP32S3Pins _pins = {};
static bool _mic_recording = false;
static int16_t* _mic_buffer = nullptr;
static size_t _mic_samples = 0;
static uint32_t _mic_sample_rate = 16000;
static uint8_t _spk_volume = 128;

// I2S port assignments
#define MIC_I2S_PORT I2S_NUM_0
#define SPK_I2S_PORT I2S_NUM_1  // Only used in full-duplex mode

static i2s_port_t spk_port() {
    return _pins.full_duplex ? SPK_I2S_PORT : MIC_I2S_PORT;
}

// --- Mic callbacks ---

static bool mic_init(const RoamCastAudioInputConfig* cfg) {
    i2s_config_t i2s_cfg = {};
    i2s_cfg.mode = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_RX);
    i2s_cfg.sample_rate = cfg->sample_rate;
    i2s_cfg.bits_per_sample = I2S_BITS_PER_SAMPLE_16BIT;
    i2s_cfg.channel_format = I2S_CHANNEL_FMT_ONLY_LEFT;
    i2s_cfg.communication_format = I2S_COMM_FORMAT_STAND_I2S;
    i2s_cfg.intr_alloc_flags = ESP_INTR_FLAG_LEVEL1;
    i2s_cfg.dma_buf_count = cfg->dma_buf_count;
    i2s_cfg.dma_buf_len = cfg->dma_buf_len;
    i2s_cfg.use_apll = false;

    esp_err_t err = i2s_driver_install(MIC_I2S_PORT, &i2s_cfg, 0, NULL);
    if (err != ESP_OK) {
        Serial.printf("[GenericESP32S3] I2S mic driver install failed: %d\n", err);
        return false;
    }

    i2s_pin_config_t pin_cfg = {};
    pin_cfg.bck_io_num = _pins.mic_pin_clock;
    pin_cfg.ws_io_num = _pins.mic_pin_ws;
    pin_cfg.data_in_num = _pins.mic_pin_data;
    pin_cfg.data_out_num = I2S_PIN_NO_CHANGE;

    err = i2s_set_pin(MIC_I2S_PORT, &pin_cfg);
    if (err != ESP_OK) {
        Serial.printf("[GenericESP32S3] I2S mic set_pin failed: %d\n", err);
        return false;
    }

    _mic_sample_rate = cfg->sample_rate;
    Serial.println("[GenericESP32S3] Mic I2S initialized");
    return true;
}

static bool mic_begin() {
    i2s_start(MIC_I2S_PORT);
    return true;
}

static void mic_end() {
    i2s_stop(MIC_I2S_PORT);
}

static bool mic_record(int16_t* buffer, size_t samples, uint32_t sample_rate) {
    _mic_buffer = buffer;
    _mic_samples = samples;
    _mic_recording = true;
    return true;
}

static bool mic_is_recording_done() {
    if (!_mic_recording) return true;

    // Perform synchronous I2S read
    size_t bytes_read = 0;
    size_t bytes_wanted = _mic_samples * sizeof(int16_t);
    esp_err_t err = i2s_read(MIC_I2S_PORT, _mic_buffer, bytes_wanted,
                              &bytes_read, portMAX_DELAY);

    _mic_recording = false;

    if (err != ESP_OK || bytes_read != bytes_wanted) {
        memset(_mic_buffer, 0, bytes_wanted);
    }

    return true;
}

static bool mic_is_full_duplex() {
    return _pins.full_duplex;
}

// --- Speaker callbacks ---

static bool spk_init(const RoamCastAudioOutputConfig* cfg) {
    if (_pins.spk_pin_data < 0) return false;

    i2s_config_t i2s_cfg = {};
    i2s_cfg.mode = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_TX);
    i2s_cfg.sample_rate = cfg->sample_rate;
    i2s_cfg.bits_per_sample = I2S_BITS_PER_SAMPLE_16BIT;
    i2s_cfg.channel_format = I2S_CHANNEL_FMT_ONLY_LEFT;
    i2s_cfg.communication_format = I2S_COMM_FORMAT_STAND_I2S;
    i2s_cfg.intr_alloc_flags = ESP_INTR_FLAG_LEVEL1;
    i2s_cfg.dma_buf_count = cfg->dma_buf_count;
    i2s_cfg.dma_buf_len = cfg->dma_buf_len;
    i2s_cfg.use_apll = false;

    i2s_port_t port = spk_port();
    esp_err_t err = i2s_driver_install(port, &i2s_cfg, 0, NULL);
    if (err != ESP_OK) {
        Serial.printf("[GenericESP32S3] I2S spk driver install failed: %d\n", err);
        return false;
    }

    i2s_pin_config_t pin_cfg = {};
    pin_cfg.bck_io_num = _pins.spk_pin_clock;
    pin_cfg.ws_io_num = _pins.spk_pin_ws;
    pin_cfg.data_out_num = _pins.spk_pin_data;
    pin_cfg.data_in_num = I2S_PIN_NO_CHANGE;

    err = i2s_set_pin(port, &pin_cfg);
    if (err != ESP_OK) {
        Serial.printf("[GenericESP32S3] I2S spk set_pin failed: %d\n", err);
        return false;
    }

    Serial.println("[GenericESP32S3] Speaker I2S initialized");
    return true;
}

static bool spk_begin() {
    i2s_start(spk_port());
    return true;
}

static void spk_end() {
    i2s_stop(spk_port());
}

static bool spk_play_raw(const int16_t* data, size_t samples, uint32_t sample_rate,
                          bool stereo, int repeat, int channel, bool stop_current) {
    // Apply software volume scaling
    static int16_t scaled_buf[320];
    size_t copy_samples = (samples > 320) ? 320 : samples;
    float vol_scale = (float)_spk_volume / 255.0f;
    for (size_t i = 0; i < copy_samples; i++) {
        scaled_buf[i] = (int16_t)((float)data[i] * vol_scale);
    }

    size_t bytes_written = 0;
    i2s_write(spk_port(), scaled_buf, copy_samples * sizeof(int16_t),
              &bytes_written, portMAX_DELAY);
    return true;
}

static bool spk_is_playing() {
    // Blocking write means playback completes in playRaw
    return false;
}

static void spk_stop() {
    i2s_zero_dma_buffer(spk_port());
}

static void spk_set_volume(uint8_t vol) {
    _spk_volume = vol;
}

// --- LED callbacks ---

static int _led_pin = -1;

static void led_init_fn() {
    if (_led_pin >= 0) {
        pinMode(_led_pin, OUTPUT);
    }
}

static void led_set_color(uint8_t r, uint8_t g, uint8_t b) {
    if (_led_pin >= 0) {
        neopixelWrite(_led_pin, r, g, b);
    }
}

static void led_off() {
    if (_led_pin >= 0) {
        neopixelWrite(_led_pin, 0, 0, 0);
    }
}

// --- Button callbacks ---

static int _btn_pin = -1;
static bool _btn_last_state = true;  // INPUT_PULLUP: idle = HIGH
static unsigned long _btn_press_start = 0;
static bool _btn_click_pending = false;

static void btn_init_fn() {
    if (_btn_pin >= 0) {
        pinMode(_btn_pin, INPUT_PULLUP);
    }
}

static bool btn_was_clicked() {
    if (_btn_pin < 0) return false;

    bool current = digitalRead(_btn_pin);
    bool clicked = false;

    // Detect rising edge (release after press)
    if (current == HIGH && _btn_last_state == LOW) {
        // Only count as click if held < 1 second (not a long press)
        if (millis() - _btn_press_start < 1000) {
            clicked = true;
        }
    }
    if (current == LOW && _btn_last_state == HIGH) {
        _btn_press_start = millis();
    }

    _btn_last_state = current;
    return clicked;
}

static bool btn_pressed_for(uint32_t ms) {
    if (_btn_pin < 0) return false;
    bool current = digitalRead(_btn_pin);
    if (current == LOW && _btn_press_start > 0) {
        return (millis() - _btn_press_start >= ms);
    }
    return false;
}

// --- Static callback struct instances ---

static AudioInputCallbacks _mic_cbs = {
    mic_init, mic_begin, mic_end, mic_record,
    mic_is_recording_done, mic_is_full_duplex
};

static AudioOutputCallbacks _spk_cbs = {
    spk_init, spk_begin, spk_end, spk_play_raw,
    spk_is_playing, spk_stop, spk_set_volume
};

static StatusIndicatorCallbacks _led_cbs = { led_init_fn, led_set_color, led_off };
static ButtonCallbacks _btn_cbs = { btn_init_fn, btn_was_clicked, btn_pressed_for };

// --- Board hooks ---

static void board_init() {
    Serial.begin(115200);
    delay(100);
    Serial.println("[GenericESP32S3] Board initialized");
}

// --- Public API ---

RoamCastConfig GenericESP32S3::config(GenericESP32S3Pins pins) {
    _pins = pins;
    _led_pin = pins.led_pin;
    _btn_pin = pins.button_pin;

    RoamCastConfig cfg = roamcast_default_config();
    cfg.hardware_model = "generic_esp32s3";
    cfg.device_id_prefix = "roamcast";

    cfg.audio_input = &_mic_cbs;
    cfg.audio_output = (pins.spk_pin_data >= 0) ? &_spk_cbs : nullptr;
    cfg.status_indicator = (pins.led_pin >= 0) ? &_led_cbs : nullptr;
    cfg.button = (pins.button_pin >= 0) ? &_btn_cbs : nullptr;

    cfg.board_init = board_init;
    cfg.board_loop = nullptr;

    // Generic boards don't have M5Stack-specific hardware
    cfg.features.modules_enabled = false;
    cfg.features.presence_enabled = false;
    cfg.features.mdns_enabled = true;

    return cfg;
}
