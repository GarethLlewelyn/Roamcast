#include "audio_capture.h"
#include "audio_dsp.h"
#include "../RoamCastInternal.h"
#include "../RoamCastLog.h"
#include "../core/runtime_config.h"
#include "../core/mqtt_client.h"

#include <WiFiUdp.h>
#include <math.h>

// Audio constants
#define AUDIO_SAMPLE_RATE   16000
#define AUDIO_FRAME_MS      20
#define AUDIO_FRAME_SAMPLES (AUDIO_SAMPLE_RATE * AUDIO_FRAME_MS / 1000)  // 320
#define AUDIO_FRAME_BYTES   (AUDIO_FRAME_SAMPLES * 2)                    // 640
#define UDP_HEADER_SIZE     12
#define UDP_PACKET_SIZE     (UDP_HEADER_SIZE + AUDIO_FRAME_BYTES)        // 652
#define VAD_PREBUFFER_FRAMES 10

// Double-buffer for mic recording
static int16_t buffer_a[AUDIO_FRAME_SAMPLES];
static int16_t buffer_b[AUDIO_FRAME_SAMPLES];
static int16_t* fill_buffer = buffer_a;
static int16_t* send_buffer = buffer_b;

// State
static bool streaming = false;
static bool recording_active = false;
static uint32_t sequence_number = 0;
static uint32_t stream_start_ms = 0;
static unsigned long last_level_report_ms = 0;
static float current_rms = 0.0f;
static float current_peak = 0.0f;

// DSP state
static DcBlockerState dc_state = {0.0f, 0.0f};
static VoiceGateState gate_state = {0, false, false};
static float _dc_block_alpha = 0.995f;
static uint16_t _gate_threshold = 150;
static uint8_t _gate_hold_frames = 5;
static uint16_t _audio_level_report_ms = 500;

// Frame timing for stale-frame detection
static unsigned long record_started_at = 0;

// Diagnostic counters
static uint32_t _dma_underruns = 0;
static uint32_t _udp_send_failures = 0;

// Pre-buffer ring: stores DC-blocked frames so speech onsets aren't lost
static int16_t prebuf[VAD_PREBUFFER_FRAMES][AUDIO_FRAME_SAMPLES];
static uint8_t prebuf_write = 0;
static uint8_t prebuf_count = 0;

// UDP
static WiFiUDP udp;
static uint8_t udp_packet[UDP_PACKET_SIZE];
static uint32_t numeric_device_id = 0;
static const char* stored_device_id = nullptr;
static uint16_t _udp_audio_port = 5100;

static void send_udp_packet(const int16_t* pcm_data) {
    uint32_t timestamp = millis() - stream_start_ms;

    memcpy(udp_packet + 0, &numeric_device_id, 4);
    memcpy(udp_packet + 4, &sequence_number, 4);
    memcpy(udp_packet + 8, &timestamp, 4);
    sequence_number++;

    memcpy(udp_packet + UDP_HEADER_SIZE, pcm_data, AUDIO_FRAME_BYTES);

    udp.beginPacket(rc_get_server_ip(), _udp_audio_port);
    udp.write(udp_packet, UDP_PACKET_SIZE);
    if (!udp.endPacket()) {
        _udp_send_failures++;
    }
}

static void prebuf_push(const int16_t* frame) {
    memcpy(prebuf[prebuf_write], frame, AUDIO_FRAME_BYTES);
    prebuf_write = (prebuf_write + 1) % VAD_PREBUFFER_FRAMES;
    if (prebuf_count < VAD_PREBUFFER_FRAMES) prebuf_count++;
}

static void prebuf_flush() {
    if (prebuf_count == 0) return;
    uint8_t read_idx = (prebuf_write - prebuf_count + VAD_PREBUFFER_FRAMES) % VAD_PREBUFFER_FRAMES;
    for (uint8_t i = 0; i < prebuf_count; i++) {
        send_udp_packet(prebuf[read_idx]);
        read_idx = (read_idx + 1) % VAD_PREBUFFER_FRAMES;
    }
    prebuf_count = 0;
}

void rc_audio_capture_init(const char* device_id, uint16_t udp_audio_port,
                           float dc_block_alpha, uint16_t gate_threshold,
                           uint8_t gate_hold_frames, uint16_t audio_level_report_ms) {
    RC_DBG("AudioCapture: init()");
    stored_device_id = device_id;
    _udp_audio_port = udp_audio_port;
    _dc_block_alpha = dc_block_alpha;
    _gate_threshold = gate_threshold;
    _gate_hold_frames = gate_hold_frames;
    _audio_level_report_ms = audio_level_report_ms;

    auto* input = roamcast::internal::getAudioInput();
    if (!input) {
        RC_LOG("AudioCapture: ERROR — no audio_input callbacks!");
        return;
    }
    RC_DBG("AudioCapture: input=%p, init=%p begin=%p record=%p isDone=%p",
            input, input->init, input->begin, input->record, input->isRecordingDone);

    // Initialize audio input hardware via callbacks
    const auto* cfg = roamcast::internal::getConfig();
    RoamCastAudioInputConfig input_cfg = {
        AUDIO_SAMPLE_RATE, 16, 8, 2, 256
    };
    if (cfg) {
        // Let board preset override magnification etc via the config struct's audio input config
        // (these are set in the board preset's init callback)
    }
    bool init_ok = input->init(&input_cfg);
    RC_DBG("AudioCapture: input->init()=%d", init_ok);

    // Ensure mic is active
    bool begin_ok = input->begin();
    RC_DBG("AudioCapture: input->begin()=%d", begin_ok);

    // Initialize UDP socket
    udp.begin(0);
    RC_DBG("AudioCapture: UDP target=%s:%d",
            rc_get_server_ip() ? rc_get_server_ip() : "NULL", _udp_audio_port);

    // Derive numeric device_id from last 4 bytes of MAC
    uint8_t mac[6];
    esp_efuse_mac_get_default(mac);
    numeric_device_id = ((uint32_t)mac[2] << 24) | ((uint32_t)mac[3] << 16) |
                        ((uint32_t)mac[4] << 8) | (uint32_t)mac[5];

    RC_DBG("AudioCapture: Ready (id=0x%08X, rate=%d, frame=%d)",
            numeric_device_id, AUDIO_SAMPLE_RATE, AUDIO_FRAME_SAMPLES);
}

void rc_audio_capture_start() {
    RC_DBG("AudioCapture: start()");
    auto* input = roamcast::internal::getAudioInput();
    if (!input) {
        RC_LOG("AudioCapture: start() FAILED — no audio input!");
        return;
    }

    streaming = true;
    sequence_number = 0;
    stream_start_ms = millis();
    last_level_report_ms = millis();

    bool rec_ok = input->record(fill_buffer, AUDIO_FRAME_SAMPLES, AUDIO_SAMPLE_RATE);
    record_started_at = millis();
    recording_active = true;

    RC_DBG("AudioCapture: Streaming started (record=%d)", rec_ok);
}

void rc_audio_capture_stop() {
    streaming = false;
    recording_active = false;
    RC_DBG("AudioCapture: Streaming stopped");
}

// Diagnostic counters for periodic reporting
static uint32_t _loop_calls = 0;
static uint32_t _frames_sent = 0;
static uint32_t _underrun_total = 0;
static uint32_t _recording_done_count = 0;
static unsigned long _last_capture_diag_ms = 0;

void rc_audio_capture_loop() {
    if (!streaming) return;

    auto* input = roamcast::internal::getAudioInput();
    if (!input) return;

    _loop_calls++;

    // Periodic capture diagnostics every 10 seconds
    unsigned long diag_now = millis();
    if (diag_now - _last_capture_diag_ms >= 10000) {
        _last_capture_diag_ms = diag_now;
        bool is_done = input->isRecordingDone();
        RC_DBG("AudioCapture: loops=%u done=%u sent=%u underruns=%u isDone=%d active=%d seq=%u",
                _loop_calls, _recording_done_count, _frames_sent, _dma_underruns, is_done, recording_active, sequence_number);
        _loop_calls = 0;
        _recording_done_count = 0;
        _frames_sent = 0;
    }

    // Check if current recording is complete
    if (input->isRecordingDone() && recording_active) {
        _recording_done_count++;
        unsigned long now = millis();
        unsigned long elapsed = now - record_started_at;

        // Stale-frame detection: a 20ms frame that completes in < 5ms
        // was served from the DMA backlog, not captured in real time.
        if (record_started_at > 0 && elapsed < (AUDIO_FRAME_MS / 4)) {
            _dma_underruns++;
            if (fill_buffer == buffer_a) fill_buffer = buffer_b;
            else fill_buffer = buffer_a;
            input->record(fill_buffer, AUDIO_FRAME_SAMPLES, AUDIO_SAMPLE_RATE);
            record_started_at = millis();
            return;
        }

        // Swap buffers
        if (fill_buffer == buffer_a) {
            fill_buffer = buffer_b;
            send_buffer = buffer_a;
        } else {
            fill_buffer = buffer_a;
            send_buffer = buffer_b;
        }

        // Start next recording immediately to minimize gaps
        input->record(fill_buffer, AUDIO_FRAME_SAMPLES, AUDIO_SAMPLE_RATE);
        record_started_at = millis();

        // DSP chain: DC block → pre-buffer → VAD gate → send
        dc_block_inplace(send_buffer, AUDIO_FRAME_SAMPLES, _dc_block_alpha, dc_state);
        prebuf_push(send_buffer);
        voice_gate_update(send_buffer, AUDIO_FRAME_SAMPLES, _gate_threshold,
                         _gate_hold_frames, gate_state);

        if (gate_state.open) {
            if (!gate_state.was_open) {
                prebuf_flush();
            }
            calculate_levels(send_buffer, AUDIO_FRAME_SAMPLES, current_rms, current_peak);
            send_udp_packet(send_buffer);
            _frames_sent++;
        } else {
            static int16_t silence[AUDIO_FRAME_SAMPLES] = {0};
            current_rms = 0.0f;
            current_peak = 0.0f;
            send_udp_packet(silence);
            _frames_sent++;
        }

        // Report audio level via MQTT periodically
        if (now - last_level_report_ms >= _audio_level_report_ms) {
            last_level_report_ms = now;
            rc_mqtt_publish_audio_level(current_rms, current_peak, gate_state.open);
        }
    }
}

bool rc_audio_capture_is_streaming() { return streaming; }
float rc_audio_capture_get_rms() { return current_rms; }
float rc_audio_capture_get_peak() { return current_peak; }
uint32_t rc_audio_capture_get_dma_underruns() { return _dma_underruns; }
uint32_t rc_audio_capture_get_udp_send_failures() { return _udp_send_failures; }
