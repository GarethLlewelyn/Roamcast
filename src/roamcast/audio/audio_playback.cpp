#include "audio_playback.h"
#include "audio_capture.h"
#include "../led/led_controller.h"
#include "../RoamCastInternal.h"
#include "../RoamCastLog.h"

#include <WiFiUdp.h>
#include <math.h>

// Audio constants (must match audio_capture)
#define AUDIO_SAMPLE_RATE   16000
#define AUDIO_FRAME_SAMPLES 320
#define AUDIO_FRAME_BYTES   640

enum PlaybackState {
    PB_IDLE,
    PB_STARTING_SPEAKER,
    PB_PLAYING,
    PB_STOPPING_SPEAKER,
    PB_RESTARTING_MIC,
    PB_TTS_STARTING,
    PB_TTS_ACTIVE,
    PB_TTS_STOPPING
};

static PlaybackState pb_state = PB_IDLE;
static uint8_t speaker_volume = 128;
static uint16_t tone_freq = 1000;
static uint16_t tone_duration_ms = 200;
static unsigned long play_start_ms = 0;
static bool was_streaming = false;
static RcLedState prev_led_state = RC_LED_BLUE_SOLID;

// Tone buffer — max 1 second at 16kHz = 16000 samples = 32KB
#define TONE_SAMPLE_RATE 16000
#define MAX_TONE_SAMPLES TONE_SAMPLE_RATE
static int16_t tone_buffer[MAX_TONE_SAMPLES];

// TTS UDP receiver
static WiFiUDP tts_udp;
static bool tts_udp_started = false;
static uint16_t _tts_udp_port = 5101;

#define RX_RING_FRAMES 16
#define TTS_FRAME_SAMPLES AUDIO_FRAME_SAMPLES  // 320
static int16_t rx_ring_buffer[RX_RING_FRAMES * TTS_FRAME_SAMPLES];
static volatile int rx_write_idx = 0;
static volatile int rx_read_idx = 0;

// playRaw: DMA reads from pointer (no copy here).
#define PB_NUM_BUFFERS 3
static int16_t pb_buffers[PB_NUM_BUFFERS][TTS_FRAME_SAMPLES];
static int pb_buf_idx = 0;

#define STREAM_CHANNEL 0
#define MAX_QUEUE_PER_LOOP 3

static unsigned long tts_last_packet_ms = 0;
#define TTS_TIMEOUT_MS 3000
#define MUSIC_TIMEOUT_MS 30000

static bool music_mode = false;

static inline int rx_available() {
    int w = rx_write_idx;
    int r = rx_read_idx;
    return (w - r + RX_RING_FRAMES) % RX_RING_FRAMES;
}

static void restart_mic_and_streaming() {
    auto* input = roamcast::internal::getAudioInput();
    if (!input) return;

    if (!roamcast::internal::isFullDuplex()) {
        const auto* cfg = roamcast::internal::getConfig();
        RoamCastAudioInputConfig input_cfg = {
            AUDIO_SAMPLE_RATE, 16, 8, 2, 256
        };
        input->init(&input_cfg);
        input->begin();
    }

    if (was_streaming) {
        rc_audio_capture_start();
        rc_led_set(RC_LED_BLUE_PULSE);
    } else {
        rc_led_set(prev_led_state);
    }
}

void rc_audio_playback_init() {
    pb_state = PB_IDLE;
    const auto* cfg = roamcast::internal::getConfig();
    if (cfg) {
        _tts_udp_port = cfg->tts_udp_port;
    }
    auto* output = roamcast::internal::getAudioOutput();
    RC_DBG("Playback: init (speaker=%d, tts_port=%d)", roamcast::internal::hasSpeaker(), _tts_udp_port);
}

void rc_audio_playback_set_volume(uint8_t volume) {
    speaker_volume = volume;
    RC_DBG("Speaker volume=%d", volume);
}

uint8_t rc_audio_playback_get_volume() {
    return speaker_volume;
}

bool rc_audio_playback_is_playing() {
    return pb_state != PB_IDLE;
}

void rc_audio_playback_play_tone(uint16_t freq_hz, uint16_t duration_ms_arg) {
    RC_DBG("Playback: play_tone freq=%d dur=%d state=%d", freq_hz, duration_ms_arg, pb_state);
    if (!roamcast::internal::hasSpeaker()) {
        RC_DBG("Playback: play_tone rejected — no speaker");
        return;
    }
    if (pb_state != PB_IDLE) {
        RC_DBG("Playback: play_tone rejected — busy (state=%d)", pb_state);
        return;
    }
    tone_freq = freq_hz;
    tone_duration_ms = min((uint16_t)1000, duration_ms_arg);
    pb_state = PB_STARTING_SPEAKER;
}

void rc_audio_playback_tts_start() {
    if (!roamcast::internal::hasSpeaker()) return;
    if (pb_state != PB_IDLE) return;
    pb_state = PB_TTS_STARTING;
    RC_LOG("TTS start requested");
}

bool rc_audio_playback_tts_active() {
    return pb_state == PB_TTS_ACTIVE || pb_state == PB_TTS_STARTING;
}

void rc_audio_playback_music_start() {
    if (!roamcast::internal::hasSpeaker()) return;
    if (pb_state != PB_IDLE) return;
    music_mode = true;
    pb_state = PB_TTS_STARTING;
    RC_LOG("Music start requested");
}

void rc_audio_playback_music_stop() {
    music_mode = false;
    if (pb_state == PB_TTS_ACTIVE || pb_state == PB_TTS_STARTING) {
        pb_state = PB_TTS_STOPPING;
        RC_LOG("Music stop requested");
    }
}

void rc_audio_playback_music_flush() {
    if (!roamcast::internal::hasSpeaker()) return;
    if (pb_state != PB_TTS_ACTIVE) return;

    auto* output = roamcast::internal::getAudioOutput();
    if (!output) return;

    output->stop();

    rx_write_idx = 0;
    rx_read_idx = 0;
    pb_buf_idx = 0;

    while (tts_udp.parsePacket() > 0) {
        tts_udp.flush();
    }

    output->begin();
    output->setVolume(speaker_volume);

    RC_DBG("Music flush: cleared buffers");
}

void rc_audio_playback_loop() {
    if (!roamcast::internal::hasSpeaker()) return;

    auto* input = roamcast::internal::getAudioInput();
    auto* output = roamcast::internal::getAudioOutput();
    if (!output) return;

    bool full_duplex = roamcast::internal::isFullDuplex();

    switch (pb_state) {
        case PB_IDLE:
            break;

        case PB_STARTING_SPEAKER: {
            RC_DBG("Playback: PB_STARTING_SPEAKER");
            was_streaming = rc_audio_capture_is_streaming();
            prev_led_state = rc_led_get_state();

            if (was_streaming) {
                rc_audio_capture_stop();
            }

            if (!full_duplex && input) {
                input->end();
            }

            rc_led_set(RC_LED_GREEN_FLASH);

            RoamCastAudioOutputConfig tone_out_cfg = { TONE_SAMPLE_RATE, 256, 8 };
            bool init_ok = output->init(&tone_out_cfg);
            bool begin_ok = output->begin();
            output->setVolume(speaker_volume);
            RC_DBG("Playback: speaker init=%d begin=%d vol=%d", init_ok, begin_ok, speaker_volume);

            uint32_t num_samples = ((uint32_t)TONE_SAMPLE_RATE * tone_duration_ms) / 1000;
            if (num_samples > MAX_TONE_SAMPLES) num_samples = MAX_TONE_SAMPLES;

            for (uint32_t i = 0; i < num_samples; i++) {
                float t = (float)i / TONE_SAMPLE_RATE;
                tone_buffer[i] = (int16_t)(sinf(2.0f * M_PI * tone_freq * t) * 16000);
            }

            bool play_ok = output->playRaw(tone_buffer, num_samples, TONE_SAMPLE_RATE, false, 1, 1, true);
            play_start_ms = millis();
            pb_state = PB_PLAYING;
            RC_DBG("Playback: playRaw=%d samples=%u", play_ok, num_samples);
            break;
        }

        case PB_PLAYING:
            if (!output->isPlaying() ||
                (millis() - play_start_ms > tone_duration_ms + 200)) {
                RC_DBG("Playback: tone done (elapsed=%lu)", millis() - play_start_ms);
                pb_state = PB_STOPPING_SPEAKER;
            }
            break;

        case PB_STOPPING_SPEAKER:
            output->stop();
            output->end();
            if (tts_udp_started) {
                tts_udp.stop();
                tts_udp_started = false;
            }
            if (full_duplex) {
                if (was_streaming) {
                    rc_audio_capture_start();
                    rc_led_set(RC_LED_BLUE_PULSE);
                } else {
                    rc_led_set(prev_led_state);
                }
                pb_state = PB_IDLE;
            } else {
                pb_state = PB_RESTARTING_MIC;
            }
            RC_DBG("Playback: speaker stopped");
            break;

        case PB_RESTARTING_MIC:
            restart_mic_and_streaming();
            pb_state = PB_IDLE;
            RC_DBG("Playback: mic restarted, complete");
            break;

        case PB_TTS_STARTING: {
            RC_DBG("Playback: TTS starting...");
            was_streaming = rc_audio_capture_is_streaming();
            prev_led_state = rc_led_get_state();

            if (was_streaming) {
                rc_audio_capture_stop();
            }

            if (!full_duplex && input) {
                input->end();
            }

            rc_led_set(RC_LED_GREEN_FLASH);

            RoamCastAudioOutputConfig out_cfg = { AUDIO_SAMPLE_RATE, 256, 8 };
            bool init_ok = output->init(&out_cfg);
            bool begin_ok = output->begin();
            output->setVolume(speaker_volume);
            RC_DBG("Playback: TTS speaker init=%d begin=%d vol=%d", init_ok, begin_ok, speaker_volume);

            tts_udp.begin(_tts_udp_port);
            tts_udp_started = true;

            rx_write_idx = 0;
            rx_read_idx = 0;
            pb_buf_idx = 0;
            tts_last_packet_ms = millis();

            pb_state = PB_TTS_ACTIVE;
            RC_DBG("Playback: TTS active on UDP %d", _tts_udp_port);
            break;
        }

        case PB_TTS_ACTIVE: {
            int packetSize = tts_udp.parsePacket();
            while (packetSize > 0) {
                int expected_bytes = TTS_FRAME_SAMPLES * 2;
                if (packetSize == expected_bytes) {
                    int next_write = (rx_write_idx + 1) % RX_RING_FRAMES;
                    if (next_write != rx_read_idx) {
                        int frame_offset = rx_write_idx * TTS_FRAME_SAMPLES;
                        int bytes_read = tts_udp.read(
                            (uint8_t*)&rx_ring_buffer[frame_offset],
                            expected_bytes);

                        if (bytes_read == expected_bytes) {
                            rx_write_idx = next_write;
                        } else if (bytes_read > 0) {
                            memset(
                                ((uint8_t*)&rx_ring_buffer[frame_offset]) + bytes_read,
                                0,
                                expected_bytes - bytes_read);
                            rx_write_idx = next_write;
                        }
                    } else {
                        tts_udp.flush();
                    }
                } else {
                    tts_udp.flush();
                }

                tts_last_packet_ms = millis();
                packetSize = tts_udp.parsePacket();
            }

            int queued = 0;
            while (rx_available() > 0 && queued < MAX_QUEUE_PER_LOOP) {
                int rx_offset = rx_read_idx * TTS_FRAME_SAMPLES;
                memcpy(pb_buffers[pb_buf_idx], &rx_ring_buffer[rx_offset],
                       TTS_FRAME_SAMPLES * sizeof(int16_t));

                output->playRaw(
                    pb_buffers[pb_buf_idx],
                    TTS_FRAME_SAMPLES,
                    AUDIO_SAMPLE_RATE,
                    false,              // stereo = false
                    1,                  // repeat = 1
                    STREAM_CHANNEL,     // channel = 0 (fixed)
                    false               // stop_current = false (queue)
                );

                rx_read_idx = (rx_read_idx + 1) % RX_RING_FRAMES;
                pb_buf_idx = (pb_buf_idx + 1) % PB_NUM_BUFFERS;
                queued++;
            }

            // Timeout check
            unsigned long timeout = music_mode ? MUSIC_TIMEOUT_MS : TTS_TIMEOUT_MS;
            if (millis() - tts_last_packet_ms > timeout) {
                RC_LOG("%s timeout (%lu ms)", music_mode ? "Music" : "TTS", timeout);
                pb_state = PB_TTS_STOPPING;
            }
            break;
        }

        case PB_TTS_STOPPING:
            if (!output->isPlaying()) {
                output->stop();
                output->end();
                if (tts_udp_started) {
                    tts_udp.stop();
                    tts_udp_started = false;
                }
                music_mode = false;
                if (full_duplex) {
                    if (was_streaming) {
                        rc_audio_capture_start();
                        rc_led_set(RC_LED_BLUE_PULSE);
                    } else {
                        rc_led_set(prev_led_state);
                    }
                    pb_state = PB_IDLE;
                } else {
                    pb_state = PB_RESTARTING_MIC;
                }
                RC_DBG("Playback: TTS stopped");
            }
            break;
    }
}
