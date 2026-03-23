#pragma once

#include <Arduino.h>

#define CSI_MAX_REPORTED_SUBCARRIERS 64

struct CsiMotionData {
    float motion_intensity;     // 0.0 - 1.0 normalized motion intensity
    float raw_variance;         // Unnormalized mean subcarrier variance
    uint32_t packet_count;      // CSI packets processed since last publish
    unsigned long timestamp_ms;

    // --- Enriched fields from wifi_csi_info_t ---
    int8_t rssi;                // Per-frame RSSI (dBm)
    int8_t noise_floor;         // Noise floor (dBm)
    int8_t snr;                 // Computed SNR = rssi - noise_floor
    uint8_t channel;            // Primary WiFi channel
    uint8_t secondary_channel;  // 0=none, 1=above, 2=below
    uint8_t bandwidth;          // 0=20MHz, 1=40MHz
    uint8_t sig_mode;           // 0=non-HT, 1=HT, 3=VHT
    uint8_t rate;               // PHY rate / MCS index
    uint8_t stbc;               // STBC indicator
    uint32_t hw_timestamp_us;   // Hardware timestamp (microseconds)
    char source_mac[18];        // Source MAC "AA:BB:CC:DD:EE:FF"

    // Per-subcarrier data (latest frame)
    float amplitudes[CSI_MAX_REPORTED_SUBCARRIERS];
    float phases[CSI_MAX_REPORTED_SUBCARRIERS];
    int subcarrier_count;       // Actual subcarrier count in latest frame
    float mean_amplitude;       // Mean amplitude across subcarriers
};

void csi_motion_init(uint16_t publish_interval_ms, uint32_t keepalive_ms,
                     float publish_threshold, float smoothing_alpha,
                     uint8_t history_depth, float motion_max_variance);
void csi_motion_loop();
bool csi_motion_available();
CsiMotionData csi_motion_get();
