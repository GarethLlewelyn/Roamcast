#pragma once

#include <Arduino.h>

// ---------------------------------------------------------------------------
// RoamCast logging macros
//
// RC_LOG(fmt, ...)  — Always printed. For essential info: WiFi, MQTT, mDNS,
//                     hub auth, commands, errors.
// RC_DBG(fmt, ...)  — Only printed when debug_level >= 1. For internal state,
//                     audio pipeline details, step-by-step init progress.
// ---------------------------------------------------------------------------

namespace roamcast {
namespace log {

// Set once during begin(), read by macros
extern uint8_t debug_level;

inline void setDebugLevel(uint8_t level) { debug_level = level; }
inline uint8_t getDebugLevel() { return debug_level; }

} // namespace log
} // namespace roamcast

// Always print — essential library output (prefixed so origin is clear)
#define RC_LOG(fmt, ...) Serial.printf("[RoamCast] " fmt "\n", ##__VA_ARGS__)

// Debug only — verbose internals, suppressed when debug_level == 0
#define RC_DBG(fmt, ...) do { \
    if (roamcast::log::debug_level >= 1) \
        Serial.printf("[RoamCast DBG] " fmt "\n", ##__VA_ARGS__); \
} while(0)
