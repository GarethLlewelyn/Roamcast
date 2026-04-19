#pragma once

#include <Arduino.h>

// RC_LOG: always printed. RC_DBG: when debug_level >= 1.

namespace roamcast {
namespace log {

extern uint8_t debug_level;

inline void setDebugLevel(uint8_t level) { debug_level = level; }
inline uint8_t getDebugLevel() { return debug_level; }

} // namespace log
} // namespace roamcast

#define RC_LOG(fmt, ...) Serial.printf("[RoamCast] " fmt "\n", ##__VA_ARGS__)

#define RC_DBG(fmt, ...) do { \
    if (roamcast::log::debug_level >= 1) \
        Serial.printf("[RoamCast DBG] " fmt "\n", ##__VA_ARGS__); \
} while(0)
