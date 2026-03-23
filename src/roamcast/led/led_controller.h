#pragma once

#include <Arduino.h>

enum RcLedState {
    RC_LED_OFF,
    RC_LED_BLUE_SOLID,       // Connected, idle
    RC_LED_BLUE_PULSE,       // Connected, streaming audio
    RC_LED_GREEN_SOLID,      // Device selected
    RC_LED_GREEN_FLASH,      // TTS playing
    RC_LED_ORANGE_SOLID,     // Connecting
    RC_LED_RED_SOLID,        // Error
    RC_LED_WHITE_PULSE,      // Presence detected
    RC_LED_PURPLE_SOLID      // OTA update (future)
};

void rc_led_init();
void rc_led_set(RcLedState state);
void rc_led_loop();
RcLedState rc_led_get_state();
