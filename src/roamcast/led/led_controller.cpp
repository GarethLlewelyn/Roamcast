#include "led_controller.h"
#include "../RoamCastInternal.h"
#include <math.h>

static RcLedState current_state = RC_LED_OFF;

static void set_color(uint8_t r, uint8_t g, uint8_t b) {
    auto* indicator = roamcast::internal::getStatusIndicator();
    if (indicator && indicator->setColor) {
        indicator->setColor(r, g, b);
    }
}

void rc_led_init() {
    auto* indicator = roamcast::internal::getStatusIndicator();
    if (indicator && indicator->init) {
        indicator->init();
    }
    set_color(0, 0, 0);
    Serial.println("[RoamCast] LED controller initialized");
}

void rc_led_set(RcLedState state) {
    current_state = state;

    switch (state) {
        case RC_LED_OFF:
            set_color(0, 0, 0);
            break;
        case RC_LED_BLUE_SOLID:
            set_color(0, 0, 80);
            break;
        case RC_LED_GREEN_SOLID:
            set_color(0, 80, 0);
            break;
        case RC_LED_ORANGE_SOLID:
            set_color(80, 40, 0);
            break;
        case RC_LED_RED_SOLID:
            set_color(80, 0, 0);
            break;
        case RC_LED_PURPLE_SOLID:
            set_color(60, 0, 80);
            break;
        default:
            // Animated states handled in rc_led_loop()
            break;
    }
}

void rc_led_loop() {
    switch (current_state) {
        case RC_LED_BLUE_PULSE: {
            float phase = sinf((float)millis() / 1000.0f * M_PI);
            uint8_t brightness = (uint8_t)(20 + 60 * (phase * 0.5f + 0.5f));
            set_color(0, 0, brightness);
            break;
        }
        case RC_LED_GREEN_FLASH: {
            uint32_t t = millis() % 2000;
            if (t < 100 || (t > 200 && t < 300)) {
                set_color(0, 80, 0);
            } else {
                set_color(0, 0, 0);
            }
            break;
        }
        case RC_LED_WHITE_PULSE: {
            float phase = sinf((float)millis() / 1500.0f * M_PI);
            uint8_t brightness = (uint8_t)(10 + 50 * (phase * 0.5f + 0.5f));
            set_color(brightness, brightness, brightness);
            break;
        }
        default:
            break;
    }
}

RcLedState rc_led_get_state() {
    return current_state;
}
