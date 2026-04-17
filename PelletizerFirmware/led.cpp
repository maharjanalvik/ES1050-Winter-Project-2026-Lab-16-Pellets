#include "project.h"

static Adafruit_NeoPixel px(1, PIN_RGB_LED, NEO_GRB + NEO_KHZ800);
static uint32_t led_lastMs = 0;

void led_init() {
    px.begin();
    px.setBrightness(60);
    px.setPixelColor(0, px.Color(20, 20, 20));
    px.show();
}

void led_update() {
    uint32_t now = millis();
    if (now - led_lastMs < LED_INTERVAL_MS) return;
    led_lastMs = now;

    uint32_t c;
    switch (systemState) {
        case SystemState::RUNNING:
            c = px.Color(0, 255, 0);
            break;
        case SystemState::PAUSED: {
            float phase = (float)(now % 2000) / 2000.0f;
            uint8_t b = (uint8_t)(127.5f + 127.5f * sinf(phase * 2.0f * PI));
            c = px.Color(b, b, 0);
            break;
        }
        case SystemState::INSERT_FILAMENT:
            c = px.Color(0, 0, 255);
            break;
        case SystemState::ERROR:
            c = ((now / 500) & 1) ? px.Color(255, 0, 0) : px.Color(0, 0, 0);
            break;
        case SystemState::JAM_CLEARING:
            c = px.Color(128, 0, 255);
            break;
        case SystemState::RUNOUT:
            c = ((now / 750) & 1) ? px.Color(255, 60, 0) : px.Color(0, 0, 0);
            break;
        default:
            c = px.Color(20, 20, 20);
            break;
    }
    px.setPixelColor(0, c);
    px.show();
}
