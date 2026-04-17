#include "project.h"

struct ToneStep { uint16_t freq; uint16_t ms; };

static const ToneStep sClick[]  = { {4000,30}, {0,0} };
static const ToneStep sStart[]  = { {800,100}, {1000,100}, {1200,150}, {0,0} };
static const ToneStep sPause[]  = { {1000,100}, {0,80}, {1000,100}, {0,0} };
static const ToneStep sError[]  = { {800,200}, {0,200}, {800,200}, {0,200}, {800,200}, {0,0} };
static const ToneStep sJam[]    = { {600,150}, {900,150}, {600,150}, {0,0} };
static const ToneStep sRunout[] = { {500,300}, {0,700}, {500,300}, {0,700}, {0,0} };

static const ToneStep* const bzPatterns[] = {
    nullptr, sClick, sStart, sPause, sError, sJam, sRunout
};
static constexpr uint8_t BZ_PATTERN_COUNT = sizeof(bzPatterns) / sizeof(bzPatterns[0]);

static const ToneStep* bzCurPat  = nullptr;
static uint8_t  bzStepIdx   = 0;
static uint32_t bzStepStart = 0;
static bool     bzActive    = false;

void buzzer_init() {
    pinMode(PIN_BUZZER, OUTPUT);
    noTone(PIN_BUZZER);
}

void buzzer_play(BuzzerPattern p) {
    if (!settings.buzzerEnabled && p != BuzzerPattern::ERROR_ALARM) return;
    uint8_t i = static_cast<uint8_t>(p);
    if (i >= BZ_PATTERN_COUNT || !bzPatterns[i]) return;
    bzCurPat   = bzPatterns[i];
    bzStepIdx  = 0;
    bzStepStart = millis();
    bzActive   = true;
    if (bzCurPat[0].freq) tone(PIN_BUZZER, bzCurPat[0].freq);
    else                  noTone(PIN_BUZZER);
}

void buzzer_stop() {
    noTone(PIN_BUZZER);
    bzActive = false;
    bzCurPat = nullptr;
}

void buzzer_update() {
    if (!bzActive || !bzCurPat) return;
    if (millis() - bzStepStart < bzCurPat[bzStepIdx].ms) return;
    bzStepIdx++;
    if (bzCurPat[bzStepIdx].freq == 0 && bzCurPat[bzStepIdx].ms == 0) {
        buzzer_stop();
        return;
    }
    bzStepStart = millis();
    if (bzCurPat[bzStepIdx].freq) tone(PIN_BUZZER, bzCurPat[bzStepIdx].freq);
    else                          noTone(PIN_BUZZER);
}
