#include "project.h"

static volatile bool estopISRFlag = false;

static void estopISR() {
    digitalWrite(PIN_MOSFET, LOW);
    estopISRFlag = true;
}

struct Btn {
    uint8_t  pin;
    bool     raw;
    bool     stable;
    uint32_t lastEdgeMs;
    uint32_t pressStartMs;
    bool     held;
    bool     longFired;
};

static Btn btns[3];
static bool flagUp      = false;
static bool flagDown    = false;
static bool flagSelect  = false;
static bool flagSelLong = false;

static bool     filRawPrev   = true;
static uint32_t filEdgeMs    = 0;
static bool     filStable    = true;
static uint32_t sens_lastTickMs = 0;

void sensors_init() {
    pinMode(PIN_ESTOP,           INPUT_PULLUP);
    pinMode(PIN_BTN_UP,          INPUT_PULLUP);
    pinMode(PIN_BTN_DOWN,        INPUT_PULLUP);
    pinMode(PIN_BTN_SELECT,      INPUT_PULLUP);
    pinMode(PIN_FILAMENT_RUNOUT, INPUT_PULLUP);

    attachInterrupt(digitalPinToInterrupt(PIN_ESTOP), estopISR, FALLING);

    btns[0] = { PIN_BTN_UP,     true, true, 0, 0, false, false };
    btns[1] = { PIN_BTN_DOWN,   true, true, 0, 0, false, false };
    btns[2] = { PIN_BTN_SELECT, true, true, 0, 0, false, false };

    filStable  = (digitalRead(PIN_FILAMENT_RUNOUT) == LOW);
    filRawPrev = filStable;
}

void sensors_update() {
    uint32_t now = millis();
    if (now - sens_lastTickMs < SENSOR_INTERVAL_MS) return;
    sens_lastTickMs = now;

    if (estopISRFlag) {
        estopISRFlag = false;
        if (digitalRead(PIN_ESTOP) == LOW)
            eventQueue.push(Event::ESTOP);
    }

    for (uint8_t i = 0; i < 3; i++) {
        bool reading = digitalRead(btns[i].pin);
        if (reading != btns[i].raw) btns[i].lastEdgeMs = now;
        btns[i].raw = reading;
        if ((now - btns[i].lastEdgeMs) < DEBOUNCE_MS) continue;
        bool prev = btns[i].stable;
        btns[i].stable = reading;
        if (!btns[i].stable && prev) {
            btns[i].pressStartMs = now;
            btns[i].held = true;
            btns[i].longFired = false;
        }
        if (btns[i].stable && !prev && btns[i].held) {
            if (!btns[i].longFired) {
                if (i == 0) flagUp     = true;
                if (i == 1) flagDown   = true;
                if (i == 2) flagSelect = true;
            }
            btns[i].held = false;
        }
        if (i == 2 && btns[i].held && !btns[i].longFired &&
            (now - btns[i].pressStartMs) >= LONG_PRESS_MS) {
            flagSelLong       = true;
            btns[i].longFired = true;
        }
    }

    bool present = (digitalRead(PIN_FILAMENT_RUNOUT) == LOW);
    if (present != filRawPrev) filEdgeMs = now;
    filRawPrev = present;
    if ((now - filEdgeMs) >= DEBOUNCE_MS && present != filStable) {
        bool wasPresent = filStable;
        filStable = present;
        if (wasPresent && !filStable)
            eventQueue.push(Event::FILAMENT_RUNOUT);
    }
}

bool sensors_btn_up_pressed()     { if (flagUp)      { flagUp      = false; return true; } return false; }
bool sensors_btn_down_pressed()   { if (flagDown)    { flagDown    = false; return true; } return false; }
bool sensors_btn_select_pressed() { if (flagSelect)  { flagSelect  = false; return true; } return false; }
bool sensors_btn_select_long()    { if (flagSelLong) { flagSelLong = false; return true; } return false; }
bool sensors_estop_active()       { return (digitalRead(PIN_ESTOP) == LOW); }
bool sensors_filament_present()   { return filStable; }
