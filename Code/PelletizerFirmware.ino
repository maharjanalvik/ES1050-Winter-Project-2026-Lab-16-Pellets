#include "config.h"
#include "ui.h"
#include "motor.h"
#include "input.h"
#include "sensors.h"
#include "state.h"
#include "effects.h"

void setup() {
    initUI();
    initMotor();
    initInput();
    initSensors();
    initEffects();
}

void loop() {
    readInput();
    updateSensors();
    updateState();
    updateMotor();
    updateUI();
    updateEffects();
}
