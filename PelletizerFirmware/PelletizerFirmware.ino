#include "project.h"

void setup() {
    pinMode(PIN_MOSFET, OUTPUT);
    digitalWrite(PIN_MOSFET, LOW);

    storage_init();
    sensors_init();
    motor_control_init();
    buzzer_init();
    led_init();
    ui_init();
    state_machine_init();
}

void loop() {
    sensors_update();
    state_machine_update();
    motor_control_update();
    ui_update();
    buzzer_update();
    led_update();
    storage_update();
}
