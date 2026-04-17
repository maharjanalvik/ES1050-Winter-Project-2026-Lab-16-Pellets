#include "project.h"

static void hw_pwm_setup(int pin) {
    gpio_set_function(pin, GPIO_FUNC_PWM);
    uint slice = pwm_gpio_to_slice_num(pin);
    pwm_set_wrap(slice, 255);
    pwm_set_enabled(slice, true);
}

static void hw_pwm_write(int pin, int value) {
    uint slice   = pwm_gpio_to_slice_num(pin);
    uint channel = pwm_gpio_to_channel(pin);
    pwm_set_chan_level(slice, channel, value);
}


static byte mks_checksum(byte* buf, int len) {
    byte sum = 0;
    for (int i = 0; i < len; i++) sum += buf[i];
    return sum & 0xFF;
}

static void mks_flush(unsigned long quietMs = 0) {
    if (quietMs == 0) {
        while (Serial1.available()) Serial1.read();
    } else {
        unsigned long last = millis();
        while (millis() - last < quietMs) {
            if (Serial1.available()) { Serial1.read(); last = millis(); }
        }
    }
}

static int mks_readBytes(byte* buf, int count, unsigned long timeoutMs = 30) {
    unsigned long start = millis();
    int idx = 0;
    while (idx < count && millis() - start < timeoutMs) {
        if (Serial1.available()) buf[idx++] = Serial1.read();
    }
    return idx;
}

static void mks_sendControl(byte* cmd, int len, int ackBytes, unsigned long timeoutMs) {
    mks_flush();
    Serial1.write(cmd, len);
    byte ack[4];
    mks_readBytes(ack, ackBytes, timeoutMs);
}

static int mks_sendRead(byte* cmd, int cmdLen, byte* resp, int respBytes, unsigned long timeoutMs) {
    mks_flush();
    Serial1.write(cmd, cmdLen);
    return mks_readBytes(resp, respBytes, timeoutMs);
}

static void mks_sendControl_blocking(byte* cmd, int len, int ackBytes = 2) {
    mks_flush(80);
    Serial1.write(cmd, len);
    byte ack[4];
    mks_readBytes(ack, ackBytes, 300);
    mks_flush(80);
}

static int mks_sendRead_blocking(byte* cmd, int cmdLen, byte* resp, int respBytes) {
    mks_flush(80);
    Serial1.write(cmd, cmdLen);
    int n = mks_readBytes(resp, respBytes, 300);
    mks_flush(80);
    return n;
}


static void stepper_setEnable(bool en, bool blocking = false) {
    byte cmd[] = { STEPPER_ADDR, 0xF3, (byte)(en ? 0x01 : 0x00), 0x00 };
    cmd[3] = mks_checksum(cmd, 3);
    if (blocking) mks_sendControl_blocking(cmd, 4, 2);
    else          mks_sendControl(cmd, 4, 2, STEPPER_CMD_TIMEOUT_MS);
}

static void stepper_setAcceleration(uint16_t acc) {
    byte cmd[] = { STEPPER_ADDR, 0xA4, (byte)(acc >> 8), (byte)(acc & 0xFF), 0x00 };
    cmd[4] = mks_checksum(cmd, 4);
    mks_sendControl_blocking(cmd, 5, 2);
}

static void stepper_setMicrosteps(byte mstep) {
    byte cmd[] = { STEPPER_ADDR, 0x84, mstep, 0x00 };
    cmd[3] = mks_checksum(cmd, 3);
    mks_sendControl_blocking(cmd, 4, 2);
}

static void stepper_setProtection(bool en) {
    byte cmd[] = { STEPPER_ADDR, 0x88, (byte)(en ? 0x01 : 0x00), 0x00 };
    cmd[3] = mks_checksum(cmd, 3);
    mks_sendControl_blocking(cmd, 4, 2);
}

static void stepper_clearProtection() {
    byte cmd[] = { STEPPER_ADDR, 0x3D, 0x00 };
    cmd[2] = mks_checksum(cmd, 2);
    mks_sendControl(cmd, 3, 2, STEPPER_CMD_TIMEOUT_MS);
}

static void stepper_sendStop(bool blocking = false) {
    byte cmd[] = { STEPPER_ADDR, 0xF7, 0x00 };
    cmd[2] = mks_checksum(cmd, 2);
    if (blocking) mks_sendControl_blocking(cmd, 3, 2);
    else          mks_sendControl(cmd, 3, 2, STEPPER_CMD_TIMEOUT_MS);
}

static void stepper_sendFD(byte dir, byte speed, uint32_t pulses, bool blocking = false) {
    byte val = (speed & 0x7F) | ((dir & 0x01) << 7);
    byte cmd[8];
    cmd[0] = STEPPER_ADDR;
    cmd[1] = 0xFD;
    cmd[2] = val;
    cmd[3] = (pulses >> 24) & 0xFF;
    cmd[4] = (pulses >> 16) & 0xFF;
    cmd[5] = (pulses >>  8) & 0xFF;
    cmd[6] =  pulses        & 0xFF;
    cmd[7] = mks_checksum(cmd, 7);
    if (blocking) {
        mks_flush(80);
        Serial1.write(cmd, 8);
        byte ack[2];
        mks_readBytes(ack, 2, 300);
        mks_flush(80);
    } else {
        mks_flush();
        Serial1.write(cmd, 8);
        byte ack[2];
        mks_readBytes(ack, 2, STEPPER_CMD_TIMEOUT_MS);
    }
}

static bool stepper_pollStallStatus() {
    byte cmd[] = { STEPPER_ADDR, 0x3E, 0x00 };
    cmd[2] = mks_checksum(cmd, 2);
    byte resp[2];
    int n = mks_sendRead(cmd, 3, resp, 2, STEPPER_CMD_TIMEOUT_MS);
    if (n >= 2 && resp[0] == STEPPER_ADDR)
        return (resp[1] == 0x01);
    return false;
}


#define STEPPER_MIN_SPEED_VAL   5

static uint8_t mmsToSpeedVal(float mmps) {
    float ratio = fabsf(mmps) / STEPPER_MAX_SPEED_MMS;
    if (ratio > 1.0f) ratio = 1.0f;
    uint8_t val = (uint8_t)(ratio * STEPPER_MAX_SPEED_VAL);
    if (val < STEPPER_MIN_SPEED_VAL && mmps > 0.1f) val = STEPPER_MIN_SPEED_VAL;
    return val;
}


static uint32_t mmToPulses(float mm) {
    float revs = fabsf(mm) / PULLER_CIRCUMF_MM;
    return (uint32_t)(revs * STEPPER_PULSES_PER_REV);
}


static float    mc_feedSpeedMmS = 0;
static bool     mc_feedFwd      = true;
static bool     mc_stepperRunning = false;
static byte     mc_stepperDir   = 0;
static byte     mc_stepperSpeed = 0;
static float    mc_filPosMm     = 0;
static float    mc_cutPhaseMs   = 0;

static uint8_t  mc_cutTgtPWM    = 0;
static uint8_t  mc_cutCurPWM    = 0;
static bool     mc_cutRunning   = false;
static uint32_t mc_cutStartMs   = 0;

static bool     mc_reversing       = false;
static float    mc_revDistMm       = 0;
static uint32_t mc_reverseStartMs  = 0;
static uint32_t mc_reverseTimeMs   = 0;

static bool     mc_stallFlag       = false;

static bool     mc_pulsedMode      = false;
static const float MC_PULSE_DIST_MM = 5.0f;
static const uint32_t MC_PULSE_WAIT_MS = 5000;
static bool     mc_pulseFeeding    = false;
static uint32_t mc_pulseStartMs    = 0;
static uint32_t mc_pulseFeedTimeMs = 0;

static uint32_t mc_lastFastMs      = 0;
static uint32_t mc_lastReissueMs   = 0;
static uint32_t mc_lastPollMs      = 0;


static void cutter_dir(bool fwd) {
    digitalWrite(PIN_CUTTER_IN_A, fwd ? HIGH : LOW);
    digitalWrite(PIN_CUTTER_IN_B, fwd ? LOW  : HIGH);
}

static void cutter_off() {
    hw_pwm_write(PIN_CUTTER_ENA, 0);
    digitalWrite(PIN_CUTTER_IN_A, LOW);
    digitalWrite(PIN_CUTTER_IN_B, LOW);
    mc_cutRunning = false;
    mc_cutCurPWM  = 0;
    mc_cutTgtPWM  = 0;
}

static uint8_t calc_cutter_pwm() {
    if (settings.pelletLenMm <= 0) return 0;
    float cutFreq = settings.feedSpeedMmS / settings.pelletLenMm;
    float tgtRPM  = cutFreq * 60.0f / CUTTER_NUM_BLADES;
    float ratio   = tgtRPM / CUTTER_MAX_RPM;
    if (ratio > 1.0f) ratio = 1.0f;
    return (uint8_t)(CUTTER_MIN_PWM + ratio * (CUTTER_MAX_PWM - CUTTER_MIN_PWM));
}

static void stepper_startFD(float speedMmS, bool forward, uint32_t pulses) {
    mc_feedSpeedMmS  = speedMmS;
    mc_feedFwd       = forward;
    mc_stepperDir    = forward ? 1 : 0;
    mc_stepperSpeed  = mmsToSpeedVal(speedMmS);
    mc_stepperRunning = true;

    if (mc_stallFlag) stepper_clearProtection();
    mc_stallFlag = false;

    stepper_setEnable(true);
    stepper_sendFD(mc_stepperDir, mc_stepperSpeed, pulses);
    mc_lastReissueMs = millis();
}

static void stepper_fullStop() {
    stepper_sendStop();
    mc_stepperRunning = false;
    mc_feedSpeedMmS   = 0;
    stepper_setEnable(false);
}


void motor_control_init() {
    Serial1.setTX(PIN_STEPPER_TX);
    Serial1.setRX(PIN_STEPPER_RX);
    Serial1.begin(STEPPER_UART_BAUD);

    pinMode(PIN_CUTTER_IN_A, OUTPUT);
    pinMode(PIN_CUTTER_IN_B, OUTPUT);
    hw_pwm_setup(PIN_CUTTER_ENA);
    cutter_off();

    delay(500);
    stepper_setEnable(true, true);
    stepper_setAcceleration(STEPPER_ACC);
    stepper_setMicrosteps(STEPPER_MSTEP);
    stepper_setProtection(true);
    stepper_setEnable(false, true);
}

static void startPulseFeed() {
    uint32_t pulses = mmToPulses(MC_PULSE_DIST_MM);
    stepper_startFD(settings.feedSpeedMmS, true, pulses);
    mc_pulseFeeding    = true;
    mc_pulseStartMs    = millis();
    mc_pulseFeedTimeMs = (uint32_t)(MC_PULSE_DIST_MM / settings.feedSpeedMmS * 1000.0f * 2.5f) + 500;
}

void motor_start() {
    mc_reversing = false;
    digitalWrite(PIN_MOSFET, HIGH);
    delay(100);

    if (mc_pulsedMode) {
        stepper_setEnable(true);
        startPulseFeed();
    } else {
        stepper_startFD(settings.feedSpeedMmS, true, STEPPER_BIG_PULSE_COUNT);
    }

    mc_cutTgtPWM  = calc_cutter_pwm();
    mc_cutCurPWM  = 0;
    mc_cutRunning = true;
    mc_cutStartMs = millis();
    cutter_dir(true);
}

void motor_stop() {
    stepper_fullStop();
    cutter_off();
    mc_reversing = false;
    digitalWrite(PIN_MOSFET, LOW);
}

void motor_pause() {
    stepper_sendStop();
    mc_stepperRunning = false;
    mc_feedSpeedMmS   = 0;
    stepper_setEnable(false);
    cutter_off();
}

void motor_resume(const MotorSnapshot& snap) {
    mc_filPosMm     = snap.filamentPosMm;
    mc_cutPhaseMs   = snap.cutterPhaseMs;
    mc_reversing    = false;
    mc_stallFlag    = false;

    digitalWrite(PIN_MOSFET, HIGH);
    delay(100);

    stepper_startFD(snap.feedSpeedMmS, true, STEPPER_BIG_PULSE_COUNT);

    mc_cutTgtPWM  = calc_cutter_pwm();
    mc_cutCurPWM  = 0;
    mc_cutRunning = true;
    mc_cutStartMs = millis();
    cutter_dir(true);
}

void motor_set_insert_mode(bool en) {
    if (en) {
        mc_reversing = false;
        mc_stallFlag = false;
        digitalWrite(PIN_MOSFET, HIGH);
        delay(100);
        stepper_startFD(INSERT_FEED_SPEED_MMS, true, STEPPER_BIG_PULSE_COUNT);
        cutter_off();
    } else {
        stepper_fullStop();
        cutter_off();
        digitalWrite(PIN_MOSFET, LOW);
    }
}

void motor_reverse(float mm) {
    mc_reversing      = true;
    mc_revDistMm      = mm;
    mc_reverseStartMs = millis();
    mc_reverseTimeMs  = (uint32_t)(mm / JAM_REVERSE_SPEED_MMS * 1000.0f * 2.5f) + 500;

    cutter_off();

    uint32_t pulses = mmToPulses(mm);
    stepper_startFD(JAM_REVERSE_SPEED_MMS, false, pulses);
}

bool motor_reverse_complete() {
    if (!mc_reversing) return true;
    return (millis() - mc_reverseStartMs) >= mc_reverseTimeMs;
}

static bool     mc_cutRevActive   = false;
static uint32_t mc_cutRevStartMs  = 0;
static const uint32_t MC_CUT_REV_RAMP_MS = 2000;

void motor_cutter_reverse(bool en) {
    if (en) {
        digitalWrite(PIN_MOSFET, HIGH);
        delay(100);
        cutter_dir(false);
        hw_pwm_write(PIN_CUTTER_ENA, 0);
        mc_cutRevActive  = true;
        mc_cutRevStartMs = millis();
    } else {
        mc_cutRevActive = false;
        hw_pwm_write(PIN_CUTTER_ENA, 0);
        digitalWrite(PIN_CUTTER_IN_A, LOW);
        digitalWrite(PIN_CUTTER_IN_B, LOW);
        digitalWrite(PIN_MOSFET, LOW);
    }
}

bool  motor_is_stalled()          { return mc_stallFlag; }
float motor_get_filament_pos_mm() { return mc_filPosMm; }
float motor_get_feed_rate()       { return mc_feedSpeedMmS; }

float motor_get_cut_frequency() {
    if (settings.pelletLenMm <= 0) return 0;
    return mc_feedSpeedMmS / settings.pelletLenMm;
}

MotorSnapshot motor_get_snapshot() {
    return { mc_filPosMm, mc_cutPhaseMs, mc_feedSpeedMmS };
}

void motor_control_update() {
    uint32_t now = millis();

    if (now - mc_lastFastMs >= MOTOR_INTERVAL_MS) {
        float dt = (float)(now - mc_lastFastMs) / 1000.0f;
        mc_lastFastMs = now;

        if (mc_stepperRunning && mc_feedSpeedMmS > 0) {
            float dMm = mc_feedSpeedMmS * dt;
            if (mc_feedFwd) mc_filPosMm += dMm;
            else            mc_filPosMm -= dMm;
        }

        if (mc_cutRunning && mc_cutCurPWM < mc_cutTgtPWM) {
            uint32_t el = now - mc_cutStartMs;
            mc_cutCurPWM = (el < CUTTER_SOFT_START_MS)
                ? (uint8_t)((float)mc_cutTgtPWM * el / CUTTER_SOFT_START_MS)
                : mc_cutTgtPWM;
            hw_pwm_write(PIN_CUTTER_ENA, mc_cutCurPWM);
        }

        if (mc_cutRevActive) {
            uint32_t el = now - mc_cutRevStartMs;
            uint8_t pwm = (el < MC_CUT_REV_RAMP_MS)
                ? (uint8_t)((float)CUTTER_MAX_PWM * el / MC_CUT_REV_RAMP_MS)
                : CUTTER_MAX_PWM;
            hw_pwm_write(PIN_CUTTER_ENA, pwm);
        }

        if (mc_cutRunning && mc_feedSpeedMmS > 0 && settings.pelletLenMm > 0) {
            float periodMs = 1000.0f * settings.pelletLenMm / mc_feedSpeedMmS;
            mc_cutPhaseMs += dt * 1000.0f;
            while (mc_cutPhaseMs >= periodMs) mc_cutPhaseMs -= periodMs;
        }

        if (mc_reversing && motor_reverse_complete()) {
            mc_reversing = false;
            mc_stepperRunning = false;
            mc_feedSpeedMmS = 0;
            mc_feedFwd = true;
            stepper_sendStop();
            stepper_setEnable(false);
            mc_filPosMm -= mc_revDistMm;
            motorSnapshot.filamentPosMm = mc_filPosMm;
            eventQueue.push(Event::JAM_CLEARED);
        }

        if (mc_pulsedMode && !mc_reversing &&
            systemState == SystemState::RUNNING) {
            if (mc_pulseFeeding && (now - mc_pulseStartMs >= mc_pulseFeedTimeMs)) {
                mc_pulseFeeding = false;
                mc_pulseStartMs = now;
            } else if (!mc_pulseFeeding && (now - mc_pulseStartMs >= MC_PULSE_WAIT_MS)) {
                startPulseFeed();
            }
        }
    }

    if (mc_stepperRunning && !mc_reversing && !mc_pulsedMode &&
        (now - mc_lastReissueMs >= STEPPER_REISSUE_MS)) {
        mc_lastReissueMs = now;
        mks_flush();
        stepper_sendFD(mc_stepperDir, mc_stepperSpeed, STEPPER_BIG_PULSE_COUNT);
    }

    if (now - mc_lastPollMs >= STEPPER_POLL_MS) {
        mc_lastPollMs = now;

        if (mc_stepperRunning && !mc_reversing) {
            bool stall = stepper_pollStallStatus();
            if (stall && !mc_stallFlag) {
                mc_stallFlag = true;
                if (systemState == SystemState::RUNNING)
                    eventQueue.push(Event::JAM_DETECTED);
            } else if (!stall) {
                mc_stallFlag = false;
            }
        }
    }
}
