#include "project.h"

static uint32_t    sm_lastTickMs       = 0;
static uint32_t    sm_pauseStartMs     = 0;
static uint32_t    sm_accumulatedRunMs = 0;
static uint32_t    sm_currentRunStart  = 0;
static uint32_t    sm_lastStatSecMs    = 0;
static uint8_t     sm_jamRetries       = 0;
static SystemState sm_prevState        = SystemState::IDLE;

static void sm_addLog(const char* msg) {
    eventLog.add(msg, millis() / 1000);
}

static void transitionTo(SystemState ns, bool resume = false) {
    sm_prevState = systemState;
    systemState  = ns;

    switch (ns) {
        case SystemState::IDLE:
            motor_stop();
            sm_accumulatedRunMs = 0;
            sm_addLog("Idle");
            break;
        case SystemState::RUNNING:
            if (resume) {
                motor_resume(motorSnapshot);
                sm_addLog("Resumed");
            } else {
                sm_accumulatedRunMs = 0;
                motor_start();
                sm_addLog("Started");
            }
            sm_currentRunStart = millis();
            buzzer_play(BuzzerPattern::START);
            break;
        case SystemState::PAUSED:
            motorSnapshot       = motor_get_snapshot();
            sm_accumulatedRunMs += millis() - sm_currentRunStart;
            motor_pause();
            sm_pauseStartMs = millis();
            buzzer_play(BuzzerPattern::PAUSE_BEEP);
            sm_addLog("Paused");
            break;
        case SystemState::INSERT_FILAMENT:
            if (sm_prevState == SystemState::RUNNING) {
                motorSnapshot       = motor_get_snapshot();
                sm_accumulatedRunMs += millis() - sm_currentRunStart;
            }
            motor_set_insert_mode(true);
            sm_addLog("Insert filament");
            break;
        case SystemState::ERROR:
            motor_stop();
            buzzer_play(BuzzerPattern::ERROR_ALARM);
            stats.errorCount++;
            storage_mark_stats_dirty();
            sm_addLog("ERROR");
            break;
        case SystemState::JAM_CLEARING:
            motorSnapshot       = motor_get_snapshot();
            sm_accumulatedRunMs += millis() - sm_currentRunStart;
            sm_jamRetries++;
            motor_reverse(settings.jamReverseMm);
            buzzer_play(BuzzerPattern::JAM_WARN);
            stats.jamCount++;
            storage_mark_stats_dirty();
            sm_addLog("Jam clearing");
            break;
        case SystemState::RUNOUT:
            if (sm_prevState == SystemState::RUNNING)
                sm_accumulatedRunMs += millis() - sm_currentRunStart;
            motor_stop();
            buzzer_play(BuzzerPattern::RUNOUT_BEEP);
            sm_addLog("Filament runout");
            break;
    }
}

void state_machine_init() {
    systemState  = SystemState::IDLE;
    sm_prevState = SystemState::IDLE;
    sm_jamRetries = 0;
}

uint32_t state_machine_get_run_time_ms() {
    if (systemState == SystemState::RUNNING)
        return sm_accumulatedRunMs + (millis() - sm_currentRunStart);
    return sm_accumulatedRunMs;
}

void state_machine_update() {
    uint32_t now = millis();
    if (now - sm_lastTickMs < STATE_INTERVAL_MS) return;
    sm_lastTickMs = now;

    if (systemState == SystemState::RUNNING && (now - sm_lastStatSecMs >= 1000)) {
        sm_lastStatSecMs = now;
        stats.totalRuntimeSec++;
        static float lastTrackedPosMm = 0;
        float curPosMm = motor_get_filament_pos_mm();
        if (curPosMm > lastTrackedPosMm) {
            stats.totalFilamentMm += (uint32_t)(curPosMm - lastTrackedPosMm);
            lastTrackedPosMm = curPosMm;
        }
        storage_mark_stats_dirty();
    }

    while (!eventQueue.isEmpty()) {
        Event e = eventQueue.pop();

        if (e == Event::ESTOP) {
            if (cutterReversing) { motor_cutter_reverse(false); cutterReversing = false; }
            sm_addLog("E-STOP");
            transitionTo(SystemState::ERROR);
            return;
        }
        if (e == Event::FILAMENT_RUNOUT &&
            (systemState == SystemState::RUNNING ||
             systemState == SystemState::INSERT_FILAMENT)) {
            transitionTo(SystemState::RUNOUT);
            return;
        }

        switch (systemState) {
            case SystemState::IDLE:
                if (e == Event::START_CMD)            { sm_jamRetries = 0; transitionTo(SystemState::RUNNING); }
                else if (e == Event::INSERT_FILAMENT_CMD) transitionTo(SystemState::INSERT_FILAMENT);
                else if (e == Event::REVERSE_CUTTER_CMD) {
                    motor_cutter_reverse(true);
                    sm_addLog("Cutter reverse");
                }
                else if (e == Event::STOP_REVERSE_CUTTER_CMD) {
                    motor_cutter_reverse(false);
                    sm_addLog("Cutter stopped");
                }
                break;
            case SystemState::RUNNING:
                if (e == Event::PAUSE_CMD)            transitionTo(SystemState::PAUSED);
                else if (e == Event::JAM_DETECTED) {
                    if (sm_jamRetries >= JAM_RETRY_MAX) { sm_addLog("Jam limit"); transitionTo(SystemState::ERROR); }
                    else                                transitionTo(SystemState::JAM_CLEARING);
                }
                break;
            case SystemState::PAUSED:
                if (e == Event::RESUME_CMD || e == Event::START_CMD) { sm_jamRetries = 0; transitionTo(SystemState::RUNNING, true); }
                else if (e == Event::INSERT_FILAMENT_CMD) transitionTo(SystemState::INSERT_FILAMENT);
                break;
            case SystemState::INSERT_FILAMENT:
                if (e == Event::EXIT_INSERT_CMD) {
                    motor_set_insert_mode(false);
                    if (sm_prevState == SystemState::RUNNING || sm_prevState == SystemState::PAUSED)
                        transitionTo(SystemState::PAUSED);
                    else transitionTo(SystemState::IDLE);
                }
                break;
            case SystemState::ERROR:
                if (e == Event::CLEAR_ERROR) transitionTo(SystemState::IDLE);
                break;
            case SystemState::JAM_CLEARING:
                if (e == Event::JAM_CLEARED)      transitionTo(SystemState::RUNNING, true);
                else if (e == Event::JAM_FAILED)  transitionTo(SystemState::ERROR);
                break;
            case SystemState::RUNOUT:
                if (e == Event::INSERT_FILAMENT_CMD) transitionTo(SystemState::INSERT_FILAMENT);
                else if (e == Event::CLEAR_ERROR)    transitionTo(SystemState::IDLE);
                break;
        }
    }

    if (systemState == SystemState::PAUSED && (now - sm_pauseStartMs) > IDLE_TIMEOUT_MS) {
        sm_addLog("Idle timeout");
        transitionTo(SystemState::IDLE);
    }
}
