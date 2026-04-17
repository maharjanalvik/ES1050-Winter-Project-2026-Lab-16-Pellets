#ifndef PROJECT_H
#define PROJECT_H

#include <Arduino.h>
#include <EEPROM.h>
#include <Wire.h>
#include <U8g2lib.h>
#include <Adafruit_NeoPixel.h>
#include "hardware/pwm.h"

#ifdef ERROR
#undef ERROR
#endif

#define FW_VERSION "1.0.0"

#define PIN_STEPPER_TX       0
#define PIN_STEPPER_RX       1
#define PIN_CUTTER_IN_A      2
#define PIN_CUTTER_IN_B      3
#define PIN_MOSFET           4
#define PIN_ESTOP            5
#define PIN_BTN_UP           6
#define PIN_BTN_DOWN         7
#define PIN_CUTTER_ENA       8
#define PIN_RGB_LED          9
#define PIN_BTN_SELECT      10
#define PIN_FILAMENT_RUNOUT 11
#define PIN_BUZZER          15
#define PIN_OLED_SDA        16
#define PIN_OLED_SCL        17

#define STEPPER_UART_BAUD       115200
#define STEPPER_ADDR            0xE0
#define STEPPER_MAX_SPEED_VAL   127
#define STEPPER_MAX_SPEED_MMS   250.0f
#define PULLER_DIAMETER_MM      12.7f
#define PULLER_CIRCUMF_MM       (PULLER_DIAMETER_MM * PI)
#define STEPPER_MSTEP           236
#define STEPPER_ACC             0x0050
#define STEPPER_PULSES_PER_REV  (STEPPER_MSTEP * 200UL)
#define STEPPER_BIG_PULSE_COUNT 4720000UL
#define STEPPER_REISSUE_MS      8000
#define STEPPER_POLL_MS         1500
#define STEPPER_CMD_TIMEOUT_MS  50

#define CUTTER_NUM_BLADES       1
#define CUTTER_SOFT_START_MS    500
#define CUTTER_MIN_PWM          60
#define CUTTER_MAX_PWM          255
#define CUTTER_MAX_RPM          3000.0f

#define DEFAULT_PELLET_LEN_MM    3.0f
#define MIN_PELLET_LEN_MM        1.0f
#define MAX_PELLET_LEN_MM       10.0f
#define PELLET_LEN_STEP_MM       0.5f

#define DEFAULT_FEED_SPEED_MMS   6.0f
#define MIN_FEED_SPEED_MMS       5.0f
#define MAX_FEED_SPEED_MMS     100.0f
#define FEED_SPEED_STEP_MMS      5.0f

#define DEFAULT_JAM_REVERSE_MM  10.0f
#define JAM_RETRY_MAX            3
#define JAM_STALL_TIMEOUT_MS     200
#define JAM_REVERSE_SPEED_MMS    10.0f
#define JAM_PAUSE_MS             500
#define INSERT_FEED_SPEED_MMS    5.0f

#define SENSOR_INTERVAL_MS          5
#define STATE_INTERVAL_MS          10
#define MOTOR_INTERVAL_MS           1
#define UI_INTERVAL_MS             50
#define LED_INTERVAL_MS            50
#define STORAGE_FLUSH_INTERVAL_MS  60000

#define DEBOUNCE_MS             50
#define LONG_PRESS_MS          800
#define SCREEN_TIMEOUT_MS    60000
#define OLED_DIM_CONTRAST        0
#define OLED_FULL_CONTRAST     255

#define IDLE_TIMEOUT_MS       300000

#define EEPROM_SIZE            256
#define SETTINGS_ADDR            0
#define STATS_ADDR              64

enum class SystemState : uint8_t {
    IDLE,
    RUNNING,
    PAUSED,
    INSERT_FILAMENT,
    ERROR,
    JAM_CLEARING,
    RUNOUT
};

enum class MenuScreen : uint8_t {
    HOME,
    MAIN_MENU,
    SETTINGS,
    SETTINGS_PELLET_LEN,
    SETTINGS_FEED_SPEED,
    WARNINGS,
    MAINTENANCE,
    SYSTEM_INFO
};

enum class Event : uint8_t {
    NONE,
    ESTOP,
    FILAMENT_RUNOUT,
    JAM_DETECTED,
    JAM_CLEARED,
    JAM_FAILED,
    START_CMD,
    PAUSE_CMD,
    RESUME_CMD,
    INSERT_FILAMENT_CMD,
    EXIT_INSERT_CMD,
    IDLE_TIMEOUT,
    CLEAR_ERROR,
    REVERSE_CUTTER_CMD,
    STOP_REVERSE_CUTTER_CMD
};

enum class BuzzerPattern : uint8_t {
    NONE, CLICK, START, PAUSE_BEEP, ERROR_ALARM, JAM_WARN, RUNOUT_BEEP
};

struct Settings {
    float    pelletLenMm;
    float    feedSpeedMmS;
    float    jamReverseMm;
    bool     buzzerEnabled;
    uint8_t  _pad[3];
    uint32_t screenTimeoutMs;
    uint8_t  crc;
};

struct Stats {
    uint32_t totalRuntimeSec;
    uint32_t totalFilamentMm;
    uint16_t jamCount;
    uint16_t errorCount;
    uint8_t  crc;
};

struct MotorSnapshot {
    float filamentPosMm;
    float cutterPhaseMs;
    float feedSpeedMmS;
};

#define EVENT_QUEUE_SIZE 16

class EventQueue {
public:
    void push(Event e) {
        if (count_ < EVENT_QUEUE_SIZE) {
            buf_[(head_ + count_) % EVENT_QUEUE_SIZE] = e;
            count_++;
        }
    }
    Event pop() {
        if (count_ == 0) return Event::NONE;
        Event e = buf_[head_];
        head_ = (head_ + 1) % EVENT_QUEUE_SIZE;
        count_--;
        return e;
    }
    bool    isEmpty() const { return count_ == 0; }
    uint8_t size()   const  { return count_; }
    void    clear()         { head_ = 0; count_ = 0; }
private:
    Event   buf_[EVENT_QUEUE_SIZE] = {};
    uint8_t head_  = 0;
    uint8_t count_ = 0;
};

#define MAX_LOG_ENTRIES 5
#define LOG_MSG_LEN    22

struct LogEntry {
    uint32_t timestampSec;
    char     message[LOG_MSG_LEN];
};

class EventLog {
public:
    void add(const char* msg, uint32_t timeSec) {
        strncpy(entries_[writeIdx_].message, msg, LOG_MSG_LEN - 1);
        entries_[writeIdx_].message[LOG_MSG_LEN - 1] = '\0';
        entries_[writeIdx_].timestampSec = timeSec;
        writeIdx_ = (writeIdx_ + 1) % MAX_LOG_ENTRIES;
        if (count_ < MAX_LOG_ENTRIES) count_++;
    }
    uint8_t count() const { return count_; }
    const LogEntry& get(uint8_t idx) const {
        int pos = ((int)writeIdx_ - 1 - (int)idx + MAX_LOG_ENTRIES) % MAX_LOG_ENTRIES;
        return entries_[pos];
    }
private:
    LogEntry entries_[MAX_LOG_ENTRIES] = {};
    uint8_t  writeIdx_ = 0;
    uint8_t  count_     = 0;
};

extern EventQueue    eventQueue;
extern EventLog      eventLog;
extern Settings      settings;
extern Stats         stats;
extern SystemState   systemState;
extern MenuScreen    currentScreen;
extern MotorSnapshot motorSnapshot;
extern uint32_t      runStartMs;
extern bool          cutterReversing;

void storage_init();
void storage_update();
void storage_save_settings();
void storage_save_stats();
void storage_mark_settings_dirty();
void storage_mark_stats_dirty();
void storage_reset_defaults();

void sensors_init();
void sensors_update();
bool sensors_btn_up_pressed();
bool sensors_btn_down_pressed();
bool sensors_btn_select_pressed();
bool sensors_btn_select_long();
bool sensors_estop_active();
bool sensors_filament_present();

void buzzer_init();
void buzzer_update();
void buzzer_play(BuzzerPattern pattern);
void buzzer_stop();

void led_init();
void led_update();

void motor_control_init();
void motor_control_update();
void motor_start();
void motor_stop();
void motor_pause();
void motor_resume(const MotorSnapshot& snap);
void motor_set_insert_mode(bool enable);
void motor_reverse(float mm);
bool motor_reverse_complete();
bool motor_is_stalled();
float motor_get_filament_pos_mm();
float motor_get_feed_rate();
float motor_get_cut_frequency();
MotorSnapshot motor_get_snapshot();
void motor_cutter_reverse(bool en);

void state_machine_init();
void state_machine_update();
uint32_t state_machine_get_run_time_ms();

void ui_init();
void ui_update();

#endif
