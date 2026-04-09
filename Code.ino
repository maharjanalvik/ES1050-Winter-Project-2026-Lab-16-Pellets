// =====================================================================
//  ES1050 Lab 16 Pellets — Plastic Filament Pelletizer Firmware
//  Target: Raspberry Pi Pico (RP2040) — Arduino IDE + Earle Philhower core
//  Libraries: U8g2 (OLED), Adafruit NeoPixel (RGB LED)
// =====================================================================
//
//  WHAT THIS MACHINE DOES
//  ──────────────────────
//  This is a desktop plastic filament pelletizer. It pulls 3D-printer
//  filament (typically 1.75 mm PLA/PETG/ABS) through a set of puller
//  wheels driven by a stepper motor, then chops it into uniform pellets
//  with a spinning rotary cutter blade. The pellet length and feed speed
//  are user-adjustable via an OLED menu.
//
//  BILL OF MATERIALS (BOM)
//  ───────────────────────
//  1x  Raspberry Pi Pico (RP2040) .............. Microcontroller (3.3 V logic)
//  1x  MKS SERVO42C closed-loop stepper driver . Filament puller motor (UART control)
//  1x  NEMA 17 stepper motor ................... Attached to the MKS SERVO42C
//  1x  Puller wheel / gear (12.7 mm diameter) .. Mounted on the NEMA 17 shaft
//  1x  L298N H-bridge motor driver ............. Drives the DC cutter motor
//  1x  12 V DC motor ........................... Cutter blade (1 blade on rotor)
//  1x  SH1106 128x64 OLED display (I2C) ....... User interface
//  1x  WS2812B NeoPixel RGB LED ................ Status indicator
//  1x  Passive piezo buzzer .................... Audio feedback
//  3x  Momentary push-button (N.O.) ............ UP / DOWN / SELECT navigation
//  1x  E-Stop mushroom button (N.C.) ........... Emergency stop, breaks to GND
//  1x  Filament runout switch (N.O.) ........... Microswitch; closed when filament present
//  1x  N-channel MOSFET (e.g. IRLZ44N) ........ Switches 12 V rail to motor drivers
//  1x  12 V / 5 A power supply ................. Main power (12 V for motors, 5 V for Pico via VSYS)
//
//  POWER DISTRIBUTION
//  ──────────────────
//  12 V PSU ──┬── MOSFET drain ── source → MKS SERVO42C Vmot (12 V in)
//             │                           → L298N +12 V input
//             └── 5 V buck/regulator ──→ Pico VSYS pin (5 V)
//                                      → L298N +5 V logic (if jumper removed)
//
//  The MOSFET gate is driven by PIN_MOSFET (GP4) through a 1 kΩ resistor
//  with a 10 kΩ pull-down to GND. When HIGH, it connects 12 V to both
//  the stepper driver and the L298N, allowing motors to run.
//  When the E-Stop fires (ISR), PIN_MOSFET is driven LOW immediately,
//  cutting power to all motors.
//
//  COMPLETE WIRING TABLE
//  ─────────────────────
//  Pico GP Pin  │ Connects To                        │ Notes
//  ─────────────┼────────────────────────────────────┼─────────────────────────────────────
//  GP0  (TX)    │ MKS SERVO42C UART RX               │ Serial1 TX → stepper UART (115200 baud)
//  GP1  (RX)    │ MKS SERVO42C UART TX               │ Serial1 RX ← stepper UART
//  GP2          │ L298N IN1                           │ Cutter motor direction A
//  GP3          │ L298N IN2                           │ Cutter motor direction B
//  GP4          │ MOSFET gate (via 1 kΩ)             │ 12 V power switch for all motors
//  GP5          │ E-Stop button (N.C. to GND)        │ INPUT_PULLUP; LOW = triggered
//  GP6          │ UP button (N.O. to GND)            │ INPUT_PULLUP; LOW = pressed
//  GP7          │ DOWN button (N.O. to GND)          │ INPUT_PULLUP; LOW = pressed
//  GP8          │ L298N ENA (Enable A)               │ RP2040 hardware PWM for speed ctrl
//  GP9          │ WS2812B NeoPixel DIN               │ 3.3 V data; power LED from 5 V rail
//  GP10         │ SELECT button (N.O. to GND)        │ INPUT_PULLUP; LOW = pressed
//  GP11         │ Filament runout switch              │ INPUT_PULLUP; LOW = filament present
//  GP15         │ Piezo buzzer (+)                    │ Buzzer (–) to GND; driven with tone()
//  GP16 (SDA)   │ SH1106 OLED SDA                    │ I2C data; use 4.7 kΩ pull-up to 3.3 V
//  GP17 (SCL)   │ SH1106 OLED SCL                    │ I2C clock; use 4.7 kΩ pull-up to 3.3 V
//
//  MKS SERVO42C CONNECTIONS
//  ────────────────────────
//  The MKS SERVO42C is a closed-loop stepper driver with built-in encoder.
//  It communicates over UART (TX/RX) at 115200 baud, address 0xE0.
//  Motor+ / Motor– connect to the NEMA 17 stepper coils.
//  Vmot (12 V) is powered through the MOSFET switch.
//  The puller wheel (12.7 mm / 0.5" diameter) is mounted directly on the
//  motor shaft, giving a circumference of ~39.9 mm per revolution.
//
//  L298N CONNECTIONS
//  ─────────────────
//  +12 V input ← MOSFET output (switched 12 V)
//  GND ← common ground
//  IN1 ← GP2 (direction A)
//  IN2 ← GP3 (direction B)
//  ENA ← GP8 (PWM speed control, remove the ENA jumper on the L298N board)
//  OUT1 / OUT2 → DC cutter motor terminals
//
//  BUTTON WIRING (all identical)
//  ─────────────────────────────
//  Each button has one terminal to the Pico GPIO and the other to GND.
//  The Pico's internal pull-up is enabled, so pressing a button pulls
//  the pin LOW. Debounce is handled in software (50 ms).
//
//  E-STOP WIRING
//  ─────────────
//  The E-Stop is normally-closed (N.C.) to GND. When the mushroom button
//  is pressed, the circuit OPENS, and the pull-up takes the pin HIGH —
//  wait, actually the opposite: the E-Stop connects GP5 to GND via N.C.
//  contacts. Pressing the button opens the contact → pin goes HIGH.
//  But we use INPUT_PULLUP and trigger on FALLING edge, meaning:
//  Normal state = HIGH (button released, N.C. open). Actually:
//  The E-Stop is wired N.C. between GP5 and GND. Normally the pin reads
//  LOW (shorted to GND). Pressing the E-Stop breaks the connection, pin
//  floats HIGH via pull-up. A FALLING-edge ISR fires when GP5 goes LOW,
//  which detects the button being re-engaged or the circuit closing.
//  In practice: digitalRead LOW = E-Stop is engaged/active.
//
//  FILAMENT RUNOUT SENSOR
//  ──────────────────────
//  A mechanical microswitch that is closed (LOW) when filament is present.
//  When filament runs out, the switch opens, the pull-up takes the pin
//  HIGH, and after debounce a FILAMENT_RUNOUT event is pushed.
//
// =====================================================================

#include <Arduino.h>
#include <EEPROM.h>
#include <Wire.h>
#include <U8g2lib.h>
#include <Adafruit_NeoPixel.h>
#include "hardware/pwm.h"   // RP2040 hardware PWM registers (used for cutter motor)

// The U8g2 library defines ERROR as a macro which conflicts with our enum.
#ifdef ERROR
#undef ERROR
#endif

// =====================================================================
//  CONFIG — Pin definitions, constants, default values
// =====================================================================

#define FW_VERSION "1.0.0"

// ── GPIO Pin Assignments ──
// See wiring table in header for physical connections.
#define PIN_STEPPER_TX       0   // GP0 → MKS SERVO42C UART RX (Serial1 TX)
#define PIN_STEPPER_RX       1   // GP1 ← MKS SERVO42C UART TX (Serial1 RX)
#define PIN_CUTTER_IN_A      2   // GP2 → L298N IN1 (cutter direction A)
#define PIN_CUTTER_IN_B      3   // GP3 → L298N IN2 (cutter direction B)
#define PIN_MOSFET           4   // GP4 → MOSFET gate (switches 12 V to motor drivers)
#define PIN_ESTOP            5   // GP5 ← E-Stop (N.C. to GND; LOW = stop active)
#define PIN_BTN_UP           6   // GP6 ← UP button (N.O. to GND)
#define PIN_BTN_DOWN         7   // GP7 ← DOWN button (N.O. to GND)
#define PIN_CUTTER_ENA       8   // GP8 → L298N ENA pin (hardware PWM, 0-255)
#define PIN_RGB_LED          9   // GP9 → WS2812B NeoPixel data in
#define PIN_BTN_SELECT      10   // GP10 ← SELECT button (N.O. to GND)
#define PIN_FILAMENT_RUNOUT 11   // GP11 ← Filament microswitch (closed=LOW when present)
#define PIN_BUZZER          15   // GP15 → Passive piezo buzzer (+), other leg to GND
#define PIN_OLED_SDA        16   // GP16 → SH1106 OLED SDA (I2C, 4.7 kΩ pull-up to 3.3 V)
#define PIN_OLED_SCL        17   // GP17 → SH1106 OLED SCL (I2C, 4.7 kΩ pull-up to 3.3 V)

// ── MKS SERVO42C Stepper Driver Constants ──
// The MKS SERVO42C is a UART-controlled closed-loop stepper with built-in
// encoder. Commands are sent over Serial1 at 115200 baud.
// The puller wheel is 12.7 mm (1/2") diameter, mounted directly on the
// NEMA 17 motor shaft. One full revolution feeds ~39.9 mm of filament.
#define STEPPER_UART_BAUD       115200     // MKS SERVO42C UART baud rate
#define STEPPER_ADDR            0xE0       // Default MKS address byte (first byte of every command)
#define STEPPER_MAX_SPEED_VAL   127        // MKS speed field is 7 bits: 0-127
#define STEPPER_MAX_SPEED_MMS   250.0f     // Calibrated top speed at speed_val=127 (~250 mm/s)
#define PULLER_DIAMETER_MM      12.7f      // Puller wheel outside diameter (determines mm/rev)
#define PULLER_CIRCUMF_MM       (PULLER_DIAMETER_MM * PI)  // ~39.9 mm per revolution
#define STEPPER_MSTEP           236        // Microstep setting register value (sent to MKS 0x84 cmd)
#define STEPPER_ACC             0x0050     // Acceleration register value (sent to MKS 0xA4 cmd)
#define STEPPER_PULSES_PER_REV  (STEPPER_MSTEP * 200UL)   // Total microsteps per full revolution
#define STEPPER_BIG_PULSE_COUNT 4720000UL  // Very large pulse count for "continuous" FD moves
#define STEPPER_REISSUE_MS      8000       // Re-send FD command every 8 s to keep motor running
#define STEPPER_POLL_MS         1500       // Poll stall/protection status every 1.5 s
#define STEPPER_CMD_TIMEOUT_MS  50         // UART response timeout for non-blocking commands

// ── L298N Cutter Motor Constants ──
// The L298N drives a small 12 V DC motor with a single cutting blade.
// Speed is controlled via hardware PWM on the ENA pin (0-255 duty cycle).
// Direction is set by IN1/IN2 (one HIGH, one LOW = spin; both LOW = brake).
// The cutter speed is automatically calculated from feed speed and pellet
// length to maintain the correct cut frequency.
#define CUTTER_NUM_BLADES       1          // Number of blades on the cutter rotor
#define CUTTER_SOFT_START_MS    500        // Ramp cutter PWM from 0 to target over 500 ms
#define CUTTER_MIN_PWM          60         // Minimum PWM duty (below this the motor stalls)
#define CUTTER_MAX_PWM          255        // Maximum PWM duty (full speed)
#define CUTTER_MAX_RPM          3000.0f    // Measured/estimated max RPM at PWM=255

// ── Pellet Length Settings ──
// Pellet length determines how far the filament feeds between each cut.
// Adjustable via the OLED menu in 0.5 mm increments.
#define DEFAULT_PELLET_LEN_MM    3.0f      // Default pellet length (mm)
#define MIN_PELLET_LEN_MM        1.0f      // Minimum allowed pellet length
#define MAX_PELLET_LEN_MM       10.0f      // Maximum allowed pellet length
#define PELLET_LEN_STEP_MM       0.5f      // Adjustment step size in settings menu

// ── Feed Speed Settings ──
// Feed speed is how fast filament is pulled through (mm/s).
// Adjustable via the OLED menu in 5 mm/s increments.
#define DEFAULT_FEED_SPEED_MMS   6.0f      // Default continuous feed speed (mm/s)
#define MIN_FEED_SPEED_MMS       5.0f      // Minimum allowed feed speed
#define MAX_FEED_SPEED_MMS     100.0f      // Maximum allowed feed speed
#define FEED_SPEED_STEP_MMS      5.0f      // Adjustment step size in settings menu

// ── Jam Detection & Recovery ──
// When the MKS SERVO42C detects a stall (motor can't turn), the firmware
// reverses the stepper to try to clear the jam, then resumes. After
// JAM_RETRY_MAX consecutive failures, the system enters ERROR state.
#define DEFAULT_JAM_REVERSE_MM  10.0f      // How far to reverse on jam (mm)
#define JAM_RETRY_MAX            3         // Max consecutive jam-clear attempts before ERROR
#define JAM_STALL_TIMEOUT_MS     200       // (reserved) minimum stall duration to confirm jam
#define JAM_REVERSE_SPEED_MMS    10.0f     // Reverse speed during jam clearing (mm/s)
#define JAM_PAUSE_MS             500       // (reserved) pause after reverse before resuming
#define INSERT_FEED_SPEED_MMS    5.0f      // Slow feed speed during manual filament insertion

// ── Timing Intervals (cooperative multitasking) ──
// The main loop calls each subsystem's update() function every iteration.
// Each subsystem uses its own timer to throttle to the interval below.
#define SENSOR_INTERVAL_MS          5      // Button/sensor polling (200 Hz)
#define STATE_INTERVAL_MS          10      // State machine tick (100 Hz)
#define MOTOR_INTERVAL_MS           1      // Motor position tracking & cutter ramp (1 kHz)
#define UI_INTERVAL_MS             50      // OLED redraw (20 fps)
#define LED_INTERVAL_MS            50      // NeoPixel color update (20 fps)
#define STORAGE_FLUSH_INTERVAL_MS  60000   // Write dirty settings/stats to flash (every 60 s)

// ── Button & UI Timing ──
#define DEBOUNCE_MS             50         // Software debounce window for all buttons & sensors
#define LONG_PRESS_MS          800         // Hold SELECT this long for a long-press action
#define SCREEN_TIMEOUT_MS    60000         // Dim OLED after 60 s of no button input
#define OLED_DIM_CONTRAST        0         // OLED contrast when dimmed (0 = nearly off)
#define OLED_FULL_CONTRAST     255         // OLED contrast at full brightness

#define IDLE_TIMEOUT_MS       300000       // Auto-transition from PAUSED → IDLE after 5 min

// ── EEPROM Layout ──
// The RP2040 emulates EEPROM in flash. Settings and stats are stored at
// fixed offsets with a CRC-8 byte for corruption detection.
#define EEPROM_SIZE            256         // Total emulated EEPROM size (bytes)
#define SETTINGS_ADDR            0         // Byte offset of Settings struct
#define STATS_ADDR              64         // Byte offset of Stats struct

// =====================================================================
//  TYPES — Enums, structs, classes
// =====================================================================

// The system is a 7-state finite state machine (FSM). All state
// transitions are driven by events pushed into a queue.
//
// State diagram:
//   IDLE ──Start──→ RUNNING ──Pause──→ PAUSED ──Resume──→ RUNNING
//     │                │                  │
//     │                ├─Jam──→ JAM_CLEARING ──Cleared──→ RUNNING
//     │                │                      ──Failed──→ ERROR
//     │                └─Runout──→ RUNOUT
//     └─Insert──→ INSERT_FILAMENT ──Exit──→ (previous state)
//
//   Any state + E-Stop → ERROR
//   ERROR ──ClearError──→ IDLE
enum class SystemState : uint8_t {
    IDLE,              // Motors off, waiting for user to press Start
    RUNNING,           // Stepper feeding filament, cutter spinning
    PAUSED,            // Motors stopped, snapshot saved for resume
    INSERT_FILAMENT,   // Stepper runs slowly (5 mm/s) to load new filament
    ERROR,             // Hard stop — E-Stop hit or jam limit exceeded
    JAM_CLEARING,      // Stepper reversing to clear a detected jam
    RUNOUT             // Filament runout sensor triggered; motors stopped
};

// OLED display screens — navigated with UP/DOWN/SELECT buttons.
enum class MenuScreen : uint8_t {
    HOME,                // Default: shows state, speed, position, cut freq
    MAIN_MENU,           // Start/Pause/Resume, Insert, Reverse Cutter, Settings...
    SETTINGS,            // Pellet length, feed speed, back
    SETTINGS_PELLET_LEN, // Adjust pellet length with UP/DOWN
    SETTINGS_FEED_SPEED, // Adjust feed speed with UP/DOWN
    WARNINGS,            // Active warnings + recent event log
    MAINTENANCE,         // Lifetime runtime, filament used, jam/error counts
    SYSTEM_INFO          // Firmware version, MCU type, current state
};

// Events are produced by sensors, timers, or UI actions and consumed
// by the state machine on each tick.
enum class Event : uint8_t {
    NONE,                    // Empty queue sentinel
    ESTOP,                   // E-Stop ISR fired (GP5 went LOW)
    FILAMENT_RUNOUT,         // Filament microswitch opened (GP11 went HIGH)
    JAM_DETECTED,            // MKS SERVO42C reported stall via 0x3E poll
    JAM_CLEARED,             // Reverse completed successfully
    JAM_FAILED,              // Reverse did not clear the jam
    START_CMD,               // User pressed Start (from IDLE)
    PAUSE_CMD,               // User pressed Pause (from RUNNING)
    RESUME_CMD,              // User pressed Resume (from PAUSED)
    INSERT_FILAMENT_CMD,     // User selected "Insert Filament" in menu
    EXIT_INSERT_CMD,         // User selected "Exit Insert" in menu
    IDLE_TIMEOUT,            // (reserved) Paused too long
    CLEAR_ERROR,             // User cleared an error from menu
    REVERSE_CUTTER_CMD,      // User selected "Reverse Cutter" for unclogging
    STOP_REVERSE_CUTTER_CMD  // User selected "Stop Reverse" to end unclog
};

// Buzzer tone sequences — each is a non-blocking pattern of freq/duration steps.
enum class BuzzerPattern : uint8_t {
    NONE, CLICK, START, PAUSE_BEEP, ERROR_ALARM, JAM_WARN, RUNOUT_BEEP
};

// Persisted to EEPROM at SETTINGS_ADDR. CRC-8 in last byte for validation.
struct Settings {
    float    pelletLenMm;      // Desired pellet length (mm) — sets cutter frequency
    float    feedSpeedMmS;     // Filament feed speed (mm/s) — sets stepper speed
    float    jamReverseMm;     // How far to reverse stepper on jam (mm)
    bool     buzzerEnabled;    // If false, only ERROR_ALARM sounds play
    uint8_t  _pad[3];         // Padding to keep alignment consistent
    uint32_t screenTimeoutMs;  // OLED dim timeout (ms)
    uint8_t  crc;              // CRC-8 over all preceding bytes
};

// Persisted to EEPROM at STATS_ADDR. Lifetime counters for maintenance tracking.
struct Stats {
    uint32_t totalRuntimeSec;  // Cumulative RUNNING-state seconds
    uint32_t totalFilamentMm;  // Cumulative filament fed (mm, integer)
    uint16_t jamCount;         // Lifetime jam events
    uint16_t errorCount;       // Lifetime error events
    uint8_t  crc;              // CRC-8 over all preceding bytes
};

// Captured when transitioning out of RUNNING so we can resume exactly
// where we left off (stepper position, cutter phase, speed).
struct MotorSnapshot {
    float filamentPosMm;       // Open-loop estimated filament position
    float cutterPhaseMs;       // Current position in the cutter cycle
    float feedSpeedMmS;        // Feed speed at time of snapshot
};

// Lock-free circular buffer for inter-module event passing.
// Sensors, timers, and UI push events; the state machine pops and processes them.
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

// Circular log of recent events shown on the Warnings screen.
// Newest entry is at index 0 via get().
#define MAX_LOG_ENTRIES 5
#define LOG_MSG_LEN    22

struct LogEntry {
    uint32_t timestampSec;         // Uptime in seconds when event occurred
    char     message[LOG_MSG_LEN]; // Human-readable event description
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

// =====================================================================
//  GLOBALS
// =====================================================================

EventQueue    eventQueue;
EventLog      eventLog;
Settings      settings;
Stats         stats;
SystemState   systemState   = SystemState::IDLE;
MenuScreen    currentScreen = MenuScreen::HOME;
MotorSnapshot motorSnapshot = {0, 0, 0};
uint32_t      runStartMs    = 0;
bool          cutterReversing = false;

// =====================================================================
//  FORWARD DECLARATIONS (non-static public API of each module)
// =====================================================================

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

// =====================================================================
//  STORAGE — Flash-backed EEPROM with CRC-8 validation
// =====================================================================
//
//  The RP2040 has no true EEPROM. The Earle Philhower Arduino core
//  emulates EEPROM in a reserved flash sector. EEPROM.commit() writes
//  the buffer to flash (slow, ~100 ms). To avoid wearing out flash,
//  we only flush dirty data every 60 seconds (STORAGE_FLUSH_INTERVAL_MS).
//
//  Both the Settings and Stats structs end with a CRC-8 byte. On boot,
//  if the CRC doesn't match, defaults are loaded (settings) or zeroed (stats).

static bool     stor_settingsDirty = false;  // true if settings changed since last flush
static bool     stor_statsDirty    = false;  // true if stats changed since last flush
static uint32_t stor_lastFlushMs   = 0;      // Timestamp of last EEPROM flush

static uint8_t crc8(const uint8_t* data, size_t len) {
    uint8_t crc = 0;
    for (size_t i = 0; i < len; i++) {
        crc ^= data[i];
        for (uint8_t j = 0; j < 8; j++)
            crc = (crc & 0x80) ? ((crc << 1) ^ 0x07) : (crc << 1);
    }
    return crc;
}

static void loadDefaults() {
    settings.pelletLenMm     = DEFAULT_PELLET_LEN_MM;
    settings.feedSpeedMmS    = DEFAULT_FEED_SPEED_MMS;
    settings.jamReverseMm    = DEFAULT_JAM_REVERSE_MM;
    settings.buzzerEnabled   = true;
    settings.screenTimeoutMs = SCREEN_TIMEOUT_MS;
}

void storage_init() {
    EEPROM.begin(EEPROM_SIZE);
    EEPROM.get(SETTINGS_ADDR, settings);
    if (crc8((const uint8_t*)&settings, sizeof(Settings) - 1) != settings.crc) {
        loadDefaults();
        stor_settingsDirty = true;
    }
    EEPROM.get(STATS_ADDR, stats);
    if (crc8((const uint8_t*)&stats, sizeof(Stats) - 1) != stats.crc) {
        memset(&stats, 0, sizeof(Stats));
        stor_statsDirty = true;
    }
}

void storage_mark_settings_dirty() { stor_settingsDirty = true; }
void storage_mark_stats_dirty()    { stor_statsDirty    = true; }

void storage_save_settings() {
    settings.crc = crc8((const uint8_t*)&settings, sizeof(Settings) - 1);
    EEPROM.put(SETTINGS_ADDR, settings);
    EEPROM.commit();
    stor_settingsDirty = false;
}

void storage_save_stats() {
    stats.crc = crc8((const uint8_t*)&stats, sizeof(Stats) - 1);
    EEPROM.put(STATS_ADDR, stats);
    EEPROM.commit();
    stor_statsDirty = false;
}

void storage_update() {
    uint32_t now = millis();
    if (now - stor_lastFlushMs < STORAGE_FLUSH_INTERVAL_MS) return;
    stor_lastFlushMs = now;
    if (stor_settingsDirty) storage_save_settings();
    if (stor_statsDirty)    storage_save_stats();
}

void storage_reset_defaults() {
    loadDefaults();
    memset(&stats, 0, sizeof(Stats));
    stor_settingsDirty = true;
    stor_statsDirty    = true;
}

// =====================================================================
//  SENSORS — Buttons, E-Stop ISR, filament runout
// =====================================================================
//
//  Hardware:
//    - 3 navigation buttons (UP/DOWN/SELECT) wired between GPIO and GND
//      with internal pull-ups. LOW = pressed.
//    - E-Stop: mushroom button with N.C. contacts between GP5 and GND.
//      Normal (safe) = pin reads LOW (shorted to GND).
//      Pressed (emergency) = circuit opens, pin floats HIGH via pull-up.
//      A FALLING-edge ISR catches the transition when E-Stop re-engages,
//      but the main check is: digitalRead(GP5) == LOW means active.
//      The ISR immediately drives PIN_MOSFET LOW to kill motor power
//      with minimal latency (hardware-level safety).
//    - Filament runout: mechanical microswitch. When filament pushes the
//      lever, the switch closes → GP11 reads LOW (filament present).
//      When filament runs out, switch opens → GP11 reads HIGH (runout).
//
//  Debouncing: All inputs use a 50 ms software debounce. A reading must
//  remain stable for DEBOUNCE_MS before the stable state is updated.
//  SELECT supports long-press detection (hold ≥ 800 ms).

static volatile bool estopISRFlag = false;

// ISR: fires on FALLING edge of GP5. Immediately cuts motor power.
static void estopISR() {
    digitalWrite(PIN_MOSFET, LOW);   // Kill 12 V to motors ASAP
    estopISRFlag = true;             // Flag for main loop to process
}

// Per-button debounce state
struct Btn {
    uint8_t  pin;           // GPIO pin number
    bool     raw;           // Last raw digitalRead value
    bool     stable;        // Debounced stable state (true=HIGH=released)
    uint32_t lastEdgeMs;    // Timestamp of last raw-level change
    uint32_t pressStartMs;  // When the current press started
    bool     held;          // Currently being held down
    bool     longFired;     // Long-press event already fired for this press
};

static Btn btns[3];           // [0]=UP, [1]=DOWN, [2]=SELECT
static bool flagUp      = false;  // Set when UP short-press detected, cleared on read
static bool flagDown    = false;
static bool flagSelect  = false;
static bool flagSelLong = false;  // Set when SELECT long-press detected

// Filament runout debounce state
static bool     filRawPrev   = true;   // Previous raw reading
static uint32_t filEdgeMs    = 0;      // Timestamp of last raw change
static bool     filStable    = true;   // Debounced: true = filament present
static uint32_t sens_lastTickMs = 0;

void sensors_init() {
    // All inputs use internal pull-ups: unpressed/open = HIGH, pressed/closed = LOW
    pinMode(PIN_ESTOP,           INPUT_PULLUP);
    pinMode(PIN_BTN_UP,          INPUT_PULLUP);
    pinMode(PIN_BTN_DOWN,        INPUT_PULLUP);
    pinMode(PIN_BTN_SELECT,      INPUT_PULLUP);
    pinMode(PIN_FILAMENT_RUNOUT, INPUT_PULLUP);

    // E-Stop ISR fires on FALLING edge (pin going LOW = E-Stop active)
    attachInterrupt(digitalPinToInterrupt(PIN_ESTOP), estopISR, FALLING);

    btns[0] = { PIN_BTN_UP,     true, true, 0, 0, false, false };
    btns[1] = { PIN_BTN_DOWN,   true, true, 0, 0, false, false };
    btns[2] = { PIN_BTN_SELECT, true, true, 0, 0, false, false };

    // Read initial filament sensor state (LOW = filament present)
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

// =====================================================================
//  BUZZER — Non-blocking tone pattern sequencer
// =====================================================================
//
//  Hardware: Passive piezo buzzer connected between GP15 (+) and GND (–).
//  Driven using the Arduino tone() function which generates a square wave
//  at the specified frequency. "Passive" means the buzzer needs an AC
//  signal to produce sound (unlike "active" buzzers which just need DC).
//
//  Each pattern is an array of {frequency_Hz, duration_ms} steps.
//  freq=0 means silence for that duration. The sentinel {0,0} marks end.
//  Patterns are played non-blocking — buzzer_update() advances steps
//  based on elapsed time.

struct ToneStep { uint16_t freq; uint16_t ms; };

static const ToneStep sClick[]  = { {4000,30}, {0,0} };                                    // Short click for button presses
static const ToneStep sStart[]  = { {800,100}, {1000,100}, {1200,150}, {0,0} };            // Ascending 3-tone for start
static const ToneStep sPause[]  = { {1000,100}, {0,80}, {1000,100}, {0,0} };               // Double beep for pause
static const ToneStep sError[]  = { {800,200}, {0,200}, {800,200}, {0,200}, {800,200}, {0,0} }; // Triple alarm for errors
static const ToneStep sJam[]    = { {600,150}, {900,150}, {600,150}, {0,0} };              // Warble for jam detected
static const ToneStep sRunout[] = { {500,300}, {0,700}, {500,300}, {0,700}, {0,0} };       // Slow double beep for runout

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

// =====================================================================
//  LED — WS2812B NeoPixel state-color mapping
// =====================================================================
//
//  Hardware: Single WS2812B RGB LED. Data pin = GP9. Power the LED from
//  the 5 V rail (VBUS or regulator output), NOT from the Pico's 3.3 V pin.
//  The WS2812B accepts 3.3 V logic on the data line from the RP2040.
//  If the LED is more than ~10 cm from the Pico, add a 470 Ω resistor
//  in series on the data line and a 100 µF capacitor across LED VCC/GND.
//
//  Color mapping:
//    IDLE           → dim white (20, 20, 20)
//    RUNNING        → solid green
//    PAUSED         → breathing yellow (sinusoidal 2-second cycle)
//    INSERT_FILAMENT→ solid blue
//    ERROR          → flashing red (500 ms on/off)
//    JAM_CLEARING   → solid purple
//    RUNOUT         → flashing orange (750 ms on/off)

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

// =====================================================================
//  MOTOR CONTROL — MKS SERVO42C UART + L298N cutter (hardware PWM)
// =====================================================================
//
//  This module controls two independent motors:
//
//  1. FILAMENT PULLER (stepper via MKS SERVO42C)
//     ─────────────────────────────────────────
//     The MKS SERVO42C is wired to Serial1 (GP0=TX, GP1=RX) at 115200 baud.
//     It speaks a simple binary protocol: each command starts with the
//     address byte (0xE0), followed by a command byte, payload, and a
//     checksum (sum of all preceding bytes, masked to 8 bits).
//
//     Key commands used:
//       0xFD — Move: [addr, 0xFD, dir|speed, pulse3, pulse2, pulse1, pulse0, checksum]
//              dir|speed byte: bit 7 = direction (0=CW, 1=CCW), bits 6:0 = speed (0-127)
//              pulse count: 4-byte big-endian number of microsteps to move
//       0xF3 — Enable/disable motor: [addr, 0xF3, 0x01/0x00, checksum]
//       0xF7 — Emergency stop: [addr, 0xF7, checksum]
//       0x3E — Read protection status: returns [addr, status] where 0x01 = stalled
//       0xA4 — Set acceleration: [addr, 0xA4, acc_hi, acc_lo, checksum]
//       0x84 — Set microstep mode: [addr, 0x84, mstep_val, checksum]
//       0x88 — Enable/disable stall protection: [addr, 0x88, 0x01/0x00, checksum]
//       0x3D — Clear protection/stall flag: [addr, 0x3D, checksum]
//
//     For continuous feeding, we issue an FD command with a very large pulse
//     count (STEPPER_BIG_PULSE_COUNT = 4,720,000) and re-issue it every 8 s
//     so the motor never runs out of pulses.
//
//  2. CUTTER (DC motor via L298N)
//     ────────────────────────────
//     The L298N ENA pin is driven with RP2040 hardware PWM (not Arduino
//     analogWrite, which doesn't work reliably on the Pico). The PWM wrap
//     is set to 255 for 8-bit duty cycle control.
//     IN1 (GP2) and IN2 (GP3) set direction:
//       IN1=HIGH, IN2=LOW  → forward (normal cutting direction)
//       IN1=LOW,  IN2=HIGH → reverse (for unclogging)
//       Both LOW           → motor brakes/stops
//     Cutter speed (PWM duty) is auto-calculated from feed speed and pellet
//     length so the blade RPM matches the required cut frequency.
//
//  SPEED MAPPING
//  ─────────────
//  The MKS speed value (0-127) maps linearly to 0 – STEPPER_MAX_SPEED_MMS.
//  At speed_val=127, the puller feeds ~250 mm/s. The calibration point
//  was speed_val 15 ≈ 30 mm/s.
//
//  DISTANCE → PULSES
//  ──────────────────
//  mm → revolutions = mm / PULLER_CIRCUMF_MM (~39.9 mm)
//  revolutions → pulses = revs * STEPPER_PULSES_PER_REV (MSTEP * 200)

// ── Hardware PWM helpers (replaces analogWrite for Pico) ──
// The RP2040's analogWrite() is unreliable; we use the hardware PWM
// peripheral directly. Each GPIO maps to a PWM slice and channel.

static void hw_pwm_setup(int pin) {
    gpio_set_function(pin, GPIO_FUNC_PWM);
    uint slice = pwm_gpio_to_slice_num(pin);
    pwm_set_wrap(slice, 255);          // 8-bit resolution (0-255)
    pwm_set_enabled(slice, true);
}

static void hw_pwm_write(int pin, int value) {
    uint slice   = pwm_gpio_to_slice_num(pin);
    uint channel = pwm_gpio_to_channel(pin);
    pwm_set_chan_level(slice, channel, value);
}

// ── MKS SERVO42C UART protocol (low-level) ──
// All commands are framed as: [address, cmd, ...payload, checksum]
// Checksum = sum of all preceding bytes, masked to 8 bits.
// The driver responds with [address, status] for most commands.

static byte mks_checksum(byte* buf, int len) {
    byte sum = 0;
    for (int i = 0; i < len; i++) sum += buf[i];
    return sum & 0xFF;
}

// Drain any pending bytes from the UART RX buffer.
// If quietMs > 0, keeps draining until no new bytes arrive for quietMs.
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

// Read up to 'count' bytes with timeout. Returns number of bytes read.
static int mks_readBytes(byte* buf, int count, unsigned long timeoutMs = 30) {
    unsigned long start = millis();
    int idx = 0;
    while (idx < count && millis() - start < timeoutMs) {
        if (Serial1.available()) buf[idx++] = Serial1.read();
    }
    return idx;
}

// Non-blocking send: flush RX, send command, read ack with short timeout.
// Used during normal operation where we can't afford to block.
static void mks_sendControl(byte* cmd, int len, int ackBytes, unsigned long timeoutMs) {
    mks_flush();
    Serial1.write(cmd, len);
    byte ack[4];
    mks_readBytes(ack, ackBytes, timeoutMs);
}

// Non-blocking send with response: returns the response bytes read.
static int mks_sendRead(byte* cmd, int cmdLen, byte* resp, int respBytes, unsigned long timeoutMs) {
    mks_flush();
    Serial1.write(cmd, cmdLen);
    return mks_readBytes(resp, respBytes, timeoutMs);
}

// Blocking send for setup(): uses 80 ms quiet-period flush and 300 ms
// response timeout. OK to block here because setup() only runs once.
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

// ── High-level stepper commands ──
// Each function builds a raw command packet and sends it via UART.

// Enable/disable the stepper driver (0xF3). When disabled, the motor
// shaft is free to spin. When enabled, the driver holds position.
static void stepper_setEnable(bool en, bool blocking = false) {
    byte cmd[] = { STEPPER_ADDR, 0xF3, (byte)(en ? 0x01 : 0x00), 0x00 };
    cmd[3] = mks_checksum(cmd, 3);
    if (blocking) mks_sendControl_blocking(cmd, 4, 2);
    else          mks_sendControl(cmd, 4, 2, STEPPER_CMD_TIMEOUT_MS);
}

// Set acceleration register (0xA4). Higher = faster ramp to target speed.
static void stepper_setAcceleration(uint16_t acc) {
    byte cmd[] = { STEPPER_ADDR, 0xA4, (byte)(acc >> 8), (byte)(acc & 0xFF), 0x00 };
    cmd[4] = mks_checksum(cmd, 4);
    mks_sendControl_blocking(cmd, 5, 2);
}

// Set microstep resolution (0x84). Value 236 sets the subdivision mode.
static void stepper_setMicrosteps(byte mstep) {
    byte cmd[] = { STEPPER_ADDR, 0x84, mstep, 0x00 };
    cmd[3] = mks_checksum(cmd, 3);
    mks_sendControl_blocking(cmd, 4, 2);
}

// Enable/disable stall protection (0x88). When enabled, the driver
// sets an internal flag if the motor stalls (load exceeds torque).
// We poll this flag with stepper_pollStallStatus() for jam detection.
static void stepper_setProtection(bool en) {
    byte cmd[] = { STEPPER_ADDR, 0x88, (byte)(en ? 0x01 : 0x00), 0x00 };
    cmd[3] = mks_checksum(cmd, 3);
    mks_sendControl_blocking(cmd, 4, 2);
}

// Clear the stall protection flag (0x3D) after a jam has been resolved.
static void stepper_clearProtection() {
    byte cmd[] = { STEPPER_ADDR, 0x3D, 0x00 };
    cmd[2] = mks_checksum(cmd, 2);
    mks_sendControl(cmd, 3, 2, STEPPER_CMD_TIMEOUT_MS);
}

// Emergency stop (0xF7). Immediately halts the stepper.
static void stepper_sendStop(bool blocking = false) {
    byte cmd[] = { STEPPER_ADDR, 0xF7, 0x00 };
    cmd[2] = mks_checksum(cmd, 2);
    if (blocking) mks_sendControl_blocking(cmd, 3, 2);
    else          mks_sendControl(cmd, 3, 2, STEPPER_CMD_TIMEOUT_MS);
}

// Start a move (0xFD command). This is the primary motion command.
// The motor runs at 'speed' (0-127) in direction 'dir' (0=CW, 1=CCW)
// for 'pulses' microsteps. For continuous feeding, we use a very large
// pulse count and re-issue before it expires.
//
// Packet format: [0xE0, 0xFD, dir|speed, pulse_B3, B2, B1, B0, checksum]
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

// Poll stall/protection status (0x3E). Returns true if the MKS reports
// that the motor is stalled (can't maintain position under load).
static bool stepper_pollStallStatus() {
    byte cmd[] = { STEPPER_ADDR, 0x3E, 0x00 };
    cmd[2] = mks_checksum(cmd, 2);
    byte resp[2];
    int n = mks_sendRead(cmd, 3, resp, 2, STEPPER_CMD_TIMEOUT_MS);
    if (n >= 2 && resp[0] == STEPPER_ADDR)
        return (resp[1] == 0x01);
    return false;
}

// ── Speed mapping: mm/s → MKS speed value (0-127) ──
// Linear mapping: speed_val = (desired_mm_s / MAX_MM_S) * 127
// Clamped to [1, 127]. Returns 0 only if input is near zero.

static uint8_t mmsToSpeedVal(float mmps) {
    float ratio = fabsf(mmps) / STEPPER_MAX_SPEED_MMS;
    if (ratio > 1.0f) ratio = 1.0f;
    uint8_t val = (uint8_t)(ratio * STEPPER_MAX_SPEED_VAL);
    if (val < 1 && mmps > 0.1f) val = 1;
    return val;
}

// ── Distance → pulse count conversion ──
// Converts a linear distance (mm) to the number of microsteps needed.
// Uses the puller wheel circumference to determine revolutions, then
// multiplies by pulses-per-revolution.

static uint32_t mmToPulses(float mm) {
    float revs = fabsf(mm) / PULLER_CIRCUMF_MM;
    return (uint32_t)(revs * STEPPER_PULSES_PER_REV);
}

// ── Module internal state ──
// These variables track the real-time state of both motors.

static float    mc_feedSpeedMmS = 0;        // Current commanded feed speed (mm/s)
static bool     mc_feedFwd      = true;      // true=forward (feeding), false=reverse (jam clearing)
static bool     mc_stepperRunning = false;   // true if an FD command is active
static byte     mc_stepperDir   = 0;         // MKS direction: 0=CW, 1=CCW (CCW=feed forward)
static byte     mc_stepperSpeed = 0;         // MKS speed value currently commanded (0-127)
static float    mc_filPosMm     = 0;         // Open-loop filament position estimate (mm)
static float    mc_cutPhaseMs   = 0;         // Tracks where we are in the current cut cycle (ms)

static uint8_t  mc_cutTgtPWM    = 0;         // Target cutter PWM after ramp
static uint8_t  mc_cutCurPWM    = 0;         // Current cutter PWM (ramping up)
static bool     mc_cutRunning   = false;     // Cutter soft-start ramp in progress
static uint32_t mc_cutStartMs   = 0;         // When cutter was started (for ramp timing)

static bool     mc_reversing       = false;  // true during jam-clearing reverse move
static float    mc_revDistMm       = 0;      // Distance to reverse (mm)
static uint32_t mc_reverseStartMs  = 0;      // When reverse started
static uint32_t mc_reverseTimeMs   = 0;      // Estimated time for reverse to complete

static bool     mc_stallFlag       = false;  // true if MKS reported a stall

// Pulsed feed mode (currently disabled — set to false for continuous feed).
// When true, the stepper feeds MC_PULSE_DIST_MM then waits MC_PULSE_WAIT_MS,
// then feeds again. Used for testing/calibration.
static bool     mc_pulsedMode      = false;
static const float MC_PULSE_DIST_MM = 5.0f;       // Distance per pulse burst
static const uint32_t MC_PULSE_WAIT_MS = 5000;    // Wait between bursts (ms)
static bool     mc_pulseFeeding    = false;        // true=currently feeding, false=waiting
static uint32_t mc_pulseStartMs    = 0;            // Start of current feed/wait phase
static uint32_t mc_pulseFeedTimeMs = 0;            // Estimated duration of feed phase

static uint32_t mc_lastFastMs      = 0;      // Timestamp of last 1 ms fast tick
static uint32_t mc_lastReissueMs   = 0;      // Timestamp of last FD command re-issue
static uint32_t mc_lastPollMs      = 0;      // Timestamp of last stall status poll

// ── Cutter helpers (L298N hardware PWM) ──

// Set cutter direction via L298N IN1/IN2.
// fwd=true: IN1=HIGH, IN2=LOW (normal cutting direction)
// fwd=false: IN1=LOW, IN2=HIGH (reverse for unclogging)
static void cutter_dir(bool fwd) {
    digitalWrite(PIN_CUTTER_IN_A, fwd ? HIGH : LOW);
    digitalWrite(PIN_CUTTER_IN_B, fwd ? LOW  : HIGH);
}

// Stop cutter: zero PWM duty and set both direction pins LOW (brake).
static void cutter_off() {
    hw_pwm_write(PIN_CUTTER_ENA, 0);
    digitalWrite(PIN_CUTTER_IN_A, LOW);
    digitalWrite(PIN_CUTTER_IN_B, LOW);
    mc_cutRunning = false;
    mc_cutCurPWM  = 0;
    mc_cutTgtPWM  = 0;
}

// Calculate the cutter PWM duty cycle needed to match the current feed
// speed and pellet length. The required cut frequency is:
//   cutFreq (Hz) = feedSpeed (mm/s) / pelletLen (mm)
// With N blades, the required RPM is:
//   rpm = cutFreq * 60 / N_blades
// PWM is linearly mapped from [MIN_PWM..MAX_PWM] for [0..MAX_RPM].
static uint8_t calc_cutter_pwm() {
    if (settings.pelletLenMm <= 0) return 0;
    float cutFreq = settings.feedSpeedMmS / settings.pelletLenMm;
    float tgtRPM  = cutFreq * 60.0f / CUTTER_NUM_BLADES;
    float ratio   = tgtRPM / CUTTER_MAX_RPM;
    if (ratio > 1.0f) ratio = 1.0f;
    return (uint8_t)(CUTTER_MIN_PWM + ratio * (CUTTER_MAX_PWM - CUTTER_MIN_PWM));
}

// Start the stepper motor: convert mm/s to MKS speed value, set direction,
// clear any stall flags, enable the driver, and issue the FD move command.
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

// Full stop: send stop command, mark stepper as not running, disable driver.
static void stepper_fullStop() {
    stepper_sendStop();
    mc_stepperRunning = false;
    mc_feedSpeedMmS   = 0;
    stepper_setEnable(false);
}

// ── Motor public API ──

// Called once from setup(). Initializes UART for the MKS SERVO42C,
// configures the L298N cutter pins, and sends initial config to the stepper.
void motor_control_init() {
    // Set up Serial1 on GP0 (TX) and GP1 (RX) for the MKS SERVO42C
    Serial1.setTX(PIN_STEPPER_TX);
    Serial1.setRX(PIN_STEPPER_RX);
    Serial1.begin(STEPPER_UART_BAUD);

    // L298N cutter: IN1/IN2 as GPIO outputs, ENA as hardware PWM
    pinMode(PIN_CUTTER_IN_A, OUTPUT);
    pinMode(PIN_CUTTER_IN_B, OUTPUT);
    hw_pwm_setup(PIN_CUTTER_ENA);
    cutter_off();

    // Configure MKS SERVO42C over UART (blocking calls are OK in setup).
    // Wait 500 ms for the driver to boot after power-on.
    delay(500);
    stepper_setEnable(true, true);       // Enable driver
    stepper_setAcceleration(STEPPER_ACC);// Set acceleration ramp
    stepper_setMicrosteps(STEPPER_MSTEP);// Set microstep resolution
    stepper_setProtection(true);         // Enable stall detection
    stepper_setEnable(false, true);      // Disable driver (idle until Start)
}

// Kick off one pulsed feed segment (only used if mc_pulsedMode is true).
// Calculates the pulse count for MC_PULSE_DIST_MM and starts the stepper.
static void startPulseFeed() {
    uint32_t pulses = mmToPulses(MC_PULSE_DIST_MM);
    stepper_startFD(settings.feedSpeedMmS, true, pulses);
    mc_pulseFeeding    = true;
    mc_pulseStartMs    = millis();
    // Estimate how long this feed will take (with 2.5x safety margin)
    mc_pulseFeedTimeMs = (uint32_t)(MC_PULSE_DIST_MM / settings.feedSpeedMmS * 1000.0f * 2.5f) + 500;
}

// Start both motors: turn on 12 V MOSFET, start stepper feeding forward
// at the configured speed, and start the cutter with soft-start ramp.
void motor_start() {
    mc_reversing = false;
    digitalWrite(PIN_MOSFET, HIGH);  // Enable 12 V to motor drivers
    delay(100);                      // Let power stabilize

    if (mc_pulsedMode) {
        stepper_setEnable(true);
        startPulseFeed();
    } else {
        // Continuous mode: issue a very long FD command; re-issue timer keeps it alive
        stepper_startFD(settings.feedSpeedMmS, true, STEPPER_BIG_PULSE_COUNT);
    }

    // Start cutter motor with soft-start (PWM ramps from 0 to target)
    mc_cutTgtPWM  = calc_cutter_pwm();
    mc_cutCurPWM  = 0;
    mc_cutRunning = true;
    mc_cutStartMs = millis();
    cutter_dir(true);  // Forward direction for cutting
}

// Full stop: halt stepper, stop cutter, cut 12 V power.
void motor_stop() {
    stepper_fullStop();
    cutter_off();
    mc_reversing = false;
    digitalWrite(PIN_MOSFET, LOW);  // Cut 12 V to motor drivers
}

// Pause: stop stepper and cutter but leave MOSFET on (faster resume).
void motor_pause() {
    stepper_sendStop();
    mc_stepperRunning = false;
    mc_feedSpeedMmS   = 0;
    stepper_setEnable(false);
    cutter_off();
}

// Resume from a snapshot: restore position/phase, restart both motors.
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

// Insert filament mode: run stepper slowly (5 mm/s) forward with no
// cutter, so the user can feed new filament through the puller.
void motor_set_insert_mode(bool en) {
    if (en) {
        mc_reversing = false;
        mc_stallFlag = false;
        digitalWrite(PIN_MOSFET, HIGH);
        delay(100);
        stepper_startFD(INSERT_FEED_SPEED_MMS, true, STEPPER_BIG_PULSE_COUNT);
        cutter_off();  // No cutting during insertion
    } else {
        stepper_fullStop();
        cutter_off();
        digitalWrite(PIN_MOSFET, LOW);
    }
}

// Reverse the stepper a fixed distance to try to clear a filament jam.
// The cutter is stopped during reversal.
void motor_reverse(float mm) {
    mc_reversing      = true;
    mc_revDistMm      = mm;
    mc_reverseStartMs = millis();
    mc_reverseTimeMs  = (uint32_t)(mm / JAM_REVERSE_SPEED_MMS * 1000.0f * 2.5f) + 500;

    cutter_off();

    uint32_t pulses = mmToPulses(mm);
    stepper_startFD(JAM_REVERSE_SPEED_MMS, false, pulses);  // false = reverse direction
}

// Check if the timed reverse move has completed.
bool motor_reverse_complete() {
    if (!mc_reversing) return true;
    return (millis() - mc_reverseStartMs) >= mc_reverseTimeMs;
}

// Cutter reverse mode state (for unclogging the cutter blade manually)
static bool     mc_cutRevActive   = false;
static uint32_t mc_cutRevStartMs  = 0;
static const uint32_t MC_CUT_REV_RAMP_MS = 2000;  // 2-second ramp to full power

// Run the cutter in reverse (for unclogging). Ramps up over 2 seconds.
// Only available from IDLE state via the menu.
void motor_cutter_reverse(bool en) {
    if (en) {
        digitalWrite(PIN_MOSFET, HIGH);
        delay(100);
        cutter_dir(false);               // Reverse direction
        hw_pwm_write(PIN_CUTTER_ENA, 0); // Start at 0 (ramp will increase)
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

// Called every loop() iteration. Handles three timing tiers:
//   1. Fast tick (1 ms) — position tracking, cutter ramp, reverse completion
//   2. Reissue tick (8 s) — re-send FD command to prevent stepper timeout
//   3. Poll tick (1.5 s) — check MKS stall status for jam detection
void motor_control_update() {
    uint32_t now = millis();

    // ── Fast tick (~1 ms): position tracking + cutter soft-start ────
    if (now - mc_lastFastMs >= MOTOR_INTERVAL_MS) {
        float dt = (float)(now - mc_lastFastMs) / 1000.0f;
        mc_lastFastMs = now;

        // Open-loop filament position estimate: integrate speed × time.
        // No encoder feedback — this is an estimate for the UI display.
        if (mc_stepperRunning && mc_feedSpeedMmS > 0) {
            float dMm = mc_feedSpeedMmS * dt;
            if (mc_feedFwd) mc_filPosMm += dMm;
            else            mc_filPosMm -= dMm;
        }

        // Cutter soft-start: linearly ramp PWM from 0 to target over
        // CUTTER_SOFT_START_MS (500 ms) to avoid current spikes.
        if (mc_cutRunning && mc_cutCurPWM < mc_cutTgtPWM) {
            uint32_t el = now - mc_cutStartMs;
            mc_cutCurPWM = (el < CUTTER_SOFT_START_MS)
                ? (uint8_t)((float)mc_cutTgtPWM * el / CUTTER_SOFT_START_MS)
                : mc_cutTgtPWM;
            hw_pwm_write(PIN_CUTTER_ENA, mc_cutCurPWM);
        }

        // Cutter reverse ramp (for unclogging): ramps to full power over 2 s.
        if (mc_cutRevActive) {
            uint32_t el = now - mc_cutRevStartMs;
            uint8_t pwm = (el < MC_CUT_REV_RAMP_MS)
                ? (uint8_t)((float)CUTTER_MAX_PWM * el / MC_CUT_REV_RAMP_MS)
                : CUTTER_MAX_PWM;
            hw_pwm_write(PIN_CUTTER_ENA, pwm);
        }

        // Open-loop cutter phase tracking: keeps a running estimate of
        // where the blade is in its rotation cycle. Used for snapshot/resume.
        if (mc_cutRunning && mc_feedSpeedMmS > 0 && settings.pelletLenMm > 0) {
            float periodMs = 1000.0f * settings.pelletLenMm / mc_feedSpeedMmS;
            mc_cutPhaseMs += dt * 1000.0f;
            while (mc_cutPhaseMs >= periodMs) mc_cutPhaseMs -= periodMs;
        }

        // Jam-clearing reverse completion: when the estimated reverse time
        // has elapsed, stop the stepper, update position, and fire JAM_CLEARED.
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

        // Pulsed feed cycle (only active if mc_pulsedMode is true):
        // Alternates between feeding MC_PULSE_DIST_MM and waiting MC_PULSE_WAIT_MS.
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

    // ── Re-issue FD command every 8 s (continuous mode only) ──
    // The MKS SERVO42C counts down its pulse count. Without re-issue,
    // the motor would eventually stop when pulses run out.
    if (mc_stepperRunning && !mc_reversing && !mc_pulsedMode &&
        (now - mc_lastReissueMs >= STEPPER_REISSUE_MS)) {
        mc_lastReissueMs = now;
        mks_flush();
        stepper_sendFD(mc_stepperDir, mc_stepperSpeed, STEPPER_BIG_PULSE_COUNT);
    }

    // ── Stall detection poll (~every 1.5 s) ──
    // Asks the MKS SERVO42C if it has detected a stall (motor blocked).
    // If stall detected while RUNNING, pushes a JAM_DETECTED event.
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

// =====================================================================
//  STATE MACHINE — 7-state FSM with E-stop override
// =====================================================================
//
//  The state machine processes events from the queue and manages
//  transitions between system states. Each transition triggers the
//  appropriate motor commands, buzzer feedback, and logging.
//
//  E-Stop overrides any state → ERROR immediately.
//  Filament runout from RUNNING or INSERT_FILAMENT → RUNOUT.
//  Jam detection in RUNNING → JAM_CLEARING (up to JAM_RETRY_MAX times).
//  PAUSED → IDLE after IDLE_TIMEOUT_MS (5 min auto-shutdown).

static uint32_t    sm_lastTickMs       = 0;      // Rate limiter for state machine tick
static uint32_t    sm_pauseStartMs     = 0;      // When we entered PAUSED (for idle timeout)
static uint32_t    sm_accumulatedRunMs = 0;       // Total run time across pause/resume cycles
static uint32_t    sm_currentRunStart  = 0;       // When the current RUNNING segment started
static uint32_t    sm_lastStatSecMs    = 0;       // Last 1-second stats tick
static uint8_t     sm_jamRetries       = 0;       // Consecutive jam-clear attempts this run
static SystemState sm_prevState        = SystemState::IDLE; // State before last transition

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

// =====================================================================
//  UI — SH1106 128x64 OLED display + 3-button menu navigation
// =====================================================================
//
//  Hardware: SH1106 128x64 monochrome OLED, connected via I2C:
//    SDA = GP16 (with 4.7 kΩ external pull-up to 3.3 V)
//    SCL = GP17 (with 4.7 kΩ external pull-up to 3.3 V)
//    VCC = 3.3 V (or 5 V if module has onboard regulator)
//    GND = common ground
//
//  Navigation (3 buttons):
//    UP (GP6)     — scroll up / increment value
//    DOWN (GP7)   — scroll down / decrement value
//    SELECT (GP10)— short press: confirm / enter submenu
//                   long press (800 ms): return to HOME from any screen
//
//  Screen auto-dims to contrast 0 after 60 s of no input.
//  Any button press wakes the display; the first press only wakes
//  (doesn't trigger a menu action).

static U8G2_SH1106_128X64_NONAME_F_HW_I2C u8g2(U8G2_R0, U8X8_PIN_NONE);
static uint32_t ui_lastDrawMs  = 0;
static uint32_t ui_lastInputMs = 0;
static bool     ui_dimmed      = false;
static uint8_t  ui_menuIdx     = 0;

static const char* stateStr(SystemState s) {
    switch (s) {
        case SystemState::IDLE:            return "IDLE";
        case SystemState::RUNNING:         return "RUNNING";
        case SystemState::PAUSED:          return "PAUSED";
        case SystemState::INSERT_FILAMENT: return "INSERT FIL.";
        case SystemState::ERROR:           return "ERROR";
        case SystemState::JAM_CLEARING:    return "JAM CLEAR";
        case SystemState::RUNOUT:          return "RUNOUT";
        default:                           return "???";
    }
}

static void fmtTime(uint32_t ms, char* buf, uint8_t len) {
    uint32_t sec = ms / 1000;
    unsigned h = sec / 3600, m = (sec % 3600) / 60, s = sec % 60;
    snprintf(buf, len, "%02u:%02u:%02u", h, m, s);
}

static void drawInvStr(uint8_t x, uint8_t y, const char* s) {
    u8g2.setDrawColor(1);
    u8g2.drawBox(0, y - 10, 128, 12);
    u8g2.setDrawColor(0);
    u8g2.drawStr(x + 2, y, s);
    u8g2.setDrawColor(1);
}

static void drawHome() {
    char buf[22];
    u8g2.setFont(u8g2_font_6x10_tf);
    u8g2.drawStr(0, 10, stateStr(systemState));
    fmtTime(state_machine_get_run_time_ms(), buf, sizeof(buf));
    u8g2.drawStr(80, 10, buf);
    float meters = motor_get_filament_pos_mm() / 1000.0f;
    snprintf(buf, sizeof(buf), "Fed:%5.2fm %4.1fp/s", (double)meters, (double)motor_get_cut_frequency());
    u8g2.drawStr(0, 24, buf);
    snprintf(buf, sizeof(buf), "Spd:%4.1f mm/s", (double)motor_get_feed_rate());
    u8g2.drawStr(0, 38, buf);
    u8g2.setFont(u8g2_font_5x7_tf);
    uint8_t ix = 0;
    if (!sensors_filament_present()) { u8g2.drawStr(ix, 52, "[R]"); ix += 20; }
    if (motor_is_stalled())          { u8g2.drawStr(ix, 52, "[J]"); ix += 20; }
    if (sensors_estop_active())      { u8g2.drawStr(ix, 52, "[E]"); ix += 20; }
    u8g2.drawStr(92, 52, "v" FW_VERSION);
    u8g2.drawStr(0, 63, "SEL=Menu");
}

#define MAIN_MENU_ITEMS 7
#define MENU_VISIBLE     4

static const char* mainMenuLabel(uint8_t i) {
    if (i == 0) {
        switch (systemState) {
            case SystemState::IDLE:            return "Start";
            case SystemState::RUNNING:         return "Pause";
            case SystemState::PAUSED:          return "Resume";
            case SystemState::ERROR:           return "Clear Error";
            case SystemState::RUNOUT:          return "Reset";
            case SystemState::INSERT_FILAMENT: return "Exit Insert";
            default:                           return "---";
        }
    }
    if (i == 2) return cutterReversing ? "Stop Reverse" : "Reverse Cutter";
    static const char* labels[] = { nullptr, "Insert Filament", nullptr, "Settings", "Warnings / Logs", "Maintenance", "System Info" };
    return labels[i];
}

static void drawMainMenu() {
    u8g2.setFont(u8g2_font_6x10_tf);
    u8g2.drawStr(30, 10, "== MENU ==");
    u8g2.drawHLine(0, 12, 128);
    uint8_t offset = (ui_menuIdx >= MENU_VISIBLE) ? ui_menuIdx - MENU_VISIBLE + 1 : 0;
    for (uint8_t i = 0; i < MENU_VISIBLE && (i + offset) < MAIN_MENU_ITEMS; i++) {
        uint8_t y = 25 + i * 12;
        const char* label = mainMenuLabel(i + offset);
        if ((i + offset) == ui_menuIdx) drawInvStr(0, y, label);
        else                            u8g2.drawStr(2, y, label);
    }
}

#define SETTINGS_ITEMS 3

static void drawSettings() {
    u8g2.setFont(u8g2_font_6x10_tf);
    u8g2.drawStr(18, 10, "== SETTINGS ==");
    u8g2.drawHLine(0, 12, 128);
    static char pBuf[22], fBuf[22];
    snprintf(pBuf, sizeof(pBuf), "Pellet: %.1fmm",  (double)settings.pelletLenMm);
    snprintf(fBuf, sizeof(fBuf), "Feed: %.0fmm/s",  (double)settings.feedSpeedMmS);
    const char* labels[SETTINGS_ITEMS] = { pBuf, fBuf, "< Back" };
    for (uint8_t i = 0; i < SETTINGS_ITEMS; i++) {
        uint8_t y = 25 + i * 12;
        if (i == ui_menuIdx) drawInvStr(0, y, labels[i]);
        else                 u8g2.drawStr(2, y, labels[i]);
    }
}

static void drawEditPellet() {
    char buf[22];
    u8g2.setFont(u8g2_font_6x10_tf);
    u8g2.drawStr(10, 10, "Pellet Length");
    u8g2.drawHLine(0, 12, 128);
    snprintf(buf, sizeof(buf), "%.1f mm", (double)settings.pelletLenMm);
    u8g2.setFont(u8g2_font_10x20_tf);
    u8g2.drawStr(30, 38, buf);
    u8g2.setFont(u8g2_font_5x7_tf);
    snprintf(buf, sizeof(buf), "Cut freq: %.1f Hz", (double)(settings.feedSpeedMmS / settings.pelletLenMm));
    u8g2.drawStr(0, 52, buf);
    u8g2.drawStr(0, 63, "UP/DN adj  SEL=OK");
}

static void drawEditFeed() {
    char buf[22];
    u8g2.setFont(u8g2_font_6x10_tf);
    u8g2.drawStr(20, 10, "Feed Speed");
    u8g2.drawHLine(0, 12, 128);
    snprintf(buf, sizeof(buf), "%.0f mm/s", (double)settings.feedSpeedMmS);
    u8g2.setFont(u8g2_font_10x20_tf);
    u8g2.drawStr(22, 38, buf);
    u8g2.setFont(u8g2_font_5x7_tf);
    snprintf(buf, sizeof(buf), "Cut freq: %.1f Hz", (double)(settings.feedSpeedMmS / settings.pelletLenMm));
    u8g2.drawStr(0, 52, buf);
    u8g2.drawStr(0, 63, "UP/DN adj  SEL=OK");
}

static void drawWarnings() {
    u8g2.setFont(u8g2_font_6x10_tf);
    u8g2.drawStr(12, 10, "== WARNINGS ==");
    u8g2.drawHLine(0, 12, 128);
    u8g2.setFont(u8g2_font_5x7_tf);
    uint8_t y = 23;
    if (motor_is_stalled())          { u8g2.drawStr(0, y, "! Jam detected");    y += 9; }
    if (!sensors_filament_present()) { u8g2.drawStr(0, y, "! Filament runout"); y += 9; }
    if (sensors_estop_active())      { u8g2.drawStr(0, y, "! E-Stop active");   y += 9; }
    if (y == 23) { u8g2.drawStr(0, y, "No active warnings"); y += 9; }
    y += 2;
    u8g2.drawHLine(0, y - 3, 128);
    uint8_t logCount = eventLog.count();
    uint8_t show = (logCount < 3) ? logCount : 3;
    for (uint8_t i = 0; i < show && y < 64; i++) {
        const LogEntry& le = eventLog.get(i);
        char buf[22];
        snprintf(buf, sizeof(buf), "%lus: %s", (unsigned long)le.timestampSec, le.message);
        u8g2.drawStr(0, y, buf);
        y += 8;
    }
}

static void drawMaintenance() {
    char buf[22];
    u8g2.setFont(u8g2_font_6x10_tf);
    u8g2.drawStr(6, 10, "== MAINTENANCE ==");
    u8g2.drawHLine(0, 12, 128);
    unsigned long hrs = stats.totalRuntimeSec / 3600, mins = (stats.totalRuntimeSec % 3600) / 60;
    snprintf(buf, sizeof(buf), "Runtime: %luh %lum", hrs, mins);
    u8g2.drawStr(0, 28, buf);
    snprintf(buf, sizeof(buf), "Filament: %.1fm", (double)((float)stats.totalFilamentMm / 1000.0f));
    u8g2.drawStr(0, 40, buf);
    snprintf(buf, sizeof(buf), "Jams: %u  Errs: %u", stats.jamCount, stats.errorCount);
    u8g2.drawStr(0, 52, buf);
}

static void drawSystemInfo() {
    char buf[22];
    u8g2.setFont(u8g2_font_6x10_tf);
    u8g2.drawStr(6, 10, "== SYSTEM INFO ==");
    u8g2.drawHLine(0, 12, 128);
    u8g2.drawStr(0, 28, "FW:  " FW_VERSION);
    u8g2.drawStr(0, 40, "MCU: RP2040");
    snprintf(buf, sizeof(buf), "State: %s", stateStr(systemState));
    u8g2.drawStr(0, 52, buf);
}

static void handleInput() {
    bool up   = sensors_btn_up_pressed();
    bool down = sensors_btn_down_pressed();
    bool sel  = sensors_btn_select_pressed();
    bool selL = sensors_btn_select_long();
    if (!(up || down || sel || selL)) return;

    ui_lastInputMs = millis();
    if (ui_dimmed) {
        u8g2.setContrast(OLED_FULL_CONTRAST);
        ui_dimmed = false;
        return;
    }
    buzzer_play(BuzzerPattern::CLICK);

    if (selL && currentScreen != MenuScreen::HOME) {
        currentScreen = MenuScreen::HOME;
        ui_menuIdx = 0;
        return;
    }

    switch (currentScreen) {
    case MenuScreen::HOME:
        if (sel) { currentScreen = MenuScreen::MAIN_MENU; ui_menuIdx = 0; }
        break;
    case MenuScreen::MAIN_MENU:
        if (up   && ui_menuIdx > 0)                   ui_menuIdx--;
        if (down && ui_menuIdx < MAIN_MENU_ITEMS - 1) ui_menuIdx++;
        if (sel) {
            switch (ui_menuIdx) {
                case 0:
                    switch (systemState) {
                        case SystemState::IDLE:            eventQueue.push(Event::START_CMD);      break;
                        case SystemState::RUNNING:         eventQueue.push(Event::PAUSE_CMD);      break;
                        case SystemState::PAUSED:          eventQueue.push(Event::RESUME_CMD);     break;
                        case SystemState::ERROR:
                        case SystemState::RUNOUT:          eventQueue.push(Event::CLEAR_ERROR);    break;
                        case SystemState::INSERT_FILAMENT: eventQueue.push(Event::EXIT_INSERT_CMD);break;
                        default: break;
                    }
                    currentScreen = MenuScreen::HOME;
                    break;
                case 1: eventQueue.push(Event::INSERT_FILAMENT_CMD); currentScreen = MenuScreen::HOME; break;
                case 2:
                    if (cutterReversing) {
                        eventQueue.push(Event::STOP_REVERSE_CUTTER_CMD);
                        cutterReversing = false;
                    } else {
                        eventQueue.push(Event::REVERSE_CUTTER_CMD);
                        cutterReversing = true;
                    }
                    currentScreen = MenuScreen::HOME;
                    break;
                case 3: currentScreen = MenuScreen::SETTINGS;    ui_menuIdx = 0; break;
                case 4: currentScreen = MenuScreen::WARNINGS;    break;
                case 5: currentScreen = MenuScreen::MAINTENANCE; break;
                case 6: currentScreen = MenuScreen::SYSTEM_INFO; break;
            }
        }
        break;
    case MenuScreen::SETTINGS:
        if (up   && ui_menuIdx > 0)                  ui_menuIdx--;
        if (down && ui_menuIdx < SETTINGS_ITEMS - 1) ui_menuIdx++;
        if (sel) {
            switch (ui_menuIdx) {
                case 0: currentScreen = MenuScreen::SETTINGS_PELLET_LEN;  break;
                case 1: currentScreen = MenuScreen::SETTINGS_FEED_SPEED;  break;
                case 2: currentScreen = MenuScreen::MAIN_MENU; ui_menuIdx = 3; break;
            }
        }
        break;
    case MenuScreen::SETTINGS_PELLET_LEN:
        if (up)   { settings.pelletLenMm += PELLET_LEN_STEP_MM; if (settings.pelletLenMm > MAX_PELLET_LEN_MM) settings.pelletLenMm = MAX_PELLET_LEN_MM; }
        if (down) { settings.pelletLenMm -= PELLET_LEN_STEP_MM; if (settings.pelletLenMm < MIN_PELLET_LEN_MM) settings.pelletLenMm = MIN_PELLET_LEN_MM; }
        if (sel)  { storage_mark_settings_dirty(); currentScreen = MenuScreen::SETTINGS; ui_menuIdx = 0; }
        break;
    case MenuScreen::SETTINGS_FEED_SPEED:
        if (up)   { settings.feedSpeedMmS += FEED_SPEED_STEP_MMS; if (settings.feedSpeedMmS > MAX_FEED_SPEED_MMS) settings.feedSpeedMmS = MAX_FEED_SPEED_MMS; }
        if (down) { settings.feedSpeedMmS -= FEED_SPEED_STEP_MMS; if (settings.feedSpeedMmS < MIN_FEED_SPEED_MMS) settings.feedSpeedMmS = MIN_FEED_SPEED_MMS; }
        if (sel)  { storage_mark_settings_dirty(); currentScreen = MenuScreen::SETTINGS; ui_menuIdx = 1; }
        break;
    case MenuScreen::WARNINGS:
    case MenuScreen::MAINTENANCE:
    case MenuScreen::SYSTEM_INFO:
        if (sel || selL) { currentScreen = MenuScreen::MAIN_MENU; ui_menuIdx = 4; }
        break;
    }
}

void ui_init() {
    Wire.setSDA(PIN_OLED_SDA);
    Wire.setSCL(PIN_OLED_SCL);
    u8g2.begin();
    u8g2.setContrast(OLED_FULL_CONTRAST);
    u8g2.clearBuffer();
    u8g2.setFont(u8g2_font_7x14B_tf);
    u8g2.drawStr(18, 28, "PELLETIZER");
    u8g2.setFont(u8g2_font_6x10_tf);
    u8g2.drawStr(18, 46, "Pelletizer " FW_VERSION);
    u8g2.sendBuffer();
    delay(1500);
    ui_lastInputMs = millis();
}

void ui_update() {
    uint32_t now = millis();
    handleInput();
    if (!ui_dimmed && (now - ui_lastInputMs) > settings.screenTimeoutMs) {
        u8g2.setContrast(OLED_DIM_CONTRAST);
        ui_dimmed = true;
    }
    if (now - ui_lastDrawMs < UI_INTERVAL_MS) return;
    ui_lastDrawMs = now;
    u8g2.clearBuffer();
    switch (currentScreen) {
        case MenuScreen::HOME:               drawHome();        break;
        case MenuScreen::MAIN_MENU:          drawMainMenu();    break;
        case MenuScreen::SETTINGS:           drawSettings();    break;
        case MenuScreen::SETTINGS_PELLET_LEN:drawEditPellet();  break;
        case MenuScreen::SETTINGS_FEED_SPEED:drawEditFeed();    break;
        case MenuScreen::WARNINGS:           drawWarnings();    break;
        case MenuScreen::MAINTENANCE:        drawMaintenance(); break;
        case MenuScreen::SYSTEM_INFO:        drawSystemInfo();  break;
    }
    u8g2.sendBuffer();
}

// =====================================================================
//  SETUP & LOOP
// =====================================================================
//
//  Power-on sequence:
//  1. MOSFET OFF immediately (motors unpowered until explicitly started)
//  2. Load settings/stats from EEPROM (or defaults if CRC fails)
//  3. Initialize sensors (buttons, E-Stop ISR, filament sensor)
//  4. Initialize motor control (UART to MKS SERVO42C, L298N PWM)
//  5. Initialize buzzer, NeoPixel LED
//  6. Initialize OLED (splash screen for 1.5 s)
//  7. Initialize state machine to IDLE
//
//  Main loop is cooperative multitasking — each subsystem's update()
//  function is called every iteration and uses internal timers to
//  self-throttle. No interrupts are used except for the E-Stop ISR.
//  Typical loop time: < 1 ms.

void setup() {
    // Ensure motors are OFF before anything else
    pinMode(PIN_MOSFET, OUTPUT);
    digitalWrite(PIN_MOSFET, LOW);

    storage_init();         // Load settings & stats from EEPROM
    sensors_init();         // Configure button/sensor pins and E-Stop ISR
    motor_control_init();   // UART init + MKS SERVO42C configuration
    buzzer_init();          // Configure buzzer pin
    led_init();             // Initialize WS2812B NeoPixel
    ui_init();              // I2C + OLED init + splash screen
    state_machine_init();   // Set initial state to IDLE
}

void loop() {
    sensors_update();          // Poll buttons, E-Stop, filament sensor
    state_machine_update();    // Process events, manage state transitions
    motor_control_update();    // Position tracking, FD reissue, stall poll
    ui_update();               // Handle input + redraw OLED
    buzzer_update();           // Advance buzzer tone patterns
    led_update();              // Update NeoPixel color based on state
    storage_update();          // Flush dirty settings/stats to EEPROM
}