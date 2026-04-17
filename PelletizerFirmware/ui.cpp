#include "project.h"

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
