#include "project.h"

static bool     stor_settingsDirty = false;
static bool     stor_statsDirty    = false;
static uint32_t stor_lastFlushMs   = 0;

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
