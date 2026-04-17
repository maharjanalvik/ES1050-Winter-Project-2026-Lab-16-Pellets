#include "project.h"

EventQueue    eventQueue;
EventLog      eventLog;
Settings      settings;
Stats         stats;
SystemState   systemState   = SystemState::IDLE;
MenuScreen    currentScreen = MenuScreen::HOME;
MotorSnapshot motorSnapshot = {0, 0, 0};
uint32_t      runStartMs    = 0;
bool          cutterReversing = false;
