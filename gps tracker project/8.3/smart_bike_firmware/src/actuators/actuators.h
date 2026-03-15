#pragma once

#include "../core/app_state.h"

void relayOn();
void relayOff();
void setLed(bool on);
void buzzerStartAlarm(uint32_t durationMs);
void buzzerStartLockPattern();
void buzzerStartArmPattern();
void buzzerStartUnlockPattern();
void buzzerStopAll();
void buzzerUpdate();
