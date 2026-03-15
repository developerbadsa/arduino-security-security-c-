#include "actuators.h"

namespace {

enum BuzzerMode : uint8_t {
  BUZ_OFF = 0,
  BUZ_ALARM = 1,
  BUZ_PATTERN = 2
};

constexpr uint16_t BEEP_GAP_MS = 200;

const BeepStep PAT_LOCK[] = {{2000, 1}};
const BeepStep PAT_ARM[] = {{500, 1}, {BEEP_GAP_MS, 0}, {500, 1}, {BEEP_GAP_MS, 0}, {2000, 1}};
const BeepStep PAT_UNLOCK[] = {{3000, 1}, {BEEP_GAP_MS, 0}, {500, 1}, {BEEP_GAP_MS, 0}, {500, 1}};

BuzzerMode buzMode = BUZ_OFF;
const BeepStep* curPat = nullptr;
uint8_t curPatLen = 0;
uint8_t curIdx = 0;
uint32_t stepUntil = 0;
uint32_t alarmUntil = 0;

void buzzerOn() {
  digitalWrite(BUZZER_RELAY_PIN, RELAY_ACTIVE_LOW ? LOW : HIGH);
}

void buzzerOff() {
  digitalWrite(BUZZER_RELAY_PIN, RELAY_ACTIVE_LOW ? HIGH : LOW);
}

void buzApply(uint8_t on) {
  if (on) buzzerOn();
  else buzzerOff();
}

void buzzerStartPattern(const BeepStep* pat, uint8_t len) {
  if (!pat || !len || buzMode == BUZ_ALARM) return;
  buzMode = BUZ_PATTERN;
  curPat = pat;
  curPatLen = len;
  curIdx = 0;
  buzApply(curPat[0].on);
  stepUntil = millis() + curPat[0].ms;
}

}  // namespace

void relayOn() {
  digitalWrite(RELAY_PIN, RELAY_ACTIVE_LOW ? LOW : HIGH);
  if (stateMutex && xSemaphoreTake(stateMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
    locked = true;
    xSemaphoreGive(stateMutex);
  }
}

void relayOff() {
  digitalWrite(RELAY_PIN, RELAY_ACTIVE_LOW ? HIGH : LOW);
  if (stateMutex && xSemaphoreTake(stateMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
    locked = false;
    xSemaphoreGive(stateMutex);
  }
}

void setLed(bool on) {
  if (LED_PIN >= 0) digitalWrite(LED_PIN, on ? HIGH : LOW);
  if (ONBOARD_LED_PIN >= 0) digitalWrite(ONBOARD_LED_PIN, on ? HIGH : LOW);
}

void buzzerStartAlarm(uint32_t durationMs) {
  buzMode = BUZ_ALARM;
  alarmUntil = millis() + durationMs;
  buzApply(1);
}

void buzzerStartLockPattern() {
  buzzerStartPattern(PAT_LOCK, static_cast<uint8_t>(sizeof(PAT_LOCK) / sizeof(PAT_LOCK[0])));
}

void buzzerStartArmPattern() {
  buzzerStartPattern(PAT_ARM, static_cast<uint8_t>(sizeof(PAT_ARM) / sizeof(PAT_ARM[0])));
}

void buzzerStartUnlockPattern() {
  buzzerStartPattern(PAT_UNLOCK, static_cast<uint8_t>(sizeof(PAT_UNLOCK) / sizeof(PAT_UNLOCK[0])));
}

void buzzerStopAll() {
  buzMode = BUZ_OFF;
  curPat = nullptr;
  curPatLen = 0;
  curIdx = 0;
  alarmUntil = 0;
  buzApply(0);
}

void buzzerUpdate() {
  const uint32_t now = millis();
  if (buzMode == BUZ_ALARM) {
    if (now >= alarmUntil) buzzerStopAll();
    return;
  }
  if (buzMode == BUZ_PATTERN) {
    if (now < stepUntil) return;
    if (++curIdx >= curPatLen) {
      buzzerStopAll();
      return;
    }
    buzApply(curPat[curIdx].on);
    stepUntil = now + curPat[curIdx].ms;
  }
}
