#include "security.h"

#include "../actuators/actuators.h"
#include "../core/command_processor.h"
#include "../logging/logger.h"
#include "../sensors/sensors.h"

namespace {

void triggerAlarm(const __FlashStringHelper* reason) {
  const uint32_t now = millis();
  bool inCooldown = true;
  if (xSemaphoreTake(stateMutex, pdMS_TO_TICKS(20)) == pdTRUE) {
    inCooldown = (now < alarmCooldownUntil);
    if (!inCooldown) {
      alarmLatched = true;
      alarmCooldownUntil = now + ALARM_COOLDOWN_MS;
    }
    xSemaphoreGive(stateMutex);
  }
  if (inCooldown) return;

  buzzerStartAlarm(ALARM_BUZZ_MS);
  if (DBG_SEC) {
    String msg = F("ALARM: ");
    msg += reason;
    logPrintln("SEC", msg);
  }
  queueReportFromLoop("ALARM_TRIGGERED", "");
}

}  // namespace

void updateSecurityAlarm() {
  bool isArmed = false;
  bool isLocked = false;
  float filteredSpd = 0.0f;
  if (xSemaphoreTake(stateMutex, pdMS_TO_TICKS(20)) == pdTRUE) {
    isArmed = armed;
    isLocked = locked;
    filteredSpd = getFilteredSpeedKmph_locked();
    xSemaphoreGive(stateMutex);
  }

  if (!isArmed || !isLocked) return;

  const uint32_t now = millis();
  bool isSettling = false;
  if (xSemaphoreTake(stateMutex, pdMS_TO_TICKS(20)) == pdTRUE) {
    isSettling = (now < armSettleUntil);
    xSemaphoreGive(stateMutex);
  }
  if (isSettling) {
    knockCount = 0;
    firstKnockAt = 0;
    tamperMotionSince = 0;
    touchSince = 0;
    return;
  }

  if (filteredSpd >= MIN_SPEED_KMPH) {
    knockCount = 0;
    firstKnockAt = 0;
    tamperMotionSince = 0;
    touchSince = 0;
    triggerAlarm(F("GPS MOTION"));
    return;
  }

  if (!imuReady) return;

  const bool touchNow = (lastADeltaG > TOUCH_DELTA_G) || (lastGMag > TOUCH_GYRO);
  if (touchNow) {
    if (!touchSince) touchSince = now;
    if ((now - touchSince >= TOUCH_TRIGGER_MS) && (now - lastTouchFireAt >= TOUCH_DEBOUNCE_MS)) {
      lastTouchFireAt = now;
      touchSince = 0;
      triggerAlarm(F("TOUCH"));
      return;
    }
  } else {
    touchSince = 0;
  }

  const bool hit = (lastADeltaG > KNOCK_DELTA_G) || (lastGMag > KNOCK_GYRO);
  if (hit && (now - lastKnockAt >= KNOCK_DEBOUNCE_MS)) {
    lastKnockAt = now;
    if (!knockCount) firstKnockAt = now;
    if (++knockCount >= KNOCK_COUNT_REQ && (now - firstKnockAt <= KNOCK_WINDOW_MS)) {
      knockCount = 0;
      triggerAlarm(F("KNOCK"));
      return;
    }
  }
  if (knockCount && (now - firstKnockAt > KNOCK_WINDOW_MS)) knockCount = 0;

  const bool motionNow = (lastADeltaG > 0.10f) || (lastGMag > 0.35f) || imuMoving;
  if (motionNow) {
    if (!tamperMotionSince) tamperMotionSince = now;
    if (now - tamperMotionSince >= TAMPER_MOTION_MS) {
      tamperMotionSince = 0;
      triggerAlarm(F("SUSTAINED MOTION"));
      return;
    }
  } else {
    tamperMotionSince = 0;
  }
}
