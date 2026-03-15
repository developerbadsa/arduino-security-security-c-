#include "security.h"

#include "../actuators/actuators.h"
#include "../core/command_processor.h"

namespace {

void triggerAlarm(const __FlashStringHelper* reason) {
  const uint32_t now = millis();
  xSemaphoreTake(stateMutex, portMAX_DELAY);
  const bool inCooldown = (now < alarmCooldownUntil);
  if (!inCooldown) {
    alarmLatched = true;
    alarmCooldownUntil = now + ALARM_COOLDOWN_MS;
  }
  xSemaphoreGive(stateMutex);
  if (inCooldown) return;

  buzzerStartAlarm(ALARM_BUZZ_MS);
  if (DBG_SEC) {
    Serial.print(F("[SEC] ALARM: "));
    Serial.println(reason);
  }
  queueReportFromLoop("ALARM_TRIGGERED", "");
}

}  // namespace

void updateSecurityAlarm() {
  if (!imuReady) return;

  xSemaphoreTake(stateMutex, portMAX_DELAY);
  const bool isArmed = armed;
  const bool isLocked = locked;
  xSemaphoreGive(stateMutex);

  if (!isArmed || !isLocked) return;

  const bool spdValid = gps.speed.isValid() && (gps.speed.age() <= 3000);
  const float spd = spdValid ? gps.speed.kmph() : 0.0f;
  if (spdValid && spd >= 2.0f) {
    knockCount = 0;
    tamperMotionSince = 0;
    touchSince = 0;
    return;
  }

  const uint32_t now = millis();

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
