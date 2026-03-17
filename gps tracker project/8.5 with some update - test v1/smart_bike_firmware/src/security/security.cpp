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
  logPrintln("SEC", reason);
  queueReportFromLoop("ALARM_TRIGGERED", "");
}

}  // namespace

void updateSecurityAlarm() {
  const uint32_t now = millis();
  bool isArmed = false;
  bool isLocked = false;
  float filteredSpd = 0.0f;
  bool isSettling = false;
  uint8_t localKnockCount = 0;
  uint32_t localFirstKnockAt = 0;
  uint32_t localLastKnockAt = 0;
  uint32_t localTouchSince = 0;
  uint32_t localLastTouchFireAt = 0;
  uint32_t localTamperMotionSince = 0;

  if (xSemaphoreTake(stateMutex, pdMS_TO_TICKS(20)) != pdTRUE) return;

  isArmed = armed;
  isLocked = locked;
  filteredSpd = getFilteredSpeedKmph_locked();
  isSettling = (now < armSettleUntil);
  localKnockCount = knockCount;
  localFirstKnockAt = firstKnockAt;
  localLastKnockAt = lastKnockAt;
  localTouchSince = touchSince;
  localLastTouchFireAt = lastTouchFireAt;
  localTamperMotionSince = tamperMotionSince;
  xSemaphoreGive(stateMutex);

  if (!isArmed || !isLocked) return;

  bool writeBackNeeded = false;
  bool alarmTriggered = false;
  const __FlashStringHelper* alarmReason = nullptr;

  if (isSettling) {
    localKnockCount = 0;
    localFirstKnockAt = 0;
    localTamperMotionSince = 0;
    localTouchSince = 0;
    writeBackNeeded = true;
  } else if (filteredSpd >= MIN_SPEED_KMPH) {
    localKnockCount = 0;
    localFirstKnockAt = 0;
    localTamperMotionSince = 0;
    localTouchSince = 0;
    writeBackNeeded = true;
    alarmTriggered = true;
    alarmReason = F("GPS MOTION");
  } else {
    if (!imuReady) return;

    const bool touchNow = (lastADeltaG > TOUCH_DELTA_G) || (lastGMag > TOUCH_GYRO);
    if (touchNow) {
      if (!localTouchSince) {
        localTouchSince = now;
        writeBackNeeded = true;
      }
      if ((now - localTouchSince >= TOUCH_TRIGGER_MS) &&
          (now - localLastTouchFireAt >= TOUCH_DEBOUNCE_MS)) {
        localLastTouchFireAt = now;
        localTouchSince = 0;
        writeBackNeeded = true;
        alarmTriggered = true;
        alarmReason = F("TOUCH");
      }
    } else if (localTouchSince != 0) {
      localTouchSince = 0;
      writeBackNeeded = true;
    }

    if (!alarmTriggered) {
      const bool hit = (lastADeltaG > KNOCK_DELTA_G) || (lastGMag > KNOCK_GYRO);
      if (hit && (now - localLastKnockAt >= KNOCK_DEBOUNCE_MS)) {
        localLastKnockAt = now;
        if (!localKnockCount) localFirstKnockAt = now;
        localKnockCount++;
        writeBackNeeded = true;
        if (localKnockCount >= KNOCK_COUNT_REQ && (now - localFirstKnockAt <= KNOCK_WINDOW_MS)) {
          localKnockCount = 0;
          writeBackNeeded = true;
          alarmTriggered = true;
          alarmReason = F("KNOCK");
        }
      }
      if (!alarmTriggered && localKnockCount && (now - localFirstKnockAt > KNOCK_WINDOW_MS)) {
        localKnockCount = 0;
        writeBackNeeded = true;
      }
    }

    if (!alarmTriggered) {
      const bool motionNow = (lastADeltaG > 0.10f) || (lastGMag > 0.35f) || imuMoving;
      if (motionNow) {
        if (!localTamperMotionSince) {
          localTamperMotionSince = now;
          writeBackNeeded = true;
        }
        if (now - localTamperMotionSince >= TAMPER_MOTION_MS) {
          localTamperMotionSince = 0;
          writeBackNeeded = true;
          alarmTriggered = true;
          alarmReason = F("SUSTAINED MOTION");
        }
      } else if (localTamperMotionSince != 0) {
        localTamperMotionSince = 0;
        writeBackNeeded = true;
      }
    }
  }

  if (writeBackNeeded) {
    // Write back the counters in one short critical section to avoid split updates.
    if (xSemaphoreTake(stateMutex, pdMS_TO_TICKS(20)) != pdTRUE) return;
    knockCount = localKnockCount;
    firstKnockAt = localFirstKnockAt;
    lastKnockAt = localLastKnockAt;
    touchSince = localTouchSince;
    lastTouchFireAt = localLastTouchFireAt;
    tamperMotionSince = localTamperMotionSince;
    xSemaphoreGive(stateMutex);
  }

  if (alarmTriggered) {
    triggerAlarm(alarmReason);
  }
}
