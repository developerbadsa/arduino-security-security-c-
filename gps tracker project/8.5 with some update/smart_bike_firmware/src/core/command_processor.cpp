#include "command_processor.h"

#include "../actuators/actuators.h"
#include "../sensors/sensors.h"

void queueReportFromLoop(const char* event, const char* cmdId) {
  if (!reportQueue) return;

  ReportMsg msg = {};
  strncpy(msg.event, event, sizeof(msg.event) - 1);
  msg.event[sizeof(msg.event) - 1] = 0;
  strncpy(msg.cmdId, cmdId ? cmdId : "", sizeof(msg.cmdId) - 1);
  msg.cmdId[sizeof(msg.cmdId) - 1] = 0;
  xQueueSend(reportQueue, &msg, 0);
}

void executeCommand(const char* cmd, const char* cmdId) {
  if (strcmp(cmd, "BUZZ") != 0) buzzerStopAll();

  if (strcmp(cmd, "LOCK") == 0) {
    buzzerStartLockPattern();
    relayOn();
    setLed(false);
    queueReportFromLoop("LOCK_DONE", cmdId ? cmdId : "");
    return;
  }

  if (strcmp(cmd, "UNLOCK") == 0) {
    relayOff();
    setLed(true);
    xSemaphoreTake(stateMutex, portMAX_DELAY);
    armed = false;
    alarmLatched = false;
    armSettleUntil = 0;
    xSemaphoreGive(stateMutex);
    knockCount = 0;
    firstKnockAt = 0;
    lastKnockAt = 0;
    tamperMotionSince = 0;
    touchSince = 0;
    lastTouchFireAt = 0;
    buzzerStartUnlockPattern();
    queueReportFromLoop("UNLOCK_DONE", cmdId ? cmdId : "");
    return;
  }

  if (strcmp(cmd, "ARM") == 0) {
    xSemaphoreTake(stateMutex, pdMS_TO_TICKS(10));
    const bool isLocked = locked;
    xSemaphoreGive(stateMutex);
    if (!isLocked) {
      queueReportFromLoop("ARM_REJECT_UNLOCKED", cmdId ? cmdId : "");
      return;
    }

    recalibrateImuBias();

    xSemaphoreTake(stateMutex, pdMS_TO_TICKS(10));
    armed = true;
    alarmLatched = false;
    armSettleUntil = millis() + ARM_SETTLE_MS;
    xSemaphoreGive(stateMutex);
    knockCount = 0;
    firstKnockAt = 0;
    lastKnockAt = 0;
    tamperMotionSince = 0;
    touchSince = 0;
    lastTouchFireAt = 0;
    buzzerStartArmPattern();
    queueReportFromLoop("ARM_ON", cmdId ? cmdId : "");
    return;
  }

  if (strcmp(cmd, "BUZZ") == 0) {
    buzzerStartAlarm(ALARM_BUZZ_MS);
    queueReportFromLoop("BUZZ_ON", cmdId ? cmdId : "");
    return;
  }

  queueReportFromLoop("UNKNOWN_CMD", cmdId ? cmdId : "");
}
