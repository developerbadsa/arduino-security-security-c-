#include "command_processor.h"

#include "../actuators/actuators.h"

void queueReportFromLoop(const char* event, const char* cmdId) {
  if (!reportQueue) return;

  ReportMsg msg = {};
  strncpy(msg.event, event, sizeof(msg.event) - 1);
  msg.event[sizeof(msg.event) - 1] = 0;
  strncpy(msg.cmdId, cmdId ? cmdId : "", sizeof(msg.cmdId) - 1);
  msg.cmdId[sizeof(msg.cmdId) - 1] = 0;
  xQueueSend(reportQueue, &msg, 0);
}

void executeCommand(const String& cmd, const String& cmdId) {
  if (cmd != "BUZZ") buzzerStopAll();

  if (cmd == "LOCK") {
    buzzerStartLockPattern();
    relayOn();
    setLed(false);
    queueReportFromLoop("LOCK_DONE", cmdId.c_str());
    return;
  }

  if (cmd == "UNLOCK") {
    relayOff();
    setLed(true);
    xSemaphoreTake(stateMutex, portMAX_DELAY);
    armed = false;
    alarmLatched = false;
    xSemaphoreGive(stateMutex);
    knockCount = 0;
    tamperMotionSince = 0;
    touchSince = 0;
    buzzerStartUnlockPattern();
    queueReportFromLoop("UNLOCK_DONE", cmdId.c_str());
    return;
  }

  if (cmd == "ARM") {
    xSemaphoreTake(stateMutex, portMAX_DELAY);
    const bool isLocked = locked;
    xSemaphoreGive(stateMutex);
    if (!isLocked) {
      queueReportFromLoop("ARM_REJECT_UNLOCKED", cmdId.c_str());
      return;
    }

    xSemaphoreTake(stateMutex, portMAX_DELAY);
    armed = true;
    alarmLatched = false;
    xSemaphoreGive(stateMutex);
    knockCount = 0;
    tamperMotionSince = 0;
    touchSince = 0;
    buzzerStartArmPattern();
    queueReportFromLoop("ARM_ON", cmdId.c_str());
    return;
  }

  if (cmd == "BUZZ") {
    buzzerStartAlarm(ALARM_BUZZ_MS);
    queueReportFromLoop("BUZZ_ON", cmdId.c_str());
    return;
  }

  queueReportFromLoop("UNKNOWN_CMD", cmdId.c_str());
}
