#include "command_processor.h"

#include "../actuators/actuators.h"
#include "../logging/logger.h"
#include "../sensors/sensors.h"

void queueReportFromLoop(const char* event, const char* cmdId);

namespace {

bool takeStateMutexOrReport(const char* cmdId) {
  if (xSemaphoreTake(stateMutex, pdMS_TO_TICKS(50)) == pdTRUE) return true;

  logPrintln("CMD", F("WARN stateMutex timeout"));
  queueReportFromLoop("MUTEX_TIMEOUT", cmdId ? cmdId : "");
  return false;
}

}  // namespace

void queueReportFromLoop(const char* event, const char* cmdId) {
  if (!reportQueue) return;

  ReportMsg msg = {};
  strncpy(msg.event, event, sizeof(msg.event) - 1);
  msg.event[sizeof(msg.event) - 1] = 0;
  const bool omitCmdId = !cmdId || !cmdId[0] || strcmp(cmdId, "LOCAL") == 0 || strcmp(cmdId, "null") == 0;
  strncpy(msg.cmdId, omitCmdId ? "" : cmdId, sizeof(msg.cmdId) - 1);
  msg.cmdId[sizeof(msg.cmdId) - 1] = 0;
  if (xQueueSend(reportQueue, &msg, 0) != pdTRUE) {
    LOG_PRINTLN_IF(DBG_NET, "CMD", F("reportQueue full, dropping report"));
    return;
  }

  LOG_PRINTF_IF(DBG_NET,
                "CMD",
                "queued report event=%s cmdId=%s",
                msg.event,
                msg.cmdId[0] ? msg.cmdId : "none");
}

void executeCommand(const char* cmd, const char* cmdId) {
  LOG_PRINTF_IF(DBG_NET,
                "CMD",
                "execute cmd=%s cmdId=%s",
                cmd ? cmd : "null",
                (cmdId && cmdId[0]) ? cmdId : "none");

  if (strcmp(cmd, "BUZZ") != 0) buzzerStopAll();

  if (strcmp(cmd, "LOCK") == 0) {
    buzzerStartLockPattern();
    relayOn();
    setLed(false);

    if (!takeStateMutexOrReport(cmdId)) return;
    locked = true;
    xSemaphoreGive(stateMutex);

    logPrintln("CMD", F("LOCK applied"));
    queueReportFromLoop("LOCK_DONE", cmdId ? cmdId : "");
    return;
  }

  if (strcmp(cmd, "UNLOCK") == 0) {
    relayOff();
    setLed(true);

    if (!takeStateMutexOrReport(cmdId)) return;
    locked = false;
    armed = false;
    alarmLatched = false;
    armSettleUntil = 0;
    knockCount = 0;
    firstKnockAt = 0;
    lastKnockAt = 0;
    tamperMotionSince = 0;
    touchSince = 0;
    lastTouchFireAt = 0;
    xSemaphoreGive(stateMutex);
    buzzerStartUnlockPattern();
    logPrintln("CMD", F("UNLOCK applied"));
    queueReportFromLoop("UNLOCK_DONE", cmdId ? cmdId : "");
    return;
  }

  if (strcmp(cmd, "ARM") == 0) {
    if (!takeStateMutexOrReport(cmdId)) return;
    const bool isLocked = locked;
    xSemaphoreGive(stateMutex);
    if (!isLocked) {
      logPrintln("CMD", F("ARM rejected: unlocked"));
      queueReportFromLoop("ARM_REJECT_UNLOCKED", cmdId ? cmdId : "");
      return;
    }

    if (!isBikeStationary()) {
      logPrintln("CMD", F("ARM rejected: bike not stationary"));
      queueReportFromLoop("ARM_REJECT_MOVING", cmdId ? cmdId : "");
      return;
    }

    if (!imuReady) {
      logPrintln("CMD", F("ARM rejected: IMU not ready"));
      queueReportFromLoop("ARM_REJECT_IMU", cmdId ? cmdId : "");
      return;
    }

    if (!recalibrateImuBias()) {
      logPrintln("CMD", F("ARM rejected: IMU recalibration failed"));
      queueReportFromLoop("ARM_REJECT_IMU", cmdId ? cmdId : "");
      return;
    }

    if (!takeStateMutexOrReport(cmdId)) return;
    armed = true;
    alarmLatched = false;
    armSettleUntil = millis() + ARM_SETTLE_MS;
    knockCount = 0;
    firstKnockAt = 0;
    lastKnockAt = 0;
    tamperMotionSince = 0;
    touchSince = 0;
    lastTouchFireAt = 0;
    xSemaphoreGive(stateMutex);
    buzzerStartArmPattern();
    logPrintln("CMD", F("ARM enabled"));
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
