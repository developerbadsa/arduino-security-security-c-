#include "tracking.h"

#include "../core/state_logic.h"
#include "../logging/logger.h"
#include "../sensors/sensors.h"

namespace {

double deg2rad(double d) {
  return d * 3.14159265358979323846 / 180.0;
}

double distanceSquaredMeters(double lat1, double lon1, double lat2, double lon2) {
  // Equirectangular approximation squared
  const double R = 6371000.0;
  const double x = deg2rad(lon2 - lon1) * cos(deg2rad((lat1 + lat2) / 2.0));
  const double y = deg2rad(lat2 - lat1);
  return (R * R) * (x * x + y * y);
}

bool isLeapYear(int year) {
  return ((year % 4) == 0 && (year % 100) != 0) || ((year % 400) == 0);
}

uint32_t gpsUtcEpoch() {
  if (!gps.date.isValid() || !gps.time.isValid()) return 0;

  const int year = gps.date.year();
  const int month = gps.date.month();
  const int day = gps.date.day();
  const int hour = gps.time.hour();
  const int minute = gps.time.minute();
  const int second = gps.time.second();

  if (year < 1970 || month < 1 || month > 12 || day < 1 || day > 31) return 0;
  if (hour > 23 || minute > 59 || second > 59) return 0;

  static const uint16_t daysBeforeMonth[] = {
    0,   31,  59,  90,  120, 151,
    181, 212, 243, 273, 304, 334
  };

  uint32_t days = 0;
  for (int y = 1970; y < year; y++) {
    days += isLeapYear(y) ? 366UL : 365UL;
  }

  days += daysBeforeMonth[month - 1];
  if (month > 2 && isLeapYear(year)) {
    days += 1;
  }
  days += static_cast<uint32_t>(day - 1);

  return (days * 86400UL) +
         (static_cast<uint32_t>(hour) * 3600UL) +
         (static_cast<uint32_t>(minute) * 60UL) +
         static_cast<uint32_t>(second);
}

void addPoint_locked(double lat, double lon, float spd, DeviceStateId st) {
  TrackPoint tp{lat, lon, spd, millis(), gpsUtcEpoch(), static_cast<uint8_t>(st)};
  if (rawCount < MAX_RAW_POINTS) {
    rawBuf[rawCount++] = tp;
  } else {
    memmove(rawBuf, rawBuf + 1, (MAX_RAW_POINTS - 1) * sizeof(TrackPoint));
    rawBuf[MAX_RAW_POINTS - 1] = tp;
  }
}

}  // namespace

void recordSamplePoint() {
  if (!isGoodFix()) return;
  if (isBikeStationary()) return;

  const double lat = gps.location.lat();
  const double lon = gps.location.lng();
  if (!isInsideBangladesh(lat, lon)) return;

  if (xSemaphoreTake(stateMutex, pdMS_TO_TICKS(10)) != pdTRUE) return;

  const float spd = getFilteredSpeedKmph_locked();
  if (spd >= MAX_SPEED_KMPH) {
    xSemaphoreGive(stateMutex);
    return;
  }

  if (rawCount > 0) {
    const double d2 = distanceSquaredMeters(rawBuf[rawCount - 1].lat, rawBuf[rawCount - 1].lon, lat, lon);
    if (d2 > (MAX_JUMP_DIST_M * MAX_JUMP_DIST_M) || d2 < (MIN_SAMPLE_DIST_M * MIN_SAMPLE_DIST_M)) {
      xSemaphoreGive(stateMutex);
      return;
    }
  }

  addPoint_locked(lat, lon, spd, ST_MOVING);
  xSemaphoreGive(stateMutex);
}

void recordHeartbeatPoint(const __FlashStringHelper* reason) {
  if (!isGoodFix()) return;

  const double lat = gps.location.lat();
  const double lon = gps.location.lng();
  if (!isInsideBangladesh(lat, lon)) return;

  if (xSemaphoreTake(stateMutex, pdMS_TO_TICKS(10)) != pdTRUE) return;

  if (rawCount > 0) {
    const double d2 = distanceSquaredMeters(rawBuf[rawCount - 1].lat, rawBuf[rawCount - 1].lon, lat, lon);
    if (d2 > (MAX_JUMP_DIST_M * MAX_JUMP_DIST_M)) {
      xSemaphoreGive(stateMutex);
      return;
    }
  }

  const DeviceStateId st = getCurrentState_locked();
  addPoint_locked(lat, lon, 0.0f, st);
  xSemaphoreGive(stateMutex);

  if (DBG_GPS) {
    String msg = F("HB ");
    msg += reason;
    logPrintln("GPS", msg);
  }
}

int buildCompressedTrack(TrackPoint* out, int maxOut, const TrackPoint* src, int srcCount) {
  if (!out || maxOut <= 0 || srcCount <= 0) return 0;
  if (srcCount <= maxOut) {
    memcpy(out, src, srcCount * sizeof(TrackPoint));
    return srcCount;
  }

  int outCount = 0;
  out[outCount++] = src[0];
  int lastIdx = 0;
  for (int i = 1; i < srcCount - 1 && outCount < (maxOut - 1); i++) {
    const double d2 = distanceSquaredMeters(src[i].lat, src[i].lon, src[lastIdx].lat, src[lastIdx].lon);
    if (d2 >= (MIN_SAMPLE_DIST_M * MIN_SAMPLE_DIST_M)) {
      out[outCount++] = src[i];
      lastIdx = i;
    }
  }
  out[outCount++] = src[srcCount - 1];
  return outCount;
}
