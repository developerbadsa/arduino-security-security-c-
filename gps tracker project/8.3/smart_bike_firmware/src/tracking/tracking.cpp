#include "tracking.h"

#include "../core/state_logic.h"
#include "../sensors/sensors.h"

namespace {

double deg2rad(double d) {
  return d * 3.14159265358979323846 / 180.0;
}

double distanceMeters(double la1, double lo1, double la2, double lo2) {
  const double r = 6371000.0;
  const double dLat = deg2rad(la2 - la1);
  const double dLon = deg2rad(lo2 - lo1);
  const double a = sin(dLat / 2) * sin(dLat / 2) +
                   cos(deg2rad(la1)) * cos(deg2rad(la2)) *
                     sin(dLon / 2) * sin(dLon / 2);
  return r * 2.0 * atan2(sqrt(a), sqrt(1.0 - a));
}

void addPoint_locked(double lat, double lon, float spd, DeviceStateId st) {
  TrackPoint tp{lat, lon, spd, millis(), static_cast<uint8_t>(st)};
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
    const double d = distanceMeters(rawBuf[rawCount - 1].lat, rawBuf[rawCount - 1].lon, lat, lon);
    if (d > MAX_JUMP_DIST_M || d < MIN_SAMPLE_DIST_M) {
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
    const double d = distanceMeters(rawBuf[rawCount - 1].lat, rawBuf[rawCount - 1].lon, lat, lon);
    if (d > MAX_JUMP_DIST_M) {
      xSemaphoreGive(stateMutex);
      return;
    }
  }

  const DeviceStateId st = getCurrentState_locked();
  addPoint_locked(lat, lon, 0.0f, st);
  xSemaphoreGive(stateMutex);

  if (DBG_GPS) {
    Serial.print(F("[GPS][HB] "));
    Serial.println(reason);
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
    const double d = distanceMeters(src[i].lat, src[i].lon, src[lastIdx].lat, src[lastIdx].lon);
    if (d >= MIN_SAMPLE_DIST_M) {
      out[outCount++] = src[i];
      lastIdx = i;
    }
  }
  out[outCount++] = src[srcCount - 1];
  return outCount;
}
