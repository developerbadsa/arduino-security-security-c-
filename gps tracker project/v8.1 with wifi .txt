#include <Arduino.h>
#include <HardwareSerial.h>
#include <TinyGPSPlus.h>
#include <Wire.h>
#include "mpu6500.h"
#include <math.h>
#include <WiFi.h>
#include <WiFiClient.h>
#include <HTTPClient.h>
#include <WebServer.h>
#include <DNSServer.h>
#include "esp32-hal-bt.h"
#include <esp_bt.h>

// ============================================================
// SmartBike Firmware v7.x + Local Access UI (ESP32 + WIFI + NEO GPS + MPU6500)
// - WIFI STA (pocket router / USB 4G dongle router)
// - Local offline access: SoftAP + Captive Portal UI (LOCK/UNLOCK/ARM)
// - MPU sensitive movement/tap -> wake Local WiFi when STA is offline
// - Existing server poll/push/report unchanged
// - Relay + buzzer + LED blink (non-blocking)
// - Command ACK beep patterns (LOCK/ARM/UNLOCK)
// - Bluetooth OFF
// ============================================================

// ================== SERVER CONFIG ==================
static const char* SERVER_URL_PUSH   = "http://smartbike.ashikdev.com/api/device/push";
static const char* SERVER_URL_POLL   = "http://smartbike.ashikdev.com/api/device/poll";
static const char* SERVER_URL_REPORT = "http://smartbike.ashikdev.com/api/device/report";
static const char* DEVICE_ID         = "BIKE01";

// ================== WIFI CONFIG (STA) ==================
static const char* WIFI_SSID = "a";
static const char* WIFI_PASS = "87654321q";

// ================== LOCAL ACCESS (SoftAP + UI) ==================
static const char* LOCAL_AP_PASS = "bikelocal88";   // >= 8 chars
static const char* LOCAL_PIN     = "2234";          // 4-digit pin
static constexpr uint32_t LOCAL_AP_IDLE_OFF_MS = 5UL * 60UL * 1000UL;

// MPU -> Local AP wake (when STA offline)
static constexpr uint32_t LOCAL_WAKE_DEBOUNCE_MS = 3500;
static constexpr uint32_t LOCAL_WAKE_TOUCH_MS    = 160;
static constexpr float    LOCAL_WAKE_DELTA_G     = 0.055f;
static constexpr float    LOCAL_WAKE_GYRO_RADPS  = 0.22f;
static constexpr float    LOCAL_WAKE_MAX_SPEED   = 6.0f;

// Local UI PIN lockout
static constexpr uint8_t  LOCAL_PIN_FAIL_MAX     = 5;
static constexpr uint32_t LOCAL_PIN_LOCK_MS      = 30000;

// ================== DEBUG ==================
static constexpr bool DBG_NET       = true;
static constexpr bool DBG_HTTP_BODY = true;
static constexpr bool DBG_GPS       = true;
static constexpr bool DBG_IMU       = false;
static constexpr bool DBG_SEC       = true;
static constexpr bool DBG_LOCAL     = true;

// ================== UART PINS ==================
static constexpr int      GPS_RX   = 16;
static constexpr int      GPS_TX   = 17;
static constexpr uint32_t GPS_BAUD = 9600;
HardwareSerial SerialGPS(1);
TinyGPSPlus gps;

// ================== DEVICE STATE ==================
enum DeviceStateId : uint8_t {
  ST_LOCKED  = 0,
  ST_UNLOCKED= 1,
  ST_MOVING  = 2,
  ST_ALARM   = 3
};

static bool locked = false;
static bool armed  = false;
static bool alarmLatched = false;
static uint32_t alarmCooldownUntil = 0;

static const char* stateToString(DeviceStateId st) {
  switch (st) {
    case ST_LOCKED:   return "LOCKED";
    case ST_UNLOCKED: return "UNLOCKED";
    case ST_MOVING:   return "MOVING";
    case ST_ALARM:    return "ALARM";
    default:          return "UNLOCKED";
  }
}

static DeviceStateId getCurrentCanonicalState() {
  if (alarmLatched) return ST_ALARM;
  return locked ? ST_LOCKED : ST_UNLOCKED;
}

// ================== TRACK TYPES (MUST BE ABOVE PROTOTYPES) ==================
struct TrackPoint {
  double   lat;
  double   lon;
  float    speed;
  uint32_t tMs;
  uint8_t  st;
};

static constexpr int MAX_RAW_POINTS   = 20;
static constexpr int MAX_TRACK_POINTS = 5;
static TrackPoint rawBuf[MAX_RAW_POINTS];
static int rawCount = 0;

// ================== BUZZER PATTERN TYPES (MUST BE ABOVE PROTOTYPES) ==================
struct BeepStep { uint16_t ms; uint8_t on; };

// ================== LOCAL PORTAL TYPES (MUST BE ABOVE PROTOTYPES) ==================
static WebServer localServer(80);
static DNSServer localDns;
static bool      localApActive = false;
static uint32_t  localApStartedAt = 0;
static uint32_t  localApLastActivityAt = 0;
static uint32_t  localWakeCooldownUntil = 0;
static uint32_t  localTouchSince = 0;

static uint8_t   localPinFailCount = 0;
static uint32_t  localPinLockedUntil = 0;

// ================== PROTOTYPES (PREVENT ARDUINO AUTO-PROTOTYPE BUG) ==================
static void gpsPump();

static inline float mag3(float x, float y, float z);
static inline double deg2rad(double deg);
static double distanceMeters(double lat1, double lon1, double lat2, double lon2);
static bool isInsideBangladesh(double lat, double lon);
static bool isGoodFix();

static float getFilteredSpeedKmph();
static bool isBikeStationary();
static bool isSpeedLikelyLowForLocal();

static void addPointToBuffer(double lat, double lon, float spd, DeviceStateId st);
static void recordSamplePoint();
static int  buildCompressedTrack(TrackPoint* outArr, int outMax);
static void recordHeartbeatPoint(const __FlashStringHelper* reason);

static void wifiInit();
static void wifiMaintenance();
static bool wifiIsConnected();

static bool httpPostJSON(const char* url, const String& json, String* outBody);

static String jsonFindString(const String& body, const char* key);

static void startBuzz(uint32_t durationMs);
static void queueReport(const String& commandId, const String& event);
static void reportCommandDone(const String& commandId, const String& event);
static void pumpPendingReport();

static void executeCommand(const String& cmd, const String& cmdId);
static void pushTrackBatch();
static void pushStatePing(DeviceStateId st);
static void pollCommand();

static void initIMU();
static void calibrateGyroBias();
static void updateImuMotion();
static void updateSecurityAlarm();
static void triggerAlarm(const __FlashStringHelper* reason);

static void buzzerStartAlarm(uint32_t durationMs);
static void buzzerStartPattern(const BeepStep* pat, uint8_t len);
static void buzzerStopAll();
static void buzzerUpdate();

// Local portal
static void localPortalStart();
static void localPortalStop();
static void localPortalMaintenance();
static void localPortalMarkActivity();
static void localPortalSetupRoutes();
static void localHandleRoot();
static void localHandleStatus();
static void localHandleCmd(const String& cmd);
static void localHandleNotFound();
static bool localCheckPinOk();

// ================== I2C / IMU ==================
static constexpr int I2C_SDA = 21;
static constexpr int I2C_SCL = 22;

bfs::Mpu6500 imu(&Wire, bfs::Mpu6500::I2C_ADDR_PRIM);

struct GyroBias { float x=0, y=0, z=0; };
static GyroBias imuBias;
static bool imuReady = false;

static bool     imuMoving = false;
static uint32_t imuMovingHoldUntil = 0;
static uint32_t lastImuReadAt = 0;

static float lastADeltaG = 0.0f;
static float lastGMag    = 0.0f;
static uint32_t tamperMotionSince = 0;

// ================== IMU TUNING ==================
static constexpr uint32_t IMU_READ_EVERY_MS = 20;
static constexpr uint32_t MOVING_HOLD_MS    = 1200;

static constexpr float GYRO_MOVING_ON_RADPS  = 0.30f;
static constexpr float GYRO_MOVING_OFF_RADPS = 0.22f;

static constexpr float ACCEL_DELTA_ON_G      = 0.25f;
static constexpr float ACCEL_DELTA_OFF_G     = 0.15f;

static constexpr float G_MPS2 = 9.80665f;

static constexpr uint16_t CAL_SAMPLES  = 250;
static constexpr uint16_t CAL_DELAY_MS = 10;

// ================== SECURITY (ARM/TAMPER) ==================
static constexpr uint32_t ALARM_BUZZ_MS       = 12000;
static constexpr uint32_t ALARM_COOLDOWN_MS   = 2000;

static constexpr uint32_t KNOCK_WINDOW_MS     = 2500;
static constexpr uint32_t KNOCK_DEBOUNCE_MS   = 180;
static constexpr uint8_t  KNOCK_COUNT_REQ     = 2;

static constexpr float KNOCK_DELTA_G         = 0.18f;
static constexpr float KNOCK_GYRO_RADPS      = 0.80f;

static constexpr uint32_t TAMPER_MOTION_MS   = 900;

static uint8_t  knockCount = 0;
static uint32_t firstKnockAt = 0;
static uint32_t lastKnockAt  = 0;

static constexpr float    TOUCH_DELTA_G_ON   = 0.06f;
static constexpr float    TOUCH_GYRO_ON      = 0.22f;
static constexpr uint32_t TOUCH_TRIGGER_MS   = 180;
static constexpr uint32_t TOUCH_DEBOUNCE_MS  = 120;
static uint32_t touchSince = 0;
static uint32_t lastTouchFireAt = 0;

// ================== TIMERS ==================
static constexpr uint32_t SAMPLE_INTERVAL_MS_MOVING   = 1000UL;
static constexpr uint32_t SAMPLE_INTERVAL_MS_STILL    = 2000UL;

static constexpr uint32_t PUSH_INTERVAL_MS_MOVING     = 15000UL;
static constexpr uint32_t PUSH_INTERVAL_MS_STILL      = 120000UL;

static constexpr uint32_t POLL_INTERVAL_MS_FAST       = 1200UL;
static constexpr uint32_t POLL_INTERVAL_MS_NORMAL     = 2500UL;
static constexpr uint32_t POLL_INTERVAL_MS_NO_NET     = 180000UL;

static constexpr uint32_t HEARTBEAT_INTERVAL_MS       = 5UL * 60UL * 1000UL;

static constexpr uint32_t PUSH_INTERVAL_MS_NO_NET     = 300000UL;

static uint32_t lastSample    = 0;
static uint32_t lastPush      = 0;
static uint32_t lastPoll      = 0;
static uint32_t lastHeartbeat = 0;

// ================== RELAY / BUZZER / LED ==================
static constexpr int RELAY_PIN        = 25;
static constexpr int BUZZER_RELAY_PIN = 32;
static constexpr int LED_PIN          = 33;
static constexpr int ONBOARD_LED_PIN  = 2;

static constexpr bool RELAY_ACTIVE_LOW = true;

static inline void relayOn()   { digitalWrite(RELAY_PIN,        RELAY_ACTIVE_LOW ? LOW  : HIGH); locked = true; }
static inline void relayOff()  { digitalWrite(RELAY_PIN,        RELAY_ACTIVE_LOW ? HIGH : LOW); locked = false; }
static inline void buzzerOn()  { digitalWrite(BUZZER_RELAY_PIN, RELAY_ACTIVE_LOW ? LOW  : HIGH); }
static inline void buzzerOff() { digitalWrite(BUZZER_RELAY_PIN, RELAY_ACTIVE_LOW ? HIGH : LOW); }

static inline void setLed(bool on) {
  if (LED_PIN >= 0) digitalWrite(LED_PIN, on ? HIGH : LOW);
  if (ONBOARD_LED_PIN >= 0) digitalWrite(ONBOARD_LED_PIN, on ? HIGH : LOW);
}

// ================== WIFI BACKOFF (non-blocking) ==================
static uint32_t nextNetTryAt = 0;
static uint32_t netBackoffMs = 15000;
static constexpr uint32_t NET_BACKOFF_MAX_MS = 5UL * 60UL * 1000UL;
static uint8_t netFailStreak = 0;

static bool wifiConnecting = false;
static bool wifiWasConnected = false;
static uint32_t wifiConnectStartedAt = 0;
static constexpr uint32_t WIFI_CONNECT_TIMEOUT_MS = 15000;

static inline bool netAllowAttempt() { return millis() >= nextNetTryAt; }

static void netOnSuccess() {
  netFailStreak = 0;
  netBackoffMs = 15000;
  nextNetTryAt = millis();
}

static void netOnFail() {
  netFailStreak++;
  uint32_t next = netBackoffMs * 2;
  netBackoffMs = (next > NET_BACKOFF_MAX_MS) ? NET_BACKOFF_MAX_MS : next;
  nextNetTryAt = millis() + netBackoffMs;
}

// ================== SPEED FILTER ==================
static constexpr float MIN_SPEED_KMPH       = 2.0f;
static constexpr int   STOP_COUNT_REQ       = 4;
static constexpr float MAX_VALID_SPEED_KMPH = 130.0f;
static constexpr float SPEED_ALPHA          = 0.4f;

static int   lowSpeedCount   = 0;
static float filteredSpeed   = 0.0f;
static bool  hasFilteredInit = false;

// ================== PENDING REPORT ==================
static bool   pendingReport = false;
static String pendingReportCmdId;
static String pendingReportEvent;
static uint32_t lastReportTry = 0;
static constexpr uint32_t REPORT_RETRY_MS = 5000;

// ================== LOG HELPERS ==================
static inline void logInfo(const __FlashStringHelper* msg) { Serial.print(F("[INFO] ")); Serial.println(msg); }
static inline void logWarn(const __FlashStringHelper* msg) { Serial.print(F("[WARN] ")); Serial.println(msg); }
static inline void logErr (const __FlashStringHelper* msg) { Serial.print(F("[ERR] "));  Serial.println(msg); }

// ============================================================
// BUZZER PATTERN ENGINE (ACK patterns + ALARM override)
// ============================================================
enum BuzzerMode : uint8_t { BUZ_OFF=0, BUZ_ALARM=1, BUZ_PATTERN=2 };
static BuzzerMode buzMode = BUZ_OFF;

static constexpr uint16_t BEEP_GAP_MS = 200;

static const BeepStep PAT_LOCK[]   = { {2000,1} };

static const BeepStep PAT_ARM[]    = {
  {500,1}, {BEEP_GAP_MS,0},
  {500,1}, {BEEP_GAP_MS,0},
  {2000,1}
};

static const BeepStep PAT_UNLOCK[] = {
  {3000,1}, {BEEP_GAP_MS,0},
  {500,1},  {BEEP_GAP_MS,0},
  {500,1}
};

static const BeepStep* curPat = nullptr;
static uint8_t curPatLen = 0;
static uint8_t curIdx = 0;
static uint32_t stepUntil = 0;
static uint32_t alarmUntil = 0;

static inline void buzApply(uint8_t on) { if (on) buzzerOn(); else buzzerOff(); }

static void buzzerStartAlarm(uint32_t durationMs) {
  buzMode = BUZ_ALARM;
  alarmUntil = millis() + durationMs;
  buzApply(1);
}

static void buzzerStartPattern(const BeepStep* pat, uint8_t len) {
  if (!pat || !len) return;
  if (buzMode == BUZ_ALARM) return;
  buzMode = BUZ_PATTERN;
  curPat = pat;
  curPatLen = len;
  curIdx = 0;
  buzApply(curPat[0].on);
  stepUntil = millis() + curPat[0].ms;
}

static void buzzerStopAll() {
  buzMode = BUZ_OFF;
  curPat = nullptr;
  curPatLen = 0;
  curIdx = 0;
  alarmUntil = 0;
  buzApply(0);
}

static void buzzerUpdate() {
  const uint32_t now = millis();

  if (buzMode == BUZ_ALARM) {
    if (now >= alarmUntil) buzzerStopAll();
    return;
  }

  if (buzMode == BUZ_PATTERN) {
    if (now < stepUntil) return;
    curIdx++;
    if (curIdx >= curPatLen) {
      buzzerStopAll();
      return;
    }
    buzApply(curPat[curIdx].on);
    stepUntil = now + curPat[curIdx].ms;
  }
}

static void startBuzz(uint32_t durationMs) {
  buzzerStartAlarm(durationMs);
}

// ============================================================
// GPS
// ============================================================
static void gpsPump() {
  while (SerialGPS.available()) gps.encode((char)SerialGPS.read());
}

// ============================================================
// Utility
// ============================================================
static inline float mag3(float x, float y, float z) { return sqrtf(x*x + y*y + z*z); }

static inline double deg2rad(double deg) {
  return deg * 3.14159265358979323846 / 180.0;
}

static double distanceMeters(double lat1, double lon1, double lat2, double lon2) {
  const double R = 6371000.0;
  const double dLat = deg2rad(lat2 - lat1);
  const double dLon = deg2rad(lon2 - lon1);
  const double a =
    sin(dLat / 2) * sin(dLat / 2) +
    cos(deg2rad(lat1)) * cos(deg2rad(lat2)) *
    sin(dLon / 2) * sin(dLon / 2);
  const double c = 2.0 * atan2(sqrt(a), sqrt(1.0 - a));
  return R * c;
}

static bool isInsideBangladesh(double lat, double lon) {
  if (lat < 20.0 || lat > 27.0) return false;
  if (lon < 88.0 || lon > 93.0) return false;
  return true;
}

static bool isGoodFix() {
  if (!gps.location.isValid()) return false;
  if (gps.location.age() > 5000) return false;
  if (!gps.satellites.isValid() || gps.satellites.value() < 4) return false;
  if (gps.hdop.isValid() && gps.hdop.hdop() > 3.0) return false;
  return true;
}

// ============================================================
// IMU
// ============================================================
static void calibrateGyroBias() {
  Serial.println(F("[IMU] Calibrating gyro bias... keep STILL (~3s)"));

  float sx=0, sy=0, sz=0;
  uint16_t ok=0;

  for (int i=0; i<40; i++) { imu.Read(); delay(5); }

  for (uint16_t i=0; i<CAL_SAMPLES; i++) {
    if (imu.Read()) {
      sx += imu.gyro_x_radps();
      sy += imu.gyro_y_radps();
      sz += imu.gyro_z_radps();
      ok++;
    }
    delay(CAL_DELAY_MS);
  }

  if (ok == 0) {
    Serial.println(F("[IMU] Bias calibration FAILED (no reads)."));
    imuBias = {};
    imuReady = false;
    return;
  }

  imuBias.x = sx / ok;
  imuBias.y = sy / ok;
  imuBias.z = sz / ok;

  Serial.print(F("[IMU] Bias rad/s: "));
  Serial.print(imuBias.x, 6); Serial.print(F(", "));
  Serial.print(imuBias.y, 6); Serial.print(F(", "));
  Serial.println(imuBias.z, 6);
}

static void initIMU() {
  Wire.begin(I2C_SDA, I2C_SCL);
  Wire.setClock(400000);

  if (!imu.Begin()) {
    Serial.println(F("[IMU] Begin() failed."));
    imuReady = false;
    return;
  }

  imu.ConfigAccelRange(bfs::Mpu6500::ACCEL_RANGE_4G);
  imu.ConfigGyroRange(bfs::Mpu6500::GYRO_RANGE_500DPS);
  imu.ConfigDlpfBandwidth(bfs::Mpu6500::DLPF_BANDWIDTH_20HZ);

  imuReady = true;
  Serial.println(F("[IMU] Init OK."));
  calibrateGyroBias();
}

static void updateImuMotion() {
  if (!imuReady) return;

  const uint32_t now = millis();
  if (now - lastImuReadAt < IMU_READ_EVERY_MS) return;
  lastImuReadAt = now;

  if (!imu.Read()) return;

  const float ax = imu.accel_x_mps2();
  const float ay = imu.accel_y_mps2();
  const float az = imu.accel_z_mps2();

  const float gx = imu.gyro_x_radps() - imuBias.x;
  const float gy = imu.gyro_y_radps() - imuBias.y;
  const float gz = imu.gyro_z_radps() - imuBias.z;

  const float aMag    = mag3(ax, ay, az);
  const float gMag    = mag3(gx, gy, gz);
  const float aDeltaG = fabsf(aMag - G_MPS2) / G_MPS2;

  lastADeltaG = aDeltaG;
  lastGMag    = gMag;

  if (!imuMoving) {
    const bool moveNow = (gMag > GYRO_MOVING_ON_RADPS) || (aDeltaG > ACCEL_DELTA_ON_G);
    if (moveNow) {
      imuMoving = true;
      imuMovingHoldUntil = now + MOVING_HOLD_MS;
      if (DBG_IMU) Serial.println(F("[STATE] IMU -> MOVING"));
    }
  } else {
    if (now < imuMovingHoldUntil) return;

    const bool stillNow = (gMag < GYRO_MOVING_OFF_RADPS) && (aDeltaG < ACCEL_DELTA_OFF_G);
    if (stillNow) {
      imuMoving = false;
      if (DBG_IMU) Serial.println(F("[STATE] IMU -> STILL"));
    } else {
      imuMovingHoldUntil = now + 120;
    }
  }
}

// ============================================================
// SPEED FILTER
// ============================================================
static float getFilteredSpeedKmph() {
  if (!gps.speed.isValid() || gps.speed.age() > 3000) {
    return hasFilteredInit ? filteredSpeed : 0.0f;
  }

  const float rawSpeed = gps.speed.kmph();
  if (rawSpeed > MAX_VALID_SPEED_KMPH) return filteredSpeed;

  if (!hasFilteredInit) {
    filteredSpeed = rawSpeed;
    hasFilteredInit = true;
  } else {
    filteredSpeed = SPEED_ALPHA * rawSpeed + (1.0f - SPEED_ALPHA) * filteredSpeed;
  }

  if (filteredSpeed < MIN_SPEED_KMPH) {
    lowSpeedCount++;
    if (lowSpeedCount >= STOP_COUNT_REQ) filteredSpeed = 0.0f;
  } else {
    lowSpeedCount = 0;
  }

  return filteredSpeed;
}

static bool isBikeStationary() {
  const float spd = getFilteredSpeedKmph();
  if (!imuReady) return spd < 1.0f;
  return (!imuMoving && spd < 3.0f);
}

static bool isSpeedLikelyLowForLocal() {
  if (!gps.speed.isValid() || gps.speed.age() > 3000) {
    return !imuMoving;
  }
  return gps.speed.kmph() < LOCAL_WAKE_MAX_SPEED;
}

// ============================================================
// Track sampling + compression
// ============================================================
static void addPointToBuffer(double lat, double lon, float spd, DeviceStateId st) {
  TrackPoint tp {lat, lon, spd, millis(), (uint8_t)st};

  if (rawCount < MAX_RAW_POINTS) {
    rawBuf[rawCount++] = tp;
  } else {
    for (int i=1; i<MAX_RAW_POINTS; i++) rawBuf[i - 1] = rawBuf[i];
    rawBuf[MAX_RAW_POINTS - 1] = tp;
  }
}

static void recordSamplePoint() {
  if (isBikeStationary()) return;
  if (!isGoodFix()) return;

  const double lat = gps.location.lat();
  const double lon = gps.location.lng();
  if (!isInsideBangladesh(lat, lon)) return;

  const float spd = getFilteredSpeedKmph();
  if (spd > MAX_VALID_SPEED_KMPH) return;

  static constexpr float  MIN_SAMPLE_DIST = 12.0f;
  static constexpr double MAX_JUMP_DIST   = 50000.0;

  if (rawCount > 0) {
    const double d = distanceMeters(rawBuf[rawCount - 1].lat, rawBuf[rawCount - 1].lon, lat, lon);
    if (d > MAX_JUMP_DIST) return;
    if (d < MIN_SAMPLE_DIST) return;
  }

  addPointToBuffer(lat, lon, spd, ST_MOVING);
}

static int buildCompressedTrack(TrackPoint* outArr, int outMax) {
  if (!outArr || outMax <= 0) return 0;
  if (rawCount <= 0) return 0;

  if (rawCount <= outMax) {
    for (int i=0; i<rawCount; i++) outArr[i] = rawBuf[i];
    return rawCount;
  }

  static constexpr float MIN_CHOSEN_DIST = 12.0f;

  int outCount = 0;
  outArr[outCount++] = rawBuf[0];

  int lastIdx = 0;
  for (int i=1; i<rawCount - 1 && outCount < (outMax - 1); i++) {
    const double d = distanceMeters(rawBuf[i].lat, rawBuf[i].lon, rawBuf[lastIdx].lat, rawBuf[lastIdx].lon);
    if (d >= MIN_CHOSEN_DIST) {
      outArr[outCount++] = rawBuf[i];
      lastIdx = i;
    }
  }

  outArr[outCount++] = rawBuf[rawCount - 1];
  return outCount;
}

static void recordHeartbeatPoint(const __FlashStringHelper* reason) {
  if (!isGoodFix()) return;

  const double lat = gps.location.lat();
  const double lon = gps.location.lng();
  if (!isInsideBangladesh(lat, lon)) return;

  if (rawCount > 0) {
    const double d = distanceMeters(rawBuf[rawCount - 1].lat, rawBuf[rawCount - 1].lon, lat, lon);
    if (d > 50000.0) return;
  }

  DeviceStateId st = alarmLatched ? ST_ALARM : (locked ? ST_LOCKED : ST_UNLOCKED);
  addPointToBuffer(lat, lon, 0.0f, st);

  Serial.print(F("[GPS][HB] "));
  Serial.println(reason);
}

// ============================================================
// WIFI (non-blocking connect + backoff) - STA
// ============================================================
static bool wifiIsConnected() {
  return WiFi.status() == WL_CONNECTED;
}

static void wifiInit() {
  WiFi.mode(WIFI_STA);
  WiFi.persistent(false);
  WiFi.setAutoReconnect(true);
  WiFi.setSleep(false);
  WiFi.disconnect(false, false);
  nextNetTryAt = 0;
}

static void wifiStartConnect() {
  if (DBG_NET) {
    Serial.print(F("[WIFI] connect -> "));
    Serial.println(WIFI_SSID);
  }
  WiFi.disconnect(false, false);
  WiFi.begin(WIFI_SSID, WIFI_PASS);
  wifiConnecting = true;
  wifiConnectStartedAt = millis();
}

static void wifiMaintenance() {
  const bool connected = wifiIsConnected();

  if (connected) {
    if (!wifiWasConnected) {
      wifiWasConnected = true;
      wifiConnecting = false;
      netOnSuccess();
      if (DBG_NET) {
        Serial.print(F("[WIFI] connected, IP="));
        Serial.println(WiFi.localIP());
      }
    }
    return;
  }

  if (wifiWasConnected) {
    wifiWasConnected = false;
    wifiConnecting = false;
    netFailStreak = 0;
    netBackoffMs = 15000;
    nextNetTryAt = millis();
    if (DBG_NET) Serial.println(F("[WIFI] disconnected"));
  }

  if (wifiConnecting) {
    if (millis() - wifiConnectStartedAt >= WIFI_CONNECT_TIMEOUT_MS) {
      wifiConnecting = false;
      netOnFail();
      WiFi.disconnect(false, false);
      if (DBG_NET) Serial.println(F("[WIFI] connect timeout"));
    }
    return;
  }

  if (netAllowAttempt()) wifiStartConnect();
}

// ============================================================
// HTTP (WiFi)  (keep small timeouts to avoid long blocking)
// ============================================================
static bool httpPostJSON(const char* url, const String& json, String* outBody) {
  if (!wifiIsConnected()) return false;

  WiFiClient client;
  client.setTimeout(2500);

  HTTPClient http;
  http.setTimeout(2500);
  http.setReuse(false);
  http.useHTTP10(true);

  if (!http.begin(client, url)) return false;
  http.addHeader("Content-Type", "application/json");

  if (DBG_NET) {
    Serial.print(F("[HTTP] POST "));
    Serial.println(url);
  }

  const int code = http.POST((uint8_t*)json.c_str(), json.length());

  if (code <= 0) {
    if (DBG_NET) {
      Serial.print(F("[HTTP] POST fail code="));
      Serial.println(code);
    }
    http.end();
    return false;
  }

  if (outBody) {
    String body = http.getString();
    if (body.length() > 2048) body = body.substring(0, 2048);
    *outBody = body;

    if (DBG_HTTP_BODY) {
      Serial.print(F("[HTTP] body="));
      Serial.println(body);
    }
  }

  if (DBG_NET) {
    Serial.print(F("[HTTP] status="));
    Serial.println(code);
  }

  http.end();
  return (code >= 200 && code < 300);
}

// ============================================================
// JSON parse (simple)
// ============================================================
static String jsonFindString(const String& body, const char* key) {
  const String k = String("\"") + key + "\"";
  const int p = body.indexOf(k);
  if (p < 0) return "";

  const int colon = body.indexOf(':', p);
  if (colon < 0) return "";

  const int nullPos = body.indexOf("null", colon);
  if (nullPos >= 0 && nullPos - colon < 12) return "null";

  const int q1 = body.indexOf('"', colon);
  if (q1 < 0) return "";

  const int q2 = body.indexOf('"', q1 + 1);
  if (q2 < 0) return "";

  return body.substring(q1 + 1, q2);
}

// ============================================================
// Device APIs
// ============================================================
static void queueReport(const String& commandId, const String& event) {
  pendingReport = true;
  pendingReportCmdId = commandId;
  pendingReportEvent = event;
  lastReportTry = 0;
}

static void reportCommandDone(const String& commandId, const String& event) {
  queueReport(commandId, event);
  pumpPendingReport();
}

static void pumpPendingReport() {
  if (!pendingReport) return;
  if (!wifiIsConnected()) return;

  const uint32_t now = millis();
  if (now - lastReportTry < REPORT_RETRY_MS) return;
  lastReportTry = now;

  String json;
  json.reserve(220);
  json += "{";
  json += "\"deviceId\":\"";  json += DEVICE_ID;           json += "\",";

  if (pendingReportCmdId.length() && pendingReportCmdId != "null") {
    json += "\"commandId\":\""; json += pendingReportCmdId; json += "\",";
  }

  json += "\"event\":\"";     json += pendingReportEvent;  json += "\"";
  json += "}";

  String body;
  const bool ok = httpPostJSON(SERVER_URL_REPORT, json, &body);
  if (ok) {
    pendingReport = false;
    pendingReportCmdId = "";
    pendingReportEvent = "";
  }
}

static void executeCommand(const String& cmd, const String& cmdId) {
  if (cmd != "BUZZ") {
    buzzerStopAll();
  }

  if (cmd == "LOCK") {
    buzzerStartPattern(PAT_LOCK, (uint8_t)(sizeof(PAT_LOCK)/sizeof(PAT_LOCK[0])));
    relayOn();
    setLed(false);
    reportCommandDone(cmdId, "LOCK_DONE");
    return;
  }

  if (cmd == "UNLOCK") {
    relayOff();
    setLed(true);

    armed = false;
    alarmLatched = false;
    knockCount = 0;
    tamperMotionSince = 0;
    touchSince = 0;

    buzzerStopAll();
    buzzerStartPattern(PAT_UNLOCK, (uint8_t)(sizeof(PAT_UNLOCK)/sizeof(PAT_UNLOCK[0])));

    reportCommandDone(cmdId, "UNLOCK_DONE");
    return;
  }

  if (cmd == "ARM") {
    if (!locked) {
      reportCommandDone(cmdId, "ARM_REJECT_UNLOCKED");
      return;
    }
    armed = true;
    alarmLatched = false;
    knockCount = 0;
    tamperMotionSince = 0;
    touchSince = 0;

    buzzerStartPattern(PAT_ARM, (uint8_t)(sizeof(PAT_ARM)/sizeof(PAT_ARM[0])));

    reportCommandDone(cmdId, "ARM_ON");
    return;
  }

  if (cmd == "BUZZ") {
    startBuzz(ALARM_BUZZ_MS);
    reportCommandDone(cmdId, "BUZZ_ON");
    return;
  }

  reportCommandDone(cmdId, "UNKNOWN_CMD");
}

static void pushStatePing(DeviceStateId st) {
  if (!wifiIsConnected()) return;

  String json;
  json.reserve(220);
  json += "{";
  json += "\"deviceId\":\""; json += DEVICE_ID; json += "\",";
  json += "\"state\":\"";    json += stateToString(st); json += "\"";

  if (isGoodFix()) {
    const double lat = gps.location.lat();
    const double lon = gps.location.lng();
    if (isInsideBangladesh(lat, lon)) {
      const float spd = getFilteredSpeedKmph();
      json += ",\"lat\":";   json += String(lat, 6);
      json += ",\"lon\":";   json += String(lon, 6);
      json += ",\"speed\":"; json += String(spd, 2);
    }
  }

  json += "}";
  (void)httpPostJSON(SERVER_URL_PUSH, json, nullptr);
}

static void pushTrackBatch() {
  if (rawCount == 0) return;
  if (!wifiIsConnected()) return;

  TrackPoint track[MAX_TRACK_POINTS];
  const int n = buildCompressedTrack(track, MAX_TRACK_POINTS);
  if (n <= 0) { rawCount = 0; return; }

  DeviceStateId topSt = alarmLatched ? ST_ALARM : (isBikeStationary() ? getCurrentCanonicalState() : ST_MOVING);

  String json;
  json.reserve(820);

  json += "{";
  json += "\"deviceId\":\""; json += DEVICE_ID; json += "\",";
  json += "\"state\":\"";    json += stateToString(topSt); json += "\",";
  json += "\"track\":[";

  for (int i=0; i<n; i++) {
    if (i) json += ",";
    json += "{";
    json += "\"lat\":";   json += String(track[i].lat, 6);   json += ",";
    json += "\"lon\":";   json += String(track[i].lon, 6);   json += ",";
    json += "\"speed\":"; json += String(track[i].speed, 2); json += ",";
    json += "\"state\":\""; json += stateToString((DeviceStateId)track[i].st); json += "\"";
    json += "}";
  }

  json += "]}";

  Serial.print(F("[PUSH] points="));
  Serial.print(n);
  Serial.print(F(" rawCount="));
  Serial.println(rawCount);

  const bool ok = httpPostJSON(SERVER_URL_PUSH, json, nullptr);
  if (ok) rawCount = 0;
  else logWarn(F("pushTrackBatch failed; keeping buffer"));
}

static void pollCommand() {
  if (!wifiIsConnected()) return;

  String json;
  json.reserve(72);
  json += "{\"deviceId\":\"";
  json += DEVICE_ID;
  json += "\"}";

  String body;
  const bool ok = httpPostJSON(SERVER_URL_POLL, json, &body);
  if (!ok) return;

  const String command   = jsonFindString(body, "command");
  const String commandId = jsonFindString(body, "commandId");

  if (command.length() == 0 || command == "null") return;
  executeCommand(command, commandId);
}

// ============================================================
// SECURITY
// ============================================================
static void triggerAlarm(const __FlashStringHelper* reason) {
  const uint32_t now = millis();
  if (now < alarmCooldownUntil) return;

  alarmLatched = true;
  alarmCooldownUntil = now + ALARM_COOLDOWN_MS;

  startBuzz(ALARM_BUZZ_MS);

  if (DBG_SEC) {
    Serial.print(F("[SEC] ALARM TRIGGERED: "));
    Serial.println(reason);
  }

  queueReport("", "ALARM_TRIGGERED");
  pushStatePing(ST_ALARM);
  lastPush = 0;
}

static void updateSecurityAlarm() {
  if (!imuReady) return;
  if (!armed) return;
  if (!locked) return;

  const bool spdValid = gps.speed.isValid() && (gps.speed.age() <= 3000);
  const float spd = spdValid ? getFilteredSpeedKmph() : 0.0f;
  const bool lowSpeed = (!spdValid) ? true : (spd < 2.0f);

  const uint32_t now = millis();
  if (!lowSpeed) {
    knockCount = 0;
    tamperMotionSince = 0;
    touchSince = 0;
    return;
  }

  const bool touchNow = (lastADeltaG > TOUCH_DELTA_G_ON) || (lastGMag > TOUCH_GYRO_ON);
  if (touchNow) {
    if (touchSince == 0) touchSince = now;
    if (now - touchSince >= TOUCH_TRIGGER_MS && now - lastTouchFireAt >= TOUCH_DEBOUNCE_MS) {
      lastTouchFireAt = now;
      touchSince = 0;
      triggerAlarm(F("TOUCH"));
      return;
    }
  } else {
    touchSince = 0;
  }

  const bool hit = (lastADeltaG > KNOCK_DELTA_G) || (lastGMag > KNOCK_GYRO_RADPS);
  if (hit) {
    if (now - lastKnockAt >= KNOCK_DEBOUNCE_MS) {
      lastKnockAt = now;

      if (knockCount == 0) firstKnockAt = now;
      knockCount++;

      if (DBG_SEC) {
        Serial.print(F("[SEC] knock="));
        Serial.print(knockCount);
        Serial.print(F(" aDeltaG="));
        Serial.print(lastADeltaG, 3);
        Serial.print(F(" gMag="));
        Serial.println(lastGMag, 3);
      }

      if (now - firstKnockAt <= KNOCK_WINDOW_MS && knockCount >= KNOCK_COUNT_REQ) {
        knockCount = 0;
        triggerAlarm(F("KNOCK"));
        return;
      }
    }
  }

  if (knockCount > 0 && (now - firstKnockAt > KNOCK_WINDOW_MS)) {
    knockCount = 0;
  }

  const bool motionNow = (lastADeltaG > 0.10f) || (lastGMag > 0.35f) || imuMoving;
  if (motionNow) {
    if (tamperMotionSince == 0) tamperMotionSince = now;
    if (now - tamperMotionSince >= TAMPER_MOTION_MS) {
      tamperMotionSince = 0;
      triggerAlarm(F("SUSTAINED MOTION"));
      return;
    }
  } else {
    tamperMotionSince = 0;
  }
}

// ============================================================
// LOCAL PORTAL (SoftAP + Captive Portal UI)
// ============================================================
static void localPortalMarkActivity() {
  localApLastActivityAt = millis();
}

static bool localCheckPinOk() {
  const uint32_t now = millis();
  if (now < localPinLockedUntil) return false;

  String pin = localServer.arg("pin");
  if (pin.length() == 0) {
    pin = localServer.arg("p");
  }

  if (pin != LOCAL_PIN) {
    localPinFailCount++;
    if (localPinFailCount >= LOCAL_PIN_FAIL_MAX) {
      localPinFailCount = 0;
      localPinLockedUntil = now + LOCAL_PIN_LOCK_MS;
    }
    return false;
  }

  localPinFailCount = 0;
  localPinLockedUntil = 0;
  return true;
}

static void localHandleRoot() {
  localPortalMarkActivity();

  const char* html =
    "<!doctype html><html><head><meta charset='utf-8'/>"
    "<meta name='viewport' content='width=device-width,initial-scale=1'/>"
    "<title>Bike Local</title>"
    "<style>"
    "body{font-family:system-ui,Arial;margin:0;background:#0b1220;color:#e5e7eb}"
    ".wrap{max-width:780px;margin:0 auto;padding:16px}"
    ".card{background:#0f172a;border:1px solid rgba(255,255,255,.08);border-radius:14px;padding:14px;margin:12px 0}"
    "h1{font-size:18px;margin:0 0 10px}"
    "label{font-size:12px;color:#94a3b8}"
    "input{width:100%;padding:12px;border-radius:10px;border:1px solid rgba(255,255,255,.10);background:#0b1220;color:#e5e7eb}"
    ".row{display:flex;gap:10px;flex-wrap:wrap}"
    "button{flex:1;min-width:140px;padding:12px 14px;border-radius:12px;border:0;background:#22c55e;color:#06240f;font-weight:700;cursor:pointer}"
    "button.warn{background:#f97316;color:#2a1100}"
    "button.danger{background:#ef4444;color:#2a0505}"
    "button.gray{background:#334155;color:#e5e7eb}"
    ".kv{display:grid;grid-template-columns:140px 1fr;gap:6px;font-size:13px}"
    ".muted{color:#94a3b8}"
    ".ok{color:#22c55e}.bad{color:#ef4444}"
    "</style></head><body><div class='wrap'>"
    "<h1>BIKE LOCAL CONTROL</h1>"
    "<div class='card'>"
    "<div class='kv'>"
    "<div class='muted'>State</div><div id='st'>-</div>"
    "<div class='muted'>Locked</div><div id='locked'>-</div>"
    "<div class='muted'>Armed</div><div id='armed'>-</div>"
    "<div class='muted'>Alarm</div><div id='alarm'>-</div>"
    "<div class='muted'>WiFi STA</div><div id='sta'>-</div>"
    "<div class='muted'>GPS</div><div id='gps'>-</div>"
    "<div class='muted'>Speed</div><div id='spd'>-</div>"
    "<div class='muted'>IMU</div><div id='imu'>-</div>"
    "<div class='muted'>Buffer</div><div id='buf'>-</div>"
    "</div></div>"
    "<div class='card'>"
    "<label>4-digit PIN</label><input id='pin' inputmode='numeric' maxlength='4' placeholder='1234'/>"
    "<div style='height:10px'></div>"
    "<div class='row'>"
    "<button class='danger' onclick=\"sendCmd('lock')\">LOCK</button>"
    "<button onclick=\"sendCmd('unlock')\">UNLOCK</button>"
    "<button class='warn' onclick=\"sendCmd('arm')\">ARM</button>"
    "<button class='gray' onclick=\"sendCmd('buzz')\">BUZZ</button>"
    "</div>"
    "<div id='msg' class='muted' style='margin-top:10px'></div>"
    "</div>"
    "<div class='muted'>If STA internet is down, move/tap the bike to wake local WiFi.</div>"
    "<script>"
    "async function refresh(){"
    "try{const r=await fetch('/status');const j=await r.json();"
    "document.getElementById('st').textContent=j.state;"
    "document.getElementById('locked').innerHTML=j.locked?'<span class=bad>YES</span>':'<span class=ok>NO</span>';"
    "document.getElementById('armed').innerHTML=j.armed?'<span class=warn>YES</span>':'<span class=ok>NO</span>';"
    "document.getElementById('alarm').innerHTML=j.alarm?'<span class=bad>YES</span>':'<span class=ok>NO</span>';"
    "document.getElementById('sta').innerHTML=j.wifi?'<span class=ok>ONLINE</span>':'<span class=bad>OFFLINE</span>';"
    "document.getElementById('gps').textContent=j.gps;"
    "document.getElementById('spd').textContent=j.speed;"
    "document.getElementById('imu').textContent=j.imu;"
    "document.getElementById('buf').textContent=j.buf;"
    "}catch(e){}"
    "}"
    "async function sendCmd(c){"
    "const p=document.getElementById('pin').value||'';"
    "document.getElementById('msg').textContent='Sending...';"
    "try{const r=await fetch('/api/'+c+'?pin='+encodeURIComponent(p),{method:'POST'});"
    "const t=await r.text();document.getElementById('msg').textContent=t;refresh();}"
    "catch(e){document.getElementById('msg').textContent='ERR';}"
    "}"
    "setInterval(refresh,1200);refresh();"
    "</script></div></body></html>";

  localServer.send(200, "text/html", html);
}

static void localHandleStatus() {
  localPortalMarkActivity();

  const bool wifi = wifiIsConnected();
  String gpsStr;
  gpsStr.reserve(64);
  gpsStr += (gps.location.isValid() ? "valid" : "no");
  gpsStr += " age="; gpsStr += String(gps.location.age());
  gpsStr += " sats="; gpsStr += String(gps.satellites.isValid() ? gps.satellites.value() : -1);
  gpsStr += " hdop="; gpsStr += String(gps.hdop.isValid() ? gps.hdop.hdop() : -1);

  String spdStr;
  spdStr.reserve(24);
  spdStr += String(gps.speed.isValid() ? gps.speed.kmph() : -1);
  spdStr += " km/h";

  String imuStr;
  imuStr.reserve(40);
  imuStr += (imuReady ? "ok" : "off");
  imuStr += " moving="; imuStr += (imuMoving ? "1" : "0");
  imuStr += " dG="; imuStr += String(lastADeltaG, 3);
  imuStr += " g=";  imuStr += String(lastGMag, 3);

  DeviceStateId st = alarmLatched ? ST_ALARM : (isBikeStationary() ? getCurrentCanonicalState() : ST_MOVING);

  String out;
  out.reserve(360);
  out += "{";
  out += "\"state\":\""; out += stateToString(st); out += "\",";
  out += "\"locked\":"; out += (locked ? "true" : "false"); out += ",";
  out += "\"armed\":";  out += (armed ? "true" : "false"); out += ",";
  out += "\"alarm\":";  out += (alarmLatched ? "true" : "false"); out += ",";
  out += "\"wifi\":";   out += (wifi ? "true" : "false"); out += ",";
  out += "\"gps\":\"";  out += gpsStr; out += "\",";
  out += "\"speed\":\""; out += spdStr; out += "\",";
  out += "\"imu\":\"";  out += imuStr; out += "\",";
  out += "\"buf\":\"";  out += String(rawCount); out += " raw\"";
  out += "}";

  localServer.send(200, "application/json", out);
}

static void localHandleCmd(const String& cmd) {
  localPortalMarkActivity();

  const uint32_t now = millis();
  if (now < localPinLockedUntil) {
    localServer.send(429, "text/plain", "PIN LOCKED, WAIT");
    return;
  }

  if (!localCheckPinOk()) {
    localServer.send(401, "text/plain", "BAD PIN");
    return;
  }

  executeCommand(cmd, "LOCAL");
  localServer.send(200, "text/plain", "OK");
}

static void localHandleNotFound() {
  localPortalMarkActivity();
  localServer.sendHeader("Location", String("http://") + WiFi.softAPIP().toString() + String("/"), true);
  localServer.send(302, "text/plain", "");
}

static void localPortalSetupRoutes() {
  localServer.on("/", HTTP_ANY, localHandleRoot);
  localServer.on("/status", HTTP_ANY, localHandleStatus);

  localServer.on("/api/lock", HTTP_ANY, [](){ localHandleCmd("LOCK"); });
  localServer.on("/api/unlock", HTTP_ANY, [](){ localHandleCmd("UNLOCK"); });
  localServer.on("/api/arm", HTTP_ANY, [](){ localHandleCmd("ARM"); });
  localServer.on("/api/buzz", HTTP_ANY, [](){ localHandleCmd("BUZZ"); });

  localServer.on("/generate_204", HTTP_ANY, localHandleRoot);
  localServer.on("/gen_204", HTTP_ANY, localHandleRoot);
  localServer.on("/hotspot-detect.html", HTTP_ANY, localHandleRoot);
  localServer.on("/connecttest.txt", HTTP_ANY, localHandleRoot);
  localServer.on("/ncsi.txt", HTTP_ANY, localHandleRoot);

  localServer.onNotFound(localHandleNotFound);
}

static void localPortalStart() {
  if (localApActive) return;

  char ssid[40];
  snprintf(ssid, sizeof(ssid), "%s-LOCAL", DEVICE_ID);

  WiFi.mode(WIFI_AP_STA);
  delay(50);

  const bool ok = WiFi.softAP(ssid, LOCAL_AP_PASS);
  if (!ok) return;

  localApActive = true;
  localApStartedAt = millis();
  localApLastActivityAt = millis();

  IPAddress ip = WiFi.softAPIP();
  localDns.start(53, "*", ip);

  localPortalSetupRoutes();
  localServer.begin();

  if (DBG_LOCAL) {
    Serial.print(F("[LOCAL] AP ON ssid="));
    Serial.print(ssid);
    Serial.print(F(" ip="));
    Serial.println(ip);
  }
}

static void localPortalStop() {
  if (!localApActive) return;

  localServer.stop();
  localDns.stop();
  WiFi.softAPdisconnect(true);

  localApActive = false;
  localApStartedAt = 0;
  localApLastActivityAt = 0;

  WiFi.mode(WIFI_STA);
  delay(50);

  if (DBG_LOCAL) Serial.println(F("[LOCAL] AP OFF"));
}

static void localPortalMaintenance() {
  const uint32_t now = millis();

  if (localApActive) {
    localDns.processNextRequest();
    localServer.handleClient();

    const int clients = WiFi.softAPgetStationNum();
    if (clients > 0) {
      localApLastActivityAt = now;
    }

    if (clients == 0 && (now - localApLastActivityAt >= LOCAL_AP_IDLE_OFF_MS)) {
      localPortalStop();
    }
    return;
  }

  if (now < localWakeCooldownUntil) return;
  if (wifiIsConnected()) return;

  if (!imuReady) return;
  if (!isSpeedLikelyLowForLocal()) return;

  const bool touchNow = (lastADeltaG > LOCAL_WAKE_DELTA_G) || (lastGMag > LOCAL_WAKE_GYRO_RADPS);

  if (touchNow) {
    if (localTouchSince == 0) localTouchSince = now;
    if (now - localTouchSince >= LOCAL_WAKE_TOUCH_MS) {
      localTouchSince = 0;
      localWakeCooldownUntil = now + LOCAL_WAKE_DEBOUNCE_MS;
      localPortalStart();
    }
  } else {
    localTouchSince = 0;
  }
}

// ============================================================
// Setup / Loop
// ============================================================
void setup() {
  Serial.begin(115200);
  delay(600);

  Serial.println(F("\n=== SmartBike Firmware v7.x + LOCAL UI ==="));

  btStop();
  esp_bt_controller_mem_release(ESP_BT_MODE_BTDM);

  pinMode(RELAY_PIN, OUTPUT);
  pinMode(BUZZER_RELAY_PIN, OUTPUT);
  if (LED_PIN >= 0) pinMode(LED_PIN, OUTPUT);
  if (ONBOARD_LED_PIN >= 0) pinMode(ONBOARD_LED_PIN, OUTPUT);

  relayOff();
  buzzerOff();
  setLed(false);

  SerialGPS.begin(GPS_BAUD, SERIAL_8N1, GPS_RX, GPS_TX);

  initIMU();
  if (!imuReady) logWarn(F("IMU not ready; security alarm disabled"));

  wifiInit();

  const uint32_t now = millis();
  lastSample    = now;
  lastPush      = now;
  lastPoll      = now;
  lastHeartbeat = now;

  Serial.println(F("=== Setup done ==="));
}

void loop() {
  gpsPump();
  wifiMaintenance();

  updateImuMotion();
  updateSecurityAlarm();
  pumpPendingReport();

  localPortalMaintenance();

  buzzerUpdate();

  static uint32_t lastBlink = 0;
  static bool ledState = false;

  const bool noNet = !wifiIsConnected();
  uint32_t blinkMs = 900;
  if (alarmLatched) blinkMs = 140;
  else if (noNet)   blinkMs = 250;

  if (millis() - lastBlink >= blinkMs) {
    lastBlink = millis();
    ledState = !ledState;
    setLed(ledState);
  }

  const uint32_t now = millis();
  const bool still = isBikeStationary();

  const uint32_t sampleInterval = still ? SAMPLE_INTERVAL_MS_STILL : SAMPLE_INTERVAL_MS_MOVING;

  const uint32_t pushInterval =
    noNet ? PUSH_INTERVAL_MS_NO_NET :
    (still ? PUSH_INTERVAL_MS_STILL : PUSH_INTERVAL_MS_MOVING);

  const uint32_t pollInterval =
    noNet ? POLL_INTERVAL_MS_NO_NET :
    (armed || alarmLatched ? POLL_INTERVAL_MS_FAST : POLL_INTERVAL_MS_NORMAL);

  if (now - lastSample >= sampleInterval) {
    lastSample = now;
    recordSamplePoint();
  }

  if (now - lastHeartbeat >= HEARTBEAT_INTERVAL_MS) {
    lastHeartbeat = now;
    if (still) recordHeartbeatPoint(F("STILL heartbeat buffered (1 point)"));
  }

  if (now - lastPush >= pushInterval) {
    lastPush = now;
    pushTrackBatch();
  }

  if (now - lastPoll >= pollInterval) {
    lastPoll = now;
    pollCommand();
  }

  if (DBG_GPS) {
    static uint32_t lastDbg = 0;
    if (millis() - lastDbg > 10000) {
      lastDbg = millis();

      Serial.print(F("[GPS] valid=")); Serial.print(gps.location.isValid());
      Serial.print(F(" age=")); Serial.print(gps.location.age());
      Serial.print(F(" sats=")); Serial.print(gps.satellites.isValid() ? gps.satellites.value() : -1);
      Serial.print(F(" hdop=")); Serial.print(gps.hdop.isValid() ? gps.hdop.hdop() : -1);
      Serial.print(F(" spd=")); Serial.print(gps.speed.isValid() ? gps.speed.kmph() : -1);
      Serial.print(F(" imuMoving=")); Serial.print(imuMoving);
      Serial.print(F(" locked=")); Serial.print(locked);
      Serial.print(F(" armed=")); Serial.print(armed);
      Serial.print(F(" alarm=")); Serial.print(alarmLatched);
      Serial.print(F(" rawCount=")); Serial.print(rawCount);
      Serial.print(F(" wifi=")); Serial.print(wifiIsConnected());
      Serial.print(F(" localAP=")); Serial.println(localApActive);
    }
  }
}