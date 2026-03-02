#include <Arduino.h>
#include <HardwareSerial.h>
#include <TinyGPSPlus.h>
#include <Wire.h>
#include "mpu6500.h"
#include <math.h>

// ========================v5==================================== 
// SmartBike Firmware (ESP32 + SIM800 + NEO GPS + MPU6500)
// - Robust HTTPDATA flow: DOWNLOAD -> send body -> OK -> HTTPACTION
// - push / poll / report endpoints
// - relay + buzzer (non-blocking) + LED blink
// - IMU motion gating + GPS speed filter
// - track buffer + compression
// - heartbeat every 5 minutes while still
// ============================================================

// ================== SERVER CONFIG ==================
static const char* SERVER_URL_PUSH   = "http://smartbike.ashikdev.com/api/device/push";
static const char* SERVER_URL_POLL   = "http://smartbike.ashikdev.com/api/device/poll";
static const char* SERVER_URL_REPORT = "http://smartbike.ashikdev.com/api/device/report";
static const char* DEVICE_ID         = "BIKE01";

// ================== DEBUG ==================
static constexpr bool DBG_SIM_AT    = true;
static constexpr bool DBG_HTTP_BODY = true;
static constexpr bool DBG_GPS       = true;
static constexpr bool DBG_IMU       = true;

// ================== UART PINS ==================
static constexpr int      GPS_RX   = 16;
static constexpr int      GPS_TX   = 17;
static constexpr uint32_t GPS_BAUD = 9600;
HardwareSerial SerialGPS(1);

static constexpr int      SIM_RX            = 26;
static constexpr int      SIM_TX            = 27;
static constexpr uint32_t SIM_BAUD_PRIMARY  = 9600;
static constexpr uint32_t SIM_BAUD_FALLBACK = 115200;
HardwareSerial SerialSIM(2);

TinyGPSPlus gps;

// ================== TRACK BUFFER TYPES (MUST BE BEFORE FUNCTIONS) ==================
struct TrackPoint {
  double   lat;
  double   lon;
  float    speed;
  uint32_t tMs;
};

static constexpr int MAX_RAW_POINTS   = 20;
static constexpr int MAX_TRACK_POINTS = 5;
static TrackPoint rawBuf[MAX_RAW_POINTS];
static int rawCount = 0;

// ================== PROTOTYPES (avoid Arduino auto-prototype bugs) ==================
static void gpsPump();

static inline float mag3(float x, float y, float z);
static inline double deg2rad(double deg);
static double distanceMeters(double lat1, double lon1, double lat2, double lon2);
static bool isInsideBangladesh(double lat, double lon);
static bool isGoodFix();

static float getFilteredSpeedKmph();
static bool isBikeStationary();

static void recordSamplePoint();
static int buildCompressedTrack(TrackPoint* outArr, int outMax);
static void recordHeartbeatPoint(const __FlashStringHelper* reason);

static void simFlushInput();
static bool simReadLine(uint32_t timeoutMs, String& outLine);
static bool simCmd(const String& cmd, uint32_t timeoutMs, String* outAll = nullptr);
static bool simWaitLineContains(const char* token, uint32_t timeoutMs);
static bool simWaitOk(uint32_t timeoutMs);
static bool simTryBaud(uint32_t baud);
static bool simAutoBaud();
static bool simWaitRegistered(uint32_t timeoutMs);
static bool simGetImsi(String& imsiOut);
static void autoConfigureApn();
static bool bearerHasIP(String* outIp = nullptr);

static bool initGPRS();
static bool ensureGPRS();
static bool httpSetUrl(const char* url);
static bool httpReadBodyExact(int expectedLen, String& outBody);
static bool httpPostJSON(const char* url, const String& json, String* outBody);

static String jsonFindString(const String& body, const char* key);

static void startBuzz(uint32_t durationMs);
static void reportCommandDone(const String& commandId, const String& event);
static void executeCommand(const String& cmd, const String& cmdId);
static void pushTrackBatch();
static void pollCommand();

static void initIMU();
static void calibrateGyroBias();
static void updateImuMotion();

// ================== I2C / IMU ==================
static constexpr int I2C_SDA = 21;
static constexpr int I2C_SCL = 22;

bfs::Mpu6500 imu(&Wire, bfs::Mpu6500::I2C_ADDR_PRIM); // 0x68

struct GyroBias { float x=0, y=0, z=0; };
static GyroBias imuBias;
static bool imuReady = false;

static bool     imuMoving = false;
static uint32_t imuMovingHoldUntil = 0;
static uint32_t lastImuReadAt = 0;

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

// ================== TIMERS ==================
static constexpr uint32_t SAMPLE_INTERVAL_MS    = 1000UL;
static constexpr uint32_t PUSH_INTERVAL_MS      = 20000UL;
static constexpr uint32_t POLL_INTERVAL_MS      = 10000UL;
static constexpr uint32_t HEARTBEAT_INTERVAL_MS = 5UL * 60UL * 1000UL;

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

static inline void relayOn()   { digitalWrite(RELAY_PIN,        RELAY_ACTIVE_LOW ? LOW  : HIGH); }
static inline void relayOff()  { digitalWrite(RELAY_PIN,        RELAY_ACTIVE_LOW ? HIGH : LOW); }
static inline void buzzerOn()  { digitalWrite(BUZZER_RELAY_PIN, RELAY_ACTIVE_LOW ? LOW  : HIGH); }
static inline void buzzerOff() { digitalWrite(BUZZER_RELAY_PIN, RELAY_ACTIVE_LOW ? HIGH : LOW); }

static inline void setLed(bool on) {
  if (LED_PIN >= 0) digitalWrite(LED_PIN, on ? HIGH : LOW);
  if (ONBOARD_LED_PIN >= 0) digitalWrite(ONBOARD_LED_PIN, on ? HIGH : LOW);
}

static bool     buzzerActive = false;
static uint32_t buzzerOffAt  = 0;
static constexpr uint32_t BUZZ_DURATION_MS = 8000;

// ================== SIM / GPRS ==================
static String gprsApn = "blweb";
static const char* GPRS_USER = "";
static const char* GPRS_PASS = "";
static bool gprsReady = false;

// ================== SPEED FILTER ==================
static constexpr float MIN_SPEED_KMPH       = 2.0f;
static constexpr int   STOP_COUNT_REQ       = 4;
static constexpr float MAX_VALID_SPEED_KMPH = 130.0f;
static constexpr float SPEED_ALPHA          = 0.4f;

static int   lowSpeedCount   = 0;
static float filteredSpeed   = 0.0f;
static bool  hasFilteredInit = false;

// ================== LOG HELPERS ==================
static inline void logInfo(const __FlashStringHelper* msg) { Serial.print(F("[INFO] ")); Serial.println(msg); }
static inline void logWarn(const __FlashStringHelper* msg) { Serial.print(F("[WARN] ")); Serial.println(msg); }
static inline void logErr (const __FlashStringHelper* msg) { Serial.print(F("[ERR] "));  Serial.println(msg); }

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
  Wire.setClock(100000);

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
      imuMovingHoldUntil = now + 150;
    }
  }
}

// ============================================================
// SPEED FILTER
// ============================================================
static float getFilteredSpeedKmph() {
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

// ============================================================
// Track sampling + compression
// ============================================================
static void addPointToBuffer(double lat, double lon, float spd) {
  if (rawCount < MAX_RAW_POINTS) {
    rawBuf[rawCount++] = {lat, lon, spd, millis()};
  } else {

    rawBuf[MAX_RAW_POINTS - 1] = {lat, lon, spd, millis()};
  }
}

static void recordSamplePoint() {
  if (isBikeStationary()) return;
  if (!isGoodFix()) return;

    rawBuf[MAX_RAW_POINTS - 1] = {lat, lon, spd, millis()};
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

  static constexpr float  MIN_SAMPLE_DIST = 15.0f;
  static constexpr double MAX_JUMP_DIST   = 50000.0;

  if (rawCount > 0) {
    const double d = distanceMeters(rawBuf[rawCount - 1].lat, rawBuf[rawCount - 1].lon, lat, lon);
    if (d > MAX_JUMP_DIST) return;
    if (d < MIN_SAMPLE_DIST) return;
  }

  addPointToBuffer(lat, lon, spd);
}

static int buildCompressedTrack(TrackPoint* outArr, int outMax) {
  if (!outArr || outMax <= 0) return 0;
  if (rawCount <= 0) return 0;

  if (rawCount <= outMax) {
    for (int i=0; i<rawCount; i++) outArr[i] = rawBuf[i];
    return rawCount;
  }

  static constexpr float MIN_CHOSEN_DIST = 15.0f;

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

  addPointToBuffer(lat, lon, 0.0f);

  Serial.print(F("[GPS][HB] "));
  Serial.println(reason);
}

// ============================================================
// SIM / HTTP (same logic as your stable recharge-style)
// ============================================================
static void simFlushInput() {
  while (SerialSIM.available()) (void)SerialSIM.read();
}

static bool simReadLine(uint32_t timeoutMs, String& outLine) {
  outLine = "";
  const uint32_t start = millis();

  while (millis() - start < timeoutMs) {
    gpsPump();

    while (SerialSIM.available()) {
      const char c = (char)SerialSIM.read();
      if (c == '\r') continue;

      if (c == '\n') {
        outLine.trim();
        if (outLine.length() == 0) break;
        return true;
      }

      outLine += c;
      if (outLine.length() > 220) {
        outLine.trim();
        return true;
      }
    }

    delay(2);
  }

  outLine.trim();
  return outLine.length() > 0;
}

static bool simCmd(const String& cmd, uint32_t timeoutMs, String* outAll) {
  if (DBG_SIM_AT) {
    Serial.print(F("\n[SIM] >> "));
    Serial.println(cmd);
  }

  simFlushInput();
  SerialSIM.print(cmd);
  SerialSIM.print("\r\n");

  String all;
  String line;
  const uint32_t start = millis();

  while (millis() - start < timeoutMs) {
    if (!simReadLine(2500, line)) continue;

    if (DBG_SIM_AT) {
      Serial.print(F("[SIM][LINE] "));
      Serial.println(line);
    }

    all += line;
    all += '\n';

    if (line == "OK") { if (outAll) *outAll = all; return true; }
    if (line.indexOf("ERROR") >= 0 || line.indexOf("+CME ERROR") >= 0) { if (outAll) *outAll = all; return false; }
  }

  if (outAll) *outAll = all;
  return false;
}

static bool simWaitLineContains(const char* token, uint32_t timeoutMs) {
  String line;
  const uint32_t start = millis();
  while (millis() - start < timeoutMs) {
    if (!simReadLine(2500, line)) continue;

    if (DBG_SIM_AT) {
      Serial.print(F("[SIM][LINE] "));
      Serial.println(line);
    }

    if (line.indexOf("ERROR") >= 0 || line.indexOf("+CME ERROR") >= 0) return false;
    if (line.indexOf(token) >= 0) return true;
  }
  return false;
}

static bool simWaitOk(uint32_t timeoutMs) {
  String line;
  const uint32_t start = millis();
  while (millis() - start < timeoutMs) {
    if (!simReadLine(2500, line)) continue;

    if (DBG_SIM_AT) {
      Serial.print(F("[SIM][LINE] "));
      Serial.println(line);
    }

    if (line == "OK") return true;
    if (line.indexOf("ERROR") >= 0 || line.indexOf("+CME ERROR") >= 0) return false;
  }
  return false;
}

static bool simTryBaud(uint32_t baud) {
  SerialSIM.end();
  delay(80);
  SerialSIM.begin(baud, SERIAL_8N1, SIM_RX, SIM_TX);
  delay(250);

  Serial.print(F("[SIM] Trying baud="));
  Serial.println(baud);

  for (int i = 0; i < 3; i++) {
    if (simCmd("AT", 1200)) return true;
    delay(150);
  }
  return false;
}

static bool simAutoBaud() {
  if (simTryBaud(SIM_BAUD_PRIMARY))  { Serial.println(F("[SIM] Baud locked: 9600")); return true; }
  if (simTryBaud(SIM_BAUD_FALLBACK)) { Serial.println(F("[SIM] Baud locked: 115200")); return true; }
  logErr(F("SIM800 no response on 9600/115200. Check wiring/power."));
  return false;
}

static bool simWaitRegistered(uint32_t timeoutMs) {
  const uint32_t start = millis();
  String all;
  while (millis() - start < timeoutMs) {
    if (simCmd("AT+CREG?", 2000, &all)) {
      if (all.indexOf(",1") >= 0 || all.indexOf(",5") >= 0) return true;
    }
    delay(700);
  }
  return false;
}

static bool simGetImsi(String& imsiOut) {
  imsiOut = "";
  if (DBG_SIM_AT) Serial.println(F("\n[SIM] >> AT+CIMI"));

  simFlushInput();
  SerialSIM.print("AT+CIMI\r\n");

  String line;
  const uint32_t start = millis();
  while (millis() - start < 4000) {
    if (!simReadLine(2500, line)) continue;

    if (DBG_SIM_AT) {
      Serial.print(F("[SIM][LINE] "));
      Serial.println(line);
    }

    if (line == "OK") break;
    if (line.indexOf("ERROR") >= 0 || line.indexOf("+CME ERROR") >= 0) return false;

    bool allDigits = true;
    for (int i=0; i<(int)line.length(); i++) if (!isDigit(line[i])) { allDigits = false; break; }
    if (allDigits && line.length() >= 10) imsiOut = line;
  }
  return imsiOut.length() >= 10;
}

static void autoConfigureApn() {
  String imsi;
  if (!simGetImsi(imsi)) {
    Serial.println(F("[APN] IMSI not available -> keep default APN"));
    return;
  }

  if      (imsi.startsWith("47001")) gprsApn = "gpinternet";
  else if (imsi.startsWith("47003")) gprsApn = "blweb";
  else if (imsi.startsWith("47002")) gprsApn = "internet";
  else if (imsi.startsWith("47004")) gprsApn = "internet";

  Serial.print(F("[APN] Using APN="));
  Serial.println(gprsApn);
}

static bool bearerHasIP(String* outIp) {
  simFlushInput();
  if (DBG_SIM_AT) Serial.println(F("\n[SIM] >> AT+SAPBR=2,1"));
  SerialSIM.print("AT+SAPBR=2,1\r\n");

  String line;
  String ip;
  int status = -1;

  const uint32_t start = millis();
  while (millis() - start < 7000) {
    if (!simReadLine(2500, line)) continue;

    if (DBG_SIM_AT) {
      Serial.print(F("[SIM][LINE] "));
      Serial.println(line);
    }

    if (line.startsWith("+SAPBR:")) {
      const int c1 = line.indexOf(',');
      const int c2 = line.indexOf(',', c1 + 1);
      if (c1 > 0 && c2 > c1) status = line.substring(c1 + 1, c2).toInt();

      const int q1 = line.indexOf('"');
      const int q2 = line.indexOf('"', q1 + 1);
      if (q1 >= 0 && q2 > q1) ip = line.substring(q1 + 1, q2);
    }

    if (line == "OK") break;
    if (line.indexOf("ERROR") >= 0 || line.indexOf("+CME ERROR") >= 0) break;
  }

  if (outIp) *outIp = ip;
  if (status != 1) return false;
  if (ip.length() == 0) return false;
  if (ip == "0.0.0.0") return false;
  return ip.indexOf('.') >= 0;
}

static bool initGPRS() {
  Serial.println(F("== GPRS INIT START =="));

  (void)simCmd("ATE0", 1500);
  (void)simCmd("AT+CMEE=2", 1500);

  if (!simCmd("AT+CPIN?", 5000)) return false;
  (void)simCmd("AT+CSQ", 2500);

  if (simWaitRegistered(20000)) Serial.println(F("[SIM] Network registered."));
  else Serial.println(F("[SIM] WARN: Network not confirmed yet."));

  autoConfigureApn();

  {
    const String cgd = String("AT+CGDCONT=1,\"IP\",\"") + gprsApn + "\"";
    (void)simCmd(cgd, 5000);
  }

  (void)simCmd("AT+HTTPTERM", 2500);
  (void)simCmd("AT+SAPBR=0,1", 8000);

  if (!simCmd("AT+SAPBR=3,1,\"CONTYPE\",\"GPRS\"", 5000)) return false;
  {
    const String apnCmd = String("AT+SAPBR=3,1,\"APN\",\"") + gprsApn + "\"";
    if (!simCmd(apnCmd, 5000)) return false;
  }

  if (!simCmd("AT+SAPBR=1,1", 65000)) return false;

  String ip;
  bool okIp = false;
  for (int i=0; i<4; i++) {
    if (bearerHasIP(&ip)) { okIp = true; break; }
    delay(700);
  }

  if (!okIp) return false;

  Serial.print(F("[SIM] Bearer IP="));
  Serial.println(ip);

  if (!simCmd("AT+HTTPINIT", 8000)) return false;
  if (!simCmd("AT+HTTPPARA=\"CID\",1", 4000)) return false;
  if (!simCmd("AT+HTTPPARA=\"CONTENT\",\"application/json\"", 4000)) return false;

  Serial.println(F("== GPRS INIT OK =="));
  return true;
}

static bool ensureGPRS() {
  String ip;
  if (gprsReady && bearerHasIP(&ip)) return true;
  logWarn(F("GPRS not ready -> re-init"));
  gprsReady = initGPRS();
  return gprsReady;
}

static bool httpSetUrl(const char* url) {
  const String cmd = String("AT+HTTPPARA=\"URL\",\"") + url + "\"";
  return simCmd(cmd, 6000);
}

static bool httpReadBodyExact(int expectedLen, String& outBody) {
  outBody = "";
  if (expectedLen <= 0) return true;

  simFlushInput();
  const String cmd = String("AT+HTTPREAD=0,") + expectedLen;

  if (DBG_SIM_AT) {
    Serial.print(F("\n[SIM] >> "));
    Serial.println(cmd);
  }

  SerialSIM.print(cmd);
  SerialSIM.print("\r\n");

  String line;
  bool headerOk = false;
  const uint32_t start = millis();
  while (millis() - start < 12000) {
    if (!simReadLine(2500, line)) continue;

    if (DBG_SIM_AT) {
      Serial.print(F("[SIM][LINE] "));
      Serial.println(line);
    }

    if (line.startsWith("+HTTPREAD:")) { headerOk = true; break; }
    if (line.indexOf("ERROR") >= 0 || line.indexOf("+CME ERROR") >= 0) return false;
  }
  if (!headerOk) return false;

  delay(30);

  char* buf = (char*)malloc((size_t)expectedLen + 1);
  if (!buf) return false;

  size_t got = 0;
  const uint32_t t0 = millis();
  while (millis() - t0 < 12000 && got < (size_t)expectedLen) {
    gpsPump();
    while (SerialSIM.available() && got < (size_t)expectedLen) {
      buf[got++] = (char)SerialSIM.read();
    }
    delay(2);
  }

  if (got < (size_t)expectedLen) { free(buf); return false; }

  buf[expectedLen] = '\0';
  outBody = String(buf);
  free(buf);

  (void)simWaitOk(2500);

  if (DBG_HTTP_BODY) {
    Serial.print(F("[HTTP] body("));
    Serial.print(expectedLen);
    Serial.print(F(")="));
    Serial.println(outBody);
  }

  return true;
}

static bool httpPostJSON(const char* url, const String& json, String* outBody) {
  if (!ensureGPRS()) return false;
  if (!httpSetUrl(url)) { gprsReady = false; return false; }

  simFlushInput();
  const String lenCmd = String("AT+HTTPDATA=") + json.length() + ",10000";

  if (DBG_SIM_AT) {
    Serial.print(F("\n[SIM] >> "));
    Serial.println(lenCmd);
  }

  SerialSIM.print(lenCmd);
  SerialSIM.print("\r\n");

  if (!simWaitLineContains("DOWNLOAD", 15000)) {
    gprsReady = false;
    return false;
  }

  SerialSIM.print(json);
  if (!simWaitOk(15000)) {
    gprsReady = false;
    return false;
  }

  simFlushInput();
  if (DBG_SIM_AT) Serial.println(F("\n[SIM] >> AT+HTTPACTION=1"));
  SerialSIM.print("AT+HTTPACTION=1\r\n");

  String line;
  int status = -1;
  int length = -1;

  const uint32_t start = millis();
  while (millis() - start < 35000) {
    if (!simReadLine(4000, line)) continue;

    if (DBG_SIM_AT) {
      Serial.print(F("[SIM][LINE] "));
      Serial.println(line);
    }

    if (line.startsWith("+HTTPACTION:")) {
      const int c1 = line.indexOf(',');
      const int c2 = line.indexOf(',', c1 + 1);
      if (c1 > 0 && c2 > c1) {
        status = line.substring(c1 + 1, c2).toInt();
        length = line.substring(c2 + 1).toInt();
      }
      break;
    }

    if (line.indexOf("ERROR") >= 0 || line.indexOf("+CME ERROR") >= 0) break;
  }

  Serial.print(F("[HTTP] status="));
  Serial.print(status);
  Serial.print(F(" len="));
  Serial.println(length);

  if (outBody) *outBody = "";
  if (length > 0 && outBody) {
    String body;
    if (!httpReadBodyExact(length, body)) { gprsReady = false; return false; }
    *outBody = body;
  } else if (length > 0) {
    String junk;
    if (!httpReadBodyExact(length, junk)) { gprsReady = false; return false; }
  }

  if (status == 601) gprsReady = false;
  return (status == 200);
}

// ============================================================
// JSON parse
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
static void startBuzz(uint32_t durationMs) {
  buzzerActive = true;
  buzzerOffAt = millis() + durationMs;
  buzzerOn();
}

static void reportCommandDone(const String& commandId, const String& event) {
  if (commandId.length() == 0 || commandId == "null") return;

  String json;
  json.reserve(180);
  json += "{";
  json += "\"deviceId\":\"";  json += DEVICE_ID;   json += "\",";
  json += "\"commandId\":\""; json += commandId;   json += "\",";
  json += "\"event\":\"";     json += event;       json += "\"";
  json += "}";

  String body;
  (void)httpPostJSON(SERVER_URL_REPORT, json, &body);
}

static void executeCommand(const String& cmd, const String& cmdId) {
  if (cmd != "BUZZ") {
    buzzerActive = false;
    buzzerOff();
  }

  if (cmd == "LOCK") {
    relayOn();
    setLed(false);
    reportCommandDone(cmdId, "LOCK_DONE");
  } else if (cmd == "UNLOCK") {
    relayOff();
    setLed(true);
    reportCommandDone(cmdId, "UNLOCK_DONE");
  } else if (cmd == "BUZZ") {
    startBuzz(BUZZ_DURATION_MS);
    reportCommandDone(cmdId, "BUZZ_ON");
  } else {
    reportCommandDone(cmdId, "UNKNOWN_CMD");
  }
}

static void pushTrackBatch() {
  if (rawCount == 0) return;

  TrackPoint track[MAX_TRACK_POINTS];
  const int n = buildCompressedTrack(track, MAX_TRACK_POINTS);
  if (n <= 0) { rawCount = 0; return; }

  String json;
  json.reserve(560);

  json += "{";
  json += "\"deviceId\":\""; json += DEVICE_ID; json += "\",";
  json += "\"state\":\"RUNNING\",";
  json += "\"track\":[";

  for (int i=0; i<n; i++) {
    if (i) json += ",";
json += "{";
json += "\"lat\":";   json += String(track[i].lat, 6);   json += ",";
json += "\"lon\":";   json += String(track[i].lon, 6);   json += ",";
json += "\"speed\":"; json += String(track[i].speed, 2); json += ",";
json += "\"t\":";     json += String(track[i].tMs);
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
  String json;
  json.reserve(64);
  json += "{\"deviceId\":\"";
  json += DEVICE_ID;
  json += "\"}";

  String body;
  const bool ok = httpPostJSON(SERVER_URL_POLL, json, &body);
  if (!ok) return;

  if (DBG_HTTP_BODY) {
    Serial.print(F("[HTTP] poll body="));
    Serial.println(body);
  }

  const String command   = jsonFindString(body, "command");
  const String commandId = jsonFindString(body, "commandId");

  if (command.length() == 0 || command == "null") return;
  executeCommand(command, commandId);
}

// ============================================================
// Setup / Loop
// ============================================================
void setup() {
  Serial.begin(115200);
  delay(700);

  Serial.println(F("\n=== SmartBike Firmware (compile-safe) ==="));

  pinMode(RELAY_PIN, OUTPUT);
  pinMode(BUZZER_RELAY_PIN, OUTPUT);
  if (LED_PIN >= 0) pinMode(LED_PIN, OUTPUT);
  if (ONBOARD_LED_PIN >= 0) pinMode(ONBOARD_LED_PIN, OUTPUT);

  relayOff();
  buzzerOff();
  setLed(false);

  SerialGPS.begin(GPS_BAUD, SERIAL_8N1, GPS_RX, GPS_TX);

  initIMU();
  if (!imuReady) logWarn(F("IMU not ready; fallback to GPS speed gating"));

  Serial.println(F("[SIM] Auto-baud..."));
  if (!simAutoBaud()) {
    logErr(F("SIM init failed. Fix wiring/power then reboot."));
  } else {
    delay(1200);
    gprsReady = initGPRS();
    if (!gprsReady) logWarn(F("GPRS not ready at boot; will retry on demand"));
    else logInfo(F("GPRS ready"));
  }

  lastSample    = millis();
  lastPush      = millis();
  lastPoll      = millis();
  lastHeartbeat = millis();

  Serial.println(F("=== Setup done ==="));
}

void loop() {
  gpsPump();
  updateImuMotion();

  if (buzzerActive && millis() >= buzzerOffAt) {
    buzzerActive = false;
    buzzerOff();
  }

  static uint32_t lastBlink = 0;
  static bool ledState = false;
  const uint32_t blinkMs = gprsReady ? 900 : 250;
  if (millis() - lastBlink >= blinkMs) {
    lastBlink = millis();
    ledState = !ledState;
    setLed(ledState);
  }

  const uint32_t now = millis();

  if (now - lastSample >= SAMPLE_INTERVAL_MS) {
    lastSample = now;
    recordSamplePoint();
  }

  if (now - lastHeartbeat >= HEARTBEAT_INTERVAL_MS) {
    lastHeartbeat = now;
    if (isBikeStationary()) {
      recordHeartbeatPoint(F("STILL heartbeat buffered (1 point)"));
    }
  }

  if (now - lastPush >= PUSH_INTERVAL_MS) {
    lastPush = now;
    pushTrackBatch();
  }

  if (now - lastPoll >= POLL_INTERVAL_MS) {
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
      Serial.print(F(" spd=")); Serial.print(gps.speed.kmph());
      Serial.print(F(" rawCount=")); Serial.println(rawCount);
    }
  }
}
