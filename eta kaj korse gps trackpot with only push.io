#include <HardwareSerial.h>
#include <TinyGPSPlus.h>

// ================== CONFIG ==================

// server url (HTTP only for SIM800)
const char* SERVER_URL = "http://smartbike.ashikdev.com/api/device/push";
const char* DEVICE_ID  = "BIKE01";

// SIM APN (Banglalink)
const char* GPRS_APN  = "blweb";
const char* GPRS_USER = "";
const char* GPRS_PASS = "";

// GPS -> Serial1 (ESP32)
//  RX1 = GPIO16 (GPS TX)
//  TX1 = GPIO17 (GPS RX)
HardwareSerial SerialGPS(1);

// SIM800 -> Serial2 (ESP32)
//  RX2 = GPIO26 (SIM TX)
//  TX2 = GPIO27 (SIM RX)
HardwareSerial SerialSIM(2);

TinyGPSPlus gps;

const unsigned long PUSH_INTERVAL_MS = 20000;
unsigned long lastPush = 0;

bool gprsReady = false;

// ================== SPEED FILTER CONFIG ==================
const float MIN_SPEED_KMPH = 2.0;     // <2 km/h => treat as 0 (noise)
const int   STOP_COUNT_REQ = 3;       // consecutive low readings to lock stop

int   lowSpeedCount = 0;
float filteredSpeed = 0;

// ================== LOG CONTROL ==================
// Keep only important logs.
// Set to 1 if you want more logs later.
#define LOG_LEVEL 0

void logInfo(const __FlashStringHelper* msg) {
  Serial.println(msg);
}
void logWarn(const __FlashStringHelper* msg) {
  Serial.print(F("[WARN] ")); Serial.println(msg);
}
void logErr(const __FlashStringHelper* msg) {
  Serial.print(F("[ERR] ")); Serial.println(msg);
}

// ================== AT HELPER ==================
bool sendAT(const char* cmd,
            const char* expect,
            uint32_t timeout = 3000,
            String* outResp = nullptr)
{
  while (SerialSIM.available()) SerialSIM.read();

#if LOG_LEVEL
  Serial.print(F("[AT] >> ")); Serial.println(cmd);
#endif

  SerialSIM.print(cmd);
  SerialSIM.print("\r\n");

  uint32_t start = millis();
  String resp;

  while (millis() - start < timeout) {
    while (SerialSIM.available()) {
      resp += (char)SerialSIM.read();
    }
    if (resp.indexOf(expect) != -1) {
#if LOG_LEVEL
      Serial.print(F("[AT] << ")); Serial.println(resp);
#endif
      if (outResp) *outResp = resp;
      return true;
    }
  }

  // Only print on FAIL
  Serial.print(F("[AT][TIMEOUT] ")); Serial.println(cmd);
  Serial.print(F("[AT][RESP] ")); Serial.println(resp);

  if (outResp) *outResp = resp;
  return false;
}

bool waitForResponse(const char* expect, uint32_t timeout = 3000)
{
  uint32_t start = millis();
  String resp;

  while (millis() - start < timeout) {
    while (SerialSIM.available()) {
      resp += (char)SerialSIM.read();
    }
    if (resp.indexOf(expect) != -1) return true;
  }

  // Only on FAIL
  Serial.print(F("[WAIT][TIMEOUT] expect="));
  Serial.println(expect);
  Serial.print(F("[WAIT][RESP] "));
  Serial.println(resp);
  return false;
}

// Check bearer has IP (robustness)
bool bearerHasIP() {
  String r;
  if (!sendAT("AT+SAPBR=2,1", "SAPBR", 5000, &r)) return false;
  // crude but effective: IP contains dot and quotes
  return (r.indexOf('.') != -1);
}

// ================== INIT GPRS + HTTP ==================
bool initGPRS()
{
  logInfo(F("== GPRS INIT START =="));

  // STEP-1 AT / autobaud
  bool ok = false;
  for (int i = 0; i < 5 && !ok; i++) {
    ok = sendAT("AT", "OK", 2000);
    if (!ok) delay(500);
  }
  if (!ok) {
    logErr(F("SIM800 no response. Check power/wiring/baud."));
    return false;
  }

  sendAT("ATE0", "OK", 2000);   // echo off
  sendAT("AT+CPIN?", "+CPIN", 5000);

  // Check signal + registration quickly
  String csqResp, cregResp, cgattResp;
  sendAT("AT+CSQ", "+CSQ", 2000, &csqResp);
  sendAT("AT+CREG?", "+CREG", 5000, &cregResp);
  sendAT("AT+CGATT?", "+CGATT", 5000, &cgattResp);

  // Show short summary
  Serial.print(F("[GPRS] ")); Serial.print(csqResp);
  Serial.print(F("[GPRS] ")); Serial.print(cregResp);
  Serial.print(F("[GPRS] ")); Serial.print(cgattResp);

  // PDP context
  {
    String cgdcontCmd = String("AT+CGDCONT=1,\"IP\",\"") + GPRS_APN + "\"";
    sendAT(cgdcontCmd.c_str(), "OK", 5000);
  }

  // Close old bearer (ignore error)
  sendAT("AT+SAPBR=0,1", "OK", 5000);

  // Bearer config
  if (!sendAT("AT+SAPBR=3,1,\"CONTYPE\",\"GPRS\"", "OK", 5000)) {
    logErr(F("SAPBR CONTYPE fail"));
    return false;
  }

  {
    String apnCmd = String("AT+SAPBR=3,1,\"APN\",\"") + GPRS_APN + "\"";
    if (!sendAT(apnCmd.c_str(), "OK", 5000)) {
      logErr(F("APN set fail"));
      return false;
    }
  }

  if (strlen(GPRS_USER) > 0) {
    String userCmd = String("AT+SAPBR=3,1,\"USER\",\"") + GPRS_USER + "\"";
    sendAT(userCmd.c_str(), "OK", 5000);
  }
  if (strlen(GPRS_PASS) > 0) {
    String passCmd = String("AT+SAPBR=3,1,\"PWD\",\"") + GPRS_PASS + "\"";
    sendAT(passCmd.c_str(), "OK", 5000);
  }

  // Open bearer (long wait)
  if (!sendAT("AT+SAPBR=1,1", "OK", 60000)) {
    logErr(F("Bearer open fail (SAPBR=1,1)"));
    return false;
  }

  // HTTP init
  sendAT("AT+HTTPTERM", "OK", 2000);   // ignore fail
  if (!sendAT("AT+HTTPINIT", "OK", 5000)) {
    logErr(F("HTTPINIT fail"));
    return false;
  }

  if (!sendAT("AT+HTTPPARA=\"CID\",1", "OK", 5000)) {
    logErr(F("HTTP CID fail"));
    return false;
  }

  {
    String urlCmd = String("AT+HTTPPARA=\"URL\",\"") + SERVER_URL + "\"";
    if (!sendAT(urlCmd.c_str(), "OK", 5000)) {
      logErr(F("HTTP URL set fail"));
      return false;
    }
  }

  if (!sendAT("AT+HTTPPARA=\"CONTENT\",\"application/json\"", "OK", 5000)) {
    logErr(F("HTTP CONTENT fail"));
    return false;
  }

  logInfo(F("== GPRS INIT OK =="));
  return true;
}

// ================== HTTP POST JSON ==================
bool httpPostJSON(const String& json)
{
  // If bearer dropped, re-init
  if (!gprsReady || !bearerHasIP()) {
    logWarn(F("GPRS not ready, re-init..."));
    gprsReady = initGPRS();
    if (!gprsReady) {
      logErr(F("GPRS init failed, skip POST"));
      return false;
    }
  }

  // HTTPDATA
  String lenCmd = String("AT+HTTPDATA=") + json.length() + ",10000";
  if (!sendAT(lenCmd.c_str(), "DOWNLOAD", 15000)) {
    logErr(F("HTTPDATA no DOWNLOAD"));
    gprsReady = false;
    return false;
  }

  // payload
  SerialSIM.print(json);

  if (!waitForResponse("OK", 15000)) {
    logErr(F("No OK after payload"));
    gprsReady = false;
    return false;
  }

  // POST action
  String httpActionResp;
  if (!sendAT("AT+HTTPACTION=1", "HTTPACTION", 30000, &httpActionResp)) {
    logErr(F("HTTPACTION timeout"));
    gprsReady = false;
    return false;
  }

  // Parse status
  int idx = httpActionResp.indexOf("+HTTPACTION:");
  int status = -1, length = -1;
  if (idx >= 0) {
    int c1 = httpActionResp.indexOf(',', idx);
    int c2 = httpActionResp.indexOf(',', c1 + 1);
    if (c1 > 0 && c2 > c1) {
      status = httpActionResp.substring(c1 + 1, c2).toInt();
      length = httpActionResp.substring(c2 + 1).toInt();
    }
  }

  // Important log only
  Serial.print(F("[HTTP] status="));
  Serial.print(status);
  Serial.print(F(" len="));
  Serial.println(length);

  if (status != 200) {
    logWarn(F("Non-200 HTTP response"));
    sendAT("AT+HTTPREAD", "OK", 5000); // try to read error body
  }

  return (status == 200);
}

// ================== SPEED FILTER ==================
float getFilteredSpeedKmph()
{
  float rawSpeed = gps.speed.kmph();

  if (rawSpeed < MIN_SPEED_KMPH) {
    lowSpeedCount++;
    if (lowSpeedCount >= STOP_COUNT_REQ) {
      filteredSpeed = 0;
    }
  } else {
    lowSpeedCount = 0;
    filteredSpeed = rawSpeed;
  }

  return filteredSpeed;
}

// ================== SETUP ==================
void setup()
{
  Serial.begin(115200);
  delay(1500);
  logInfo(F("Smart Bike – GPS + GPRS (clean logs + speed filter)"));

  SerialGPS.begin(9600, SERIAL_8N1, 16, 17);
  SerialSIM.begin(115200, SERIAL_8N1, 26, 27);

  delay(3000);

  gprsReady = initGPRS();
  if (!gprsReady) logWarn(F("GPRS not ready at boot"));
}

// ================== LOOP ==================
void loop()
{
  while (SerialGPS.available()) {
    gps.encode((char)SerialGPS.read());
  }

  unsigned long now = millis();
  if (now - lastPush >= PUSH_INTERVAL_MS) {
    lastPush = now;

    if (gps.location.isValid() && gps.location.age() < 10000) {
      double lat = gps.location.lat();
      double lon = gps.location.lng();
      float speed = getFilteredSpeedKmph();

      // Important GPS log only
      Serial.print(F("[GPS] "));
      Serial.print(lat, 6);
      Serial.print(F(", "));
      Serial.print(lon, 6);
      Serial.print(F(" speed="));
      Serial.println(speed, 2);

      // JSON payload
      String json = "{";
      json += "\"deviceId\":\""; json += DEVICE_ID; json += "\",";
      json += "\"lat\":";   json += String(lat, 6);   json += ",";
      json += "\"lon\":";   json += String(lon, 6);   json += ",";
      json += "\"speed\":"; json += String(speed, 2); json += ",";
      json += "\"state\":\"ARMED\"";
      json += "}";

      httpPostJSON(json);

    } else {
      logWarn(F("No valid GPS fix yet"));
    }
  }
}
