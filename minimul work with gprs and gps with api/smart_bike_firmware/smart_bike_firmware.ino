#include <HardwareSerial.h>
#include <TinyGPSPlus.h>

// ================== CONFIG ==================
const char* SERVER_URL_PUSH   = "http://smartbike.ashikdev.com/api/device/push";
const char* SERVER_URL_POLL   = "http://smartbike.ashikdev.com/api/device/poll";
const char* SERVER_URL_REPORT = "http://smartbike.ashikdev.com/api/device/report";
const char* DEVICE_ID         = "BIKE01";

// Banglalink APN
const char* GPRS_APN  = "blweb";
const char* GPRS_USER = "";
const char* GPRS_PASS = "";

// GPS UART (Serial1)
HardwareSerial SerialGPS(1);  // RX1=16, TX1=17

// SIM800 UART (Serial2)
HardwareSerial SerialSIM(2);  // RX2=26, TX2=27

TinyGPSPlus gps;

// intervals
const unsigned long PUSH_INTERVAL_MS = 10000;
const unsigned long POLL_INTERVAL_MS = 10000;

unsigned long lastPush = 0;
unsigned long lastPoll = 0;

bool gprsReady = false;

// ================== RELAY / IGNITION KILL ==================
const int RELAY_PIN = 25;

// IMPORTANT:
// If your LED is on the SAME pin you're testing (GPIO25),
// set LED_PIN = RELAY_PIN.
// Otherwise keep separate LED pin (recommended).
const int LED_PIN = 33;   // change to 25 if LED is on relay pin

// Relay driver with MOSFET is ACTIVE-HIGH at GPIO level,
// because MOSFET turns ON when gate HIGH.
// So keep this false unless you are using a ready-made relay module.
const bool RELAY_ACTIVE_LOW = false;

// Relay states
void relayOn()  { digitalWrite(RELAY_PIN, RELAY_ACTIVE_LOW ? LOW : HIGH); }
void relayOff() { digitalWrite(RELAY_PIN, RELAY_ACTIVE_LOW ? HIGH : LOW); }

// LED indicator states
void ledOn()  { digitalWrite(LED_PIN, HIGH); }
void ledOff() { digitalWrite(LED_PIN, LOW);  }

// ================== SPEED FILTER ==================
const float MIN_SPEED_KMPH = 2.0;
const int STOP_COUNT_REQ   = 3;
int   lowSpeedCount = 0;
float filteredSpeed = 0;

float getFilteredSpeedKmph() {
  float rawSpeed = gps.speed.kmph();

  if (rawSpeed < MIN_SPEED_KMPH) {
    lowSpeedCount++;
    if (lowSpeedCount >= STOP_COUNT_REQ) filteredSpeed = 0;
  } else {
    lowSpeedCount = 0;
    filteredSpeed = rawSpeed;
  }
  return filteredSpeed;
}

// ================== LOG HELPERS (minimal) ==================
void logWarn(const __FlashStringHelper* msg) { Serial.print(F("[WARN] ")); Serial.println(msg); }
void logErr (const __FlashStringHelper* msg) { Serial.print(F("[ERR] "));  Serial.println(msg); }

// ================== AT HELPERS ==================
bool sendAT(const char* cmd, const char* expect, uint32_t timeout = 3000, String* outResp = nullptr)
{
  while (SerialSIM.available()) SerialSIM.read();

  SerialSIM.print(cmd);
  SerialSIM.print("\r\n");

  uint32_t start = millis();
  String resp;

  while (millis() - start < timeout) {
    while (SerialSIM.available()) resp += (char)SerialSIM.read();
    if (resp.indexOf(expect) != -1) {
      if (outResp) *outResp = resp;
      return true;
    }
  }

  // print only on fail
  Serial.print(F("[AT][TIMEOUT] ")); Serial.println(cmd);
  Serial.print(F("[AT][RESP] "));    Serial.println(resp);

  if (outResp) *outResp = resp;
  return false;
}

bool waitForResponse(const char* expect, uint32_t timeout = 3000)
{
  uint32_t start = millis();
  String resp;

  while (millis() - start < timeout) {
    while (SerialSIM.available()) resp += (char)SerialSIM.read();
    if (resp.indexOf(expect) != -1) return true;
  }

  Serial.print(F("[WAIT][TIMEOUT] expect=")); Serial.println(expect);
  Serial.print(F("[WAIT][RESP] ")); Serial.println(resp);
  return false;
}

bool bearerHasIP() {
  String r;
  if (!sendAT("AT+SAPBR=2,1", "SAPBR", 5000, &r)) return false;
  return (r.indexOf('.') != -1);
}

// ================== GPRS INIT ==================
bool initGPRSWithURL(const char* url)
{
  Serial.println(F("== GPRS INIT START =="));

  bool ok = false;
  for (int i = 0; i < 5 && !ok; i++) {
    ok = sendAT("AT", "OK", 2000);
    if (!ok) delay(500);
  }
  if (!ok) {
    logErr(F("SIM800 no response. Check power/wiring/baud."));
    return false;
  }

  sendAT("ATE0", "OK", 2000);
  sendAT("AT+CPIN?", "+CPIN", 5000);

  {
    String cgdcontCmd = String("AT+CGDCONT=1,\"IP\",\"") + GPRS_APN + "\"";
    sendAT(cgdcontCmd.c_str(), "OK", 5000);
  }

  sendAT("AT+SAPBR=0,1", "OK", 5000);

  if (!sendAT("AT+SAPBR=3,1,\"CONTYPE\",\"GPRS\"", "OK", 5000)) return false;
  {
    String apnCmd = String("AT+SAPBR=3,1,\"APN\",\"") + GPRS_APN + "\"";
    if (!sendAT(apnCmd.c_str(), "OK", 5000)) return false;
  }

  if (!sendAT("AT+SAPBR=1,1", "OK", 60000)) {
    logErr(F("Bearer open fail"));
    return false;
  }

  sendAT("AT+HTTPTERM", "OK", 2000); // ignore error
  if (!sendAT("AT+HTTPINIT", "OK", 5000)) return false;
  if (!sendAT("AT+HTTPPARA=\"CID\",1", "OK", 5000)) return false;

  {
    String urlCmd = String("AT+HTTPPARA=\"URL\",\"") + url + "\"";
    if (!sendAT(urlCmd.c_str(), "OK", 5000)) return false;
  }

  if (!sendAT("AT+HTTPPARA=\"CONTENT\",\"application/json\"", "OK", 5000)) return false;

  Serial.println(F("== GPRS INIT OK =="));
  return true;
}

bool ensureGPRS(const char* url) {
  if (!gprsReady || !bearerHasIP()) {
    logWarn(F("GPRS not ready, re-init..."));
    gprsReady = initGPRSWithURL(url);
  } else {
    String urlCmd = String("AT+HTTPPARA=\"URL\",\"") + url + "\"";
    sendAT(urlCmd.c_str(), "OK", 3000);
  }
  return gprsReady;
}

// ================== HTTP POST ==================
bool httpPostJSON(const char* url, const String& json, String* outBody = nullptr)
{
  if (!ensureGPRS(url)) {
    logErr(F("GPRS init failed, skip HTTP"));
    return false;
  }

  String lenCmd = String("AT+HTTPDATA=") + json.length() + ",10000";
  if (!sendAT(lenCmd.c_str(), "DOWNLOAD", 15000)) {
    gprsReady = false;
    return false;
  }

  SerialSIM.print(json);

  if (!waitForResponse("OK", 15000)) {
    gprsReady = false;
    return false;
  }

  String actionResp;
  if (!sendAT("AT+HTTPACTION=1", "HTTPACTION", 30000, &actionResp)) {
    gprsReady = false;
    return false;
  }

  int status = -1, length = -1;
  int idx = actionResp.indexOf("+HTTPACTION:");
  if (idx >= 0) {
    int c1 = actionResp.indexOf(',', idx);
    int c2 = actionResp.indexOf(',', c1 + 1);
    if (c1 > 0 && c2 > c1) {
      status = actionResp.substring(c1 + 1, c2).toInt();
      length = actionResp.substring(c2 + 1).toInt();
    }
  }

  Serial.print(F("[HTTP] status="));
  Serial.print(status);
  Serial.print(F(" len="));
  Serial.println(length);

  if (length > 0) {
    String readResp;
    sendAT("AT+HTTPREAD", "OK", 8000, &readResp);
    if (outBody) *outBody = readResp;
  }

  return (status == 200);
}

// ================== DEVICE APIs ==================

// 1) PUSH location
void pushLocation()
{
  if (!(gps.location.isValid() && gps.location.age() < 10000)) {
    logWarn(F("No valid GPS fix"));
    return;
  }

  double lat = gps.location.lat();
  double lon = gps.location.lng();
  float  spd = getFilteredSpeedKmph();

  Serial.print(F("[GPS] "));
  Serial.print(lat, 6);
  Serial.print(F(", "));
  Serial.print(lon, 6);
  Serial.print(F(" speed="));
  Serial.println(spd, 2);

  String json = "{";
  json += "\"deviceId\":\""; json += DEVICE_ID; json += "\",";
  json += "\"lat\":";   json += String(lat, 6);   json += ",";
  json += "\"lon\":";   json += String(lon, 6);   json += ",";
  json += "\"speed\":"; json += String(spd, 2);   json += ",";
  json += "\"state\":\"ARMED\"";
  json += "}";

  httpPostJSON(SERVER_URL_PUSH, json);
}

// forward declare
void executeCommand(const String& cmd, const String& cmdId);

// 2) POLL command
void pollCommand()
{
  String json = "{";
  json += "\"deviceId\":\""; json += DEVICE_ID; json += "\"";
  json += "}";

  String body;
  bool ok = httpPostJSON(SERVER_URL_POLL, json, &body);
  if (!ok) return;

  int cmdPos = body.indexOf("\"command\"");
  if (cmdPos == -1) return;

  String command = "";
  String commandId = "";

  int colon = body.indexOf(':', cmdPos);
  if (colon != -1) {
    int q1 = body.indexOf('"', colon);
    int q2 = body.indexOf('"', q1 + 1);
    if (q1 != -1 && q2 != -1) {
      command = body.substring(q1 + 1, q2);
    }
  }

  int idPos = body.indexOf("\"commandId\"");
  if (idPos != -1) {
    int colon2 = body.indexOf(':', idPos);
    int q3 = body.indexOf('"', colon2);
    int q4 = body.indexOf('"', q3 + 1);
    if (q3 != -1 && q4 != -1) {
      commandId = body.substring(q3 + 1, q4);
    }
  }

  if (command.length() == 0 || command == "null") return;

  Serial.print(F("[CMD] "));
  Serial.print(command);
  Serial.print(F(" id="));
  Serial.println(commandId);

  executeCommand(command, commandId);
}

// 3) REPORT executed command
void reportCommandDone(const String& commandId, const String& event)
{
  String json = "{";
  json += "\"deviceId\":\""; json += DEVICE_ID; json += "\",";
  json += "\"commandId\":\""; json += commandId; json += "\",";
  json += "\"event\":\""; json += event; json += "\"";
  json += "}";

  httpPostJSON(SERVER_URL_REPORT, json);
}

// ================== COMMAND EXECUTION ==================
void executeCommand(const String& cmd, const String& cmdId)
{
  // Default: LED always OFF unless UNLOCK
  ledOff();

  if (cmd == "LOCK") {
    relayOn();
    Serial.println(F("[CMD] LOCK -> relay ON"));
    reportCommandDone(cmdId, "LOCK_DONE");
  }
  else if (cmd == "UNLOCK") {
    relayOff();
    ledOn();  // ONLY here LED ON
    Serial.println(F("[CMD] UNLOCK -> relay OFF + LED ON"));
    reportCommandDone(cmdId, "UNLOCK_DONE");
  }
  else if (cmd == "ARM") {
    Serial.println(F("[CMD] ARM"));
    reportCommandDone(cmdId, "ARM_DONE");
  }
  else if (cmd == "DISARM") {
    Serial.println(F("[CMD] DISARM"));
    reportCommandDone(cmdId, "DISARM_DONE");
  }
  else {
    Serial.println(F("[CMD] Unknown"));
    reportCommandDone(cmdId, "UNKNOWN_CMD");
  }
}

// ================== SETUP ==================
void setup()
{
  Serial.begin(115200);
  delay(1500);
  Serial.println(F("SmartBike Device Firmware (push/poll/report + relay + LED)"));

  pinMode(RELAY_PIN, OUTPUT);
  pinMode(LED_PIN, OUTPUT);

  relayOff(); // safe default
  ledOff();   // default OFF

  SerialGPS.begin(9600, SERIAL_8N1, 16, 17);
  SerialSIM.begin(115200, SERIAL_8N1, 26, 27);

  delay(3000);

  gprsReady = initGPRSWithURL(SERVER_URL_PUSH);
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
    pushLocation();
  }

  if (now - lastPoll >= POLL_INTERVAL_MS) {
    lastPoll = now;
    pollCommand();
  }
}
