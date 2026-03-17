#include "network.h"

#include "../core/state_logic.h"
#include "../logging/logger.h"
#include "../sensors/sensors.h"
#include "../tracking/tracking.h"
#include "../core/command_processor.h"

namespace {

HardwareSerial SerialAT(2);

bool modemUartStarted = false;
bool modemReady = false;
bool bearerReady = false;
uint32_t lastInternetActivityAt = 0;
int lastKnownCsq = -1;
uint32_t lastHttpLatencyMs = 0;
uint32_t lastSmsScanAt = 0;

static constexpr uint32_t SMS_SCAN_INTERVAL_MS = 2500UL;
static constexpr int SMS_MAX_BATCH = 6;

String urlToRoute(const char* url) {
  if (!url || !url[0]) return "/";

  String route = url;
  const int scheme = route.indexOf("://");
  if (scheme >= 0) {
    const int slash = route.indexOf('/', scheme + 3);
    route = (slash >= 0) ? route.substring(slash) : "/";
  }
  return route;
}

String clipForLog(const String& value, size_t limit = HTTP_LOG_BODY_LIMIT) {
  if (value.length() <= limit) return value;
  return value.substring(0, limit) + "...";
}

void logAtTx(const char* cmd) {
  if (DBG_NET_AT) {
    logPrintf("AT", "TX %s", cmd);
  }
}

void logAtRx(const char* response) {
  if (DBG_NET_AT) {
    logPrintf("AT", "RX %s", response);
  }
}

void logAtFail(const char* cmd, const char* reason, const String& response) {
  LOG_PRINTF_IF(DBG_NET,
                "AT",
                "cmd=%s fail=%s resp=%s",
                cmd,
                reason,
                clipForLog(response).c_str());
}

void drainModemRx() {
  while (SerialAT.available()) {
    SerialAT.read();
  }
}

int findLineToken(const String& response, const char* token, int startAt = 0) {
  if (!token || !token[0]) return -1;

  int pos = response.indexOf(token, startAt);
  while (pos >= 0) {
    if (pos == 0 || response[pos - 1] == '\n') return pos;
    pos = response.indexOf(token, pos + 1);
  }
  return -1;
}

bool responseHasError(const String& response) {
  return response.startsWith("ERROR") ||
         response.indexOf("\r\nERROR") != -1 ||
         response.indexOf("\nERROR") != -1 ||
         response.indexOf("+CME ERROR:") != -1 ||
         response.indexOf("+CMS ERROR:") != -1;
}

bool responseContains(const String& response, const char* token) {
  return token && token[0] && response.indexOf(token) != -1;
}

bool responseHasOperationNotAllowed(const String& response) {
  return response.indexOf("operation not allowed") != -1 ||
         response.indexOf("+CME ERROR: 3") != -1;
}

bool responseHasLineForToken(const String& response, const char* token) {
  if (!token || !token[0]) return false;

  const int marker = response.indexOf(token);
  if (marker < 0) return false;

  const int lineEndCr = response.indexOf('\r', marker);
  const int lineEndLf = response.indexOf('\n', marker);
  return (lineEndCr >= 0) || (lineEndLf >= 0);
}

bool sendATCommand(const char* cmd,
                   String& response,
                   uint32_t timeout,
                   const char* okToken,
                   const char* altToken = nullptr,
                   bool allowOperationNotAllowed = false) {
  drainModemRx();
  response = "";

  logAtTx(cmd);
  SerialAT.println(cmd);

  const uint32_t start = millis();
  while (millis() - start < timeout) {
    while (SerialAT.available()) {
      response += static_cast<char>(SerialAT.read());
    }

    if (responseContains(response, okToken) || responseContains(response, altToken)) {
      logAtRx(clipForLog(response).c_str());
      return true;
    }

    if (responseHasError(response)) {
      if (allowOperationNotAllowed &&
          (responseHasOperationNotAllowed(response) || response.indexOf("+CME ERROR:") != -1)) {
        logAtRx(clipForLog(response).c_str());
        return true;
      }
      logAtFail(cmd, "modem_error", response);
      return false;
    }

    vTaskDelay(pdMS_TO_TICKS(20));
  }

  logAtFail(cmd, "timeout", response);
  return false;
}

bool sendATOk(const char* cmd, uint32_t timeout = GSM_AT_TIMEOUT_MS) {
  String response;
  return sendATCommand(cmd, response, timeout, "OK");
}

bool sendATOkOrOperationNotAllowed(const char* cmd, uint32_t timeout = GSM_AT_TIMEOUT_MS) {
  String response;
  const bool ok = sendATCommand(cmd, response, timeout, "OK", nullptr, true);
  if (ok) return true;

  const bool cleanupCmd = strcmp(cmd, "AT+HTTPTERM") == 0 || strcmp(cmd, "AT+SAPBR=0,1") == 0;
  if (cleanupCmd && response.indexOf("+CME ERROR:") != -1) {
    logAtRx(clipForLog(response).c_str());
    return true;
  }

  return false;
}

bool sendATReadComplete(const char* cmd, String& response, uint32_t timeout) {
  drainModemRx();
  response = "";

  logAtTx(cmd);
  SerialAT.println(cmd);

  const uint32_t start = millis();
  while (millis() - start < timeout) {
    while (SerialAT.available()) {
      response += static_cast<char>(SerialAT.read());
    }

    if (response.indexOf("\r\nOK\r\n") != -1 ||
        response.indexOf("\nOK\r\n") != -1 ||
        response.startsWith("OK\r\n") ||
        response.endsWith("\r\nOK\r\n") ||
        response.endsWith("\nOK\r\n")) {
      logAtRx(clipForLog(response).c_str());
      return true;
    }

    if (responseHasError(response)) {
      logAtFail(cmd, "modem_error", response);
      return false;
    }

    vTaskDelay(pdMS_TO_TICKS(20));
  }

  logAtFail(cmd, "timeout", response);
  return false;
}

bool readAfterRawWrite(String& response, uint32_t timeout, const char* okToken) {
  const uint32_t start = millis();
  response = "";

  while (millis() - start < timeout) {
    while (SerialAT.available()) {
      response += static_cast<char>(SerialAT.read());
    }

    if (responseContains(response, okToken)) {
      logAtRx(clipForLog(response).c_str());
      return true;
    }

    if (responseHasError(response)) {
      logAtFail("RAW_WRITE", "modem_error", response);
      return false;
    }

    vTaskDelay(pdMS_TO_TICKS(20));
  }

  logAtFail("RAW_WRITE", "timeout", response);
  return false;
}

bool waitForIncomingLine(String& response, uint32_t timeout, const char* lineToken, const char* ctx) {
  const uint32_t start = millis();
  while (millis() - start < timeout) {
    while (SerialAT.available()) {
      response += static_cast<char>(SerialAT.read());
    }

    if (responseHasLineForToken(response, lineToken)) {
      logAtRx(clipForLog(response).c_str());
      return true;
    }

    if (responseHasError(response)) {
      logAtFail(ctx, "modem_error", response);
      return false;
    }

    vTaskDelay(pdMS_TO_TICKS(20));
  }

  return false;
}

void markNetHealthy() {
  const uint32_t now = millis();
  lastInternetActivityAt = now;
  lastInternetOkAt = now;
  gInternetOk = true;
}

void markNetUnhealthy() {
  gInternetOk = false;
  lastInternetOkAt = 0;
}

void markBearerDown() {
  bearerReady = false;
  netWasConnected = false;
  markNetUnhealthy();
}

void netOnSuccess() {
  netFailStreak = 0;
  netBackoffMs = 15000;
  nextNetTryAt = millis();
  bearerReady = true;
  netConnecting = false;
  netWasConnected = true;
}

void netOnFail() {
  netFailStreak++;
  const uint32_t next = netBackoffMs * 2;
  netBackoffMs = min(next, static_cast<uint32_t>(NET_BACKOFF_MAX_MS));
  nextNetTryAt = millis() + netBackoffMs;
  netConnecting = false;
  markBearerDown();

  if (DBG_NET) {
    logPrintf("GSM", "backoff %lu ms streak=%d", static_cast<unsigned long>(netBackoffMs), netFailStreak);
  }
}

int parseCSQ(const String& response) {
  const int marker = response.indexOf("+CSQ:");
  if (marker < 0) return -1;

  const int comma = response.indexOf(',', marker);
  if (comma < 0) return -1;

  String value = response.substring(marker + 5, comma);
  value.trim();

  const int rssi = value.toInt();
  return (rssi == 99) ? -1 : rssi;
}

bool isSimReady(const String& response) {
  return response.indexOf("+CPIN: READY") != -1;
}

bool isRegistered(const String& response) {
  const int marker = response.indexOf("+CREG:");
  if (marker < 0) return false;
  return response.indexOf(",1", marker) != -1 || response.indexOf(",5", marker) != -1;
}

bool isGprsAttached(const String& response) {
  return response.indexOf("+CGATT: 1") != -1;
}

bool initModemUartIfNeeded() {
  if (modemUartStarted) return true;

  SerialAT.begin(GSM_BAUD, SERIAL_8N1, GSM_RX, GSM_TX);
  modemUartStarted = true;
  delay(GSM_BOOT_DELAY_MS);
  drainModemRx();

  LOG_PRINTF_IF(DBG_NET,
                "GSM",
                "UART ready rx=%d tx=%d baud=%lu",
                GSM_RX,
                GSM_TX,
                static_cast<unsigned long>(GSM_BAUD));
  return true;
}

bool syncModem() {
  for (int i = 0; i < 4; i++) {
    String response;
    if (sendATCommand("AT", response, GSM_AT_TIMEOUT_MS, "OK")) {
      return true;
    }
    vTaskDelay(pdMS_TO_TICKS(250));
  }
  return false;
}

bool queryNetworkState(bool* simReadyOut,
                       bool* registeredOut,
                       bool* gprsAttachedOut,
                       int* csqOut) {
  String simState;
  String sigState;
  String regState;
  String gprsState;

  const bool simOk = sendATReadComplete("AT+CPIN?", simState, 4000);
  const bool sigOk = sendATReadComplete("AT+CSQ", sigState, GSM_AT_TIMEOUT_MS);
  const bool regOk = sendATReadComplete("AT+CREG?", regState, GSM_AT_TIMEOUT_MS);
  const bool gprsOk = sendATReadComplete("AT+CGATT?", gprsState, GSM_AT_TIMEOUT_MS);

  const bool simReady = simOk && isSimReady(simState);
  const bool registered = regOk && isRegistered(regState);
  const bool gprsAttached = gprsOk && isGprsAttached(gprsState);
  const int csq = sigOk ? parseCSQ(sigState) : -1;
  lastKnownCsq = csq;

  if (simReadyOut) *simReadyOut = simReady;
  if (registeredOut) *registeredOut = registered;
  if (gprsAttachedOut) *gprsAttachedOut = gprsAttached;
  if (csqOut) *csqOut = csq;

  LOG_PRINTF_IF(DBG_NET,
                "GSM",
                "SIM=%s REG=%s GPRS=%s CSQ=%d",
                simReady ? "YES" : "NO",
                registered ? "YES" : "NO",
                gprsAttached ? "YES" : "NO",
                csq);

  return simOk && sigOk && regOk && gprsOk;
}

bool waitForPacketNetwork(uint32_t timeoutMs) {
  const uint32_t start = millis();

  while (millis() - start < timeoutMs) {
    bool simReady = false;
    bool registered = false;
    bool gprsAttached = false;
    int csq = -1;

    queryNetworkState(&simReady, &registered, &gprsAttached, &csq);

    if (simReady && registered && gprsAttached && csq >= GSM_MIN_CSQ) {
      return true;
    }

    LOG_PRINTLN_IF(DBG_NET, "GSM", F("Still searching network / GPRS..."));
    vTaskDelay(pdMS_TO_TICKS(GSM_NETWORK_RETRY_STEP_MS));
  }

  LOG_PRINTLN_IF(DBG_NET, "GSM", F("Network/GPRS wait timeout"));
  return false;
}

bool configureBearerProfile() {
  if (!sendATOk("ATE0")) return false;
  if (!sendATOk("AT+CFUN=1", 5000)) return false;

  String ati;
  sendATReadComplete("ATI", ati, 4000);
  sendATOk("AT+CMEE=2", 3000);
  sendATOk("AT+CREG=0", 3000);
  sendATOk("AT+CMGF=1", 3000);  // SMS text mode
  sendATOk("AT+CNMI=2,1,0,0,0", 3000); // New message notification

  if (!waitForPacketNetwork(GSM_NETWORK_WAIT_MS)) return false;

  sendATOk("AT+SAPBR=0,1", 3000);
  if (!sendATOk("AT+SAPBR=3,1,\"Contype\",\"GPRS\"", 3000)) return false;

  char apnCmd[96];
  snprintf(apnCmd, sizeof(apnCmd), "AT+SAPBR=3,1,\"APN\",\"%s\"", GSM_APN);
  if (!sendATOk(apnCmd, 3000)) return false;

  if (GSM_USER[0]) {
    char userCmd[96];
    snprintf(userCmd, sizeof(userCmd), "AT+SAPBR=3,1,\"USER\",\"%s\"", GSM_USER);
    if (!sendATOk(userCmd, 3000)) return false;
  }

  if (GSM_PASS[0]) {
    char passCmd[96];
    snprintf(passCmd, sizeof(passCmd), "AT+SAPBR=3,1,\"PWD\",\"%s\"", GSM_PASS);
    if (!sendATOk(passCmd, 3000)) return false;
  }

  return true;
}

bool queryBearerStatus(String* ipOut = nullptr) {
  String response;
  if (!sendATReadComplete("AT+SAPBR=2,1", response, 6000)) {
    return false;
  }

  const bool up = response.indexOf("+SAPBR: 1,1") != -1;
  if (up && ipOut) {
    const int q1 = response.indexOf('"');
    const int q2 = response.indexOf('"', q1 + 1);
    *ipOut = (q1 >= 0 && q2 > q1) ? response.substring(q1 + 1, q2) : "";
  }
  return up;
}

bool openBearer() {
  if (queryBearerStatus()) {
    return true;
  }

  String openResp;
  if (!sendATReadComplete("AT+SAPBR=1,1", openResp, 15000)) {
    if (!queryBearerStatus()) {
      return false;
    }
  }

  String ip;
  const bool up = queryBearerStatus(&ip);
  if (up) {
    LOG_PRINTF_IF(DBG_NET, "GSM", "Bearer up ip=%s", ip.length() ? ip.c_str() : "n/a");
  }
  return up;
}

void closeBearerSession() {
  sendATOkOrOperationNotAllowed("AT+HTTPTERM", 2000);
  sendATOkOrOperationNotAllowed("AT+SAPBR=0,1", 5000);
  markBearerDown();
}

void recoverHttpStack(const __FlashStringHelper* reason) {
  LOG_PRINTLN_IF(DBG_NET, "HTTP", reason);
  sendATOkOrOperationNotAllowed("AT+HTTPTERM", 3000);
  sendATOkOrOperationNotAllowed("AT+SAPBR=0,1", 7000);
  markBearerDown();
  nextNetTryAt = millis();
}

bool ensureDataSession() {
  bool simReady = false;
  bool registered = false;
  bool gprsAttached = false;
  int csq = -1;

  if (!queryNetworkState(&simReady, &registered, &gprsAttached, &csq) ||
      !simReady ||
      !registered ||
      !gprsAttached ||
      csq < GSM_MIN_CSQ) {
    LOG_PRINTLN_IF(DBG_NET, "GSM", F("Session unhealthy. Rebuilding bearer..."));
    markBearerDown();
    if (!configureBearerProfile()) return false;
    return openBearer();
  }

  if (!queryBearerStatus()) {
    LOG_PRINTLN_IF(DBG_NET, "GSM", F("Bearer not open. Re-opening..."));
    markBearerDown();
    if (!configureBearerProfile()) return false;
    return openBearer();
  }

  bearerReady = true;
  return true;
}

bool ensureModemSession() {
  if (!initModemUartIfNeeded()) return false;

  if (!modemReady) {
    if (!syncModem()) {
      LOG_PRINTLN_IF(DBG_NET, "GSM", F("AT sync failed"));
      return false;
    }
    modemReady = true;
    LOG_PRINTLN_IF(DBG_NET, "GSM", F("Modem sync OK"));
  }

  if (bearerReady && ensureDataSession()) {
    return true;
  }

  if (!configureBearerProfile()) {
    return false;
  }

  if (!openBearer()) {
    return false;
  }

  bearerReady = true;
  return true;
}

bool parseHttpAction(const String& response, int* statusCode, int* bodyLength) {
  const int marker = response.indexOf("+HTTPACTION:");
  if (marker < 0) return false;

  const int comma1 = response.indexOf(',', marker);
  const int comma2 = response.indexOf(',', comma1 + 1);
  if (comma1 < 0 || comma2 < 0) return false;

  *statusCode = response.substring(comma1 + 1, comma2).toInt();

  int end = response.indexOf('\r', comma2 + 1);
  if (end < 0) end = response.indexOf('\n', comma2 + 1);
  if (end < 0) end = response.length();
  *bodyLength = response.substring(comma2 + 1, end).toInt();
  return true;
}

bool queryHttpStatus(int* statusOut, int* finishOut, int* remainOut, String* rawOut = nullptr) {
  String response;
  if (!sendATReadComplete("AT+HTTPSTATUS?", response, 4000)) {
    return false;
  }

  if (rawOut) *rawOut = response;

  const int marker = response.indexOf("+HTTPSTATUS:");
  if (marker < 0) return false;

  const int comma1 = response.indexOf(',', marker);
  const int comma2 = response.indexOf(',', comma1 + 1);
  const int comma3 = response.indexOf(',', comma2 + 1);
  if (comma1 < 0 || comma2 < 0 || comma3 < 0) return false;

  if (statusOut) *statusOut = response.substring(comma1 + 1, comma2).toInt();
  if (finishOut) *finishOut = response.substring(comma2 + 1, comma3).toInt();

  int end = response.indexOf('\r', comma3 + 1);
  if (end < 0) end = response.indexOf('\n', comma3 + 1);
  if (end < 0) end = response.length();
  if (remainOut) *remainOut = response.substring(comma3 + 1, end).toInt();
  return true;
}

bool waitForHttpActionResult(String& response, uint32_t timeoutMs) {
  if (waitForIncomingLine(response, timeoutMs, "+HTTPACTION:", "HTTPACTION_URC")) {
    return true;
  }

  String statusRaw;
  int httpStatus = -1;
  int finish = -1;
  int remain = -1;
  if (queryHttpStatus(&httpStatus, &finish, &remain, &statusRaw)) {
    LOG_PRINTF_IF(DBG_NET,
                  "HTTP",
                  "HTTPSTATUS status=%d finish=%d remain=%d",
                  httpStatus,
                  finish,
                  remain);

    if (httpStatus == 1 || httpStatus == 2) {
      LOG_PRINTLN_IF(DBG_NET, "HTTP", F("HTTP action still busy, waiting extra"));
      return waitForIncomingLine(response, 20000, "+HTTPACTION:", "HTTPACTION_URC_EXT");
    }
  }

  logAtFail("AT+HTTPACTION=1", "timeout", response);
  return false;
}

String extractHttpBody(const String& response) {
  const int marker = response.indexOf("+HTTPREAD:");
  if (marker < 0) return "";

  int bodyStart = response.indexOf('\n', marker);
  if (bodyStart < 0) return "";
  bodyStart += 1;

  int okPos = response.lastIndexOf("\r\nOK");
  if (okPos < 0) okPos = response.lastIndexOf("\nOK");
  if (okPos < bodyStart) okPos = response.length();

  String body = response.substring(bodyStart, okPos);
  body.trim();
  return body;
}

bool readHttpBody(String* outBody, int expectedLength) {
  String response;
  const uint32_t timeout = max(static_cast<uint32_t>(GSM_AT_TIMEOUT_MS),
                               static_cast<uint32_t>(expectedLength * 8 + 3000));
  if (!sendATReadComplete("AT+HTTPREAD", response, timeout)) {
    return false;
  }

  const String body = extractHttpBody(response);
  if (outBody) {
    *outBody = body;
  }

  if (DBG_HTTP_BODY && body.length()) {
    logPrintf("HTTP", "body=%s", clipForLog(body).c_str());
  }
  return true;
}

bool httpInitForJson(const char* url) {
  sendATOkOrOperationNotAllowed("AT+HTTPTERM", 2000);
  if (!sendATOk("AT+HTTPINIT", 5000)) {
    sendATOkOrOperationNotAllowed("AT+HTTPTERM", 3000);
    if (!sendATOk("AT+HTTPINIT", 5000)) return false;
  }
  if (!sendATOk("AT+HTTPPARA=\"CID\",1", 3000)) return false;
  sendATOk("AT+HTTPPARA=\"UA\",\"SmartBike-SIM800/8.5\"", 3000);

  char urlPara[196];
  snprintf(urlPara, sizeof(urlPara), "AT+HTTPPARA=\"URL\",\"%s\"", url);
  if (!sendATOk(urlPara, 5000)) return false;

  if (!sendATOk("AT+HTTPPARA=\"CONTENT\",\"application/json\"", 3000)) return false;
  return true;
}

void sendSmsReply(const char* sender, const char* text) {
  if (!sender || !text || !sender[0]) return;
  char sendCmd[48];
  snprintf(sendCmd, sizeof(sendCmd), "AT+CMGS=\"%s\"", sender);
  String resp;
  if (sendATCommand(sendCmd, resp, 5000, ">")) {
    SerialAT.print(text);
    SerialAT.write(0x1A); // CTRL+Z
    if (!readAfterRawWrite(resp, 10000, "OK")) {
      LOG_PRINTLN_IF(DBG_NET, "SMS", F("Reply send failed"));
    }
  }
}

void deleteSmsFromStorage(int index) {
  if (index < 0) return;

  char deleteCmd[20];
  snprintf(deleteCmd, sizeof(deleteCmd), "AT+CMGD=%d", index);
  sendATOk(deleteCmd, 5000);
}

bool queueSmsCommand(const char* cmd) {
  if (!cmd || !cmd[0] || !cmdQueue) return false;

  CmdMsg msg = {};
  strncpy(msg.cmd, cmd, sizeof(msg.cmd) - 1);
  msg.cmd[sizeof(msg.cmd) - 1] = 0;
  strncpy(msg.cmdId, "SMS", sizeof(msg.cmdId) - 1);
  msg.cmdId[sizeof(msg.cmdId) - 1] = 0;

  if (xQueueSend(cmdQueue, &msg, pdMS_TO_TICKS(100)) != pdTRUE) {
    LOG_PRINTLN_IF(DBG_NET, "SMS", F("cmdQueue full"));
    return false;
  }
  return true;
}

bool parseSmsHeader(const String& header, int* outIndex, String* outSender) {
  if (outIndex) *outIndex = -1;
  if (outSender) *outSender = "";

  const int colon = header.indexOf(':');
  if (colon < 0) return false;
  const int comma = header.indexOf(',', colon + 1);
  if (comma < 0) return false;

  String indexText = header.substring(colon + 1, comma);
  indexText.trim();
  const int index = indexText.toInt();
  if (index < 0) return false;

  if (outIndex) *outIndex = index;

  const int q1 = header.indexOf('"', comma + 1);
  if (q1 < 0) return true;
  const int q2 = header.indexOf('"', q1 + 1);
  if (q2 < 0) return true;
  const int q3 = header.indexOf('"', q2 + 1);
  if (q3 < 0) return true;
  const int q4 = header.indexOf('"', q3 + 1);
  if (q4 < 0) return true;

  if (outSender) *outSender = header.substring(q3 + 1, q4);
  return true;
}

bool extractNextSmsFromList(const String& response,
                            int startAt,
                            int* nextAt,
                            int* outIndex,
                            String* outSender,
                            String* outBody) {
  if (nextAt) *nextAt = response.length();
  if (outIndex) *outIndex = -1;
  if (outSender) *outSender = "";
  if (outBody) *outBody = "";

  const int headerAt = findLineToken(response, "+CMGL:", startAt);
  if (headerAt < 0) return false;

  int lineEnd = response.indexOf("\r\n", headerAt);
  int lineBreakLen = 2;
  if (lineEnd < 0) {
    lineEnd = response.indexOf('\n', headerAt);
    lineBreakLen = 1;
  }
  if (lineEnd < 0) return false;

  const String header = response.substring(headerAt, lineEnd);
  if (!parseSmsHeader(header, outIndex, outSender)) return false;

  const int bodyStart = lineEnd + lineBreakLen;
  int nextHeaderAt = response.indexOf("\r\n+CMGL:", bodyStart);
  if (nextHeaderAt >= 0) {
    nextHeaderAt += 2;
  } else {
    nextHeaderAt = response.indexOf("\n+CMGL:", bodyStart);
    if (nextHeaderAt >= 0) nextHeaderAt += 1;
  }

  int okAt = response.indexOf("\r\nOK", bodyStart);
  if (okAt < 0) okAt = response.indexOf("\nOK", bodyStart);

  int bodyEnd = response.length();
  if (nextHeaderAt >= 0) {
    bodyEnd = nextHeaderAt - 2;
    if (bodyEnd < bodyStart) bodyEnd = nextHeaderAt - 1;
  } else if (okAt >= 0) {
    bodyEnd = okAt;
  }

  if (bodyEnd < bodyStart) bodyEnd = bodyStart;

  if (outBody) {
    *outBody = response.substring(bodyStart, bodyEnd);
    outBody->trim();
  }
  if (nextAt) *nextAt = (nextHeaderAt >= 0) ? nextHeaderAt : response.length();
  return true;
}

String normalizeSmsBody(const String& body) {
  String normalized = body;
  normalized.replace("\r", " ");
  normalized.replace("\n", " ");
  normalized.trim();
  normalized.toUpperCase();
  return normalized;
}

void processSmsMessage(int index, const String& sender, const String& rawBody) {
  const String body = normalizeSmsBody(rawBody);

  if (body.indexOf(DEVICE_ID) == -1 || body.indexOf(LOCAL_PIN) == -1) {
    LOG_PRINTLN_IF(DBG_NET, "SMS", F("Unauthorized SMS (Bad ID or PIN)"));
    deleteSmsFromStorage(index);
    return;
  }

  if (body.indexOf("AP ON") != -1) {
    localApManualTrigger = true;
    localApManualTriggerAt = millis();
    LOG_PRINTLN_IF(DBG_NET, "SMS", F("AP Manual ON via SMS"));
    if (sender.length() > 0) sendSmsReply(sender.c_str(), "Local AP: ON (30min timeout)");
    deleteSmsFromStorage(index);
    return;
  }

  if (body.indexOf("AP OFF") != -1) {
    localApManualTrigger = false;
    LOG_PRINTLN_IF(DBG_NET, "SMS", F("AP Manual OFF via SMS"));
    if (sender.length() > 0) sendSmsReply(sender.c_str(), "Local AP: OFF");
    deleteSmsFromStorage(index);
    return;
  }

  if (body.indexOf("UNLOCK") != -1) {
    const bool queued = queueSmsCommand("UNLOCK");
    LOG_PRINTLN_IF(DBG_NET, "SMS", queued ? F("UNLOCK via SMS") : F("UNLOCK via SMS failed"));
    if (sender.length() > 0) sendSmsReply(sender.c_str(), queued ? "Bike: UNLOCK queued" : "Bike: BUSY");
    deleteSmsFromStorage(index);
    return;
  }

  if (body.indexOf("LOCK") != -1) {
    const bool queued = queueSmsCommand("LOCK");
    LOG_PRINTLN_IF(DBG_NET, "SMS", queued ? F("LOCK via SMS") : F("LOCK via SMS failed"));
    if (sender.length() > 0) sendSmsReply(sender.c_str(), queued ? "Bike: LOCK queued" : "Bike: BUSY");
    deleteSmsFromStorage(index);
    return;
  }

  if ((body.indexOf("STATUS") != -1 || body.indexOf("LOC") != -1) && sender.length() > 0) {
    LOG_PRINTLN_IF(DBG_NET, "SMS", F("Status/Loc requested via SMS"));

    DeviceStateId st = ST_UNLOCKED;
    if (xSemaphoreTake(stateMutex, pdMS_TO_TICKS(50)) == pdTRUE) {
      st = getCurrentState_locked();
      xSemaphoreGive(stateMutex);
    }

    char reply[160];
    int rlen = snprintf(reply, sizeof(reply), "SmartBike %s: %s. ", DEVICE_ID, stateToString(st));
    if (gps.location.isValid()) {
      snprintf(reply + rlen, sizeof(reply) - rlen, "Loc: https://www.google.com/maps?q=%.6f,%.6f (Age: %lu)",
               gps.location.lat(), gps.location.lng(), static_cast<unsigned long>(gps.location.age() / 1000));
    } else if (gps.location.age() < 3600000UL) {
      snprintf(reply + rlen, sizeof(reply) - rlen, "Last Loc: https://www.google.com/maps?q=%.6f,%.6f (Age: %lu min)",
               gps.location.lat(), gps.location.lng(), static_cast<unsigned long>(gps.location.age() / 60000));
    } else {
      snprintf(reply + rlen, sizeof(reply) - rlen, "GPS: No Fix");
    }

    sendSmsReply(sender.c_str(), reply);
    deleteSmsFromStorage(index);
    return;
  }

  LOG_PRINTLN_IF(DBG_NET, "SMS", F("Unknown SMS command"));
  if (sender.length() > 0) {
    sendSmsReply(sender.c_str(), "Format: <DEVICE_ID> <PIN> LOCK | UNLOCK | STATUS | LOC | AP ON | AP OFF");
  }
  deleteSmsFromStorage(index);
}

void handleIncomingSms() {
  const uint32_t now = millis();
  if (now - lastSmsScanAt < SMS_SCAN_INTERVAL_MS) return;
  lastSmsScanAt = now;

  String response;
  if (!sendATReadComplete("AT+CMGL=\"REC UNREAD\"", response, 12000)) return;

  int cursor = 0;
  int processed = 0;
  while (processed < SMS_MAX_BATCH) {
    int index = -1;
    int nextAt = response.length();
    String sender;
    String body;
    if (!extractNextSmsFromList(response, cursor, &nextAt, &index, &sender, &body)) break;

    LOG_PRINTF_IF(DBG_NET, "SMS", "Unread SMS index=%d", index);
    processSmsMessage(index, sender, body);
    processed++;
    cursor = nextAt;
  }

  if (processed > 0) {
    LOG_PRINTF_IF(DBG_NET, "SMS", "Processed unread batch=%d", processed);
  }
}

void gsmMaintenance() {
  if (netIsConnected()) {
    if (!netWasConnected) {
      netWasConnected = true;
      LOG_PRINTLN_IF(DBG_NET, "GSM", F("Connected & bearer ready"));
    }
    return;
  }

  if (millis() < nextNetTryAt) return;

  netConnecting = true;
  netConnectStartedAt = millis();
  LOG_PRINTLN_IF(DBG_NET, "GSM", F("Attach start"));

  const bool ok = ensureModemSession();
  if (ok) {
    netOnSuccess();
    markNetHealthy();
    LOG_PRINTLN_IF(DBG_NET, "GSM", F("Network ready"));
  } else {
    netOnFail();
    closeBearerSession();
    LOG_PRINTLN_IF(DBG_NET, "GSM", F("Attach failed"));
  }
}

void internetProbe() {
  const uint32_t now = millis();

  if (!bearerReady) {
    markNetUnhealthy();
    return;
  }

  if (lastInternetActivityAt != 0 && (now - lastInternetActivityAt < NET_OK_TTL_MS)) {
    gInternetOk = true;
    lastNetCheckAt = now;
    return;
  }

  if (now - lastNetCheckAt < NET_CHECK_EVERY_MS) {
    gInternetOk = (lastInternetOkAt != 0) && (now - lastInternetOkAt < NET_OK_TTL_MS);
    return;
  }
  lastNetCheckAt = now;

  String ip;
  if (queryBearerStatus(&ip)) {
    gInternetOk = true;
    lastInternetOkAt = now;
    LOG_PRINTF_IF(DBG_NET, "GSM", "Probe OK ip=%s", ip.length() ? ip.c_str() : "n/a");
    return;
  }

  LOG_PRINTLN_IF(DBG_NET, "GSM", F("Probe failed, bearer lost"));
  markBearerDown();
}

bool httpPostJSON(const char* url, const String& json, String* outBody) {
  if (!ensureDataSession()) {
    LOG_PRINTLN_IF(DBG_NET, "HTTP", F("Skip POST: gsm offline"));
    return false;
  }

  const String route = urlToRoute(url);
  LOG_PRINTF_IF(DBG_NET,
                "HTTP",
                "POST %s bytes=%u",
                route.c_str(),
                static_cast<unsigned>(json.length()));

  if (!httpInitForJson(url)) {
    recoverHttpStack(F("HTTP init failed, rebuilding bearer"));
    return false;
  }

  char dataCmd[40];
  snprintf(dataCmd, sizeof(dataCmd),
           "AT+HTTPDATA=%u,%lu",
           static_cast<unsigned>(json.length()),
           static_cast<unsigned long>(GSM_HTTP_DATA_TIMEOUT_MS));

  String response;
  if (!sendATCommand(dataCmd, response, 5000, "DOWNLOAD")) {
    recoverHttpStack(F("HTTPDATA failed, rebuilding bearer"));
    return false;
  }

  SerialAT.print(json);
  SerialAT.flush();
  if (!readAfterRawWrite(response, GSM_HTTP_DATA_TIMEOUT_MS + 2000, "OK")) {
    recoverHttpStack(F("Payload write failed, rebuilding bearer"));
    return false;
  }

  int httpStatus = -1;
  int bodyLength = -1;
  const uint32_t startedAt = millis();

  if (!sendATCommand("AT+HTTPACTION=1", response, 5000, "OK")) {
    recoverHttpStack(F("HTTPACTION start failed, rebuilding bearer"));
    return false;
  }

  response = "";
  if (!waitForHttpActionResult(response, GSM_HTTP_ACTION_TIMEOUT_MS)) {
    recoverHttpStack(F("HTTPACTION timeout, rebuilding bearer"));
    return false;
  }

  if (!parseHttpAction(response, &httpStatus, &bodyLength)) {
    LOG_PRINTF_IF(DBG_NET, "HTTP", "HTTPACTION parse failed: %s", clipForLog(response).c_str());
    recoverHttpStack(F("HTTPACTION parse failed, rebuilding bearer"));
    return false;
  }

  const uint32_t tookMs = millis() - startedAt;
  lastHttpLatencyMs = tookMs;
  LOG_PRINTF_IF(DBG_NET,
                "HTTP",
                "%s status=%d bodyLen=%d took=%lu",
                route.c_str(),
                httpStatus,
                bodyLength,
                static_cast<unsigned long>(tookMs));

  String body;
  if ((bodyLength > 0) || outBody || DBG_HTTP_BODY) {
    readHttpBody(&body, bodyLength);
  }

  sendATOkOrOperationNotAllowed("AT+HTTPTERM", 2000);

  if (outBody) {
    *outBody = body;
  }

  if (httpStatus > 0 && httpStatus < 600) {
    markNetHealthy();
  } else {
    markNetUnhealthy();
  }

  if (httpStatus >= 200 && httpStatus < 300) {
    return true;
  }

  LOG_PRINTF_IF(DBG_NET, "HTTP", "%s failed status=%d", route.c_str(), httpStatus);
  return false;
}

String jsonFindString(const String& body, const char* key) {
  const String k = String('"') + key + '"';
  const int p = body.indexOf(k);
  if (p < 0) return "";

  const int colon = body.indexOf(':', p);
  if (colon < 0) return "";

  const int nullPos = body.indexOf("null", colon);
  if (nullPos >= 0 && (nullPos - colon) < 12) return "null";

  const int q1 = body.indexOf('"', colon);
  if (q1 < 0) return "";
  const int q2 = body.indexOf('"', q1 + 1);
  if (q2 < 0) return "";
  return body.substring(q1 + 1, q2);
}

void pumpPendingReport() {
  ReportMsg rmsg = {};
  while (xQueueReceive(reportQueue, &rmsg, 0) == pdTRUE) {
    pendingReport = true;
    strncpy(pendingReportEvent, rmsg.event, sizeof(pendingReportEvent) - 1);
    pendingReportEvent[sizeof(pendingReportEvent) - 1] = 0;
    strncpy(pendingReportCmdId, rmsg.cmdId, sizeof(pendingReportCmdId) - 1);
    pendingReportCmdId[sizeof(pendingReportCmdId) - 1] = 0;
    lastReportTry = 0;

    LOG_PRINTF_IF(DBG_NET,
                  "REPORT",
                  "queue event=%s cmdId=%s",
                  pendingReportEvent,
                  pendingReportCmdId[0] ? pendingReportCmdId : "none");
  }

  if (!pendingReport || !netIsConnected()) return;

  const uint32_t now = millis();
  if (now - lastReportTry < REPORT_RETRY_MS) return;
  lastReportTry = now;

  char json[200];
  int len = snprintf(json, sizeof(json), "{\"deviceId\":\"%s\",", DEVICE_ID);
  if (pendingReportCmdId[0] && strcmp(pendingReportCmdId, "null") != 0) {
    len += snprintf(json + len, sizeof(json) - len, "\"commandId\":\"%s\",", pendingReportCmdId);
  }
  snprintf(json + len, sizeof(json) - len, "\"event\":\"%s\"}", pendingReportEvent);

  if (httpPostJSON(SERVER_URL_REPORT, String(json), nullptr)) {
    LOG_PRINTF_IF(DBG_NET, "REPORT", "sent event=%s", pendingReportEvent);
    pendingReport = false;
    pendingReportEvent[0] = 0;
    pendingReportCmdId[0] = 0;
  } else {
    LOG_PRINTF_IF(DBG_NET, "REPORT", "failed event=%s", pendingReportEvent);
  }
}

void pushTrackBatch() {
  if (!netIsConnected()) return;

  TrackPoint localBuf[MAX_RAW_POINTS];
  int localCount = 0;
  bool localAlarm = false;
  bool localLocked = false;
  bool localMoving = false;

  if (xSemaphoreTake(stateMutex, pdMS_TO_TICKS(20)) != pdTRUE) return;
  localCount = rawCount;
  localAlarm = alarmLatched;
  localLocked = locked;
  localMoving = imuMoving;
  if (localCount > 0) memcpy(localBuf, rawBuf, localCount * sizeof(TrackPoint));
  xSemaphoreGive(stateMutex);

  if (localCount == 0) return;

  const DeviceStateId topSt = localAlarm ? ST_ALARM
                                         : (localMoving ? ST_MOVING
                                                        : (localLocked ? ST_LOCKED : ST_UNLOCKED));

  TrackPoint track[MAX_TRACK_POINTS];
  const int n = buildCompressedTrack(track, MAX_TRACK_POINTS, localBuf, localCount);
  if (n <= 0) return;

  char json[850];
  int len = snprintf(json, sizeof(json),
                     "{\"deviceId\":\"%s\",\"state\":\"%s\",\"track\":[",
                     DEVICE_ID, stateToString(topSt));

  for (int i = 0; i < n; i++) {
    const char* pfx = (i == 0) ? "" : ",";
    len += snprintf(json + len, sizeof(json) - len,
                    "%s{\"lat\":%.6f,\"lon\":%.6f,\"speed\":%.2f,\"state\":\"%s\"}",
                    pfx, track[i].lat, track[i].lon, track[i].speed,
                    stateToString(static_cast<DeviceStateId>(track[i].st)));
  }
  snprintf(json + len, sizeof(json) - len, "]}");

  LOG_PRINTF_IF(DBG_NET, "PUSH", "send raw=%d packed=%d", localCount, n);
  const bool ok = httpPostJSON(SERVER_URL_PUSH, String(json), nullptr);

  if (ok) {
    if (xSemaphoreTake(stateMutex, pdMS_TO_TICKS(100)) == pdTRUE) {
      if (rawCount >= localCount) {
        const int remaining = rawCount - localCount;
        if (remaining > 0) memmove(rawBuf, rawBuf + localCount, remaining * sizeof(TrackPoint));
        rawCount = remaining;
      }
      xSemaphoreGive(stateMutex);
    }
    LOG_PRINTF_IF(DBG_NET, "PUSH", "success count=%d", n);
  } else {
    LOG_PRINTLN_IF(DBG_NET, "PUSH", F("Failed, buffer kept"));
  }
}

void pollCommand() {
  if (!netIsConnected()) return;

  char json[64];
  snprintf(json, sizeof(json), "{\"deviceId\":\"%s\"}", DEVICE_ID);

  String body;
  if (!httpPostJSON(SERVER_URL_POLL, String(json), &body)) return;

  const String command = jsonFindString(body, "command");
  const String commandId = jsonFindString(body, "commandId");
  if (!command.length() || command == "null") {
    LOG_PRINTLN_IF(DBG_NET, "POLL", F("No command"));
    return;
  }

  CmdMsg msg = {};
  command.toCharArray(msg.cmd, sizeof(msg.cmd));
  commandId.toCharArray(msg.cmdId, sizeof(msg.cmdId));
  if (xQueueSend(cmdQueue, &msg, pdMS_TO_TICKS(100)) != pdTRUE) {
    LOG_PRINTLN_IF(DBG_NET, "NET", F("cmdQueue full"));
    return;
  }

  LOG_PRINTF_IF(DBG_NET,
                "POLL",
                "command=%s cmdId=%s",
                msg.cmd,
                msg.cmdId[0] ? msg.cmdId : "none");
}

}  // namespace

bool netIsConnected() {
  return modemReady && bearerReady;
}

uint32_t adaptivePollInterval(uint32_t baseMs) {
  uint32_t interval = baseMs;

  if (lastKnownCsq >= 0) {
    if (lastKnownCsq <= 9) interval = max(interval, baseMs * 4U);
    else if (lastKnownCsq <= 11) interval = max(interval, baseMs * 2U);
  }

  if (lastHttpLatencyMs >= 15000UL) interval = max(interval, baseMs * 4U);
  else if (lastHttpLatencyMs >= 8000UL) interval = max(interval, baseMs * 2U);

  return interval;
}

void netTask(void* pv) {
  (void)pv;
  vTaskDelay(pdMS_TO_TICKS(3000));

  uint32_t lastPushLocal = 0;
  uint32_t lastPollLocal = 0;

  for (;;) {
    gsmMaintenance();
    handleIncomingSms();
    internetProbe();
    pumpPendingReport();

    const bool still = isBikeStationary();
    const bool noNet = !netIsConnected() || !gInternetOk;

    const uint32_t pushInt = noNet ? PUSH_INTERVAL_NONET_MS
                                   : (still ? PUSH_INTERVAL_STILL_MS : PUSH_INTERVAL_MOVING_MS);
    if (millis() - lastPushLocal >= pushInt) {
      lastPushLocal = millis();
      pushTrackBatch();
    }

    bool fastPoll = false;
    if (xSemaphoreTake(stateMutex, pdMS_TO_TICKS(100)) == pdTRUE) {
      fastPoll = armed || alarmLatched;
      xSemaphoreGive(stateMutex);
    }

    const uint32_t basePollInt = noNet ? POLL_INTERVAL_NONET_MS
                                       : (fastPoll ? POLL_INTERVAL_FAST_MS : POLL_INTERVAL_NORMAL_MS);
    const uint32_t pollInt = noNet ? basePollInt : adaptivePollInterval(basePollInt);
    if (millis() - lastPollLocal >= pollInt) {
      lastPollLocal = millis();
      pollCommand();
    }

    vTaskDelay(pdMS_TO_TICKS(100));
  }
}
