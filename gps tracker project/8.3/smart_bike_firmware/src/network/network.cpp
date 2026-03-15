#include "network.h"

#include "../core/state_logic.h"
#include "../sensors/sensors.h"
#include "../tracking/tracking.h"

namespace {

void netOnSuccess() {
  netFailStreak = 0;
  netBackoffMs = 15000;
  nextNetTryAt = millis();
}

void netOnFail() {
  netFailStreak++;
  const uint32_t next = netBackoffMs * 2;
  netBackoffMs = min(next, static_cast<uint32_t>(NET_BACKOFF_MAX_MS));
  nextNetTryAt = millis() + netBackoffMs;
  if (DBG_NET) {
    Serial.printf("[WIFI] backoff %lu ms streak=%d\n",
                  static_cast<unsigned long>(netBackoffMs), netFailStreak);
  }
}

void wifiMaintenance() {
  const bool connected = wifiIsConnected();
  if (connected) {
    if (!wifiWasConnected) {
      wifiWasConnected = true;
      wifiConnecting = false;
      netOnSuccess();
      if (DBG_NET) Serial.printf("[WIFI] connected  IP=%s\n", WiFi.localIP().toString().c_str());
    }
    return;
  }

  if (wifiWasConnected) {
    wifiWasConnected = false;
    wifiConnecting = false;
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

  if (millis() >= nextNetTryAt) {
    if (DBG_NET) Serial.printf("[WIFI] connect -> %s\n", WIFI_SSID);
    WiFi.begin(WIFI_SSID, WIFI_PASS);
    wifiConnecting = true;
    wifiConnectStartedAt = millis();
  }
}

void internetProbe() {
  const uint32_t now = millis();
  if (now - lastNetCheckAt < NET_CHECK_EVERY_MS) {
    gInternetOk = (lastInternetOkAt != 0) && (now - lastInternetOkAt < NET_OK_TTL_MS);
    return;
  }
  lastNetCheckAt = now;

  if (!wifiIsConnected()) {
    gInternetOk = false;
    lastInternetOkAt = 0;
    return;
  }

  IPAddress ip;
  bool ok = WiFi.hostByName("smartbike.ashikdev.com", ip);
  if (ok) {
    WiFiClient c;
    c.setTimeout(1500);
    ok = c.connect(ip, 80);
    c.stop();
  }
  if (ok) lastInternetOkAt = now;
  gInternetOk = ok;
  if (DBG_NET) Serial.printf("[NET] probe=%s\n", ok ? "OK" : "FAIL");
}

bool httpPostJSON(const char* url, const String& json, String* outBody) {
  if (!wifiIsConnected()) return false;

  WiFiClient client;
  client.setTimeout(HTTP_TIMEOUT_MS / 1000);

  HTTPClient http;
  http.setTimeout(HTTP_TIMEOUT_MS);
  http.setReuse(false);
  http.useHTTP10(true);

  if (!http.begin(client, url)) return false;
  http.addHeader("Content-Type", "application/json");

  if (DBG_NET) Serial.printf("[HTTP] POST %s\n", url);

  const int code = http.POST(reinterpret_cast<uint8_t*>(const_cast<char*>(json.c_str())), json.length());
  if (code <= 0) {
    if (DBG_NET) Serial.printf("[HTTP] fail code=%d\n", code);
    http.end();
    return false;
  }

  if (outBody) {
    *outBody = http.getString();
    if (outBody->length() > 2048) outBody->remove(2048);
    if (DBG_HTTP_BODY) {
      Serial.print(F("[HTTP] body="));
      Serial.println(*outBody);
    }
  }

  if (DBG_NET) Serial.printf("[HTTP] status=%d\n", code);
  http.end();
  return (code >= 200 && code < 300);
}

String jsonFindString(const String& body, const char* key) {
  const String k = String('"') + key + '"';
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

void pumpPendingReport() {
  ReportMsg rmsg = {};
  while (xQueueReceive(reportQueue, &rmsg, 0) == pdTRUE) {
    pendingReport = true;
    strncpy(pendingReportEvent, rmsg.event, sizeof(pendingReportEvent) - 1);
    pendingReportEvent[sizeof(pendingReportEvent) - 1] = 0;
    strncpy(pendingReportCmdId, rmsg.cmdId, sizeof(pendingReportCmdId) - 1);
    pendingReportCmdId[sizeof(pendingReportCmdId) - 1] = 0;
    lastReportTry = 0;
  }

  if (!pendingReport || !wifiIsConnected()) return;

  const uint32_t now = millis();
  if (now - lastReportTry < REPORT_RETRY_MS) return;
  lastReportTry = now;

  String json;
  json.reserve(200);
  json += F("{\"deviceId\":\"");
  json += DEVICE_ID;
  json += F("\",");
  if (pendingReportCmdId[0] && strcmp(pendingReportCmdId, "null") != 0) {
    json += F("\"commandId\":\"");
    json += pendingReportCmdId;
    json += F("\",");
  }
  json += F("\"event\":\"");
  json += pendingReportEvent;
  json += F("\"}");

  if (httpPostJSON(SERVER_URL_REPORT, json, nullptr)) {
    pendingReport = false;
    pendingReportEvent[0] = 0;
    pendingReportCmdId[0] = 0;
  }
}

void pushTrackBatch() {
  if (!wifiIsConnected()) return;

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

  String json;
  json.reserve(820);
  json += F("{\"deviceId\":\"");
  json += DEVICE_ID;
  json += F("\",");
  json += F("\"state\":\"");
  json += stateToString(topSt);
  json += F("\",");
  json += F("\"track\":[");

  for (int i = 0; i < n; i++) {
    if (i) json += ',';
    json += F("{\"lat\":");
    json += String(track[i].lat, 6);
    json += F(",\"lon\":");
    json += String(track[i].lon, 6);
    json += F(",\"speed\":");
    json += String(track[i].speed, 2);
    json += F(",\"state\":\"");
    json += stateToString(static_cast<DeviceStateId>(track[i].st));
    json += F("\"}");
  }
  json += F("]}");

  const bool ok = httpPostJSON(SERVER_URL_PUSH, json, nullptr);

  if (ok) {
    xSemaphoreTake(stateMutex, portMAX_DELAY);
    if (rawCount >= localCount) {
      const int remaining = rawCount - localCount;
      if (remaining > 0) memmove(rawBuf, rawBuf + localCount, remaining * sizeof(TrackPoint));
      rawCount = remaining;
    }
    xSemaphoreGive(stateMutex);
  } else {
    Serial.println(F("[PUSH] failed, keeping buffer"));
  }
}

void pollCommand() {
  if (!wifiIsConnected()) return;

  String json;
  json.reserve(48);
  json += F("{\"deviceId\":\"");
  json += DEVICE_ID;
  json += F("\"}");

  String body;
  if (!httpPostJSON(SERVER_URL_POLL, json, &body)) return;

  const String command = jsonFindString(body, "command");
  const String commandId = jsonFindString(body, "commandId");
  if (!command.length() || command == "null") return;

  CmdMsg msg = {};
  command.toCharArray(msg.cmd, sizeof(msg.cmd));
  commandId.toCharArray(msg.cmdId, sizeof(msg.cmdId));
  if (xQueueSend(cmdQueue, &msg, pdMS_TO_TICKS(100)) != pdTRUE) {
    Serial.println(F("[NET] cmdQueue full, dropped"));
  }
}

}  // namespace

bool wifiIsConnected() {
  return WiFi.status() == WL_CONNECTED;
}

void netTask(void* pv) {
  (void)pv;
  vTaskDelay(pdMS_TO_TICKS(2000));

  uint32_t lastPushLocal = 0;
  uint32_t lastPollLocal = 0;

  for (;;) {
    wifiMaintenance();
    internetProbe();
    pumpPendingReport();

    const bool still = isBikeStationary();
    const bool noNet = !wifiIsConnected() || !gInternetOk;

    const uint32_t pushInterval = noNet ? PUSH_INTERVAL_NONET_MS
                                        : (still ? PUSH_INTERVAL_STILL_MS : PUSH_INTERVAL_MOVING_MS);
    if (millis() - lastPushLocal >= pushInterval) {
      lastPushLocal = millis();
      pushTrackBatch();
    }

    bool fastPoll = false;
    xSemaphoreTake(stateMutex, portMAX_DELAY);
    fastPoll = armed || alarmLatched;
    xSemaphoreGive(stateMutex);

    const uint32_t pollInterval = noNet ? POLL_INTERVAL_NONET_MS
                                        : (fastPoll ? POLL_INTERVAL_FAST_MS : POLL_INTERVAL_NORMAL_MS);
    if (millis() - lastPollLocal >= pollInterval) {
      lastPollLocal = millis();
      pollCommand();
    }

    vTaskDelay(pdMS_TO_TICKS(50));
  }
}
