#include "local_portal.h"

#include "../core/command_processor.h"
#include "../core/state_logic.h"
#include "../logging/logger.h"
#include "../network/network.h"
#include "portal_html.h"

namespace {

bool localRoutesReady = false;

void appendJsonEscaped(String& out, const char* text) {
  out += '\"';
  if (text) {
    for (size_t i = 0; text[i]; ++i) {
      const char c = text[i];
      switch (c) {
        case '\\': out += F("\\\\"); break;
        case '\"': out += F("\\\""); break;
        case '\b': out += F("\\b"); break;
        case '\f': out += F("\\f"); break;
        case '\n': out += F("\\n"); break;
        case '\r': out += F("\\r"); break;
        case '\t': out += F("\\t"); break;
        default:
          if (static_cast<uint8_t>(c) < 0x20) {
            out += ' ';
          } else {
            out += c;
          }
          break;
      }
    }
  }
  out += '\"';
}

bool shouldKeepPortalOn() {
  return alarmLatched || !netIsConnected() || !gInternetOk;
}

bool localCheckPinOk() {
  const uint32_t now = millis();
  if (now < localPinLockedUntil) return false;

  String pin = localServer.arg("pin");
  if (!pin.length()) pin = localServer.arg("p");

  if (pin != LOCAL_PIN) {
    if (++localPinFailCount >= LOCAL_PIN_FAIL_MAX) {
      localPinFailCount = 0;
      localPinLockedUntil = now + LOCAL_PIN_LOCK_MS;
    }
    return false;
  }

  localPinFailCount = 0;
  localPinLockedUntil = 0;
  return true;
}

void localHandleRoot() {
  localApLastActivityAt = millis();
  localServer.send_P(200, "text/html", HTML_PAGE);
}

void localHandleStatus() {
  localApLastActivityAt = millis();

  bool sLocked = false;
  bool sArmed = false;
  bool sAlarm = false;
  bool sAp = false;
  bool sNet = false;
  float sDeltaG = 0.0f;
  float sGMag = 0.0f;
  bool sImuMoving = false;
  bool sImuReady = false;
  int sBufCount = 0;
  DeviceStateId sSt = ST_UNLOCKED;
  RecentLogEntry logs[RECENT_LOG_COUNT] = {};
  const size_t logCount = recentLogSnapshot(logs, RECENT_LOG_COUNT);

  if (xSemaphoreTake(stateMutex, pdMS_TO_TICKS(100)) == pdTRUE) {
    sLocked = locked;
    sArmed = armed;
    sAlarm = alarmLatched;
    sDeltaG = lastADeltaG;
    sGMag = lastGMag;
    sImuMoving = imuMoving;
    sBufCount = rawCount;
    sSt = getCurrentState_locked();
    xSemaphoreGive(stateMutex);
  }

  sImuReady = imuReady;
  sAp = localApActive;
  sNet = netIsConnected() && static_cast<bool>(gInternetOk);

  char gpsStr[80];
  snprintf(gpsStr, sizeof(gpsStr), "%s age=%lu sats=%d hdop=%.1f",
           gps.location.isValid() ? "valid" : "no",
           static_cast<unsigned long>(gps.location.age()),
           gps.satellites.isValid() ? static_cast<int>(gps.satellites.value()) : -1,
           gps.hdop.isValid() ? static_cast<double>(gps.hdop.hdop()) : -1.0);

  char spdStr[30];
  snprintf(spdStr, sizeof(spdStr), "%.1f km/h",
           gps.speed.isValid() ? static_cast<double>(gps.speed.kmph()) : -1.0);

  char imuStr[60];
  snprintf(imuStr, sizeof(imuStr), "%s moving=%d dG=%.3f gyro=%.3f",
           sImuReady ? "ok" : "off",
           static_cast<int>(sImuMoving),
           static_cast<double>(sDeltaG),
           static_cast<double>(sGMag));

  String out;
  out.reserve(3200);
  out += F("{\"state\":\"");
  out += stateToString(sSt);
  out += F("\",");
  out += F("\"now\":");
  out += millis();
  out += F(",");
  out += F("\"locked\":");
  out += sLocked ? "true" : "false";
  out += F(",");
  out += F("\"armed\":");
  out += sArmed ? "true" : "false";
  out += F(",");
  out += F("\"alarm\":");
  out += sAlarm ? "true" : "false";
  out += F(",");
  out += F("\"ap\":");
  out += sAp ? "true" : "false";
  out += F(",");
  out += F("\"net\":");
  out += sNet ? "true" : "false";
  out += F(",");
  out += F("\"gps\":\"");
  out += gpsStr;
  out += F("\",");
  out += F("\"speed\":\"");
  out += spdStr;
  out += F("\",");
  out += F("\"imu\":\"");
  out += imuStr;
  out += F("\",");
  out += F("\"buf\":\"");
  out += sBufCount;
  out += F(" raw\",");
  out += F("\"logs\":[");
  for (size_t i = 0; i < logCount; ++i) {
    if (i) out += ',';
    out += F("{\"ms\":");
    out += logs[i].atMs;
    out += F(",\"text\":");
    appendJsonEscaped(out, logs[i].text);
    out += '}';
  }
  out += F("]}");

  localServer.send(200, "application/json", out);
}

void localHandleCmd(const String& cmd) {
  localApLastActivityAt = millis();
  if (millis() < localPinLockedUntil) {
    localServer.send(429, "text/plain", "PIN LOCKED - WAIT");
    return;
  }
  if (!localCheckPinOk()) {
    localServer.send(401, "text/plain", "BAD PIN");
    return;
  }
  executeCommand(cmd.c_str(), "LOCAL");
  localServer.send(200, "text/plain", "OK");
}

void localHandleNotFound() {
  localApLastActivityAt = millis();
  String loc = "http://";
  loc += WiFi.softAPIP().toString();
  loc += "/";
  localServer.sendHeader("Location", loc, true);
  localServer.send(302, "text/plain", "");
}

void localPortalSetupRoutes() {
  if (localRoutesReady) return;
  localServer.on("/", HTTP_ANY, localHandleRoot);
  localServer.on("/status", HTTP_ANY, localHandleStatus);
  localServer.on("/api/lock", HTTP_ANY, [] { localHandleCmd("LOCK"); });
  localServer.on("/api/unlock", HTTP_ANY, [] { localHandleCmd("UNLOCK"); });
  localServer.on("/api/arm", HTTP_ANY, [] { localHandleCmd("ARM"); });
  localServer.on("/api/buzz", HTTP_ANY, [] { localHandleCmd("BUZZ"); });
  localServer.on("/generate_204", HTTP_ANY, localHandleRoot);
  localServer.on("/gen_204", HTTP_ANY, localHandleRoot);
  localServer.on("/hotspot-detect.html", HTTP_ANY, localHandleRoot);
  localServer.on("/connecttest.txt", HTTP_ANY, localHandleRoot);
  localServer.on("/ncsi.txt", HTTP_ANY, localHandleRoot);
  localServer.onNotFound(localHandleNotFound);
  localRoutesReady = true;
}

}  // namespace

void localPortalStart() {
  if (localApActive) return;

  WiFi.mode(WIFI_AP);
  delay(50);

  char apSsid[40];
  snprintf(apSsid, sizeof(apSsid), "%s-LOCAL", DEVICE_ID);
  WiFi.softAP(apSsid, LOCAL_AP_PASS);
  delay(100);

  localApLastActivityAt = millis();
  IPAddress ip = WiFi.softAPIP();
  localDns.start(53, "*", ip);
  localPortalSetupRoutes();
  localServer.begin();
  localApActive = true;

  logPrintf("LOCAL",
            "active=%d ip=%s stations=%d",
            static_cast<int>(localApActive),
            WiFi.softAPIP().toString().c_str(),
            WiFi.softAPgetStationNum());

  if (DBG_LOCAL) {
    logPrintf("LOCAL", "Portal ON ssid=%s ip=%s", apSsid, ip.toString().c_str());
  }
}

void localPortalStop() {
  if (!localApActive) return;
  localServer.stop();
  localDns.stop();
  WiFi.softAPdisconnect(true);
  WiFi.mode(WIFI_OFF);
  localApActive = false;
  localApLastActivityAt = 0;
  if (DBG_LOCAL) logPrintln("LOCAL", F("Portal OFF"));
}

void localPortalMaintenance() {
  if (!localApActive) {
    if (shouldKeepPortalOn()) {
      localPortalStart();
    }
    return;
  }

  localDns.processNextRequest();
  localServer.handleClient();

  const bool hasStations = WiFi.softAPgetStationNum() > 0;
  if (!hasStations &&
      !shouldKeepPortalOn() &&
      localApLastActivityAt != 0 &&
      (millis() - localApLastActivityAt >= LOCAL_AP_IDLE_OFF_MS)) {
    localPortalStop();
  }
}
