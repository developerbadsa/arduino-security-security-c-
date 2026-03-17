#include "local_portal.h"

#include "../core/command_processor.h"
#include "../core/state_logic.h"
#include "portal_html.h"

namespace {

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
  bool sWifi = false;
  bool sNet = false;
  float sDeltaG = 0.0f;
  float sGMag = 0.0f;
  bool sImuMoving = false;
  bool sImuReady = false;
  int sBufCount = 0;
  DeviceStateId sSt = ST_UNLOCKED;

  xSemaphoreTake(stateMutex, pdMS_TO_TICKS(100));
  sLocked = locked;
  sArmed = armed;
  sAlarm = alarmLatched;
  sDeltaG = lastADeltaG;
  sGMag = lastGMag;
  sImuMoving = imuMoving;
  sBufCount = rawCount;
  sSt = getCurrentState_locked();
  xSemaphoreGive(stateMutex);

  sImuReady = imuReady;
  sWifi = (WiFi.status() == WL_CONNECTED);
  sNet = static_cast<bool>(gInternetOk);

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
  out.reserve(450);
  out += F("{\"state\":\"");
  out += stateToString(sSt);
  out += F("\",");
  out += F("\"locked\":");
  out += sLocked ? "true" : "false";
  out += F(",");
  out += F("\"armed\":");
  out += sArmed ? "true" : "false";
  out += F(",");
  out += F("\"alarm\":");
  out += sAlarm ? "true" : "false";
  out += F(",");
  out += F("\"wifi\":");
  out += sWifi ? "true" : "false";
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
  out += F(" raw\"}");

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
}

}  // namespace

void localPortalStart() {
  if (localApActive) return;

  localApLastActivityAt = millis();
  IPAddress ip = WiFi.softAPIP();
  localDns.start(53, "*", ip);
  localPortalSetupRoutes();
  localServer.begin();
  localApActive = true;

  Serial.printf("[LOCAL] active=%d ip=%s stations=%d\n",
                static_cast<int>(localApActive),
                WiFi.softAPIP().toString().c_str(),
                WiFi.softAPgetStationNum());

  if (DBG_LOCAL) {
    char ssid[40];
    WiFi.softAPSSID().toCharArray(ssid, sizeof(ssid));
    Serial.printf("[LOCAL] Portal ON  ssid=%s  ip=%s\n", ssid, ip.toString().c_str());
  }
}

void localPortalStop() {
  if (!localApActive) return;
  localServer.stop();
  localDns.stop();
  localApActive = false;
  localApLastActivityAt = 0;
  if (DBG_LOCAL) Serial.println(F("[LOCAL] Portal OFF"));
}

void localPortalMaintenance() {
  if (!localApActive) return;
  localDns.processNextRequest();
  localServer.handleClient();
}
