/*=============================================================
 *  SmartBike v8.3 "Deep Recharge"
 *  Modular refactor with preserved runtime behavior.
 *=============================================================*/

#include "src/smart_bike_app.h"

void setup() {
  Serial.begin(921600);
  delay(600);
  Serial.println(F("\n=== SmartBike v8.3  Deep Recharge ==="));

  btStop();
  esp_bt_controller_mem_release(ESP_BT_MODE_BTDM);

  pinMode(RELAY_PIN, OUTPUT);
  pinMode(BUZZER_RELAY_PIN, OUTPUT);
  if (LED_PIN >= 0) pinMode(LED_PIN, OUTPUT);
  if (ONBOARD_LED_PIN >= 0) pinMode(ONBOARD_LED_PIN, OUTPUT);
  digitalWrite(RELAY_PIN, RELAY_ACTIVE_LOW ? HIGH : LOW);
  digitalWrite(BUZZER_RELAY_PIN, RELAY_ACTIVE_LOW ? HIGH : LOW);
  setLed(false);

  SerialGPS.begin(9600, SERIAL_8N1, GPS_RX, GPS_TX);
  initIMU();

  WiFi.persistent(false);
  WiFi.setAutoReconnect(false);
  WiFi.setSleep(false);
  WiFi.mode(WIFI_AP_STA);
  delay(100);

  char apSsid[40];
  snprintf(apSsid, sizeof(apSsid), "%s-LOCAL", DEVICE_ID);
  WiFi.softAP(apSsid, LOCAL_AP_PASS);
  delay(100);

  Serial.printf("[WIFI] AP SSID=%s  IP=%s\n", apSsid, WiFi.softAPIP().toString().c_str());

  stateMutex = xSemaphoreCreateMutex();
  cmdQueue = xQueueCreate(4, sizeof(CmdMsg));
  reportQueue = xQueueCreate(8, sizeof(ReportMsg));

  configASSERT(stateMutex);
  configASSERT(cmdQueue);
  configASSERT(reportQueue);

  localPortalStart();

  xTaskCreatePinnedToCore(
    netTask,
    "netTask",
    8192,
    nullptr,
    2,
    &netTaskHandle,
    0
  );

  const uint32_t now = millis();
  lastSample = now;
  lastHeartbeat = now;

  Serial.println(F("=== Setup done - Core 1 running ==="));
  Serial.printf("[HEAP] free=%d\n", esp_get_free_heap_size());
}

void loop() {
  gpsPump();
  updateImuMotion();
  updateSecurityAlarm();
  buzzerUpdate();
  localPortalMaintenance();

  CmdMsg cmsg = {};
  while (xQueueReceive(cmdQueue, &cmsg, 0) == pdTRUE) {
    executeCommand(cmsg.cmd, cmsg.cmdId);
  }

  {
    static uint32_t lastBlink = 0;
    static bool ledState = false;
    xSemaphoreTake(stateMutex, pdMS_TO_TICKS(10));
    const bool alarm = alarmLatched;
    xSemaphoreGive(stateMutex);
    const bool noWifi = !wifiIsConnected();
    const uint32_t blinkMs = alarm ? 140 : (noWifi ? 250 : 900);
    if (millis() - lastBlink >= blinkMs) {
      lastBlink = millis();
      ledState = !ledState;
      setLed(ledState);
    }
  }

  const uint32_t now = millis();
  const bool still = isBikeStationary();

  if (now - lastSample >= (still ? SAMPLE_INTERVAL_STILL_MS : SAMPLE_INTERVAL_MOVING_MS)) {
    lastSample = now;
    recordSamplePoint();
  }

  if (now - lastHeartbeat >= HEARTBEAT_INTERVAL_MS) {
    lastHeartbeat = now;
    if (still) recordHeartbeatPoint(F("STILL heartbeat"));
  }

  if (DBG_GPS) {
    static uint32_t lastDbg = 0;
    if (now - lastDbg >= 10000) {
      lastDbg = now;
      xSemaphoreTake(stateMutex, pdMS_TO_TICKS(10));
      const bool sLocked = locked;
      const bool sArmed = armed;
      const bool sAlarm = alarmLatched;
      const int sBuf = rawCount;
      xSemaphoreGive(stateMutex);

      Serial.printf(
        "[DBG] gps=%d age=%lu sats=%d hdop=%.1f spd=%.1f "
        "imu=%d locked=%d armed=%d alarm=%d buf=%d "
        "wifi=%d net=%d portal=%d heap=%d\n",
        static_cast<int>(gps.location.isValid()),
        static_cast<unsigned long>(gps.location.age()),
        gps.satellites.isValid() ? static_cast<int>(gps.satellites.value()) : -1,
        gps.hdop.isValid() ? static_cast<double>(gps.hdop.hdop()) : -1.0,
        gps.speed.isValid() ? static_cast<double>(gps.speed.kmph()) : -1.0,
        static_cast<int>(imuMoving),
        static_cast<int>(sLocked),
        static_cast<int>(sArmed),
        static_cast<int>(sAlarm),
        sBuf,
        static_cast<int>(wifiIsConnected()),
        static_cast<int>(static_cast<bool>(gInternetOk)),
        static_cast<int>(localApActive),
        static_cast<int>(esp_get_free_heap_size()));
    }
  }
}
