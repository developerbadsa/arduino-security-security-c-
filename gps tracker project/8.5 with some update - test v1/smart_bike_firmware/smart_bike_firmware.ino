// Smart Bike Firmware - 9
#include "src/smart_bike_app.h"
#include <esp_err.h>
#include <esp_task_wdt.h>

#if __has_include(<esp_idf_version.h>)
#include <esp_idf_version.h>
#endif

namespace {

constexpr uint32_t WDT_TIMEOUT_SEC = 30;
constexpr uint32_t WDT_TIMEOUT_MS = WDT_TIMEOUT_SEC * 1000U;

bool configureTaskWatchdog() {
#if defined(ESP_IDF_VERSION) && (ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(5, 0, 0))
  const esp_task_wdt_config_t wdtConfig = {WDT_TIMEOUT_MS, 0, true};

  esp_err_t err = esp_task_wdt_init(&wdtConfig);
  if (err == ESP_ERR_INVALID_STATE) {
    err = esp_task_wdt_reconfigure(&wdtConfig);
  }
#else
  esp_err_t err = esp_task_wdt_init(WDT_TIMEOUT_SEC, true);
#endif

  if (err != ESP_OK) {
    logPrintf("SYS", "Task watchdog init failed: %s", esp_err_to_name(err));
    return false;
  }

  err = esp_task_wdt_status(nullptr);
  if (err == ESP_ERR_NOT_FOUND) {
    err = esp_task_wdt_add(nullptr);
  }

  if (err != ESP_OK) {
    logPrintf("SYS", "Task watchdog subscribe failed: %s", esp_err_to_name(err));
    return false;
  }

  return true;
}

}  // namespace

void setup() {
  Serial.begin(SERIAL_LOG_BAUD);
  delay(2000);
  logPrintln("SYS", F("SmartBike v8.5 GSM Main Firmware"));

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
  WiFi.disconnect(true, true);
  WiFi.mode(WIFI_OFF);
  delay(100);

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

  logPrintln("SYS", F("Setup done - full features active"));

  if (configureTaskWatchdog()) {
    logPrintf("SYS", "Task watchdog enabled (%lu ms)", static_cast<unsigned long>(WDT_TIMEOUT_MS));
  }
}

void loop() {
  gpsPump();
  updateImuMotion();
  updateSecurityAlarm();
  buzzerUpdate();
  localPortalMaintenance();
  esp_task_wdt_reset(); // Feed watchdog from main loop

  CmdMsg cmsg = {};
  while (xQueueReceive(cmdQueue, &cmsg, 0) == pdTRUE) {
    executeCommand(cmsg.cmd, cmsg.cmdId);
  }

  {
    static uint32_t lastBlink = 0;
    static bool ledState = false;

    bool alarm = false;
    if (xSemaphoreTake(stateMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
      alarm = alarmLatched;
      xSemaphoreGive(stateMutex);
    }

    const bool noNet = !netIsConnected() || !gInternetOk;
    const uint32_t blinkMs = alarm ? 140 : (noNet ? 250 : 900);
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

      bool sLocked = false;
      bool sArmed = false;
      bool sAlarm = false;
      int sBuf = 0;
      if (xSemaphoreTake(stateMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
        sLocked = locked;
        sArmed = armed;
        sAlarm = alarmLatched;
        sBuf = rawCount;
        xSemaphoreGive(stateMutex);
      }

      logPrintf("DBG",
                "gps=%d age=%lu sats=%d hdop=%.1f spd=%.1f imu=%d locked=%d armed=%d alarm=%d buf=%d ap=%d gsm=%d net=%d",
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
                static_cast<int>(localApActive),
                static_cast<int>(netIsConnected()),
                static_cast<int>(static_cast<bool>(gInternetOk)));
    }
  }
}
