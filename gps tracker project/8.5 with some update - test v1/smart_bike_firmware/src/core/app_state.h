#pragma once

#if !defined(ARDUINO_ARCH_ESP32)
#error "SmartBike firmware requires an ESP32 board package. In Arduino IDE install/select an Espressif ESP32 board such as ESP32 Dev Module."
#endif

#include <Arduino.h>
#include <DNSServer.h>
#include <HardwareSerial.h>
#include <WebServer.h>
#include <WiFi.h>
#include <Wire.h>
#include <string.h>
#include <math.h>

#include "../config/app_config.h"
#include "app_types.h"
#include "compat_deps.h"

#ifdef __has_include
#if __has_include(<freertos/FreeRTOS.h>)
#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>
#include <freertos/semphr.h>
#include <freertos/task.h>
#elif __has_include(<FreeRTOS.h>)
#include <FreeRTOS.h>
#include <queue.h>
#include <semphr.h>
#include <task.h>
#endif
#endif

extern SemaphoreHandle_t stateMutex;
extern QueueHandle_t cmdQueue;
extern QueueHandle_t reportQueue;
extern TaskHandle_t netTaskHandle;

extern HardwareSerial SerialGPS;
extern TinyGPSPlus gps;
extern bfs::Mpu6500 imu;

extern volatile bool locked;
extern volatile bool armed;
extern volatile bool alarmLatched;
extern volatile uint32_t alarmCooldownUntil;

extern TrackPoint rawBuf[MAX_RAW_POINTS];
extern int rawCount;

extern float filteredSpeed;
extern bool hasFilteredInit;
extern int lowSpeedCount;
extern bool imuMoving;

extern bool imuReady;
extern GyroBias imuBias;
extern uint32_t imuMovingHoldUntil;
extern uint32_t lastImuReadAt;
extern float lastADeltaG;
extern float lastGMag;
extern uint32_t tamperMotionSince;
extern uint32_t armSettleUntil;

extern uint8_t knockCount;
extern uint32_t firstKnockAt;
extern uint32_t lastKnockAt;
extern uint32_t touchSince;
extern uint32_t lastTouchFireAt;

extern uint32_t lastSample;
extern uint32_t lastHeartbeat;

extern volatile bool gInternetOk;
extern volatile uint32_t lastInternetOkAt;
extern uint32_t lastNetCheckAt;
extern uint32_t lastPush;
extern uint32_t lastPoll;
extern bool netConnecting;
extern bool netWasConnected;
extern uint32_t netConnectStartedAt;
extern uint32_t nextNetTryAt;
extern uint32_t netBackoffMs;
extern int netFailStreak;
extern bool pendingReport;
extern char pendingReportEvent[24];
extern char pendingReportCmdId[40];
extern uint32_t lastReportTry;

extern WebServer localServer;
extern DNSServer localDns;
extern bool localApActive;
extern uint32_t localApLastActivityAt;
extern uint8_t localPinFailCount;
extern uint32_t localPinLockedUntil;

extern volatile bool localApManualTrigger;
extern uint32_t localApManualTriggerAt;

extern volatile PortalMode portalMode;
