#pragma once

#include <Arduino.h>

// Server / device config
static constexpr const char* SERVER_URL_PUSH = "http://smartbike.ashikdev.com/api/device/push";
static constexpr const char* SERVER_URL_POLL = "http://smartbike.ashikdev.com/api/device/poll";
static constexpr const char* SERVER_URL_REPORT = "http://smartbike.ashikdev.com/api/device/report";
static constexpr const char* DEVICE_ID = "BIKE01";

// Wi-Fi config
static constexpr const char* WIFI_SSID = "a";
static constexpr const char* WIFI_PASS = "87654321q";
static constexpr const char* LOCAL_AP_PASS = "bikelocal88";
static constexpr const char* LOCAL_PIN = "2234";

// Debug flags
static constexpr bool DBG_NET = true;
static constexpr bool DBG_HTTP_BODY = true;
static constexpr bool DBG_GPS = true;
static constexpr bool DBG_IMU = false;
static constexpr bool DBG_SEC = true;
static constexpr bool DBG_LOCAL = true;

// Hardware pins
static constexpr int GPS_RX = 16;
static constexpr int GPS_TX = 17;
static constexpr int I2C_SDA = 21;
static constexpr int I2C_SCL = 22;
static constexpr int RELAY_PIN = 25;
static constexpr int BUZZER_RELAY_PIN = 32;
static constexpr int LED_PIN = 33;
static constexpr int ONBOARD_LED_PIN = 2;
static constexpr bool RELAY_ACTIVE_LOW = true;

// Timing constants
static constexpr uint32_t SAMPLE_INTERVAL_MOVING_MS = 1000UL;
static constexpr uint32_t SAMPLE_INTERVAL_STILL_MS = 2000UL;
static constexpr uint32_t PUSH_INTERVAL_MOVING_MS = 15000UL;
static constexpr uint32_t PUSH_INTERVAL_STILL_MS = 120000UL;
static constexpr uint32_t PUSH_INTERVAL_NONET_MS = 300000UL;
static constexpr uint32_t POLL_INTERVAL_FAST_MS = 1200UL;
static constexpr uint32_t POLL_INTERVAL_NORMAL_MS = 2500UL;
static constexpr uint32_t POLL_INTERVAL_NONET_MS = 180000UL;
static constexpr uint32_t HEARTBEAT_INTERVAL_MS = 5UL * 60UL * 1000UL;
static constexpr uint32_t NET_CHECK_EVERY_MS = 8000UL;
static constexpr uint32_t NET_OK_TTL_MS = 15000UL;
static constexpr uint32_t REPORT_RETRY_MS = 5000UL;
static constexpr uint32_t HTTP_TIMEOUT_MS = 3000UL;

// Buzzer
static constexpr uint32_t ALARM_BUZZ_MS = 12000UL;
static constexpr uint32_t ALARM_COOLDOWN_MS = 2000UL;

// IMU
static constexpr uint32_t IMU_READ_EVERY_MS = 20UL;
static constexpr uint32_t MOVING_HOLD_MS = 1200UL;
static constexpr float GYRO_MOVING_ON = 0.30f;
static constexpr float GYRO_MOVING_OFF = 0.22f;
static constexpr float ACCEL_DELTA_ON = 0.25f;
static constexpr float ACCEL_DELTA_OFF = 0.15f;
static constexpr float G_MPS2 = 9.80665f;
static constexpr uint16_t CAL_SAMPLES = 250;
static constexpr uint16_t CAL_DELAY_MS = 10;
static constexpr uint16_t ARM_CAL_SAMPLES = 80;
static constexpr uint16_t ARM_CAL_DELAY_MS = 5;

// Security
static constexpr uint32_t ARM_SETTLE_MS = 2500UL;
static constexpr float TOUCH_DELTA_G = 0.04f;
static constexpr float TOUCH_GYRO = 0.12f;
static constexpr uint32_t TOUCH_TRIGGER_MS = 80UL;
static constexpr uint32_t TOUCH_DEBOUNCE_MS = 100UL;
static constexpr float KNOCK_DELTA_G = 0.18f;
static constexpr float KNOCK_GYRO = 0.80f;
static constexpr uint32_t KNOCK_WINDOW_MS = 2500UL;
static constexpr uint32_t KNOCK_DEBOUNCE_MS = 180UL;
static constexpr uint8_t KNOCK_COUNT_REQ = 2;
static constexpr uint32_t TAMPER_MOTION_MS = 900UL;

// Speed filter
static constexpr float MIN_SPEED_KMPH = 2.0f;
static constexpr float MAX_SPEED_KMPH = 130.0f;
static constexpr float SPEED_ALPHA = 0.4f;
static constexpr int STOP_COUNT_REQ = 4;

// Track buffer
static constexpr int MAX_RAW_POINTS = 20;
static constexpr int MAX_TRACK_POINTS = 5;
static constexpr float MIN_SAMPLE_DIST_M = 12.0f;
static constexpr double MAX_JUMP_DIST_M = 50000.0;

// Local portal PIN
static constexpr uint8_t LOCAL_PIN_FAIL_MAX = 5;
static constexpr uint32_t LOCAL_PIN_LOCK_MS = 30000UL;

// Wi-Fi backoff
static constexpr uint32_t WIFI_CONNECT_TIMEOUT_MS = 15000UL;
static constexpr uint32_t NET_BACKOFF_MAX_MS = 5UL * 60UL * 1000UL;
