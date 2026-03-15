#pragma once

#include <Arduino.h>

enum DeviceStateId : uint8_t {
  ST_LOCKED = 0,
  ST_UNLOCKED = 1,
  ST_MOVING = 2,
  ST_ALARM = 3
};

struct TrackPoint {
  double lat;
  double lon;
  float speed;
  uint32_t tMs;
  uint8_t st;
};

struct BeepStep {
  uint16_t ms;
  uint8_t on;
};

struct CmdMsg {
  char cmd[16];
  char cmdId[40];
};

struct ReportMsg {
  char event[24];
  char cmdId[40];
};

struct GyroBias {
  float x = 0.0f;
  float y = 0.0f;
  float z = 0.0f;
};
