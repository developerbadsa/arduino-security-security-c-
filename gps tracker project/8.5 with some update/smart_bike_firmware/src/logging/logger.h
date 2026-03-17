#pragma once

#include <Arduino.h>

#define LOG_PRINTF_IF(flag, tag, fmt, ...)         \
  do {                                             \
    if (flag) {                                    \
      Serial.printf("[" tag "] " fmt "\n", ##__VA_ARGS__); \
    }                                              \
  } while (0)

#define LOG_PRINTLN_IF(flag, tag, msg)             \
  do {                                             \
    if (flag) {                                    \
      Serial.print(F("[" tag "] "));               \
      Serial.println(msg);                         \
    }                                              \
  } while (0)
