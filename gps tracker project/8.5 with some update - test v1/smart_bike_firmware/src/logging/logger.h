#pragma once

#include <Arduino.h>
#include <stdarg.h>

static constexpr size_t RECENT_LOG_COUNT = 16;
static constexpr size_t RECENT_LOG_TEXT_LEN = 120;

struct RecentLogEntry {
  uint32_t atMs;
  char text[RECENT_LOG_TEXT_LEN];
};

void logPrintf(const char* tag, const char* fmt, ...);
void logPrintln(const char* tag, const char* msg);
void logPrintln(const char* tag, const String& msg);
void logPrintln(const char* tag, const __FlashStringHelper* msg);
size_t recentLogSnapshot(RecentLogEntry* out, size_t maxOut);

#define LOG_PRINTF_IF(flag, tag, fmt, ...)         \
  do {                                             \
    if (flag) {                                    \
      logPrintf(tag, fmt, ##__VA_ARGS__);          \
    }                                              \
  } while (0)

#define LOG_PRINTLN_IF(flag, tag, msg)             \
  do {                                             \
    if (flag) {                                    \
      logPrintln(tag, msg);                        \
    }                                              \
  } while (0)
