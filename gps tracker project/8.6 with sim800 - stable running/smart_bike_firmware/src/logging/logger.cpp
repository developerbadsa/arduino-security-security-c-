#include "logger.h"

namespace {

portMUX_TYPE recentLogMux = portMUX_INITIALIZER_UNLOCKED;
RecentLogEntry recentLogs[RECENT_LOG_COUNT] = {};
size_t recentLogHead = 0;
size_t recentLogCount = 0;

void sanitizeLogText(const char* src, char* dst, size_t dstSize) {
  if (!dst || dstSize == 0) return;

  if (!src) {
    dst[0] = 0;
    return;
  }

  size_t j = 0;
  for (size_t i = 0; src[i] && j + 1 < dstSize; ++i) {
    char c = src[i];
    if (c == '\r' || c == '\n' || c == '\t') c = ' ';
    dst[j++] = c;
  }
  dst[j] = 0;
}

void pushRecentLogLine(const char* line) {
  RecentLogEntry entry = {};
  entry.atMs = millis();
  sanitizeLogText(line, entry.text, sizeof(entry.text));

  portENTER_CRITICAL(&recentLogMux);
  recentLogs[recentLogHead] = entry;
  recentLogHead = (recentLogHead + 1) % RECENT_LOG_COUNT;
  if (recentLogCount < RECENT_LOG_COUNT) recentLogCount++;
  portEXIT_CRITICAL(&recentLogMux);
}

void emitTaggedLine(const char* tag, const char* msg) {
  const char* safeTag = (tag && tag[0]) ? tag : "LOG";
  const char* safeMsg = msg ? msg : "";

  Serial.printf("[%s] %s\n", safeTag, safeMsg);

  char line[RECENT_LOG_TEXT_LEN];
  snprintf(line, sizeof(line), "[%s] %s", safeTag, safeMsg);
  pushRecentLogLine(line);
}

}  // namespace

void logPrintf(const char* tag, const char* fmt, ...) {
  char msg[RECENT_LOG_TEXT_LEN];
  va_list args;
  va_start(args, fmt);
  vsnprintf(msg, sizeof(msg), fmt, args);
  va_end(args);
  emitTaggedLine(tag, msg);
}

void logPrintln(const char* tag, const char* msg) {
  emitTaggedLine(tag, msg);
}

void logPrintln(const char* tag, const String& msg) {
  emitTaggedLine(tag, msg.c_str());
}

void logPrintln(const char* tag, const __FlashStringHelper* msg) {
  String text = msg;
  emitTaggedLine(tag, text.c_str());
}

size_t recentLogSnapshot(RecentLogEntry* out, size_t maxOut) {
  if (!out || maxOut == 0) return 0;

  portENTER_CRITICAL(&recentLogMux);

  const size_t total = recentLogCount;
  const size_t take = (total < maxOut) ? total : maxOut;
  const size_t oldest = (recentLogHead + RECENT_LOG_COUNT - total) % RECENT_LOG_COUNT;
  const size_t start = (oldest + (total - take)) % RECENT_LOG_COUNT;

  for (size_t i = 0; i < take; ++i) {
    out[i] = recentLogs[(start + i) % RECENT_LOG_COUNT];
  }

  portEXIT_CRITICAL(&recentLogMux);
  return take;
}
