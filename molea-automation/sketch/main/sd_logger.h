#pragma once
#include <Arduino.h>
#include "config.h"

#if USE_SD_LOGGER
// If you already define SD_CS_PIN in config.h, this won't override it.
#ifndef SD_CS_PIN
  #define SD_CS_PIN 10
#endif

bool sd_logger_init();
bool sd_logger_ready();
const char* sd_logger_filename();
bool sd_logger_logLine(const char* line);
bool sd_logger_logEvent(const char* msg);
void sd_logger_flush();
void sd_logger_close();
#else
inline bool sd_logger_init() { return false; }
inline bool sd_logger_ready() { return false; }
inline const char* sd_logger_filename() { return ""; }
inline bool sd_logger_logLine(const char*) { return false; }
inline bool sd_logger_logEvent(const char*) { return false; }
inline void sd_logger_flush() {}
inline void sd_logger_close() {}
#endif
