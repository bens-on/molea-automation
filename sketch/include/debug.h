#ifndef DEBUG_H
#define DEBUG_H

#include <Arduino.h>
#include <Arduino_RouterBridge.h>

// Debug macros for logging
#define DEBUG_INFO(tag, msg) \
  do { \
    Monitor.print(F("[")); \
    Monitor.print(F(tag)); \
    Monitor.print(F("] INFO: ")); \
    Monitor.println(F(msg)); \
  } while(0)

#define DEBUG_ERROR(tag, msg) \
  do { \
    Monitor.print(F("[")); \
    Monitor.print(F(tag)); \
    Monitor.print(F("] ERROR: ")); \
    Monitor.println(F(msg)); \
  } while(0)

// Print a formatted header
void printHeader(const char* title) {
  Monitor.println();
  Monitor.print(F("=== "));
  Monitor.print(title);
  Monitor.println(F(" ==="));
  Monitor.println();
}

// Print component status
void printComponentStatus(const char* name, bool status, const char* description) {
  Monitor.print(F("  "));
  Monitor.print(name);
  Monitor.print(F(": "));
  if (status) {
    Monitor.print(F("[OK] "));
  } else {
    Monitor.print(F("[FAIL] "));
  }
  Monitor.println(description);
}

// Print timestamp in readable format (hours:minutes:seconds from milliseconds)
void printTimestamp(unsigned long ms) {
  unsigned long seconds = ms / 1000;
  unsigned long minutes = seconds / 60;
  unsigned long hours = minutes / 60;
  
  seconds = seconds % 60;
  minutes = minutes % 60;
  
  if (hours < 10) Monitor.print(F("0"));
  Monitor.print(hours);
  Monitor.print(F(":"));
  if (minutes < 10) Monitor.print(F("0"));
  Monitor.print(minutes);
  Monitor.print(F(":"));
  if (seconds < 10) Monitor.print(F("0"));
  Monitor.print(seconds);
}

// Log a single command line to Monitor, Serial, and Bridge (for CMD,... lines)
void logCommandLine(const char* line);

#endif
