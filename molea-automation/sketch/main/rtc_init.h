#ifndef RTC_INIT_H
#define RTC_INIT_H

#include <Arduino.h>

struct DateTime {
  uint16_t year;
  uint8_t month;
  uint8_t day;
  uint8_t hour;
  uint8_t minute;
  uint8_t second;
  bool dataValid;
};

extern bool rtc_initialized;

bool initRTC();
bool readRTC(DateTime &dt);
bool verifyRTCTime();
bool setRTC(int year, int month, int day, int hour, int minute, int second);

#endif
