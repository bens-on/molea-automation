#include "rtc_init.h"
#include "config.h"
#include <Arduino_RouterBridge.h>
#include <Wire.h>

#define PCF8523_ADDRESS 0x68
#define PCF8523_CONTROL_1 0x00
#define PCF8523_SECONDS 0x03
#define PCF8523_MINUTES 0x04
#define PCF8523_HOURS 0x05
#define PCF8523_DAYS 0x06
#define PCF8523_MONTHS 0x08
#define PCF8523_YEARS 0x09

bool rtc_initialized = false;

static uint8_t bcd2dec(uint8_t val) {
  return val - 6 * (val >> 4);
}

static uint8_t dec2bcd(uint8_t val) {
  return (uint8_t)((val / 10 * 16) + (val % 10));
}

static uint8_t readRegister(uint8_t reg) {
  Wire.beginTransmission(PCF8523_ADDRESS);
  Wire.write(reg);
  Wire.endTransmission();
  Wire.requestFrom(PCF8523_ADDRESS, 1);
  return Wire.read();
}

static void writeRegister(uint8_t reg, uint8_t val) {
  Wire.beginTransmission(PCF8523_ADDRESS);
  Wire.write(reg);
  Wire.write(val);
  Wire.endTransmission();
}

bool initRTC() {
  Wire.begin();
  Wire.beginTransmission(PCF8523_ADDRESS);
  if (Wire.endTransmission() != 0) {
    Monitor.println(F("Warning: RTC initialization failed"));
    rtc_initialized = false;
    return false;
  }
  
  uint8_t control1 = readRegister(PCF8523_CONTROL_1);
  if (control1 & 0x20) {
    writeRegister(PCF8523_CONTROL_1, control1 & ~0x20);
    delay(10);
  }
  
  Monitor.println(F("RTC initialized"));
  rtc_initialized = true;
  return true;
}

bool readRTC(DateTime &dt) {
  if (!rtc_initialized) {
    dt.dataValid = false;
    return false;
  }
  
  Wire.beginTransmission(PCF8523_ADDRESS);
  Wire.write(PCF8523_SECONDS);
  if (Wire.endTransmission() != 0) {
    dt.dataValid = false;
    return false;
  }
  
  Wire.requestFrom(PCF8523_ADDRESS, 7);
  if (Wire.available() < 7) {
    dt.dataValid = false;
    return false;
  }
  
  uint8_t seconds = Wire.read();
  uint8_t minutes = Wire.read();
  uint8_t hours = Wire.read();
  uint8_t days = Wire.read();
  Wire.read();
  uint8_t months = Wire.read();
  uint8_t years = Wire.read();
  
  dt.second = bcd2dec(seconds & 0x7F);
  dt.minute = bcd2dec(minutes & 0x7F);
  dt.hour = bcd2dec(hours & 0x3F);
  dt.day = bcd2dec(days & 0x3F);
  dt.month = bcd2dec(months & 0x1F);
  dt.year = 2000 + bcd2dec(years);
  
  if (dt.second > 59 || dt.minute > 59 || dt.hour > 23 ||
      dt.day < 1 || dt.day > 31 || dt.month < 1 || dt.month > 12) {
    dt.dataValid = false;
    return false;
  }
  
  dt.dataValid = true;
  return true;
}

bool verifyRTCTime() {
  if (!rtc_initialized) return false;
  
  DateTime dt;
  if (readRTC(dt) && dt.dataValid) {
    Monitor.print(F("RTC Time: "));
    Monitor.print(dt.year);
    Monitor.print(F("-"));
    if (dt.month < 10) Monitor.print(F("0"));
    Monitor.print(dt.month);
    Monitor.print(F("-"));
    if (dt.day < 10) Monitor.print(F("0"));
    Monitor.print(dt.day);
    Monitor.print(F(" "));
    if (dt.hour < 10) Monitor.print(F("0"));
    Monitor.print(dt.hour);
    Monitor.print(F(":"));
    if (dt.minute < 10) Monitor.print(F("0"));
    Monitor.print(dt.minute);
    Monitor.print(F(":"));
    if (dt.second < 10) Monitor.print(F("0"));
    Monitor.println(dt.second);
    return true;
  }
  return false;
}

bool setRTC(int year, int month, int day, int hour, int minute, int second) {
  if (year < 2000) year = 2000;
  if (year > 2099) year = 2099;
  if (month < 1) month = 1;
  if (month > 12) month = 12;
  if (day < 1) day = 1;
  if (day > 31) day = 31;
  if (hour < 0) hour = 0;
  if (hour > 23) hour = 23;
  if (minute < 0) minute = 0;
  if (minute > 59) minute = 59;
  if (second < 0) second = 0;
  if (second > 59) second = 59;

  uint8_t yy = (uint8_t)(year - 2000);

  Wire.beginTransmission(PCF8523_ADDRESS);
  Wire.write(PCF8523_SECONDS);
  Wire.write(dec2bcd((uint8_t)second) & 0x7F);
  Wire.write(dec2bcd((uint8_t)minute) & 0x7F);
  Wire.write(dec2bcd((uint8_t)hour) & 0x3F);
  Wire.write(dec2bcd((uint8_t)day) & 0x3F);
  Wire.write((uint8_t)0x00);
  Wire.write(dec2bcd((uint8_t)month) & 0x1F);
  Wire.write(dec2bcd(yy));

  if (Wire.endTransmission() != 0) {
    Monitor.println(F("RTC set failed (I2C write error)"));
    return false;
  }

  rtc_initialized = true;
  Monitor.println(F("RTC set OK"));
  return true;
}
