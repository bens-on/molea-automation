#include "pH_init.h"

#include <Wire.h>
#include <Arduino_RouterBridge.h>

// Atlas EZO I2C response codes:
// 1   = success
// 2   = syntax error / command failed
// 254 = still processing
// 255 = no data to send

// Definition of the global declared in pH_init.h
bool ph_initialized = false;

// Returns Wire.endTransmission() value: 0=OK, 1=data too long, 2=NACK addr, 3=NACK data, 4=other
// On Zephyr/unoq, 1 often means no device at address — check I2C wiring and that EZO is on same bus as RTC/BMP280.
static uint8_t pingI2C(uint8_t addr) {
  Wire.beginTransmission(addr);
  return (uint8_t)Wire.endTransmission();
}

bool initPH(uint8_t addr) {
  // Wire.begin() is called in sketch.ino; calling again is safe.
  Wire.begin();

  if (pingI2C(addr) != 0) {
    Monitor.print(F("pH EZO not found at 0x"));
    Monitor.println(addr, HEX);
    return false;
  }

  Monitor.print(F("pH EZO found at 0x"));
  Monitor.println(addr, HEX);
  return true;
}

bool sendPHCommand(const String& cmd, String& response, uint8_t& code, uint8_t addr) {
  response = "";
  code = 255;

  // --- write command ---
  Wire.beginTransmission(addr);
  for (size_t i = 0; i < cmd.length(); i++) {
    Wire.write((uint8_t)cmd[i]);
  }
  Wire.endTransmission();

  // --- wait based on command type ---
  // "R" (read) takes ~900 ms typical on the Atlas EZO.
  if (cmd.length() > 0 && (cmd[0] == 'R' || cmd[0] == 'r')) {
    delay(1000);
  } else {
    delay(400);
  }

  // --- read response, retry once on code 254 (still processing) ---
  // Atlas EZO response: 1 code byte + up to 30 ASCII chars + null terminator = 32 bytes max.
  for (int attempt = 0; attempt < 2; attempt++) {
    Wire.requestFrom((uint8_t)addr, (uint8_t)32);

    if (Wire.available() < 1) {
      code = 255;
      if (attempt == 0) { delay(300); continue; }
      return false;
    }

    code = (uint8_t)Wire.read(); // first byte is always the response code
    response = "";
    while (Wire.available()) {
      char c = (char)Wire.read();
      if (c == 0) break;         // Atlas null-terminates the payload
      response += c;
    }
    response.trim();

    if (code == 1)   return true;              // success
    if (code == 254) { delay(300); continue; } // still processing — retry once
    return false;                              // code 2 = error, 255 = no data
  }
  return false;
}

static bool looksNumeric(const String& s) {
  if (s.length() == 0) return false;
  char c0 = s[0];
  return (isDigit(c0) || c0 == '-' || c0 == '+');
}

bool readPH(PHData& out, uint8_t addr) {
  out.value = NAN;
  out.valid = false;
  out.code  = 255;

  String  resp;
  uint8_t code;
  bool ok = sendPHCommand("R", resp, code, addr);

  out.code = code;

  if (!ok) {
    out.valid = false;
    return false;
  }

  // Typical response looks like "7.12"
  if (looksNumeric(resp)) {
    out.value = resp.toFloat();
    // Sanity range: pH 0–14, allow slightly beyond for debugging
    if (out.value > -1.0f && out.value < 15.0f) {
      out.valid = true;
      return true;
    }
  }

  out.valid = false;
  return true; // command succeeded but value couldn't be parsed
}