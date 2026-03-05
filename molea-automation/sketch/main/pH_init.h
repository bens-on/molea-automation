#ifndef PH_INIT_H
#define PH_INIT_H

#include <Arduino.h>

struct PHData {
  float   value;    // parsed pH value (if valid)
  bool    valid;    // true if we got a valid numeric reading
  uint8_t code;     // Atlas response code (1=success, 2=fail, 254=pending, 255=no data)
};

// Atlas EZO pH default 7-bit I2C address (0x63 = 99 decimal).
// Cannot be changed on the EZO Carrier Board per schematic note.
bool initPH(uint8_t addr = 0x63);
bool sendPHCommand(const String& cmd, String& response, uint8_t& code, uint8_t addr = 0x63);
bool readPH(PHData& out, uint8_t addr = 0x63);

// Global init flag — used by test_components.cpp and sketch.ino (phInitOk).
extern bool ph_initialized;

// ---- sketch.ino-compatible wrappers (lowercase names) ----
inline bool initpH(uint8_t addr = 0x63) {
  ph_initialized = initPH(addr);
  return ph_initialized;
}

inline bool readpH(float& val, uint8_t addr = 0x63) {
  PHData d;
  bool ok = readPH(d, addr);
  val = d.valid ? d.value : NAN;
  return d.valid;
}

#endif // PH_INIT_H