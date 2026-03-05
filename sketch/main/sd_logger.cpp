#include "sd_logger.h"

#if USE_SD_LOGGER
#include <SPI.h>
#include <SD.h>

static File gFile;
static bool gReady = false;
static char gName[32] = {0};
static unsigned long gLastFlushMs = 0;

// Make a safe filename like LOG00001.CSV
static void makeFilename() {
  // If you want RTC-based filenames later, we can upgrade this.
  for (int i = 1; i < 10000; i++) {
    snprintf(gName, sizeof(gName), "LOG%05d.CSV", i);
    if (!SD.exists(gName)) return;
  }
  // fallback if somehow everything exists
  snprintf(gName, sizeof(gName), "LOG99999.CSV");
}

bool sd_logger_init() {
  gReady = false;

  // IMPORTANT: SD.begin uses SPI; you already call SPI.begin() in your sketch.
  if (!SD.begin(SD_CS_PIN)) {
    return false;
  }

  makeFilename();

  gFile = SD.open(gName, FILE_WRITE);
  if (!gFile) {
    return false;
  }

  // Header: millis,rtc,bmp280,imu,ph,state,motors(M1_speed,M2_speed,pos_mm,M1_mA,M2_mA,fault)
  gFile.println("millis,rtc,bmp280,imu,ph,state,M1,M2,pos_mm,M1_mA,M2_mA,motor_fault");
  gFile.flush();
  gLastFlushMs = millis();

  gReady = true;
  return true;
}

bool sd_logger_ready() {
  return gReady;
}

const char* sd_logger_filename() {
  return gName;
}

bool sd_logger_logLine(const char* line) {
  if (!gReady || !gFile) return false;

  gFile.println(line);

  // Periodic flush so you don't lose the whole flight if power dies.
  // (You can tune this. 1000 ms is safe.)
  unsigned long now = millis();
  if (now - gLastFlushMs >= 1000UL) {
    gFile.flush();
    gLastFlushMs = now;
  }

  return true;
}

bool sd_logger_logEvent(const char* msg) {
  if (!gReady || !gFile) return false;

  // Write an "EVENT" line that won't break your CSV parser too badly.
  // Example: EVENT,12345,LAUNCH DETECTED
  gFile.print("EVENT,");
  gFile.print(millis());
  gFile.print(",");
  gFile.println(msg);

  // Flush events immediately (events are important)
  gFile.flush();
  gLastFlushMs = millis();

  return true;
}

void sd_logger_flush() {
  if (gReady && gFile) {
    gFile.flush();
    gLastFlushMs = millis();
  }
}

void sd_logger_close() {
  if (gFile) {
    gFile.flush();
    gFile.close();
  }
  gReady = false;
}
#endif /* USE_SD_LOGGER */
