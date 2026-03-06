/*
 * MOLEA Payload System — Sequenced State Machine
 * Arduino Uno Q
 *
 * Logs to:
 *   - Arduino Lab Serial Monitor (Monitor.println)
 *   - Python app via Bridge.notify("log_line", "...")
 *   - Onboard SD memory (data folder directory)
 *
 * Features:
 *   - pH moving average filter
 *   - BMP280, RTC (20 minutes behind), IMU (accel, gyro), pH, flight state detection
 */

#include <Arduino.h>
#include <Arduino_RouterBridge.h>
#include <Wire.h>
#include <SPI.h>
#include <math.h>

#include "main/config.h"
#include "main/debug.h"
#include "main/rtc_init.h"
#include "main/BMP280_init.h"
#include "main/imu_init.h"
#include "main/motor_init.h"
#include "main/buzzer_init.h"
#include "main/pH_init.h"
#include "main/storage_init.h"
#include "main/sd_logger.h"
#include "main/test_components.h"

// ---------------------- Mission States ----------------------
enum MissionState {
  PREFLIGHT = 0,
  FLIGHT = 1,
  LANDING_CONFIRM = 2,
  DRILLING = 3,
  PH_TEST = 4,
  DISARMED = 5,
  DESCENT_BYPASS = 6
};

static MissionState currentState  = PREFLIGHT;
static MissionState lastState     = DISARMED;
static unsigned long stateEnterMs = 0;

static unsigned long missionStartTime = 0;
static unsigned long lastDataOutput   = 0;

static float lastAltitude = 0.0f;
static bool  inFlight     = false;

static unsigned long landingConfirmedMs = 0;
static unsigned long landingDeadlineMs  = 0;
static const unsigned long LANDING_WINDOW_MS = 15UL * 60UL * 1000UL;

// Mission timing (LANDING_CONFIRM and DRILLING)
FINAL static const unsigned long LANDING_CONFIRM_STABLE_MS = 8000;
FINAL static const unsigned long LANDING_CONFIRM_LEVELING_TIMEOUT_MS = 15000;  // max wait before starting M1 (fallback)
FINAL static const int           LANDING_CONFIRM_M1_SPEED  = 200;
FINAL static uint8_t             legTimeMin = LEG_TIME_MIN_DEFAULT;  // M1 run time for leg leveling (minutes); set via LEG_TIME command
FINAL static bool                landingConfirmSkipStable = false;  // set by SET_STATE LANDING_CONFIRM so M1 starts immediately
FINAL static const unsigned long Z_UP_HOLD_MS              = 1000;
FINAL static const float         Z_UP_ACCEL_Z_MIN          = 8.0f;
FINAL static const float         Z_UP_ACCEL_XY_MAX         = 3.0f;

static const uint8_t       DRILL_DURATION_MINUTES_DEFAULT = 8;
static uint8_t             drillDurationMinutes = DRILL_DURATION_MINUTES_DEFAULT;

static float lastPH      = 0.0f;
static bool  lastPHValid = false;
static bool  phInitOk    = false;

#define PH_AVG_WINDOW 5
static float phAvgBuf[PH_AVG_WINDOW];
static int   phAvgCount = 0;
static int   phAvgIdx   = 0;

// ---------------------- Bridge logging ----------------------
static void bridgeLog(const char* line) {
  if (!line) return;
  static char buf[400];
  size_t i = 0;
  for (; i < sizeof(buf) - 1 && line[i] != '\0'; i++) buf[i] = line[i];
  buf[i] = '\0';
  Bridge.notify("log_line", buf);
}

// Convenience: print to Monitor AND Bridge AND Serial
static void logAll(const char* line) {
  Monitor.println(line);
  Serial.println(line);
  bridgeLog(line);
}

static void logAll(const __FlashStringHelper* fsh) {
  Monitor.println(fsh);
  Serial.println(fsh);
  // Bridge needs a char*, so we copy from flash
  char tmp[120];
  strncpy_P(tmp, (const char*)fsh, sizeof(tmp) - 1);
  tmp[sizeof(tmp) - 1] = '\0';
  bridgeLog(tmp);
}

void logCommandLine(const char* line) {
  if (line) logAll(line);
}

// Log to Monitor and Serial only (no Bridge) — use in override path to avoid blocking on Bridge.notify
static void logMonitorSerialOnly(const __FlashStringHelper* fsh) {
  Monitor.println(fsh);
  Serial.println(fsh);
}

// #region agent log
// Debug: one NDJSON line to Serial (prefix DBG6524f0); copy lines to .cursor/debug-6524f0.log for analysis
static void debugLogLandingConfirm(const char* hypothesisId, uint8_t legSec, unsigned long legMs, unsigned long nowMs) {
  char buf[180];
  snprintf(buf, sizeof(buf),
    "DBG6524f0\t{\"hypothesisId\":\"%s\",\"message\":\"landing_confirm_m1_start\",\"legTimeMin\":%u,\"legTimeMs\":%lu,\"millis\":%lu,\"units\":\"minutes\"}",
    hypothesisId, (unsigned int)legSec, legMs, nowMs);
  Serial.println(buf);
  Monitor.println(buf);
}
// #endregion

// ---------------------- Helpers ----------------------
static void updateStateEntryTimer() {
  if (currentState != lastState) {
    lastState     = currentState;
    stateEnterMs  = millis();
  }
}

static const char* stateName(MissionState s) {
  switch (s) {
    case PREFLIGHT:       return "PREFLIGHT";
    case FLIGHT:          return "FLIGHT";
    case LANDING_CONFIRM: return "LANDING_CONFIRM";
    case DRILLING:        return "DRILLING";
    case PH_TEST:         return "PH_TEST";
    case DISARMED:        return "DISARMED";
    case DESCENT_BYPASS:  return "DESCENT_BYPASS";
  }
  return "UNKNOWN";
}

static bool soilCollected50mL() {
  return false; // TODO: wire up soil volume sensor
}

// Payload Z-up: IMU Z axis dominant and ~1g, or at-rest (calibrated Z-up gives small magnitude)
static bool imuZUp(const IMUData& imu) {
  if (!imu.dataValid) return false;
  float az = imu.accel_z;
  float ax = (float)fabs((double)imu.accel_x);
  float ay = (float)fabs((double)imu.accel_y);
  float mag = (float)sqrt((double)(imu.accel_x * imu.accel_x + imu.accel_y * imu.accel_y + imu.accel_z * imu.accel_z));
  // After calibration at rest Z-up, accel is near 0; accept that as Z-up (matches imuOrientationLabel)
  if (mag < 2.0f) return true;
  return (az > Z_UP_ACCEL_Z_MIN && ax < Z_UP_ACCEL_XY_MAX && ay < Z_UP_ACCEL_XY_MAX);
}

// Orientation: axis most aligned with gravity (largest |accel|) = that axis "up" or "down".
// After zero-g calibration, "Z up at rest" gives all axes near 0; treat small magnitude as Z_UP.
static const char* imuOrientationLabel(const IMUData& imu) {
  if (!imu.dataValid) return "IMU_INVALID";
  float ax = imu.accel_x, ay = imu.accel_y, az = imu.accel_z;
  float abx = (float)fabs((double)ax), aby = (float)fabs((double)ay), abz = (float)fabs((double)az);
  float mag = (float)sqrt((double)(ax*ax + ay*ay + az*az));
  static const float minG = 4.0f;       // dominant axis (m/s²) when gravity is visible
  static const float atRestMax = 2.0f;  // below this magnitude = at rest (calibrated Z-up pose)
  if (mag < atRestMax) return "Z_UP";
  if (abz >= abx && abz >= aby && abz >= minG) return (az > 0) ? "Z_UP" : "Z_DOWN";
  if (abx >= aby && abx >= abz && abx >= minG) return (ax > 0) ? "X_UP" : "X_DOWN";
  if (aby >= abx && aby >= abz && aby >= minG) return (ay > 0) ? "Y_UP" : "Y_DOWN";
  return "TILT";
}

// ---------------------- Command polling ----------------------
#define CMD_BUF_SIZE 64
static char    cmdBuf[CMD_BUF_SIZE];
static uint8_t cmdIdx = 0;

static void pollCommandsNonBlocking() {
  for (int n = 0; n < 64 && Monitor.available() > 0; n++) {
    int c = Monitor.read();
    if (c == '\n' || c == '\r') {
      cmdBuf[cmdIdx] = '\0';
      if (cmdIdx > 0) {
        String command = String(cmdBuf);
        command.trim();
        if (command.length() > 0 && command.length() < 50) {
          parseCommand(command);
        }
      }
      cmdIdx = 0;
    } else if (c >= 0 && cmdIdx < CMD_BUF_SIZE - 1) {
      cmdBuf[cmdIdx++] = (char)c;
    }
  }
}

// ---------------------- pH background updater ----------------------
static void updatePHBackground() {
  if (!phInitOk) return;

  const unsigned long PH_READ_INTERVAL_MS = 2000;
  static unsigned long lastPhReadMs = 0;
  if (millis() - lastPhReadMs < PH_READ_INTERVAL_MS) return;
  lastPhReadMs = millis();

  float phVal = 0.0f;
  bool ok = readpH(phVal, I2C_EZO_ADDRESS);
  if (ok) {
    phAvgBuf[phAvgIdx] = phVal;
    phAvgIdx = (phAvgIdx + 1) % PH_AVG_WINDOW;
    if (phAvgCount < PH_AVG_WINDOW) phAvgCount++;
    if (phAvgCount > 0) {
      float sum = 0.0f;
      for (int i = 0; i < phAvgCount; i++) sum += phAvgBuf[i];
      lastPH = sum / (float)phAvgCount;
    }
    lastPHValid = true;
  } else {
    lastPHValid = false;
  }
}

// ---------------------- State Handlers ----------------------
// Motor API for flight-phase code (same as serial commands). Use in handlePreflight, handleDrilling, etc.:
//   startMotor1Timed(speed, durationMs)  — e.g. startMotor1Timed(-400, 9500) = "CAL -400 9.5" (reverse M1 for 9.5 s)
//   setMotor1Speed(speed), setMotor2Speed(speed)  — -400..400; stop with 0 or stopMotor1()/stopMotor2()
//   moveM1DownMm(mm), moveM1UpMm(mm), gotoPositionMm(mm)  — soft-start position moves
//   motor_cancelMove(); stopMotor1(); stopMotor2()  — stop all; isMoveStateIdle() to check if move done
static void handlePreflight() {
  // Do not auto-stop motors on stall in PREFLIGHT so manual M1/M2 control always works.
  // Overcurrent still appears in the data line (M1mA, M2mA, fault); user can STOP if needed.

  BaroData baro;
  IMUData  imu;

  bool baroOk = readBMP280(baro) && baro.dataValid;
  bool imuOk  = readIMU(imu)    && imu.dataValid;
  if (!baroOk) return;

  float altitudeChange = baro.altitude - lastAltitude;
  lastAltitude = baro.altitude;

  float accelMag = 0.0f;
  if (imuOk) {
    accelMag = (float)sqrt((double)(
      imu.accel_x * imu.accel_x +
      imu.accel_y * imu.accel_y +
      imu.accel_z * imu.accel_z));
  }

  // --- Normal launch detection ---
  bool altitudeLaunch = altitudeChange > 5.0f;
  bool imuLaunch      = (!imuOk) || (accelMag > 11.77f);

  if (altitudeLaunch && imuLaunch && !inFlight) {
    logAll(F("LAUNCH DETECTED -> FLIGHT"));
    playStateSound((uint8_t)FLIGHT);
    currentState = FLIGHT;
    inFlight     = true;
    if (storage_initialized) storage_set_launch_time(millis());
    return;
  }

  // --- Descent bypass: fast drop detected without ever seeing a launch ---
  static float peakAltitude = -9999.0f;
  if (baro.altitude > peakAltitude) peakAltitude = baro.altitude;

  float dropFromPeak = peakAltitude - baro.altitude;

  if (dropFromPeak > 8.0f && !inFlight) {
    logAll(F("DESCENT BYPASS: large drop detected without launch -> skipping to LANDING_CONFIRM"));
    logAll(F("(launch & flight states bypassed — drop-test or missed launch)"));
    playStateSound((uint8_t)DESCENT_BYPASS);
    inFlight = true;
    if (storage_initialized) storage_set_launch_time(millis());
    currentState = DESCENT_BYPASS;
  }
}

static void handleDescentBypass() {
  if (motor_isStallDetected()) {
    logAll(F("STALL DETECTED (descent bypass) -> reset to PREFLIGHT, stop motors, continue monitoring launch/landing"));
    motor_cancelMove();
    stopMotor1();
    stopMotor2();
    inFlight = false;
    currentState = PREFLIGHT;
    return;
  }

  BaroData baro;
  IMUData  imu;

  bool baroOk = readBMP280(baro) && baro.dataValid;
  bool imuOk  = readIMU(imu)    && imu.dataValid;
  if (!baroOk) return;

  static float altitudeHistory[10] = {0};
  static int   historyIndex = 0;

  altitudeHistory[historyIndex] = baro.altitude;
  historyIndex = (historyIndex + 1) % 10;
  if (historyIndex != 0) return;

  float avgAltitude = 0.0f;
  float maxAltitude = altitudeHistory[0];
  float minAltitude = altitudeHistory[0];
  for (int i = 0; i < 10; i++) {
    avgAltitude += altitudeHistory[i];
    if (altitudeHistory[i] > maxAltitude) maxAltitude = altitudeHistory[i];
    if (altitudeHistory[i] < minAltitude) minAltitude = altitudeHistory[i];
  }
  avgAltitude /= 10.0f;

  bool altitudeStable = (avgAltitude < 50.0f) && ((maxAltitude - minAltitude) < 2.0f);

  float accelMag = 0.0f, gyroMag = 0.0f;
  if (imuOk) {
    accelMag = (float)sqrt((double)(
      imu.accel_x * imu.accel_x +
      imu.accel_y * imu.accel_y +
      imu.accel_z * imu.accel_z));
    gyroMag = (float)sqrt((double)(
      imu.gyro_x * imu.gyro_x +
      imu.gyro_y * imu.gyro_y +
      imu.gyro_z * imu.gyro_z));
  }

  bool imuStable = (!imuOk) || (((float)fabs((double)(accelMag - 9.81f)) < 3.0f) && (gyroMag < 0.5f));

  if (altitudeStable && imuStable) {
    logAll(F("DESCENT BYPASS: stable landing detected -> LANDING_CONFIRM"));
    playStateSound((uint8_t)LANDING_CONFIRM);
    currentState = LANDING_CONFIRM;
  }
}

static void handleFlight() {
  if (motor_isStallDetected()) {
    logAll(F("STALL DETECTED (flight) -> reset to PREFLIGHT, stop motors, continue monitoring launch/landing"));
    motor_cancelMove();
    stopMotor1();
    stopMotor2();
    inFlight = false;
    currentState = PREFLIGHT;
    return;
  }

  BaroData baro;
  IMUData  imu;

  bool baroOk = readBMP280(baro) && baro.dataValid;
  bool imuOk  = readIMU(imu)    && imu.dataValid;
  if (!baroOk) return;

  static float altitudeHistory[10] = {0};
  static int   historyIndex = 0;

  altitudeHistory[historyIndex] = baro.altitude;
  historyIndex = (historyIndex + 1) % 10;
  if (historyIndex != 0) return;

  float avgAltitude = 0.0f;
  float maxAltitude = altitudeHistory[0];
  float minAltitude = altitudeHistory[0];
  for (int i = 0; i < 10; i++) {
    avgAltitude += altitudeHistory[i];
    if (altitudeHistory[i] > maxAltitude) maxAltitude = altitudeHistory[i];
    if (altitudeHistory[i] < minAltitude) minAltitude = altitudeHistory[i];
  }
  avgAltitude /= 10.0f;

  bool altitudeStable = (avgAltitude < 50.0f) && ((maxAltitude - minAltitude) < 2.0f);

  float accelMag = 0.0f;
  float gyroMag  = 0.0f;
  if (imuOk) {
    accelMag = (float)sqrt((double)(
      imu.accel_x * imu.accel_x +
      imu.accel_y * imu.accel_y +
      imu.accel_z * imu.accel_z));
    gyroMag = (float)sqrt((double)(
      imu.gyro_x * imu.gyro_x +
      imu.gyro_y * imu.gyro_y +
      imu.gyro_z * imu.gyro_z));
  }

  bool imuStable = (!imuOk) || (((float)fabs((double)(accelMag - 9.81f)) < 3.0f) && (gyroMag < 0.5f));

  if (altitudeStable && imuStable) {
    logAll(F("LANDING DETECTED -> LANDING_CONFIRM"));
    playStateSound((uint8_t)LANDING_CONFIRM);
    currentState = LANDING_CONFIRM;
  }
}

enum LandingConfirmStep { LANDING_STABLE_WAIT, LANDING_M1_RUN, LANDING_Z_UP_WAIT };

static void handleLandingConfirm() {
  /* Run state-enter first so M1 always starts when entering LANDING_CONFIRM (skip-stable or after stable).
     Do not require baro/imu here — if we returned on !baroOk first, we could miss starting M1 on the only
     frame when stateEnterMs != lastLandingConfirmStateEnter. */
  static LandingConfirmStep landingConfirmStep = LANDING_STABLE_WAIT;
  static unsigned long stepEnterMs = 0;
  static unsigned long lastLandingConfirmStateEnter = 0;
  if (stateEnterMs != lastLandingConfirmStateEnter) {
    lastLandingConfirmStateEnter = stateEnterMs;
    stepEnterMs = millis();
    if (landingConfirmSkipStable) {
      landingConfirmSkipStable = false;
      unsigned long legTimeMs = (unsigned long)legTimeMin * 60UL * 1000UL;
      // #region agent log
      debugLogLandingConfirm("A", legTimeMin, legTimeMs, millis());
      // #endregion
      startMotor1Timed(LANDING_CONFIRM_M1_SPEED, legTimeMs);
      landingConfirmStep = LANDING_M1_RUN;
      { char buf[64]; snprintf(buf, sizeof(buf), "LANDING_CONFIRM: M1 leg leveling for %u min (LEG_TIME)", (unsigned int)legTimeMin); logAll(buf); }
    } else {
      landingConfirmStep = LANDING_STABLE_WAIT;
    }
  }

  BaroData baro;
  IMUData  imu;
  bool baroOk = readBMP280(baro) && baro.dataValid;
  bool imuOk  = readIMU(imu)    && imu.dataValid;
  if (!baroOk) return;

  if (landingConfirmStep == LANDING_STABLE_WAIT) {
    static float lastAlt = 0.0f;
    static unsigned long stableSinceMs = 0;
    float altDelta = (float)fabs((double)(baro.altitude - lastAlt));
    lastAlt = baro.altitude;

    bool altitudeStable = (altDelta < 0.5f);
    bool imuStable      = true;
    if (imuOk) {
      float accelMag = (float)sqrt((double)(
        imu.accel_x * imu.accel_x +
        imu.accel_y * imu.accel_y +
        imu.accel_z * imu.accel_z));
      float gyroMag = (float)sqrt((double)(
        imu.gyro_x * imu.gyro_x +
        imu.gyro_y * imu.gyro_y +
        imu.gyro_z * imu.gyro_z));
      // Low gyro = not rotating. Accept ~1g (7.3–12.3) OR near-zero (zero-g calibrated / at rest)
      imuStable = (gyroMag < 0.5f) && (
        ((float)fabs((double)(accelMag - 9.81f)) < 2.5f) ||
        (accelMag < 2.0f));
    }

    if (!(altitudeStable && imuStable)) {
      stableSinceMs = 0;
    } else {
      if (stableSinceMs == 0) stableSinceMs = millis();
      if (stableSinceMs != 0 && (millis() - stableSinceMs >= LANDING_CONFIRM_STABLE_MS)) {
        unsigned long legTimeMs = (unsigned long)legTimeMin * 60UL * 1000UL;
        startMotor1Timed(LANDING_CONFIRM_M1_SPEED, legTimeMs);
        landingConfirmStep = LANDING_M1_RUN;
        stepEnterMs = millis();
        { char buf[64]; snprintf(buf, sizeof(buf), "LANDING_CONFIRM: M1 leg leveling for %u min (LEG_TIME)", (unsigned int)legTimeMin); logAll(buf); }
      }
    }
    // Fallback: if stable condition never met, start leveling after timeout so maneuver always runs
    if (millis() - stateEnterMs >= LANDING_CONFIRM_LEVELING_TIMEOUT_MS) {
      unsigned long legTimeMs = (unsigned long)legTimeMin * 60UL * 1000UL;
      startMotor1Timed(LANDING_CONFIRM_M1_SPEED, legTimeMs);
      landingConfirmStep = LANDING_M1_RUN;
      stepEnterMs = millis();
      logAll(F("LANDING_CONFIRM: leveling timeout — starting M1 leg leveling (LEG_TIME)"));
    }
    return;
  }

  if (landingConfirmStep == LANDING_M1_RUN) {
    if (isMoveStateIdle()) {
      landingConfirmStep = LANDING_Z_UP_WAIT;
      stepEnterMs = millis();
    }
    return;
  }

  if (landingConfirmStep == LANDING_Z_UP_WAIT) {
    if (!imuZUp(imu)) {
      stepEnterMs = millis();
      return;
    }
    if (millis() - stepEnterMs >= Z_UP_HOLD_MS) {
      logAll(F("LANDING CONFIRMED (z-up) -> DRILLING"));
      landingConfirmedMs = millis();
      landingDeadlineMs  = landingConfirmedMs + LANDING_WINDOW_MS;
      if (storage_initialized) storage_set_landing_time(landingConfirmedMs);
      // No playStateSound(DRILLING) here; 3 loud slow beeps play in handleDrilling() before motors start
      currentState = DRILLING;
    }
  }
}

// Non-blocking 3 loud slow beeps before starting motors; then startSoilCollect() once. No continuous beep during drilling.
static const unsigned long DRILL_START_BEEP_ON_MS = 400;
static const unsigned long DRILL_START_BEEP_OFF_MS = 350;
static const int DRILL_START_BEEP_HZ = 1800;

static void handleDrilling() {
  if (landingDeadlineMs != 0 && millis() > landingDeadlineMs) {
    noTone(BUZZER_PIN);
    logAll(F("DRILLING: mission window ended -> DISARMED"));
    stopMotor1();
    stopMotor2();
    if (storage_initialized) storage_write_mission_summary();
    playStateSound((uint8_t)DISARMED);
    currentState = DISARMED;
    return;
  }

  const unsigned long DRILL_TIMEOUT_MS = (unsigned long)drillDurationMinutes * 60UL * 1000UL;

  // 3 loud slow beeps before starting motors (non-blocking); then startSoilCollect() once
  static unsigned long drillStateEnterLast = 0;
  static int drillBeepPhase = 0;       // 0..5: beep0 on, off, beep1 on, off, beep2 on, off
  static unsigned long drillBeepPhaseEndMs = 0;
  static bool drillMotorStarted = false;
  if (stateEnterMs != drillStateEnterLast) {
    drillStateEnterLast = stateEnterMs;
    drillBeepPhase = 0;
    drillMotorStarted = false;
    noTone(BUZZER_PIN);
    tone(BUZZER_PIN, DRILL_START_BEEP_HZ);
    drillBeepPhaseEndMs = millis() + DRILL_START_BEEP_ON_MS;
  }
  if (!drillMotorStarted && millis() >= drillBeepPhaseEndMs) {
    if (drillBeepPhase == 0) {
      noTone(BUZZER_PIN);
      drillBeepPhase = 1;
      drillBeepPhaseEndMs = millis() + DRILL_START_BEEP_OFF_MS;
    } else if (drillBeepPhase == 1) {
      tone(BUZZER_PIN, DRILL_START_BEEP_HZ);
      drillBeepPhase = 2;
      drillBeepPhaseEndMs = millis() + DRILL_START_BEEP_ON_MS;
    } else if (drillBeepPhase == 2) {
      noTone(BUZZER_PIN);
      drillBeepPhase = 3;
      drillBeepPhaseEndMs = millis() + DRILL_START_BEEP_OFF_MS;
    } else if (drillBeepPhase == 3) {
      tone(BUZZER_PIN, DRILL_START_BEEP_HZ);
      drillBeepPhase = 4;
      drillBeepPhaseEndMs = millis() + DRILL_START_BEEP_ON_MS;
    } else if (drillBeepPhase == 4) {
      noTone(BUZZER_PIN);
      drillBeepPhase = 5;
      drillBeepPhaseEndMs = millis() + DRILL_START_BEEP_OFF_MS;
    } else {
      drillMotorStarted = true;
      startSoilCollect();
      logAll(F("DRILLING: startSoilCollect()"));
    }
  }

  // pH recording during drilling
  static float lastDrillLoggedPH = -999.0f;
  if (lastPHValid && lastPH != lastDrillLoggedPH) {
    lastDrillLoggedPH = lastPH;
    char line[64];
    if (lastPH < 0.0f || lastPH > 14.0f) {
      snprintf(line, sizeof(line), "pH: %.2f | Valid = 1 | OUT_OF_RANGE (drill)", lastPH);
    } else {
      snprintf(line, sizeof(line), "pH: %.2f | Valid = 1 (drill)", lastPH);
    }
    logAll(line);
    if (storage_initialized) storage_record_ph_sample(lastPH);
  }

  if (soilCollected50mL()) {
    noTone(BUZZER_PIN);
    logAll(F("DRILLING: 50mL collected -> DISARMED"));
    stopMotor1();
    stopMotor2();
    if (storage_initialized) storage_write_mission_summary();
    playStateSound((uint8_t)DISARMED);
    currentState = DISARMED;
    return;
  }

  // SOIL_COLLECT finished (move state back to IDLE) -> disarm
  if (isMoveStateIdle()) {
    noTone(BUZZER_PIN);
    logAll(F("DRILLING: SOIL_COLLECT done -> DISARMED"));
    stopMotor1();
    stopMotor2();
    if (storage_initialized) storage_write_mission_summary();
    playStateSound((uint8_t)DISARMED);
    currentState = DISARMED;
    return;
  }

  if (millis() - stateEnterMs >= DRILL_TIMEOUT_MS) {
    noTone(BUZZER_PIN);
    logAll(F("DRILLING: timeout -> DISARMED"));
    stopMotor1();
    stopMotor2();
    if (storage_initialized) storage_write_mission_summary();
    playStateSound((uint8_t)DISARMED);
    currentState = DISARMED;
    return;
  }
}

static void handlePHTest() {
  if (landingDeadlineMs != 0 && millis() > landingDeadlineMs) {
    logAll(F("PH_TEST: mission window ended -> DISARMED"));
    if (storage_initialized) storage_write_mission_summary();
    playStateSound((uint8_t)DISARMED);
    currentState = DISARMED;
    return;
  }

  if (millis() - stateEnterMs < 60) {
    logAll(F("PH_TEST: starting pH acquisition (I2C SDA/SCL)"));
  }

  static float lastLoggedPH = -999.0f;
  if (lastPHValid && lastPH != lastLoggedPH) {
    lastLoggedPH = lastPH;
    char line[64];
    if (lastPH < 0.0f || lastPH > 14.0f) {
      snprintf(line, sizeof(line), "pH: %.2f | Valid = 1 | OUT_OF_RANGE", lastPH);
    } else {
      snprintf(line, sizeof(line), "pH: %.2f | Valid = 1", lastPH);
    }
    logAll(line);
    if (storage_initialized) storage_record_ph_sample(lastPH);
  } else if (!lastPHValid) {
    if (millis() - stateEnterMs < 60) logAll(F("pH: --- | Valid = 0"));
  }
}

// ---------------------- Output Sensor Data ----------------------
static void outputSensorData() {
  unsigned long timestamp = millis();

  DateTime rtc;
  bool rtcValid  = readRTC(rtc)    && rtc.dataValid;

  BaroData baro;
  bool baroValid = readBMP280(baro) && baro.dataValid;

  IMUData imu;
  bool imuValid  = readIMU(imu)    && imu.dataValid;

  char buf[360];
  int  n = 0;

  n += snprintf(buf + n, sizeof(buf) - n, "%lu,", timestamp);

  if (rtcValid) {
    n += snprintf(buf + n, sizeof(buf) - n,
      "%04d-%02d-%02d %02d:%02d:%02d,",
      rtc.year, rtc.month, rtc.day,
      rtc.hour, rtc.minute, rtc.second);
  } else {
    n += snprintf(buf + n, sizeof(buf) - n, "RTC_INVALID,");
  }

  if (baroValid) {
    n += snprintf(buf + n, sizeof(buf) - n,
      "T=%.2f A=%.2f,", baro.temperature, baro.altitude);
  } else {
    n += snprintf(buf + n, sizeof(buf) - n, "BMP_INVALID,");
  }

  if (imuValid) {
    n += snprintf(buf + n, sizeof(buf) - n,
      "AX=%.3f AY=%.3f AZ=%.3f GX=%.4f GY=%.4f GZ=%.4f %s,",
      imu.accel_x, imu.accel_y, imu.accel_z,
      imu.gyro_x, imu.gyro_y, imu.gyro_z,
      imuOrientationLabel(imu));
  } else {
    n += snprintf(buf + n, sizeof(buf) - n, "IMU_INVALID,");
  }

  if (!phInitOk) {
    n += snprintf(buf + n, sizeof(buf) - n, "PH_INIT_FAIL,");
  } else if (lastPHValid) {
    if (lastPH < 0.0f || lastPH > 14.0f) {
      n += snprintf(buf + n, sizeof(buf) - n, "PH=%.2f[OUT_OF_RANGE],", lastPH);
    } else {
      n += snprintf(buf + n, sizeof(buf) - n, "PH=%.2f,", lastPH);
    }
  } else {
    n += snprintf(buf + n, sizeof(buf) - n, "PH_INVALID,");
  }

  n += snprintf(buf + n, sizeof(buf) - n, "%s,", stateName(currentState));

  /* Motor data for monitor and file: M1_speed, M2_speed, M1_pos_mm, M1_mA, M2_mA, motor_fault (current sense fault detection) */
  int m1Spd = motor_getM1Speed();
  int m2Spd = motor_getM2Speed();
  float m1Pos = motor_getPositionMm();
  unsigned int m1Ma = motor_getM1CurrentMa();
  unsigned int m2Ma = motor_getM2CurrentMa();
  char faultBuf[16];
  motor_getFaultStatus(faultBuf, sizeof(faultBuf));
  n += snprintf(buf + n, sizeof(buf) - n, "M1=%d M2=%d pos=%.1f M1mA=%u M2mA=%u %s",
    m1Spd, m2Spd, m1Pos, m1Ma, m2Ma, faultBuf);

  Monitor.println(buf);
  Serial.println(buf);
  Serial.flush();
  bridgeLog(buf);
  if (sd_logger_ready()) sd_logger_logLine(buf);
}

// ---------------------- State override (serial command) ----------------------
// Uses Monitor+Serial only (no Bridge) so override path does not block on Bridge. Plays state sound (short pattern).
void setMissionStateOverride(uint8_t stateId) {
  if (stateId > (uint8_t)DESCENT_BYPASS) return;

  noTone(BUZZER_PIN);  // stop any current tone (e.g. drilling) before state change and new sound

  MissionState s = (MissionState)stateId;

  switch (s) {
    case PREFLIGHT:
      inFlight = false;
      currentState = PREFLIGHT;
      logMonitorSerialOnly(F("OVERRIDE: -> PREFLIGHT"));
      break;
    case FLIGHT:
      inFlight = true;
      if (storage_initialized) storage_set_launch_time(millis());
      currentState = FLIGHT;
      logMonitorSerialOnly(F("OVERRIDE: -> FLIGHT"));
      break;
    case DESCENT_BYPASS:
      inFlight = true;
      if (storage_initialized) storage_set_launch_time(millis());
      currentState = DESCENT_BYPASS;
      logMonitorSerialOnly(F("OVERRIDE: -> DESCENT_BYPASS"));
      break;
    case LANDING_CONFIRM:
      landingConfirmSkipStable = true;
      currentState = LANDING_CONFIRM;
      logMonitorSerialOnly(F("OVERRIDE: -> LANDING_CONFIRM (M1 will start immediately)"));
      break;
    case DRILLING:
      landingConfirmedMs = millis();
      landingDeadlineMs  = landingConfirmedMs + LANDING_WINDOW_MS;
      if (storage_initialized) storage_set_landing_time(landingConfirmedMs);
      currentState = DRILLING;
      logMonitorSerialOnly(F("OVERRIDE: -> DRILLING"));
      break;
    case PH_TEST:
      currentState = PH_TEST;
      logMonitorSerialOnly(F("OVERRIDE: -> PH_TEST"));
      break;
    case DISARMED:
      motor_cancelMove();
      stopMotor1();
      stopMotor2();
      if (storage_initialized) storage_write_mission_summary();
      currentState = DISARMED;
      logMonitorSerialOnly(F("OVERRIDE: -> DISARMED"));
      break;
  }
  lastState     = currentState;
  stateEnterMs  = millis();
  if ((MissionState)s != DRILLING) playStateSound((uint8_t)s);  // DRILLING: 3 beeps in handleDrilling() before motors
}

void setDrillDurationMinutes(uint8_t minutes) {
  drillDurationMinutes = (uint8_t)constrain((int)minutes, 1, 60);
}

void setLegTimeSec(uint8_t minutes) {
  legTimeMin = (uint8_t)constrain((int)minutes, 1, 120);
  char buf[48];
  snprintf(buf, sizeof(buf), "Leg time (LEG_TIME) set to %u min — used for M1 in LANDING_CONFIRM", (unsigned int)legTimeMin);
  logAll(buf);
}

bool isLandingConfirmLevelingInProgress(void) {
  return (currentState == LANDING_CONFIRM && !isMoveStateIdle());
}

// ---------------------- Setup / Loop ----------------------
void setup() {
  Monitor.begin(9600);
  delay(500);
  Serial.begin(SERIAL_BAUD_RATE);
  delay(500);

  logAll("*** BOOT ***");  // canary — if you don't see this, port/Monitor is broken

  Bridge.begin();
  delay(500);

  Wire.begin();
  Wire.setClock(100000); // EZO pH at 100k
  SPI.begin();

  logAll(F("=== molea-automation-full (MOLEA Payload) ==="));
  logAll(F("Initializing components..."));

  logAll(F("  -> IMU..."));
  bool imuOk = initIMU();

  logAll(F("  -> BMP280..."));
  bool bmpOk = initBMP280();

  logAll(F("  -> RTC..."));
  bool rtcOk = initRTC();

  logAll(F("  -> pH sensor..."));
  phInitOk = false;
  delay(800);  // EZO needs time after power-up before it answers I2C ping
  for (int tries = 0; tries < 3 && !phInitOk; tries++) {
    phInitOk = initpH(I2C_EZO_ADDRESS);
    if (!phInitOk) delay(400);
  }

  logAll(F("  -> Motor..."));
  bool motorOk = initMotor();

  logAll(F("  -> Buzzer..."));
  bool buzzerOk = initBuzzer();

  logAll(F("  -> Storage..."));
  bool storeOk = storage_init();
  bool sdOk = sd_logger_init();
  if (sdOk) logAll(sd_logger_filename());

  Monitor.println();
  logAll(F("=== Initialization Status ==="));
  printComponentStatus("IMU",       imuOk,    imuOk    ? "ICM-20948"            : "Failed");
  printComponentStatus("BMP280",    bmpOk,    bmpOk    ? "Barometric sensor"    : "Failed");
  printComponentStatus("RTC",       rtcOk,    rtcOk    ? "PCF8523"              : "Failed");
  printComponentStatus("pH Sensor",  phInitOk, phInitOk ? "EZO pH Carrier (I2C)" : "Failed");
  printComponentStatus("Motor",     motorOk,  motorOk  ? "Dual motor driver"    : "Failed");
  printComponentStatus("Buzzer",    buzzerOk, buzzerOk ? "Pin D6"               : "Failed");
  printComponentStatus("Storage",   storeOk,  storeOk  ? "Monitor output"       : "Failed");
  printComponentStatus("SD log",    sdOk,    sdOk     ? "CSV to file"           : "Disabled or no card");
  { // Log same init status to Serial/bridge so it appears in stored data file
    char line[64];
    snprintf(line, sizeof(line), "  IMU: %s",      imuOk    ? "[OK] ICM-20948"            : "[FAIL] Failed"); logAll(line);
    snprintf(line, sizeof(line), "  BMP280: %s",   bmpOk    ? "[OK] Barometric sensor"    : "[FAIL] Failed"); logAll(line);
    snprintf(line, sizeof(line), "  RTC: %s",      rtcOk    ? "[OK] PCF8523"              : "[FAIL] Failed"); logAll(line);
    snprintf(line, sizeof(line), "  pH Sensor: %s", phInitOk ? "[OK] EZO pH Carrier (I2C)" : "[FAIL] Failed"); logAll(line);
    snprintf(line, sizeof(line), "  Motor: %s",    motorOk  ? "[OK] Dual motor driver"    : "[FAIL] Failed"); logAll(line);
    snprintf(line, sizeof(line), "  Buzzer: %s",   buzzerOk ? "[OK] Pin D6"               : "[FAIL] Failed"); logAll(line);
    snprintf(line, sizeof(line), "  Storage: %s",  storeOk  ? "[OK] Monitor output"       : "[FAIL] Failed"); logAll(line);
  }
  Monitor.println();

  bool allCriticalOk = rtcOk && bmpOk && imuOk && motorOk && buzzerOk;
  if (!allCriticalOk || !phInitOk) playArmedWithWarningSound();
  else playArmedSound();

  currentState     = PREFLIGHT;
  missionStartTime = millis();
  if (storage_initialized) storage_set_mission_start(missionStartTime);

  if (bmpOk) calibrateBMP280AtGround(5);
  if (imuOk) calibrateIMU(100);

  logAll(F("=== System Ready ==="));
  logAll(F("Type 'HELP' for available test commands"));
  Monitor.println();
}

// Non-blocking double beep when pH init failed. Runs with the loop so no blocking.
static void updatePHFailBeep() {
  if (phInitOk) {
    noTone(BUZZER_PIN);
    return;
  }
  static unsigned long phFailBeepMs = 0;
  static uint8_t phFailPhase = 0;
  const unsigned long phaseDurMs[4] = { 50, 40, 50, 40 };  // on, off, on, off
  unsigned long now = millis();
  if (now - phFailBeepMs >= phaseDurMs[phFailPhase]) {
    phFailBeepMs = now;
    if (phFailPhase == 0 || phFailPhase == 2)
      tone(BUZZER_PIN, 1000);
    else
      noTone(BUZZER_PIN);
    phFailPhase = (phFailPhase + 1) % 4;
  }
}

void loop() {
  updateStateEntryTimer();
  pollCommandsNonBlocking();
  motor_poll();
  updatePHBackground();
  updatePHFailBeep();

  switch (currentState) {
    case PREFLIGHT:       handlePreflight();      break;
    case FLIGHT:          handleFlight();         break;
    case LANDING_CONFIRM: handleLandingConfirm(); break;
    case DRILLING:        handleDrilling();       break;
    case PH_TEST:         handlePHTest();         break;
    case DISARMED:
      motor_cancelMove();
      stopMotor1();
      stopMotor2();
      break;
    case DESCENT_BYPASS:  handleDescentBypass();   break;
  }

  unsigned long now = millis();
  if (now - lastDataOutput >= DATA_RATE_MS) {
    outputSensorData();
    lastDataOutput = now;
  }

  delay(10);
}

// Workaround includes (App Lab build requirement)
#include "main/rtc_init.cpp"
#include "main/BMP280_init.cpp"
#include "main/imu_init.cpp"
#include "main/motor_init.cpp"
#include "main/buzzer_init.cpp"
#include "main/pH_init.cpp"
#include "main/storage_init.cpp"
#include "main/sd_logger.cpp"
#include "main/test_components.cpp"
