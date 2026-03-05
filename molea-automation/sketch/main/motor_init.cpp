/*
 * Motor driver: transplanted from MotorControl/sketch.ino.
 * Same setMotorRaw/stopMotor logic, INA/INB/PWM from config.h (2,4,9 = M1; 7,8,10 = M2).
 * EN pins: when MOTOR_EN_PINS_CONNECTED==0 (default), M1EN/M2EN are not driven; leave them
 * unplugged on the shield so drivers stay enabled. Set MOTOR_EN_PINS_CONNECTED 1 to drive EN.
 *
 * --- Current sensing and fault detection ---
 * Pololu VNH5019 exposes current-sense (CS) pins: M1CS = A0, M2CS = A1. The CS output is
 * ~0.14 V/A. With 5 V ADC reference, current (mA) ≈ analogRead(CS) * 34.
 * - High current while the motor is commanded can indicate: stall, mechanical overload, or
 *   short circuit. Sustained overcurrent can damage the driver or motor.
 * - Fault status is derived from MOTOR_OVERCURRENT_MA (config.h): if M1 or M2 current
 *   exceeds this threshold, motor_getFaultStatus() returns M1_OVER, M2_OVER, or M1_M2_OVER.
 * - Every data line (monitor and SD file) includes: M1 speed, M2 speed, position (mm),
 *   M1_mA, M2_mA, and motor_fault (OK or overcurrent code). Use logged current and fault
 *   for post-flight analysis and to detect stalls/overloads.
 */
#include "motor_init.h"
#include "config.h"
#include <Arduino_RouterBridge.h>
#include <string.h>

bool motor_initialized = false;

// --- Constants (MotorControl parity) ---
// Position convention: 0 = BOTTOM (fully extended into ground), 116.84 = ORIGIN (retracted at top); 4.6 in travel, 7 s full stroke
static const float TRAVEL_MM = 116.84f;
static const float M1_POSITION_MIN = 0.0f;   // bottom (extended)
static const float M1_POSITION_MAX = 116.84f;  // origin (retracted)
static const unsigned long TEST_EXTEND_MS = 1100;
static const unsigned long GOTO_LIMIT_EXTRA_MS = 500;
static const unsigned long SOIL_M2_DOWN_MS = 3000;
static const unsigned long SOIL_M2_HALF_MS = 1500;
static const float COLLECT_FULL_DOWN_SEC = 7.0f;
static const float COLLECT_LINGER_AT_BOTTOM_SEC = 1.0f;
static const float COLLECT_HALF_UP_SEC = 4.5f;
static const float COLLECT_LINGER_SEC = 1.0f;
static const float COLLECT_HALF_DOWN_SEC = 4.5f;

// SOIL_COLLECT: step-based sequence. M2 -400, then cal 400/5, -400/5, wait 1s, cal 400/6, -400/6, wait 1s, then 10x (cal 400/7, -400/7, wait 1s), then M2 stop.
static const unsigned long SOIL_COLLECT_WAIT_MS = 1000;
static const int SOIL_COLLECT_M2_SPEED = -400;
static const int SOIL_COLLECT_STEP_COUNT = 38;  // steps 0..37: 0=M2 start, 1-2=cal 5s, 3=wait, 4-5=cal 6s, 6=wait, 7-36=10x(cal 7s, cal 7s, wait), 37=M2 stop

// --- Tunables ---
// M1 direction: on threaded rod, -400 = mechanism goes DOWN (into ground), +400 = UP (retract to origin)
static int M1_SPEED_DOWN = -400;
static int M1_SPEED_UP = 400;
static unsigned long M1_KICK_MS = 300;
static int M1_CRUISE_SPEED_DOWN = 180;
static int M1_CRUISE_SPEED_UP = -200;
static float fullTravelTimeSec = 7.0f;
static float MM_PER_SEC_AT_FULL = 116.84f / 7.0f;

// --- State ---
static float positionMm = 0.0f;
static unsigned long lastPositionUpdateMs = 0;
static bool limitsEnabled = true;
static int m1CurrentSpeed = 0;
static int m2CurrentSpeed = 0;

enum MoveState { IDLE, M1_KICK, M1_CRUISE, M1_TIMED, SOIL_TEST, AUTO_COLLECT, SOIL_COLLECT };
static MoveState moveState = IDLE;
static unsigned long moveStartMs = 0;
static unsigned long cruiseStartMs = 0;
static unsigned long cruiseTimeMs = 0;
static float moveTargetMm = 0.0f;
static int moveCruiseSpeed = 0;
static unsigned long timedRunEndMs = 0;
static float positionBeforeLastMoveMm = 0.0f;

static int soilPhase = 0;
static unsigned long soilPhaseEndMs = 0;
static unsigned long soilM1DownEndMs = 0;
static int soilCycleCount = 0;

static unsigned long collectEndMs = 0;
static int collectPhase = 0;
static unsigned long collectPhaseEndMs = 0;

static int soilCollectStep = 0;  // 0..37 for step-based SOIL_COLLECT sequence
static unsigned long soilCollectPhaseEndMs = 0;

static char motorBuf[120];

static void motorLog(const char* msg);

static float mmPerSec(int speed) {
  int absSpeed = speed < 0 ? -speed : speed;
  if (absSpeed == 0) return 0.0f;
  float rate = (fullTravelTimeSec > 0.0f) ? (TRAVEL_MM / fullTravelTimeSec) : MM_PER_SEC_AT_FULL;
  return rate * ((float)absSpeed / 400.0f);
}

static bool m1WouldExceedOriginLimit(int speed) {
  if (!limitsEnabled) return false;
  if (speed == M1_SPEED_DOWN && positionMm <= M1_POSITION_MIN) return true;  // at bottom, can't go further down
  if (speed == M1_SPEED_UP && positionMm >= M1_POSITION_MAX) return true;    // at origin, can't go further up
  return false;
}

static void setMotorRaw(int motor, int speed) {
  speed = constrain(speed, -400, 400);
  if (motor == 1 && m1WouldExceedOriginLimit(speed)) speed = 0;
  if (motor == 1) lastPositionUpdateMs = millis();

#if MOTOR_EN_PINS_CONNECTED
  digitalWrite(MOTOR_M1EN, LOW);
  digitalWrite(MOTOR_M2EN, LOW);
#endif

  int inaPin, inbPin, pwmPin;
  if (motor == 1) {
    inaPin = MOTOR_M1INA;
    inbPin = MOTOR_M1INB;
    pwmPin = MOTOR_M1PWM;
    m1CurrentSpeed = speed;
  } else {
    inaPin = MOTOR_M2INA;
    inbPin = MOTOR_M2INB;
    pwmPin = MOTOR_M2PWM;
    m2CurrentSpeed = speed;
  }
  int absSpeed = speed < 0 ? -speed : speed;
  if (speed >= 0) {
    digitalWrite(inaPin, HIGH);
    digitalWrite(inbPin, LOW);
  } else {
    digitalWrite(inaPin, LOW);
    digitalWrite(inbPin, HIGH);
  }
  if (absSpeed == 0) {
    digitalWrite(pwmPin, LOW);
  } else if (absSpeed >= 400) {
    digitalWrite(pwmPin, HIGH);
  } else {
    analogWrite(pwmPin, map(absSpeed, 0, 400, 0, 255));
  }
}

static void stopMotorRaw(int motor) {
  if (motor == 1) lastPositionUpdateMs = millis();
  int inaPin, inbPin, pwmPin;
  if (motor == 1) {
    inaPin = MOTOR_M1INA;
    inbPin = MOTOR_M1INB;
    pwmPin = MOTOR_M1PWM;
    m1CurrentSpeed = 0;
  } else {
    inaPin = MOTOR_M2INA;
    inbPin = MOTOR_M2INB;
    pwmPin = MOTOR_M2PWM;
    m2CurrentSpeed = 0;
  }
  digitalWrite(inaPin, LOW);
  digitalWrite(inbPin, LOW);
  digitalWrite(pwmPin, LOW);
}

static void motorLog(const char* msg) {
  Monitor.println(msg);
  Serial.println(msg);
}

void setLimitsEnabled(bool on) { limitsEnabled = on; }
bool getLimitsEnabled() { return limitsEnabled; }

bool initMotor() {
  positionMm = 0.0f;
  lastPositionUpdateMs = millis();
  moveState = IDLE;
  m1CurrentSpeed = 0;
  m2CurrentSpeed = 0;

  /* Set up motor control pins (MotorControl/sketch.ino setup() order). */
  pinMode(MOTOR_M1INA, OUTPUT);
  pinMode(MOTOR_M1INB, OUTPUT);
  pinMode(MOTOR_M1PWM, OUTPUT);
  pinMode(MOTOR_M2INA, OUTPUT);
  pinMode(MOTOR_M2INB, OUTPUT);
  pinMode(MOTOR_M2PWM, OUTPUT);
#if MOTOR_EN_PINS_CONNECTED
  pinMode(MOTOR_M1EN, OUTPUT);
  pinMode(MOTOR_M2EN, OUTPUT);
  digitalWrite(MOTOR_M1EN, HIGH);
  digitalWrite(MOTOR_M2EN, HIGH);
  delay(250);
  digitalWrite(MOTOR_M1EN, LOW);
  digitalWrite(MOTOR_M2EN, LOW);
#endif

  stopMotorRaw(1);
  stopMotorRaw(2);
  delay(50);
  motor_initialized = true;
  Monitor.println(F("Motor init: Pololu DualVNH5019 (full MotorControl)"));
  Serial.println(F("Motor init: Pololu DualVNH5019 (full MotorControl)"));
  Monitor.println(F("  EN pins NOT connected: M1EN/M2EN left unplugged on shield; drivers always enabled."));
  Serial.println(F("  EN pins NOT connected: M1EN/M2EN left unplugged on shield; drivers always enabled."));
  Monitor.println(F("  WIRING: D2->M1INA D4->M1INB D9->M1PWM D7->M2INA D8->M2INB D10->M2PWM A0->M1CS A1->M2CS (no EN)."));
  Serial.println(F("  WIRING: D2->M1INA D4->M1INB D9->M1PWM D7->M2INA D8->M2INB D10->M2PWM A0->M1CS A1->M2CS (no EN)."));
  return true;
}

void setMotor1Speed(int speed) {
  speed = constrain(speed, -400, 400);
  setMotorRaw(1, speed);
}

void setMotor2Speed(int speed) {
  speed = constrain(speed, -400, 400);
  setMotorRaw(2, speed);
}

void stopMotor1() { stopMotorRaw(1); }
void stopMotor2() { stopMotorRaw(2); }

void startMotor1Timed(int speed, unsigned long durationMs) {
  speed = constrain(speed, -400, 400);
  moveState = IDLE;
  stopMotorRaw(1);
  timedRunEndMs = millis() + durationMs;
  moveState = M1_TIMED;
  // #region agent log
  { char dbg[140]; snprintf(dbg, sizeof(dbg), "DBG6524f0\t{\"hypothesisId\":\"B\",\"message\":\"startMotor1Timed\",\"durationMs\":%lu,\"millis\":%lu,\"timedRunEndMs\":%lu}", durationMs, (unsigned long)millis(), timedRunEndMs); Serial.println(dbg); }
  // #endregion
  setMotorRaw(1, speed);
}

void setMotorOrigin() {
  positionMm = M1_POSITION_MAX;  // origin = retracted
  lastPositionUpdateMs = millis();
}

float getPositionMm() { return positionMm; }

void setTravelTimeSec(float sec) {
  if (sec > 0.1f && sec < 600.0f) {
    fullTravelTimeSec = sec;
    MM_PER_SEC_AT_FULL = TRAVEL_MM / fullTravelTimeSec;
  }
}

void setM1KickMs(unsigned long ms) {
  if (ms > 0 && ms <= 2000) M1_KICK_MS = ms;
}

void setM1Cruise(int down, int up) {
  M1_CRUISE_SPEED_DOWN = down;
  M1_CRUISE_SPEED_UP = up;
}

void motor_cancelMove() {
  moveState = IDLE;
  stopMotorRaw(1);
  stopMotorRaw(2);
}

bool isMoveStateIdle() { return moveState == IDLE; }

void moveM1DownMm(float mm) {
  if (mm <= 0) return;
  positionBeforeLastMoveMm = positionMm;
  moveTargetMm = positionMm - mm;  // down = toward bottom (0)
  if (moveTargetMm < M1_POSITION_MIN) moveTargetMm = M1_POSITION_MIN;
  float distanceMm = positionMm - moveTargetMm;
  float kickDist = mmPerSec(M1_SPEED_DOWN) * (M1_KICK_MS / 1000.0f);
  if (kickDist < 0) kickDist = -kickDist;
  float cruiseDist = distanceMm - kickDist;
  if (cruiseDist < 0) cruiseDist = 0;
  cruiseTimeMs = (unsigned long)((cruiseDist / mmPerSec(M1_CRUISE_SPEED_DOWN)) * 1000.0f);
  if (cruiseTimeMs == 0 && cruiseDist > 0) cruiseTimeMs = 1;
  moveCruiseSpeed = M1_SPEED_DOWN;
  moveStartMs = millis();
  moveState = M1_KICK;
  setMotorRaw(1, M1_SPEED_DOWN);
}

void moveM1UpMm(float mm) {
  if (mm <= 0) return;
  positionBeforeLastMoveMm = positionMm;
  moveTargetMm = positionMm + mm;  // up = toward origin
  if (moveTargetMm > M1_POSITION_MAX) moveTargetMm = M1_POSITION_MAX;
  float distanceMm = moveTargetMm - positionMm;
  float kickDist = mmPerSec(M1_SPEED_UP) * (M1_KICK_MS / 1000.0f);
  if (kickDist < 0) kickDist = -kickDist;
  float cruiseDist = distanceMm - kickDist;
  if (cruiseDist < 0) cruiseDist = 0;
  cruiseTimeMs = (unsigned long)((cruiseDist / mmPerSec(M1_CRUISE_SPEED_UP)) * 1000.0f);
  if (cruiseTimeMs == 0 && cruiseDist > 0) cruiseTimeMs = 1;
  moveCruiseSpeed = M1_SPEED_UP;
  moveStartMs = millis();
  moveState = M1_KICK;
  setMotorRaw(1, M1_SPEED_UP);
}

void gotoPositionMm(float targetMm) {
  if (targetMm < M1_POSITION_MIN) targetMm = M1_POSITION_MIN;
  if (targetMm > M1_POSITION_MAX) targetMm = M1_POSITION_MAX;
  float dist = targetMm - positionMm;
  if (dist > 0) {
    // toward higher position = retract (origin) = use UP
    positionBeforeLastMoveMm = positionMm;
    moveTargetMm = targetMm;
    float distanceMm = dist;
    float kickDist = mmPerSec(M1_SPEED_UP) * (M1_KICK_MS / 1000.0f);
    if (kickDist < 0) kickDist = -kickDist;
    float cruiseDist = distanceMm - kickDist;
    if (cruiseDist < 0) cruiseDist = 0;
    cruiseTimeMs = (unsigned long)((cruiseDist / mmPerSec(M1_CRUISE_SPEED_UP)) * 1000.0f);
    if (targetMm >= M1_POSITION_MAX - 0.5f) cruiseTimeMs += GOTO_LIMIT_EXTRA_MS;
    moveCruiseSpeed = M1_SPEED_UP;
    moveStartMs = millis();
    moveState = M1_KICK;
    setMotorRaw(1, M1_SPEED_UP);
  } else if (dist < 0) {
    // toward lower position = extend (bottom) = use DOWN
    positionBeforeLastMoveMm = positionMm;
    moveTargetMm = targetMm;
    float distanceMm = -dist;
    float kickDist = mmPerSec(M1_SPEED_DOWN) * (M1_KICK_MS / 1000.0f);
    if (kickDist < 0) kickDist = -kickDist;
    float cruiseDist = distanceMm - kickDist;
    if (cruiseDist < 0) cruiseDist = 0;
    cruiseTimeMs = (unsigned long)((cruiseDist / mmPerSec(M1_CRUISE_SPEED_DOWN)) * 1000.0f);
    if (targetMm <= M1_POSITION_MIN + 0.5f) cruiseTimeMs += GOTO_LIMIT_EXTRA_MS;
    moveCruiseSpeed = M1_SPEED_DOWN;
    moveStartMs = millis();
    moveState = M1_KICK;
    setMotorRaw(1, M1_SPEED_DOWN);
  }
}

void startTest() {
  positionBeforeLastMoveMm = positionMm;
  float kickDist = mmPerSec(M1_SPEED_DOWN) * (M1_KICK_MS / 1000.0f);
  if (kickDist < 0) kickDist = -kickDist;
  float cruiseMs = (float)(TEST_EXTEND_MS - M1_KICK_MS);
  if (cruiseMs < 0) cruiseMs = 0;
  float cruiseDist = mmPerSec(M1_CRUISE_SPEED_DOWN) * (cruiseMs / 1000.0f);
  if (cruiseDist < 0) cruiseDist = -cruiseDist;
  moveTargetMm = positionMm - kickDist - cruiseDist;  // extend toward bottom (0)
  if (moveTargetMm < M1_POSITION_MIN) moveTargetMm = M1_POSITION_MIN;
  cruiseTimeMs = (unsigned long)cruiseMs;
  moveCruiseSpeed = M1_SPEED_DOWN;
  moveStartMs = millis();
  moveState = M1_KICK;
  setMotorRaw(1, M1_SPEED_DOWN);
}

void startSoilTest() {
  positionBeforeLastMoveMm = positionMm;
  moveState = IDLE;
  stopMotorRaw(1);
  stopMotorRaw(2);
  soilPhase = 0;
  soilCycleCount = 0;
  soilPhaseEndMs = millis() + SOIL_M2_DOWN_MS;
  soilM1DownEndMs = millis() + (unsigned long)(fullTravelTimeSec * 1000.0f);
  setMotorRaw(1, M1_SPEED_DOWN);
  setMotorRaw(2, -400);
  moveState = SOIL_TEST;
}

void startReturn() {
  float targetMm = positionBeforeLastMoveMm;
  if (targetMm < M1_POSITION_MIN) targetMm = M1_POSITION_MIN;
  if (targetMm > M1_POSITION_MAX) targetMm = M1_POSITION_MAX;
  float dist = targetMm - positionMm;
  if (dist > 0.5f) {
    moveTargetMm = targetMm;
    float distanceMm = dist;
    float kickDist = mmPerSec(M1_SPEED_DOWN) * (M1_KICK_MS / 1000.0f);
    float cruiseDist = distanceMm - kickDist;
    if (cruiseDist < 0) cruiseDist = 0;
    cruiseTimeMs = (unsigned long)((cruiseDist / mmPerSec(M1_CRUISE_SPEED_DOWN)) * 1000.0f);
    moveCruiseSpeed = M1_CRUISE_SPEED_DOWN;
    moveStartMs = millis();
    moveState = M1_KICK;
    setMotorRaw(1, M1_SPEED_DOWN);
  } else if (dist < -0.5f) {
    moveTargetMm = targetMm;
    float distanceMm = -dist;
    float kickDist = mmPerSec(M1_SPEED_UP) * (M1_KICK_MS / 1000.0f);
    if (kickDist < 0) kickDist = -kickDist;
    float cruiseDist = distanceMm - kickDist;
    if (cruiseDist < 0) cruiseDist = 0;
    cruiseTimeMs = (unsigned long)((cruiseDist / mmPerSec(-M1_CRUISE_SPEED_UP)) * 1000.0f);
    moveCruiseSpeed = M1_CRUISE_SPEED_UP;
    moveStartMs = millis();
    moveState = M1_KICK;
    setMotorRaw(1, M1_SPEED_UP);
  }
}

void startCollect(float sec) {
  if (sec < 1.0f || sec > 86400.0f) return;
  positionBeforeLastMoveMm = positionMm;
  moveState = IDLE;
  stopMotorRaw(1);
  stopMotorRaw(2);
  collectEndMs = millis() + (unsigned long)(sec * 1000.0f);
  collectPhase = 0;
  collectPhaseEndMs = millis() + (unsigned long)(COLLECT_FULL_DOWN_SEC * 1000.0f);
  setMotorRaw(2, -400);
  setMotorRaw(1, M1_SPEED_DOWN);
  moveState = AUTO_COLLECT;
}

void startSoilCollect() {
  moveState = IDLE;
  stopMotorRaw(1);
  stopMotorRaw(2);
  setMotorRaw(2, SOIL_COLLECT_M2_SPEED);  // m2 -400
  soilCollectStep = 0;
  soilCollectPhaseEndMs = 0;  // advance from step 0 on first poll
  moveState = SOIL_COLLECT;
}

static void printDiagnostics() {
  int m1Ina = digitalRead(MOTOR_M1INA);
  int m1Inb = digitalRead(MOTOR_M1INB);
  int m2Ina = digitalRead(MOTOR_M2INA);
  int m2Inb = digitalRead(MOTOR_M2INB);
  int m1Cs = analogRead(MOTOR_M1CS);
  int m2Cs = analogRead(MOTOR_M2CS);
#if MOTOR_EN_PINS_CONNECTED
  int m1En = digitalRead(MOTOR_M1EN);
  int m2En = digitalRead(MOTOR_M2EN);
  snprintf(motorBuf, sizeof(motorBuf), "  EN: M1=%d M2=%d  DIR: M1(INA=%d INB=%d) M2(INA=%d INB=%d)  CS: M1=%d M2=%d",
    m1En, m2En, m1Ina, m1Inb, m2Ina, m2Inb, m1Cs, m2Cs);
#else
  motorLog("  EN: not connected (drivers always enabled).");
  snprintf(motorBuf, sizeof(motorBuf), "  DIR: M1(INA=%d INB=%d) M2(INA=%d INB=%d)  CS: M1=%d M2=%d",
    m1Ina, m1Inb, m2Ina, m2Inb, m1Cs, m2Cs);
#endif
  motorLog(motorBuf);
}

static void printCurrentReadings() {
  int m1Cs = analogRead(MOTOR_M1CS);
  int m2Cs = analogRead(MOTOR_M2CS);
  unsigned int m1Ma = (unsigned int)(m1Cs * 34);
  unsigned int m2Ma = (unsigned int)(m2Cs * 34);
  snprintf(motorBuf, sizeof(motorBuf), "  Current: M1=%u mA M2=%u mA", m1Ma, m2Ma);
  motorLog(motorBuf);
}

void motor_printStatus() {
  motorLog("");
  motorLog("--- MOTOR STATUS ---");
  snprintf(motorBuf, sizeof(motorBuf), "  M1 speed: %d  M2 speed: %d", m1CurrentSpeed, m2CurrentSpeed);
  motorLog(motorBuf);
  snprintf(motorBuf, sizeof(motorBuf), "  Position: %.1f mm (0 - %.1f)", positionMm, TRAVEL_MM);
  motorLog(motorBuf);
  if (positionMm <= M1_POSITION_MIN + 0.5f) motorLog("  At bottom (0 mm)");
  else if (positionMm >= M1_POSITION_MAX - 0.5f) motorLog("  At origin (116.84 mm)");
  else motorLog("  In range");
  motorLog(limitsEnabled ? "  Limits: ON (0-116.84 mm)" : "  Limits: OFF");
  motorLog("");
  printDiagnostics();
  printCurrentReadings();
  motorLog("");
}

int motor_getM1Speed(void) { return m1CurrentSpeed; }
int motor_getM2Speed(void) { return m2CurrentSpeed; }
float motor_getPositionMm(void) { return positionMm; }
unsigned int motor_getM1CurrentMa(void) {
  int raw = analogRead(MOTOR_M1CS);
  return (unsigned int)(raw * 34);
}
unsigned int motor_getM2CurrentMa(void) {
  int raw = analogRead(MOTOR_M2CS);
  return (unsigned int)(raw * 34);
}
void motor_getFaultStatus(char* buf, size_t bufLen) {
  unsigned int m1 = motor_getM1CurrentMa();
  unsigned int m2 = motor_getM2CurrentMa();
  bool o1 = (m1 >= (unsigned int)MOTOR_OVERCURRENT_MA);
  bool o2 = (m2 >= (unsigned int)MOTOR_OVERCURRENT_MA);
  if (o1 && o2) snprintf(buf, bufLen, "M1_M2_OVER");
  else if (o1) snprintf(buf, bufLen, "M1_OVER");
  else if (o2) snprintf(buf, bufLen, "M2_OVER");
  else snprintf(buf, bufLen, "OK");
}

// Debounce: require overcurrent for several consecutive reads to avoid inrush spikes triggering stall.
static const int STALL_DEBOUNCE_COUNT = 3;
static int stallDebounceCount = 0;

bool motor_isStallDetected(void) {
  char buf[16];
  motor_getFaultStatus(buf, sizeof(buf));
  bool over = (strcmp(buf, "OK") != 0);
  if (over) {
    if (stallDebounceCount < STALL_DEBOUNCE_COUNT) stallDebounceCount++;
    return (stallDebounceCount >= STALL_DEBOUNCE_COUNT);
  }
  stallDebounceCount = 0;
  return false;
}

void motor_poll() {
  // Re-apply direct motor speeds when idle (so M1/M2 commands persist even if other code touches pins)
  if (moveState == IDLE) {
    if (m1CurrentSpeed != 0) setMotorRaw(1, m1CurrentSpeed);
    if (m2CurrentSpeed != 0) setMotorRaw(2, m2CurrentSpeed);
  }

  // Position tracking
  if (m1CurrentSpeed != 0) {
    unsigned long now = millis();
    unsigned long deltaMs = now - lastPositionUpdateMs;
    lastPositionUpdateMs = now;
    float sec = (float)deltaMs / 1000.0f;
    // Down (into ground) -> position decreases toward 0; up (retract) -> position increases toward origin
    float v = (m1CurrentSpeed == M1_SPEED_DOWN ? -1.0f : (m1CurrentSpeed == M1_SPEED_UP ? 1.0f : 0.0f)) * mmPerSec(m1CurrentSpeed);
    positionMm += v * sec;
    if (positionMm < M1_POSITION_MIN) positionMm = M1_POSITION_MIN;
    if (positionMm > M1_POSITION_MAX) positionMm = M1_POSITION_MAX;
  }

  // Hard limit stop
  if (limitsEnabled && m1CurrentSpeed != 0) {
    if (positionMm <= M1_POSITION_MIN && m1CurrentSpeed == M1_SPEED_DOWN) {
      stopMotorRaw(1);
      positionMm = M1_POSITION_MIN;
      lastPositionUpdateMs = millis();
      if (moveState != AUTO_COLLECT && moveState != SOIL_COLLECT) moveState = IDLE;
    } else if (positionMm >= M1_POSITION_MAX && m1CurrentSpeed == M1_SPEED_UP) {
      stopMotorRaw(1);
      positionMm = M1_POSITION_MAX;
      lastPositionUpdateMs = millis();
      if (moveState != AUTO_COLLECT && moveState != SOIL_COLLECT) moveState = IDLE;
    }
  }

  // Move state machine
  if (moveState == M1_KICK) {
    if (millis() - moveStartMs >= M1_KICK_MS) {
      moveState = M1_CRUISE;
      cruiseStartMs = millis();
      setMotorRaw(1, moveCruiseSpeed);
    }
  } else if (moveState == M1_CRUISE) {
    bool reachedTarget = (moveCruiseSpeed > 0 && positionMm >= moveTargetMm) ||
                         (moveCruiseSpeed < 0 && positionMm <= moveTargetMm);
    if (reachedTarget || (millis() - cruiseStartMs >= cruiseTimeMs)) {
      stopMotorRaw(1);
      positionMm = moveTargetMm;
      if (positionMm < M1_POSITION_MIN) positionMm = M1_POSITION_MIN;
      if (positionMm > M1_POSITION_MAX) positionMm = M1_POSITION_MAX;
      lastPositionUpdateMs = millis();
      moveState = IDLE;
      snprintf(motorBuf, sizeof(motorBuf), ">> Move done. Position = %.1f mm", positionMm);
      motorLog(motorBuf);
    }
  } else if (moveState == M1_TIMED) {
    if (millis() >= timedRunEndMs) {
      stopMotorRaw(1);
      moveState = IDLE;
      // #region agent log
      { char dbg[120]; snprintf(dbg, sizeof(dbg), "DBG6524f0\t{\"hypothesisId\":\"C\",\"message\":\"CAL_finished\",\"millis\":%lu}", (unsigned long)millis()); Serial.println(dbg); }
      // #endregion
      motorLog(">> CAL run finished.");
    }
  } else if (moveState == SOIL_TEST) {
    if (m1CurrentSpeed == M1_SPEED_DOWN && millis() >= soilM1DownEndMs) {
      stopMotorRaw(1);
      positionMm = M1_POSITION_MIN;  // bottom
      lastPositionUpdateMs = millis();
    }
    if (millis() >= soilPhaseEndMs) {
      if (soilPhase == 0) {
        soilCycleCount = 1;
        soilPhase = 1;
        setMotorRaw(2, 400);
        soilPhaseEndMs = millis() + SOIL_M2_HALF_MS;
      } else if (soilPhase == 1) {
        soilPhase = 2;
        setMotorRaw(2, -400);
        soilPhaseEndMs = millis() + SOIL_M2_HALF_MS;
      } else {
        soilCycleCount++;
        if (soilCycleCount > 5) {
          stopMotorRaw(1);
          stopMotorRaw(2);
          moveState = IDLE;
          motorLog(">> SOIL_TEST done.");
        } else {
          soilPhase = 1;
          setMotorRaw(2, 400);
          soilPhaseEndMs = millis() + SOIL_M2_HALF_MS;
        }
      }
    }
  } else if (moveState == SOIL_COLLECT) {
    if (soilCollectStep < 37) setMotorRaw(2, SOIL_COLLECT_M2_SPEED);  // M2 -400 until final step

    if (millis() >= soilCollectPhaseEndMs) {
      stopMotorRaw(1);  // stop M1 when step completes (no-op if step was wait)
      soilCollectStep++;
      if (soilCollectStep >= 37) {
        stopMotorRaw(2);
        moveState = IDLE;
        motorLog(">> SOIL_COLLECT done.");
      } else {
        unsigned long now = millis();
        if (soilCollectStep == 1) {
          setMotorRaw(1, 400);
          soilCollectPhaseEndMs = now + 5000UL;
        } else if (soilCollectStep == 2) {
          setMotorRaw(1, -400);
          soilCollectPhaseEndMs = now + 5000UL;
        } else if (soilCollectStep == 3) {
          soilCollectPhaseEndMs = now + SOIL_COLLECT_WAIT_MS;
        } else if (soilCollectStep == 4) {
          setMotorRaw(1, 400);
          soilCollectPhaseEndMs = now + 6000UL;
        } else if (soilCollectStep == 5) {
          setMotorRaw(1, -400);
          soilCollectPhaseEndMs = now + 6000UL;
        } else if (soilCollectStep == 6) {
          soilCollectPhaseEndMs = now + SOIL_COLLECT_WAIT_MS;
        } else {
          int sub = (soilCollectStep - 7) % 3;
          if (sub == 0) {
            setMotorRaw(1, 400);
            soilCollectPhaseEndMs = now + 7000UL;
          } else if (sub == 1) {
            setMotorRaw(1, -400);
            soilCollectPhaseEndMs = now + 7000UL;
          } else {
            soilCollectPhaseEndMs = now + SOIL_COLLECT_WAIT_MS;
          }
        }
      }
    }
  } else if (moveState == AUTO_COLLECT) {
    if (millis() >= collectEndMs) {
      stopMotorRaw(1);
      stopMotorRaw(2);
      moveState = IDLE;
      motorLog(">> COLLECT done.");
    } else if (collectPhase == 0) {
      if (millis() >= collectPhaseEndMs || positionMm <= M1_POSITION_MIN) {
        stopMotorRaw(1);
        positionMm = M1_POSITION_MIN;  // at bottom
        lastPositionUpdateMs = millis();
        collectPhase = 4;
        collectPhaseEndMs = millis() + (unsigned long)(COLLECT_LINGER_AT_BOTTOM_SEC * 1000.0f);
      }
    } else if (collectPhase == 4) {
      if (millis() >= collectPhaseEndMs) {
        collectPhase = 1;
        collectPhaseEndMs = millis() + (unsigned long)(COLLECT_HALF_UP_SEC * 1000.0f);
        setMotorRaw(1, M1_SPEED_UP);
      }
    } else if (collectPhase == 1) {
      if (millis() >= collectPhaseEndMs || positionMm >= M1_POSITION_MAX) {
        stopMotorRaw(1);
        positionMm = M1_POSITION_MAX;  // at origin
        lastPositionUpdateMs = millis();
        collectPhase = 2;
        collectPhaseEndMs = millis() + (unsigned long)(COLLECT_LINGER_SEC * 1000.0f);
      }
    } else if (collectPhase == 2) {
      if (millis() >= collectPhaseEndMs) {
        collectPhase = 3;
        collectPhaseEndMs = millis() + (unsigned long)(COLLECT_HALF_DOWN_SEC * 1000.0f);
        setMotorRaw(1, M1_SPEED_DOWN);
      }
    } else {
      if (millis() >= collectPhaseEndMs || positionMm <= M1_POSITION_MIN) {
        stopMotorRaw(1);
        positionMm = M1_POSITION_MIN;  // at bottom
        lastPositionUpdateMs = millis();
        collectPhase = 4;
        collectPhaseEndMs = millis() + (unsigned long)(COLLECT_LINGER_AT_BOTTOM_SEC * 1000.0f);
      }
    }
  }
}
