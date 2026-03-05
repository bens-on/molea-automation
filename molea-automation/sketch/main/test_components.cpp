#include "test_components.h"
#include "debug.h"
#include "BMP280_init.h"
#include "motor_init.h"

static void logCmd(const char* line) {
  logCommandLine(line);
}

void parseCommand(const String& cmd) {
  String cmdUpper = cmd;
  cmdUpper.trim();
  cmdUpper.toUpperCase();
  if (cmdUpper.length() == 0) return;

  if (cmdUpper == "HELP" || cmdUpper == "?") {
    showHelp();
    logCmd("CMD,HELP");
    return;
  }

  if (cmdUpper.startsWith("SET_STATE ")) {
    String name = cmd.substring(10);
    name.trim();
    name.toUpperCase();
    int stateId = -1;
    if (name == "PREFLIGHT")       stateId = 0;
    else if (name == "FLIGHT")     stateId = 1;
    else if (name == "LANDING_CONFIRM") stateId = 2;
    else if (name == "DRILLING")   stateId = 3;
    else if (name == "PH_TEST")    stateId = 4;
    else if (name == "DISARMED")   stateId = 5;
    else if (name == "DESCENT_BYPASS") stateId = 6;
    if (stateId >= 0) {
      setMissionStateOverride((uint8_t)stateId);
      static char buf[48];
      snprintf(buf, sizeof(buf), "CMD,SET_STATE,%s", name.c_str());
      logCmd(buf);
    } else {
      Monitor.println(F("SET_STATE: use PREFLIGHT, FLIGHT, LANDING_CONFIRM, DRILLING, PH_TEST, DISARMED, or DESCENT_BYPASS"));
    }
    return;
  }

  if (cmdUpper == "STOP") {
    if (isLandingConfirmLevelingInProgress()) {
      Monitor.println(F("Leveling in progress (LANDING_CONFIRM); STOP ignored. Wait for leveling to finish or SET_STATE DISARMED."));
      return;
    }
    motor_cancelMove();
    stopMotor1();
    stopMotor2();
    logCmd("CMD,STOP");
    return;
  }

  if (cmdUpper == "LIMITS ON" || cmdUpper == "LIMITS_ON") {
    setLimitsEnabled(true);
    logCmd("CMD,LIMITS_ON");
    return;
  }
  if (cmdUpper == "LIMITS OFF" || cmdUpper == "LIMITS_OFF") {
    setLimitsEnabled(false);
    logCmd("CMD,LIMITS_OFF");
    return;
  }

  if (cmdUpper == "SET_ORIGIN") {
    setMotorOrigin();
    logCmd("CMD,SET_ORIGIN");
    return;
  }

  if (cmdUpper == "BMP_ORIGIN") {
    if (bmp280_initialized) calibrateBMP280AtGround(5);
    logCmd("CMD,BMP_ORIGIN");
    return;
  }

  if (cmdUpper == "POS" || cmdUpper == "POSITION") {
    motor_printStatus();
    logCmd("CMD,POS");
    return;
  }

  if (cmdUpper == "STATUS") {
    motor_printStatus();
    logCmd("CMD,STATUS");
    return;
  }

  if (cmdUpper.startsWith("TRAVEL_TIME ")) {
    float sec = cmd.substring(12).toFloat();
    if (sec > 0.1f && sec < 600.0f) {
      setTravelTimeSec(sec);
      logCmd("CMD,TRAVEL_TIME");
    } else {
      Monitor.println(F("TRAVEL_TIME: give seconds (e.g. 7)"));
    }
    return;
  }

  if (cmdUpper.startsWith("DRILL_MIN ")) {
    String arg = cmd.substring(10);
    arg.trim();
    int n = arg.toInt();
    if (n >= 1 && n <= 60) {
      setDrillDurationMinutes((uint8_t)n);
      static char buf[32];
      snprintf(buf, sizeof(buf), "CMD,DRILL_MIN,%d", n);
      logCmd(buf);
    } else {
      Monitor.println(F("DRILL_MIN: give minutes 1-60 (e.g. DRILL_MIN 8)"));
    }
    return;
  }

  if (cmdUpper.startsWith("LEG_TIME ")) {
    String arg = cmd.substring(9);
    arg.trim();
    int n = arg.toInt();
    if (n >= 1 && n <= 120) {
      setLegTimeSec((uint8_t)n);
      static char buf[32];
      snprintf(buf, sizeof(buf), "CMD,LEG_TIME,%d", n);
      logCmd(buf);
    } else {
      Monitor.println(F("LEG_TIME: give minutes 1-120 (e.g. LEG_TIME 2)"));
    }
    return;
  }

  if (cmdUpper == "TEST") {
    startTest();
    logCmd("CMD,TEST");
    return;
  }

  if (cmdUpper == "SOIL_TEST") {
    startSoilTest();
    logCmd("CMD,SOIL_TEST");
    return;
  }

  if (cmdUpper == "SOIL_COLLECT") {
    startSoilCollect();
    logCmd("CMD,SOIL_COLLECT");
    return;
  }

  if (cmdUpper == "RETURN") {
    startReturn();
    logCmd("CMD,RETURN");
    return;
  }

  if (cmdUpper.startsWith("CAL ")) {
    String args = cmd.substring(4);
    args.trim();
    int spaceIdx = args.indexOf(' ');
    if (spaceIdx > 0) {
      int calSpeed = args.substring(0, spaceIdx).toInt();
      float sec = args.substring(spaceIdx + 1).toFloat();
      if (sec > 0.0f && sec < 86400.0f) {
        calSpeed = constrain(calSpeed, -400, 400);
        startMotor1Timed(calSpeed, (unsigned long)(sec * 1000.0f));
        static char buf[32];
        snprintf(buf, sizeof(buf), "CMD,CAL,%d,%.1f", calSpeed, sec);
        logCmd(buf);
      } else {
        Monitor.println(F("CAL: give seconds (e.g. CAL -400 9.5)"));
      }
    } else {
      Monitor.println(F("CAL: give speed and seconds (e.g. CAL -400 9.5)"));
    }
    return;
  }

  if (cmdUpper.startsWith("M1_KICK ")) {
    long ms = cmd.substring(8).toInt();
    if (ms > 0 && ms <= 2000) {
      setM1KickMs((unsigned long)ms);
      logCmd("CMD,M1_KICK");
    } else {
      Monitor.println(F("M1_KICK: 1-2000 ms"));
    }
    return;
  }

  if (cmdUpper.startsWith("M1_CRUISE ")) {
    String rest = cmd.substring(10);
    rest.trim();
    int spaceIdx = rest.indexOf(' ');
    if (spaceIdx > 0) {
      int down = rest.substring(0, spaceIdx).toInt();
      int up = rest.substring(spaceIdx + 1).toInt();
      setM1Cruise(down, up);
    } else {
      int v = rest.toInt();
      if (v > 0) setM1Cruise(v, -v);
      else setM1Cruise(-v, v);
    }
    logCmd("CMD,M1_CRUISE");
    return;
  }

  if (cmdUpper.startsWith("M1 DOWN ")) {
    float mm = cmd.substring(8).toFloat();
    if (mm > 0) {
      moveM1DownMm(mm);
      logCmd("CMD,M1_DOWN");
    } else {
      Monitor.println(F("M1 DOWN: positive mm required"));
    }
    return;
  }

  if (cmdUpper.startsWith("M1 UP ")) {
    float mm = cmd.substring(6).toFloat();
    if (mm > 0) {
      moveM1UpMm(mm);
      logCmd("CMD,M1_UP");
    } else {
      Monitor.println(F("M1 UP: positive mm required"));
    }
    return;
  }

  if (cmdUpper.startsWith("GOTO ")) {
    float targetMm = cmd.substring(5).toFloat();
    gotoPositionMm(targetMm);
    logCmd("CMD,GOTO");
    return;
  }

  if (cmdUpper.startsWith("COLLECT ")) {
    float sec = cmd.substring(8).toFloat();
    if (sec >= 1.0f && sec <= 86400.0f) {
      startCollect(sec);
      logCmd("CMD,COLLECT");
    } else {
      Monitor.println(F("COLLECT: duration 1-86400 sec"));
    }
    return;
  }

  if (cmdUpper.startsWith("RUN ")) {
    String args = cmd.substring(4);
    args.trim();
    int spaceIdx = args.indexOf(' ');
    if (spaceIdx > 0) {
      int s1 = args.substring(0, spaceIdx).toInt();
      int s2 = args.substring(spaceIdx + 1).toInt();
      setMotor1Speed(constrain(s1, -400, 400));
      setMotor2Speed(constrain(s2, -400, 400));
      logCmd("CMD,RUN");
    } else {
      Monitor.println(F("RUN: give two speeds (e.g. RUN 200 -150)"));
    }
    return;
  }

  if (cmdUpper.startsWith("M1 ")) {
    String arg = cmd.substring(2);
    arg.trim();
    arg.toUpperCase();
    static char buf[24];
    if (arg == "STOP") {
      stopMotor1();
      logCmd("CMD,M1,STOP");
    } else {
      int speed = arg.toInt();
      speed = constrain(speed, -400, 400);
      setMotor1Speed(speed);
      snprintf(buf, sizeof(buf), "CMD,M1,%d", speed);
      logCmd(buf);
    }
    return;
  }

  if (cmdUpper.startsWith("M2 ")) {
    String arg = cmd.substring(2);
    arg.trim();
    arg.toUpperCase();
    static char buf2[24];
    if (arg == "STOP") {
      stopMotor2();
      logCmd("CMD,M2,STOP");
    } else {
      int speed = arg.toInt();
      speed = constrain(speed, -400, 400);
      setMotor2Speed(speed);
      snprintf(buf2, sizeof(buf2), "CMD,M2,%d", speed);
      logCmd(buf2);
    }
    return;
  }

  Monitor.print(F("Unknown command: "));
  Monitor.println(cmd);
  Monitor.println(F("Type HELP for commands"));
}

void showHelp() {
  Monitor.println();
  Monitor.println(F("  MOTOR CONTROL - COMMAND HELP"));
  Monitor.println(F("================================="));
  Monitor.println();
  Monitor.println(F("Commands (not case-sensitive):"));
  Monitor.println(F("  M1 <speed>      - Set Motor 1 (screw) speed"));
  Monitor.println(F("  M2 <speed>      - Set Motor 2 (auger) speed"));
  Monitor.println(F("  M1 DOWN <mm>    - Move M1 down by mm (soft-start: kick then cruise)"));
  Monitor.println(F("  M1 UP <mm>      - Move M1 up by mm (soft-start: kick then cruise)"));
  Monitor.println(F("  GOTO <mm>       - Move to position 0 - 116.84 mm"));
  Monitor.println(F("  POS / POSITION  - Show estimated position (mm)"));
  Monitor.println(F("  SET_ORIGIN      - Set current position to origin (116.84 mm, retracted)"));
  Monitor.println(F("  TRAVEL_TIME <s> - Set time (sec) for M1 at 400 to go 0->116.84 mm; enables accurate position"));
  Monitor.println(F("  TEST            - Extend 1.1 s (kick then cruise) then stop; tests kick"));
  Monitor.println(F("  SOIL_TEST       - M1 down, M2 down to bottom, then up-halfway/back-down x5"));
  Monitor.println(F("  SOIL_COLLECT    - Sequence: M2 -400, cal 400/5 -400/5, wait 1s, cal 400/6 -400/6, wait 1s, 10x(cal 400/7 -400/7 wait 1s), M2 stop"));
  Monitor.println(F("  RETURN          - Undo last move (go back to position before last M1 DOWN/UP, GOTO, TEST, CAL, SOIL_TEST, COLLECT)"));
  Monitor.println(F("  CAL <s> <sec>   - Run M1 at speed s for sec (for calibration)"));
  Monitor.println(F("  M1_KICK <ms>    - Set kick duration (default 300 ms)"));
  Monitor.println(F("  M1_CRUISE [down] [up] - Set cruise speed(s)"));
  Monitor.println(F("  RUN <s1> <s2>   - Set both motors"));
  Monitor.println(F("  STOP            - Stop all motors"));
  Monitor.println(F("  LIMITS_OFF      - Ignore 0/116.84 mm limits (drive to mechanical origin; then SET_ORIGIN, LIMITS_ON)"));
  Monitor.println(F("  LIMITS_ON       - Re-enable 0/116.84 mm limits (default)"));
  Monitor.println(F("  STATUS          - Show motor status and position"));
  Monitor.println(F("  COLLECT <sec>   - Autonomous: down 9s, linger at bottom, up 4.5s, linger at top, down 4.5s; repeat (run from origin)"));
  Monitor.println(F("  SET_STATE <name> - Override mission state (PREFLIGHT, FLIGHT, LANDING_CONFIRM, DRILLING, PH_TEST, DISARMED, DESCENT_BYPASS)"));
  Monitor.println(F("  DRILL_MIN <n>   - Set drilling phase duration in minutes (1-60, default 8)"));
  Monitor.println(F("  LEG_TIME <n>    - Set leg leveling time in minutes, M1 run in LANDING_CONFIRM (1-120, default 10)"));
  Monitor.println(F("  BMP_ORIGIN      - Set BMP280 ground altitude reference"));
  Monitor.println(F("  HELP, ?         - Show this message"));
  Monitor.println();
  Monitor.println(F("Speed range: -400 to 400. M1: down=-400 (into ground), up=400 (retract). Distance: 0 = bottom (extended), 116.84 = origin (retracted)."));
  Monitor.println(F("Hard limits: M1 stops at 0 mm (bottom) and 116.84 mm (origin). Use LIMITS_OFF to drive past, then SET_ORIGIN at retracted position."));
  Monitor.println();
  Monitor.println(F("Examples:"));
  Monitor.println(F("  test            - Test kick: extend 1.1 s then stop"));
  Monitor.println(F("  return          - Return to origin"));
  Monitor.println(F("  soil_test       - Soil collection test"));
  Monitor.println(F("  m1 down 50      - Move 50 mm down (kick then cruise)"));
  Monitor.println(F("  goto 40         - Go to 40 mm"));
  Monitor.println(F("  collect 60      - Autonomous collect for 60 s (from origin)"));
  Monitor.println(F("  limits_off      - Then drive to mechanical origin, STOP, SET_ORIGIN, limits_on"));
  Monitor.println(F("  stop            - Stop everything"));
  Monitor.println();
  Monitor.println(F("Ready! Type a command. (Motors not moving? Verify wiring and LIMITS_OFF if at end of travel.)"));
  Monitor.println();
}
