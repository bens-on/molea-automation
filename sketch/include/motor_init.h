#pragma once
#include <Arduino.h>

extern bool motor_initialized;

// Motor 0-116.84 mm limits (when true, M1 cannot move past origin or end)
void setLimitsEnabled(bool on);
bool getLimitsEnabled();

bool initMotor();
// Speed -400..400 (MotorControl scale); mapped to PWM internally
void setMotor1Speed(int speed);
void setMotor2Speed(int speed);
void stopMotor1();
void stopMotor2();

// CAL command: run M1 at speed for durationMs (non-blocking). E.g. startMotor1Timed(-400, 9500) = CAL -400 9.5
void startMotor1Timed(int speed, unsigned long durationMs);
void motor_poll();

// Motor position origin (SET_ORIGIN command)
void setMotorOrigin();
float getPositionMm();

// Position calibration: time for M1 at 400 to travel 0->116.84 mm
void setTravelTimeSec(float sec);
void setM1KickMs(unsigned long ms);
void setM1Cruise(int down, int up);

// Soft-start moves (kick then cruise)
void moveM1DownMm(float mm);
void moveM1UpMm(float mm);
void gotoPositionMm(float mm);

// Preset moves
void startTest();
void startSoilTest();
void startReturn();
void startCollect(float sec);
void startSoilCollect();

// Cancel any move and stop both motors (STOP command)
void motor_cancelMove();
bool isMoveStateIdle();

// Status and diagnostics
void motor_printStatus();

// Data logging: speeds (-400..400), position (mm), current (mA), fault status for monitor and file
int motor_getM1Speed(void);
int motor_getM2Speed(void);
float motor_getPositionMm(void);
unsigned int motor_getM1CurrentMa(void);
unsigned int motor_getM2CurrentMa(void);
// Fault status: "OK" or "M1_OVER" / "M2_OVER" / "M1_M2_OVER" when current exceeds MOTOR_OVERCURRENT_MA
void motor_getFaultStatus(char* buf, size_t bufLen);
// True when either motor reports overcurrent (stall/overload)
bool motor_isStallDetected(void);
