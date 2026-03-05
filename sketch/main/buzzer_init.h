#ifndef BUZZER_INIT_H
#define BUZZER_INIT_H

#include <Arduino.h>

extern bool buzzer_initialized;

bool initBuzzer();
void playArmedSound();
void playArmedWithWarningSound();
void playArmRejectedSound();
void playDisarmedSound();
void playRecordingSound();

/** Play the distinct sound for mission state 0-6 (PREFLIGHT, FLIGHT, LANDING_CONFIRM, DRILLING, PH_TEST, DISARMED, DESCENT_BYPASS). Short patterns so override path does not block long. */
void playStateSound(uint8_t stateId);

#endif
