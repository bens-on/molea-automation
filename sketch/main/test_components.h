#ifndef TEST_COMPONENTS_H
#define TEST_COMPONENTS_H

#include <Arduino.h>

void parseCommand(const String& cmd);
void showHelp();

/** Override mission state (0=PREFLIGHT, 1=FLIGHT, 2=LANDING_CONFIRM, 3=DRILLING, 4=PH_TEST, 5=DISARMED, 6=DESCENT_BYPASS). Runs same per-state actions as automatic transitions. */
void setMissionStateOverride(uint8_t stateId);

/** Set drilling phase duration in minutes (1–60). Used by DRILL_MIN command and for DRILLING timeout. */
void setDrillDurationMinutes(uint8_t minutes);

/** Set leg leveling time in minutes (M1 run duration in LANDING_CONFIRM). Used by LEG_TIME command. */
void setLegTimeSec(uint8_t seconds);

/** True when in LANDING_CONFIRM and M1 leveling run is in progress (STOP is ignored so leveling completes). */
bool isLandingConfirmLevelingInProgress(void);

#endif
