#ifndef STORAGE_INIT_H
#define STORAGE_INIT_H

#include <Arduino.h>

extern bool storage_initialized;

bool storage_init();
void storage_record_ph_sample(float ph);
void storage_set_mission_start(unsigned long ms);
void storage_set_launch_time(unsigned long ms);
void storage_set_landing_time(unsigned long ms);
void storage_write_mission_summary();

#endif
