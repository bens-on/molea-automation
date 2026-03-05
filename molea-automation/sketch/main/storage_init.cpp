#include "storage_init.h"
#include "config.h"
#include <Arduino_RouterBridge.h>

bool storage_initialized = false;

static unsigned long mission_start_ms = 0;
static unsigned long launch_ms = 0;
static unsigned long landing_ms = 0;
static uint32_t ph_sample_count = 0;
static float ph_min = 999.0f;
static float ph_max = -999.0f;
static double ph_sum = 0.0;

bool storage_init() {
  mission_start_ms = 0;
  launch_ms = 0;
  landing_ms = 0;
  ph_sample_count = 0;
  ph_min = 999.0f;
  ph_max = -999.0f;
  ph_sum = 0.0;
  storage_initialized = true;
  Monitor.println(F("Storage initialized (Monitor output)"));
  return true;
}

void storage_record_ph_sample(float ph) {
  if (!storage_initialized) return;
  ph_sample_count++;
  if (ph < ph_min) ph_min = ph;
  if (ph > ph_max) ph_max = ph;
  ph_sum += (double)ph;
}

void storage_set_mission_start(unsigned long ms) {
  mission_start_ms = ms;
}

void storage_set_launch_time(unsigned long ms) {
  launch_ms = ms;
}

void storage_set_landing_time(unsigned long ms) {
  landing_ms = ms;
}

void storage_write_mission_summary() {
  if (!storage_initialized) return;
  Monitor.println();
  Monitor.println(F("=== MISSION SUMMARY ==="));
  Monitor.print(F("Mission start (ms): "));
  Monitor.println(mission_start_ms);
  if (launch_ms) {
    Monitor.print(F("Launch (ms): "));
    Monitor.println(launch_ms);
  }
  if (landing_ms) {
    Monitor.print(F("Landing (ms): "));
    Monitor.println(landing_ms);
  }
  Monitor.print(F("pH samples: "));
  Monitor.println(ph_sample_count);
  if (ph_sample_count > 0) {
    Monitor.print(F("pH min: "));
    Monitor.println(ph_min, 2);
    Monitor.print(F("pH max: "));
    Monitor.println(ph_max, 2);
    Monitor.print(F("pH avg: "));
    Monitor.println((float)(ph_sum / ph_sample_count), 2);
  }
  Monitor.println(F("=== END MISSION SUMMARY ==="));
  Monitor.println();
}
