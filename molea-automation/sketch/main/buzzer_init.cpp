#include "buzzer_init.h"
#include "config.h"

bool buzzer_initialized = false;

bool initBuzzer() {
  pinMode(BUZZER_PIN, OUTPUT);
  digitalWrite(BUZZER_PIN, LOW);
  
  Monitor.println(F("Buzzer initialized"));
  buzzer_initialized = true;
  return true;
}

void playArmedSound() {
  if (!buzzer_initialized) return;
  
  // Two short high-pitched beeps to indicate electronics are armed
  tone(BUZZER_PIN, 2000, 50);  // First high-pitched beep
  delay(100);
  tone(BUZZER_PIN, 2200, 50);  // Second high-pitched beep
  delay(200);
}

void playArmedWithWarningSound() {
  if (!buzzer_initialized) return;
  // Armed pattern plus one lower-pitch beep for non-critical sensor missing
  tone(BUZZER_PIN, 2000, 50);
  delay(100);
  tone(BUZZER_PIN, 2200, 50);
  delay(100);
  tone(BUZZER_PIN, 800, 150);
  delay(200);
}

void playArmRejectedSound() {
  if (!buzzer_initialized) return;
  // Low repeated beeps - critical failure, arm rejected
  for (int i = 0; i < 3; i++) {
    tone(BUZZER_PIN, 400, 200);
    delay(300);
  }
}

void playDisarmedSound() {
  if (!buzzer_initialized) return;
  // One medium-length tone to indicate mission complete, disarmed
  tone(BUZZER_PIN, 1000, 400);
  delay(500);
}

void playRecordingSound() {
  if (!buzzer_initialized) return;
  
  tone(BUZZER_PIN, 1500, 100);
  delay(50);
  tone(BUZZER_PIN, 1500, 100);
  delay(50);
}

// State IDs: 0=PREFLIGHT, 1=FLIGHT, 2=LANDING_CONFIRM, 3=DRILLING, 4=PH_TEST, 5=DISARMED, 6=DESCENT_BYPASS
// Use tone(); delay(); noTone(); so the buzzer always stops (Uno Q/Zephyr may not honor tone(pin,freq,duration)).
void playStateSound(uint8_t stateId) {
  if (!buzzer_initialized) return;
  switch (stateId) {
    case 0: // PREFLIGHT — one short beep
      tone(BUZZER_PIN, 800);
      delay(80);
      noTone(BUZZER_PIN);
      delay(80);
      break;
    case 1: // FLIGHT — two quick high beeps
      tone(BUZZER_PIN, 1500);
      delay(60);
      noTone(BUZZER_PIN);
      delay(40);
      tone(BUZZER_PIN, 1500);
      delay(60);
      noTone(BUZZER_PIN);
      delay(80);
      break;
    case 2: // LANDING_CONFIRM — three short beeps
      tone(BUZZER_PIN, 1200);
      delay(60);
      noTone(BUZZER_PIN);
      delay(50);
      tone(BUZZER_PIN, 1200);
      delay(60);
      noTone(BUZZER_PIN);
      delay(50);
      tone(BUZZER_PIN, 1200);
      delay(60);
      noTone(BUZZER_PIN);
      delay(80);
      break;
    case 3: // DRILLING — four short beeps
      tone(BUZZER_PIN, 1000);
      delay(50);
      noTone(BUZZER_PIN);
      delay(40);
      tone(BUZZER_PIN, 1000);
      delay(50);
      noTone(BUZZER_PIN);
      delay(40);
      tone(BUZZER_PIN, 1000);
      delay(50);
      noTone(BUZZER_PIN);
      delay(40);
      tone(BUZZER_PIN, 1000);
      delay(50);
      noTone(BUZZER_PIN);
      delay(80);
      break;
    case 4: // PH_TEST — five very short beeps
      tone(BUZZER_PIN, 1800);
      delay(40);
      noTone(BUZZER_PIN);
      delay(30);
      tone(BUZZER_PIN, 1800);
      delay(40);
      noTone(BUZZER_PIN);
      delay(30);
      tone(BUZZER_PIN, 1800);
      delay(40);
      noTone(BUZZER_PIN);
      delay(30);
      tone(BUZZER_PIN, 1800);
      delay(40);
      noTone(BUZZER_PIN);
      delay(30);
      tone(BUZZER_PIN, 1800);
      delay(40);
      noTone(BUZZER_PIN);
      delay(80);
      break;
    case 5: // DISARMED — one medium low tone
      tone(BUZZER_PIN, 1000);
      delay(350);
      noTone(BUZZER_PIN);
      delay(120);
      break;
    case 6: // DESCENT_BYPASS — same as FLIGHT (two quick beeps)
      tone(BUZZER_PIN, 1500);
      delay(60);
      noTone(BUZZER_PIN);
      delay(40);
      tone(BUZZER_PIN, 1500);
      delay(60);
      noTone(BUZZER_PIN);
      delay(80);
      break;
    default:
      break;
  }
  noTone(BUZZER_PIN); // ensure off when we return
}
