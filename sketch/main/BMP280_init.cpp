#include "BMP280_init.h"
#include "config.h"
#include <Wire.h>
#include <Adafruit_BMP280.h>

#define SEA_LEVEL_PRESSURE 1013.25

Adafruit_BMP280 bmp;
static float reference_altitude_m = 0.0f;
static bool reference_altitude_set = false;

bool bmp280_initialized = false;

bool initBMP280() {
  if (!bmp.begin(I2C_BMP280_ADDRESS)) {
    Monitor.println(F("Warning: BMP280 initialization failed"));
    bmp280_initialized = false;
    return false;
  }
  
  bmp.setSampling(
    Adafruit_BMP280::MODE_NORMAL,
    Adafruit_BMP280::SAMPLING_X2,
    Adafruit_BMP280::SAMPLING_X16,
    Adafruit_BMP280::FILTER_X16,
    Adafruit_BMP280::STANDBY_MS_500
  );
  
  Monitor.println(F("BMP280 initialized"));
  bmp280_initialized = true;
  return true;
}

bool readBMP280(BaroData &data) {
  if (!bmp280_initialized) {
    data.dataValid = false;
    return false;
  }
  
  float temperature = bmp.readTemperature();
  float pressure = bmp.readPressure() / 100.0F;
  
  if (isnan(temperature) || isnan(pressure) ||
      temperature < -40 || temperature > 85 ||
      pressure < 300 || pressure > 1100) {
    data.dataValid = false;
    return false;
  }
  
  data.temperature = temperature;
  data.pressure = pressure;
  float abs_alt = 44330.0 * (1.0 - pow(pressure / SEA_LEVEL_PRESSURE, 0.1903));
  data.altitude = reference_altitude_set ? (abs_alt - reference_altitude_m) : abs_alt;
  data.dataValid = true;
  return true;
}

bool calibrateBMP280AtGround(unsigned int samples) {
  if (!bmp280_initialized || samples == 0) return false;
  float sum = 0.0f;
  unsigned int valid = 0;
  for (unsigned int i = 0; i < samples; i++) {
    float temperature = bmp.readTemperature();
    float pressure = bmp.readPressure() / 100.0F;
    if (isnan(temperature) || isnan(pressure) || pressure < 300 || pressure > 1100) continue;
    sum += 44330.0 * (1.0 - pow(pressure / SEA_LEVEL_PRESSURE, 0.1903));
    valid++;
    delay(50);
  }
  if (valid == 0) return false;
  reference_altitude_m = sum / (float)valid;
  reference_altitude_set = true;
  char msg[64];
  snprintf(msg, sizeof(msg), "BMP280 reference altitude set: %.2f m", reference_altitude_m);
  Monitor.println(msg);
  return true;
}
