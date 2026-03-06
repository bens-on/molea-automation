#ifndef BMP280_INIT_H
#define BMP280_INIT_H

#include <Arduino.h>

struct BaroData {
  float temperature;
  float pressure;
  float altitude;
  bool dataValid;
};

extern bool bmp280_initialized;

bool initBMP280();
bool readBMP280(BaroData &data);
bool calibrateBMP280AtGround(unsigned int samples = 5);

#endif
