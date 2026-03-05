#ifndef IMU_INIT_H
#define IMU_INIT_H

#include <Arduino.h>

struct IMUData {
  float accel_x, accel_y, accel_z;
  float gyro_x, gyro_y, gyro_z;
  bool dataValid;
};

extern bool imu_initialized;

bool initIMU();
bool readIMU(IMUData &data);
bool calibrateIMU(unsigned int samples = 100);

#endif
