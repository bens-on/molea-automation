// ICM-20948 driver using raw SPI (same approach as working imu app).
// SparkFun library is incompatible with Arduino_RouterBridge.

#include "imu_init.h"
#include "config.h"
#include <SPI.h>
#include <Arduino_RouterBridge.h>

// ICM-20948 register addresses (Bank 0)
#define ICM_REG_BANK_SEL  0x7F
#define ICM_WHO_AM_I      0x00
#define ICM_PWR_MGMT_1    0x06
#define ICM_PWR_MGMT_2    0x07
#define ICM_ACCEL_XOUT_H  0x2D
#define ICM_GYRO_XOUT_H   0x33
#define ICM_WHO_AM_I_VAL  0xEA

// Sensitivity: ±2g -> 16384 LSB/g, ±250 dps -> 131 LSB/dps
#define ACCEL_SENSITIVITY  16384.0f
#define GYRO_SENSITIVITY   131.0f

#define ICM_SPI_SETTINGS   SPISettings(1000000, MSBFIRST, SPI_MODE3)

static float offset_accel_x = 0.0f, offset_accel_y = 0.0f, offset_accel_z = 0.0f;
static float offset_gyro_x = 0.0f, offset_gyro_y = 0.0f, offset_gyro_z = 0.0f;
static bool imu_calibrated = false;

bool imu_initialized = false;

static void icmSelectBank(uint8_t bank) {
  digitalWrite(IMU_SPI_CS, LOW);
  delayMicroseconds(5);
  SPI.transfer(ICM_REG_BANK_SEL & 0x7F);
  SPI.transfer((bank & 0x03) << 4);
  digitalWrite(IMU_SPI_CS, HIGH);
  delayMicroseconds(5);
}

static uint8_t icmReadReg(uint8_t reg) {
  digitalWrite(IMU_SPI_CS, LOW);
  delayMicroseconds(5);
  SPI.transfer(reg | 0x80);
  uint8_t val = SPI.transfer(0x00);
  digitalWrite(IMU_SPI_CS, HIGH);
  delayMicroseconds(5);
  return val;
}

static void icmWriteReg(uint8_t reg, uint8_t val) {
  digitalWrite(IMU_SPI_CS, LOW);
  delayMicroseconds(5);
  SPI.transfer(reg & 0x7F);
  SPI.transfer(val);
  digitalWrite(IMU_SPI_CS, HIGH);
  delayMicroseconds(5);
}

static void icmReadBytes(uint8_t reg, uint8_t* buf, uint8_t len) {
  digitalWrite(IMU_SPI_CS, LOW);
  delayMicroseconds(5);
  SPI.transfer(reg | 0x80);
  for (uint8_t i = 0; i < len; i++) {
    buf[i] = SPI.transfer(0x00);
  }
  digitalWrite(IMU_SPI_CS, HIGH);
  delayMicroseconds(5);
}

static int16_t combineBytes(uint8_t h, uint8_t l) {
  return (int16_t)((h << 8) | l);
}

bool initIMU() {
  SPI.begin();
  pinMode(IMU_SPI_CS, OUTPUT);
  digitalWrite(IMU_SPI_CS, HIGH);
  delay(100);

  SPI.beginTransaction(ICM_SPI_SETTINGS);
  icmSelectBank(0);
  delay(10);
  uint8_t whoami = icmReadReg(ICM_WHO_AM_I);
  SPI.endTransaction();

  if (whoami != ICM_WHO_AM_I_VAL) {
    Monitor.println(F("Warning: IMU initialization failed (WHO_AM_I)"));
    imu_initialized = false;
    return false;
  }

  SPI.beginTransaction(ICM_SPI_SETTINGS);
  icmWriteReg(ICM_PWR_MGMT_1, 0x80);
  delay(100);
  icmWriteReg(ICM_PWR_MGMT_1, 0x01);
  delay(50);
  icmWriteReg(ICM_PWR_MGMT_2, 0x00);
  delay(50);
  /* Leave transaction open (no endTransaction) - matches working imu app so SPI stays 1MHz/MODE3 */

  Monitor.println(F("IMU initialized"));
  imu_initialized = true;
  return true;
}

bool readIMU(IMUData &data) {
  if (!imu_initialized) {
    data.dataValid = false;
    return false;
  }

  /* Single CS-low burst read from 0x2D: 12 bytes = accel (6) + gyro (6), register auto-increment */
  uint8_t raw[12];
  icmSelectBank(0);
  digitalWrite(IMU_SPI_CS, LOW);
  delayMicroseconds(5);
  SPI.transfer(ICM_ACCEL_XOUT_H | 0x80);
  for (uint8_t i = 0; i < 12; i++) {
    raw[i] = SPI.transfer(0x00);
  }
  digitalWrite(IMU_SPI_CS, HIGH);
  delayMicroseconds(5);

  int16_t raw_ax = combineBytes(raw[0], raw[1]);
  int16_t raw_ay = combineBytes(raw[2], raw[3]);
  int16_t raw_az = combineBytes(raw[4], raw[5]);
  int16_t raw_gx = combineBytes(raw[6], raw[7]);
  int16_t raw_gy = combineBytes(raw[8], raw[9]);
  int16_t raw_gz = combineBytes(raw[10], raw[11]);

  // Accel: LSB/g -> m/s² ( * 9.81f )
  float ax = (raw_ax / ACCEL_SENSITIVITY) * 9.81f;
  float ay = (raw_ay / ACCEL_SENSITIVITY) * 9.81f;
  float az = (raw_az / ACCEL_SENSITIVITY) * 9.81f;
  // Gyro: LSB/dps -> rad/s ( * PI/180 )
  const float dpsToRad = 3.14159265f / 180.0f;
  float gx = (raw_gx / GYRO_SENSITIVITY) * dpsToRad;
  float gy = (raw_gy / GYRO_SENSITIVITY) * dpsToRad;
  float gz = (raw_gz / GYRO_SENSITIVITY) * dpsToRad;

  data.accel_x = imu_calibrated ? (ax - offset_accel_x) : ax;
  data.accel_y = imu_calibrated ? (ay - offset_accel_y) : ay;
  data.accel_z = imu_calibrated ? (az - offset_accel_z) : az;
  data.gyro_x = imu_calibrated ? (gx - offset_gyro_x) : gx;
  data.gyro_y = imu_calibrated ? (gy - offset_gyro_y) : gy;
  data.gyro_z = imu_calibrated ? (gz - offset_gyro_z) : gz;
  data.dataValid = true;
  return true;
}

bool calibrateIMU(unsigned int samples) {
  if (!imu_initialized || samples == 0) return false;
  float sum_ax = 0.0f, sum_ay = 0.0f, sum_az = 0.0f;
  float sum_gx = 0.0f, sum_gy = 0.0f, sum_gz = 0.0f;
  const float dpsToRad = 3.14159265f / 180.0f;

  for (unsigned int i = 0; i < samples; i++) {
    uint8_t raw[12];
    icmSelectBank(0);
    digitalWrite(IMU_SPI_CS, LOW);
    delayMicroseconds(5);
    SPI.transfer(ICM_ACCEL_XOUT_H | 0x80);
    for (uint8_t j = 0; j < 12; j++) raw[j] = SPI.transfer(0x00);
    digitalWrite(IMU_SPI_CS, HIGH);
    delayMicroseconds(5);

    int16_t raw_ax = combineBytes(raw[0], raw[1]);
    int16_t raw_ay = combineBytes(raw[2], raw[3]);
    int16_t raw_az = combineBytes(raw[4], raw[5]);
    int16_t raw_gx = combineBytes(raw[6], raw[7]);
    int16_t raw_gy = combineBytes(raw[8], raw[9]);
    int16_t raw_gz = combineBytes(raw[10], raw[11]);

    sum_ax += (raw_ax / ACCEL_SENSITIVITY) * 9.81f;
    sum_ay += (raw_ay / ACCEL_SENSITIVITY) * 9.81f;
    sum_az += (raw_az / ACCEL_SENSITIVITY) * 9.81f;
    sum_gx += (raw_gx / GYRO_SENSITIVITY) * dpsToRad;
    sum_gy += (raw_gy / GYRO_SENSITIVITY) * dpsToRad;
    sum_gz += (raw_gz / GYRO_SENSITIVITY) * dpsToRad;
    delay(20);
  }

  offset_accel_x = sum_ax / (float)samples;
  offset_accel_y = sum_ay / (float)samples;
  offset_accel_z = sum_az / (float)samples;
  offset_gyro_x = sum_gx / (float)samples;
  offset_gyro_y = sum_gy / (float)samples;
  offset_gyro_z = sum_gz / (float)samples;
  imu_calibrated = true;
  Monitor.println(F("IMU calibration done (offsets applied)"));
  return true;
}
