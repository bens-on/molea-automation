#ifndef CONFIG_H
#define CONFIG_H

/* Set to 1 to enable SD card CSV logging (requires SD library). 0 = log to Monitor/Serial/Bridge only. */
#define USE_SD_LOGGER 0

#define SERIAL_BAUD_RATE 115200
#define DATA_RATE_MS 1000

#define I2C_BMP280_ADDRESS 0x77
#define I2C_RTC_ADDRESS 0x68
#define I2C_EZO_ADDRESS 0x63

#define IMU_SPI_MOSI 11
#define IMU_SPI_MISO 12
#define IMU_SPI_SCLK 13
#define IMU_SPI_CS 0

/* Pololu Dual VNH5019 — direct GPIO. EN pins NOT connected: leave M1EN/M2EN unplugged on shield; drivers stay enabled. */
#define MOTOR_EN_PINS_CONNECTED 0   /* 0 = EN not wired; drivers always enabled. 1 = drive EN pins (LOW = enable). */
#define MOTOR_M1INA 2
#define MOTOR_M1INB 4
#define MOTOR_M1EN 3   /* unused when MOTOR_EN_PINS_CONNECTED==0 */
#define MOTOR_M2EN 1   /* unused when MOTOR_EN_PINS_CONNECTED==0 */
#define MOTOR_M2INA 7
#define MOTOR_M2INB 8
#define MOTOR_M1PWM 9
#define MOTOR_M2PWM 10
#define MOTOR_M1CS A0
#define MOTOR_M2CS A1
/* Current sense: ~34 mA per ADC step (VNH5019 0.14 V/A). Above this = possible stall/overload. */
#define MOTOR_OVERCURRENT_MA 2500

#define BUZZER_PIN 6   /* D6 so D3 is dedicated to M1 enable (no pin sharing) */

/* LANDING_CONFIRM: M1 leg leveling run duration in MINUTES. Override at runtime via LEG_TIME command (1–120 min). */
#ifndef LEG_TIME_MIN_DEFAULT
#define LEG_TIME_MIN_DEFAULT 10
#endif

#endif
