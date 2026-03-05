# molea-automation-full

Arduino Uno Q payload system for rocket data collection and soil sampling. This app **combines the functionality** of the standalone apps (RTC, BMP280, IMU, motor, buzzer, pH) in this repository and adds the MOLEA mission state machine and serial test commands.

## Hardware

- **PCF8523 RTC** (I2C 0x68)
- **BMP280** (I2C 0x77) - Altitude and internal payload temperature
- **ICM-20948 IMU** (SPI: D11=MOSI, D12=MISO, D13=SCLK, D0=CS)
- **Pololu DualVNH5019 Motor Driver Shield** - Motor control
- **Buzzer** (D3)
- **EZO pH Carrier Board** (I2C 0x63)

## Pin Configuration

### Motor Driver
- D2 = M1INA, D4 = M1INB, D6 = M1EN, D9 = M1PWM, A0 = M1CS
- D7 = M2INA, D8 = M2INB, **D5 = M2EN** (wire shield D12 to Arduino D5; D12 is IMU MISO)
- D10 = M2PWM, A1 = M2CS
- With separate power (jumper off): Arduino and shield must share GND.

### IMU (SPI)
- D11 = MOSI
- D12 = MISO
- D13 = SCLK
- D0 = CS

### I2C Devices
- BMP280: 0x77
- RTC: 0x68
- EZO pH: 0x63

## Data Output

Sensor data is output every 0.5 seconds to Serial (115200 baud) in the format:
- Timestamp (ms)
- RTC status
- BMP280: Temperature, Altitude
- IMU: Accelerometer (X, Y, Z)
- Mission state

## Mission States

- **PREFLIGHT (0)**: Armed sound, ground detection
- **FLIGHT (1)**: Recording sound, in-air data collection
- **POSTFLIGHT (2)**: Landing detection, motor sequence, pH collection

## Serial Commands

Type `HELP` over the serial monitor for the full list. Commands include: `STATUS`, `TEST RTC` / `TRTC`, `TEST BMP280` / `TBMP`, `TEST IMU` / `TIMU`, `TEST MOTOR` / `TMOT`, `TEST BUZZER` / `TBUZ`, `TEST PH` / `TPH`, `TEST ALL` / `TALL`, `SET YYYY-MM-DD HH:MM:SS` (set RTC), `SETC` (set RTC to compile time).

## Libraries Required

- **Adafruit BMP280 Library** - Install via Arduino Library Manager
- **SparkFun ICM-20948 Arduino Library** - Available in `libraries/` folder or Library Manager
- **Arduino Wire and SPI libraries** - Included with Arduino IDE

## Code Structure

- `sketch/sketch.ino` - Main program with mission state machine
- `sketch/main/config.h` - Pin and configuration definitions
- `sketch/main/rtc_init.*` - RTC (PCF8523) init, read, and set
- `sketch/main/BMP280_init.*` - BMP280 sensor init and read
- `sketch/main/imu_init.*` - ICM-20948 IMU init and read
- `sketch/main/motor_init.*` - Motor driver control
- `sketch/main/buzzer_init.*` - Buzzer control functions
- `sketch/main/pH_init.*` - EZO pH sensor (Atlas I2C) init and read
- `sketch/main/test_components.*` - Serial command parsing and component tests



