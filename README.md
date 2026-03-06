# molea-automation-full

Arduino Uno Q payload system for rocket data collection and soil sampling. This app **combines the functionality** of the standalone apps (RTC, BMP280, IMU, motor, buzzer, pH) in this repository and adds the MOLEA mission state machine and serial test commands.

## Hardware

- **PCF8523 RTC** (I2C 0x68)
- **BMP280** (I2C 0x77) - Altitude and internal payload temperature
- **ICM-20948 IMU** (SPI: D11=MOSI, D12=MISO, D13=SCLK, D0=CS)
- **Pololu DualVNH5019 Motor Driver Shield** - Motor control
- **Buzzer** (D6)
- **EZO pH Carrier Board** (I2C 0x63)

## Pin Configuration

### Motor Driver
- D2 = M1INA, D4 = M1INB, D1 = M1EN, D9 = M1PWM, A0 = M1CS
- D7 = M2INA, D8 = M2INB, **D3 = M2EN** (wire shield D12 to Arduino D5; D12 is IMU MISO)
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
- IMU: Gyroscope (x, Y, Z)
- IMU Orientation Flag
- pH Output
- Mission state
- Motor Current Sensing
- Motor OK Flag

## Mission States

- **PREFLIGHT (0)**: Armed sound, ground detection
- **FLIGHT (1)**: Recording sound, in-air data collection
- **LANDING_CONFIRM (2)**: Landing detection, motor sequence, pH collection
- **DRILLING (3)**: Actuation of lead screw mechanism. M1 is actuation motor, M2 os the auger motor.
- **DISARMED (4)**: All motors are cutoff from the system. Data continuously collected until power off.

## Serial Commands

Type `HELP` over the serial monitor for the full list. 

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

## Directory Tree

molea-integration
├── app.yaml
├── examples
│   └── arduino_UNO_pH_sample_code.ino
├── python
│   ├── main.py
│   ├── plot_logs.py
│   └── requirements.txt
├── README.md
└── sketch
    ├── include
    │   ├── BMP280_init.h
    │   ├── buzzer_init.h
    │   ├── config.h
    │   ├── debug.h
    │   ├── imu_init.h
    │   ├── motor_init.h
    │   ├── pgmspace.h
    │   ├── pH_init.h
    │   ├── rtc_init.h
    │   ├── sd_logger.h
    │   ├── storage_init.h
    │   └── test_components.h
    ├── main
    │   ├── BMP280_init.cpp
    │   ├── buzzer_init.cpp
    │   ├── imu_init.cpp
    │   ├── motor_init.cpp
    │   ├── MOTORS_AND_CURRENT_SENSE.md
    │   ├── pH_init.cpp
    │   ├── rtc_init.cpp
    │   ├── sd_logger.cpp
    │   ├── storage_init.cpp
    │   └── test_components.cpp
    ├── sketch.ino
    └── sketch.yaml

