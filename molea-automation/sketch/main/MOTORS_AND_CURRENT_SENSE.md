# Motor data and current-sense fault detection

## Data availability

Motor information is included in every data row that is:

- Printed to the **Serial Monitor** (and Bridge)
- Written to the **SD log file** (when SD card is initialized)

Each row is emitted at `DATA_RATE_MS` (default 1 s) and contains the same CSV fields.

## CSV fields (motor columns)

After the existing columns (millis, rtc, bmp280, imu, ph, state), each line includes:

| Column      | Meaning                          | Example  |
|------------|-----------------------------------|----------|
| `M1`       | Motor 1 speed (-400..400)         | 200      |
| `M2`       | Motor 2 speed (-400..400)         | -400     |
| `pos_mm`   | M1 position in mm (0–73.6)        | 12.5     |
| `M1_mA`    | Motor 1 current sense (mA)        | 450      |
| `M2_mA`    | Motor 2 current sense (mA)        | 120      |
| `motor_fault` | Fault status (see below)       | OK       |

## Current sensing (hardware)

- **Shield:** Pololu Dual VNH5019.
- **Pins:** M1 current sense = Arduino **A0** (M1CS), M2 current sense = **A1** (M2CS).
- **Scaling:** The VNH5019 CS output is ~0.14 V/A. With a 5 V ADC reference,  
  **current (mA) ≈ analogRead(CS) × 34**.

So the logged `M1_mA` and `M2_mA` values are derived from the ADC using this scaling.

## Fault detection

- **Threshold:** `MOTOR_OVERCURRENT_MA` in `config.h` (default 2500 mA = 2.5 A).
- **Logic:** If M1 current ≥ threshold → fault includes `M1_OVER`. If M2 current ≥ threshold → fault includes `M2_OVER`.
- **`motor_fault` values:**
  - `OK` — both channels below threshold.
  - `M1_OVER` — M1 at or above overcurrent threshold.
  - `M2_OVER` — M2 at or above overcurrent threshold.
  - `M1_M2_OVER` — both at or above threshold.

High current when the motor is commanded can indicate:

- **Stall** (e.g. auger blocked).
- **Mechanical overload.**
- **Short or driver fault.**

The code does **not** auto-stop the motor on overcurrent; it only reports it in the data and fault field. You can use the logged `M1_mA`, `M2_mA`, and `motor_fault` for analysis and to add automatic shutdown in code if desired.

## API for logging

- `motor_getM1Speed()` / `motor_getM2Speed()` — commanded speed (-400..400).
- `motor_getPositionMm()` — M1 position (mm).
- `motor_getM1CurrentMa()` / `motor_getM2CurrentMa()` — current (mA) from CS pins.
- `motor_getFaultStatus(buf, len)` — writes `OK` or overcurrent code into `buf`.

## SD log file

- When SD logging is initialized, the same CSV line (including motor columns) is written to the log file via `sd_logger_logLine()`.
- The file header row includes: `...,M1,M2,pos_mm,M1_mA,M2_mA,motor_fault`.
