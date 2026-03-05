#!/usr/bin/env python3
import os
import re
from datetime import datetime

import matplotlib.pyplot as plt

# Use command: python3 python/plot_logs.py
# To output a graph to the "Logged" folder

# ----------- SETTINGS -----------
DATA_DIR = "sketch/data"
OUT_DIR  = "sketch/Logged"
# -------------------------------

# Your lines look like (accel only or accel+gyro):
# 5807,2026-02-19 18:15:47,T=23.62 A=-0.20,AX=-0.021 AY=-0.111 AZ=0.006,PH_INIT_FAIL,PREFLIGHT
# 5807,2026-02-19 18:15:47,T=23.62 A=-0.20,AX=-0.021 AY=-0.111 AZ=0.006 GX=0.001 GY=-0.002 GZ=0.000,PH_INIT_FAIL,PREFLIGHT
LINE_RE = re.compile(
    r"^\s*(?P<ms>\d+)\s*,\s*(?P<dt>\d{4}-\d{2}-\d{2}\s+\d{2}:\d{2}:\d{2})\s*,.*?"
    r"AX=(?P<ax>[-+]?\d*\.?\d+)\s+AY=(?P<ay>[-+]?\d*\.?\d+)\s+AZ=(?P<az>[-+]?\d*\.?\d+)"
    r"(?:\s+GX=(?P<gx>[-+]?\d*\.?\d+)\s+GY=(?P<gy>[-+]?\d*\.?\d+)\s+GZ=(?P<gz>[-+]?\d*\.?\d+))?",
    re.IGNORECASE
)

def newest_file(folder: str):
    files = []
    for name in os.listdir(folder):
        path = os.path.join(folder, name)
        if os.path.isfile(path):
            files.append(path)
    if not files:
        return None
    return max(files, key=lambda p: os.path.getmtime(p))

def parse_log_file(path: str):
    t_sec = []
    ax = []
    ay = []
    az = []
    gx = []
    gy = []
    gz = []

    with open(path, "r", errors="ignore") as f:
        for line in f:
            m = LINE_RE.search(line)
            if not m:
                continue
            ms = int(m.group("ms"))
            t_sec.append(ms / 1000.0)
            ax.append(float(m.group("ax")))
            ay.append(float(m.group("ay")))
            az.append(float(m.group("az")))
            gx_g = m.group("gx")
            gy_g = m.group("gy")
            gz_g = m.group("gz")
            if gx_g is not None and gy_g is not None and gz_g is not None:
                gx.append(float(gx_g))
                gy.append(float(gy_g))
                gz.append(float(gz_g))

    return t_sec, ax, ay, az, gx, gy, gz

def main():
    os.makedirs(OUT_DIR, exist_ok=True)

    log_path = newest_file(DATA_DIR)
    if not log_path:
        raise SystemExit(f"No log files found in {DATA_DIR}")

    t_sec, ax, ay, az, gx, gy, gz = parse_log_file(log_path)

    if len(t_sec) < 5:
        raise SystemExit(
            f"Parsed too few IMU points ({len(t_sec)}). "
            "Check the file format / that AX AY AZ appear in the log."
        )

    stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    base = os.path.splitext(os.path.basename(log_path))[0]

    # --- Accel plot ---
    plt.figure()
    plt.plot(t_sec, ax, label="AX")
    plt.plot(t_sec, ay, label="AY")
    plt.plot(t_sec, az, label="AZ")
    plt.xlabel("Time (s)")
    plt.ylabel("Acceleration (m/s²)")
    plt.title("IMU Accel XYZ vs Time")
    plt.legend()
    plt.grid(True)
    out_path = os.path.join(OUT_DIR, f"{base}_imu_xyz_{stamp}.png")
    plt.savefig(out_path, dpi=200, bbox_inches="tight")
    print(f"Saved plot -> {out_path}")

    # --- Gyro plot (when present) ---
    if len(gx) == len(t_sec) and len(gx) >= 5:
        plt.figure()
        plt.plot(t_sec, gx, label="GX")
        plt.plot(t_sec, gy, label="GY")
        plt.plot(t_sec, gz, label="GZ")
        plt.xlabel("Time (s)")
        plt.ylabel("Angular velocity (rad/s)")
        plt.title("IMU Gyro XYZ vs Time")
        plt.legend()
        plt.grid(True)
        gyro_path = os.path.join(OUT_DIR, f"{base}_imu_gyro_{stamp}.png")
        plt.savefig(gyro_path, dpi=200, bbox_inches="tight")
        print(f"Saved plot -> {gyro_path}")

if __name__ == "__main__":
    main()