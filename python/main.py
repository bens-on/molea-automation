#!/usr/bin/env python3
"""
molea-automation-full - Bridge Logger (Uno Q App Lab)

Receives log lines from the MCU via:
  Bridge.notify("log_line", "...")

and saves them into:
  sketch/data/molea_flight_YYYYMMDD_HHMMSS.csv
"""

from arduino.app_utils import *
import os
import time
from datetime import datetime

# Where you want the file to appear in the left sidebar
DATA_DIR = os.path.join("sketch", "data")

# Make sure folder exists
os.makedirs(DATA_DIR, exist_ok=True)

# Create a new file each run
ts = datetime.now().strftime("%Y%m%d_%H%M%S")
LOG_PATH = os.path.join(DATA_DIR, f"molea_flight_{ts}.csv")

# Open once, keep it open (fast + reliable)
log_file = open(LOG_PATH, "w", buffering=1)  # line-buffered
print(f"[logger] Writing to: {LOG_PATH}")

# Optional header (your data lines are already CSV-like, but this helps)
log_file.write("raw_line\n")

# This runs whenever the MCU does Bridge.notify("log_line", something)
def on_log_line(line):
    try:
        if line is None:
            return

        # Ensure string
        s = str(line)

        # Print to python console (so you see it live)
        print(s)

        # Write to file
        log_file.write(s.replace("\n", " ") + "\n")

    except Exception as e:
        # Never crash the logger
        print(f"[logger] error in on_log_line: {e}")

# Subscribe to notifications from the sketch
Bridge.provide("log_line", on_log_line)

def loop():
    # Keep app alive; all work happens in callback above
    time.sleep(0.2)

App.run(user_loop=loop)
