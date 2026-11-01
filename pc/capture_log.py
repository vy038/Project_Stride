"""Grab training logs from the leg. Run 'closed' in the serial monitor first (or pass --start), then close the monitor.

Usage: python capture_log.py /dev/ttyACM0 --runs 5 --amp 0.25 [--start]
Writes log_00.csv, log_01.csv, ... in the current directory. Needs: pip install pyserial
"""
import argparse, sys, time
import serial

ap = argparse.ArgumentParser()
ap.add_argument("port"); ap.add_argument("--runs", type=int, default=5)
ap.add_argument("--amp", type=float, default=0.25); ap.add_argument("--start", action="store_true")
a = ap.parse_args()

s = serial.Serial(a.port, 115200, timeout=8)
time.sleep(1.0); s.reset_input_buffer()

def cmd(c): s.write((c + "\n").encode())

if a.start:
    cmd("closed"); print("calibrating..."); time.sleep(6); s.reset_input_buffer()

for k in range(a.runs):
    cmd(f"log {a.amp}")
    rows, inside = [], False
    while True:
        ln = s.readline().decode(errors="ignore").strip()
        if not ln: sys.exit("timed out. Is the joint in closed loop? ('log needs closed with ctrl pid')")
        if "log needs" in ln: sys.exit(ln)
        if ln == "#LOG_BEGIN": inside = True; continue
        if ln == "#LOG_END": break
        if inside: rows.append(ln)
    name = f"log_{k:02d}.csv"
    open(name, "w").write("\n".join(rows) + "\n")
    print(f"{name}: {len(rows) - 1} samples")
    time.sleep(0.5)
