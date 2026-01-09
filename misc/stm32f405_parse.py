#!/usr/bin/env python3
"""
Intake STM32F405 JSON messages and parse into:
- Wheel velocities (per wheel)
- IMU orientation/gyro

Also forwards a compact wheel payload over UDP.
"""

import serial
import json
import time
import socket

# ----- SERIAL (STM32) CONFIG -----
SERIAL_PORT = "/dev/ttyACM0"   # your STM32 USB CDC device
SERIAL_BAUD = 115200

# ----- UDP CONFIG -----
UDP_IP = "0.0.0.0"           # change to target IP if sending to another machine
UDP_PORT = 5678
SEND_HZ = 100
MIN_DT = 1.0 / SEND_HZ

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.setblocking(False)


def main():
    print(f"Opening {SERIAL_PORT} at {SERIAL_BAUD} baud...")
    ser = serial.Serial(SERIAL_PORT, SERIAL_BAUD, timeout=1)

    # Give the STM32 some time after opening the port
    time.sleep(2.0)

    last_send = 0.0  # local to main, avoids global weirdness

    while True:
        try:
            line = ser.readline()
            if not line:
                continue

            line = line.decode("utf-8", errors="ignore").strip()
            if not line:
                continue

            # Ignore non-JSON noise (optional)
            if not line.startswith("{"):
                # print("Raw non-json:", repr(line))  # uncomment if debugging
                continue

            data = json.loads(line)

            # Extract fields safely
            t_ms     = data.get("t")
            w1_ticks = data.get("w1_ticks")
            w2_ticks = data.get("w2_ticks")

            w1_w = data.get("w1_w")
            w1_v = data.get("w1_v")
            w2_w = data.get("w2_w")
            w2_v = data.get("w2_v")

            yaw   = data.get("yaw")
            pitch = data.get("pitch")
            roll  = data.get("roll")

            gx = data.get("gx")
            gy = data.get("gy")
            gz = data.get("gz")

            # Optional: skip if any critical values are missing
            if None in (t_ms, w1_ticks, w2_ticks, w1_w, w1_v, w2_w, w2_v, yaw, pitch, roll, gx, gy, gz):
                # malformed / partial JSON, ignore
                continue

            # Print a compact status line
            print(
                f"t={t_ms:6d} | "
                f"W1: ticks={w1_ticks:6d} w={w1_w:7.3f} v={w1_v:6.3f} | "
                f"W2: ticks={w2_ticks:6d} w={w2_w:7.3f} v={w2_v:6.3f} | "
                f"Yaw={yaw:7.2f} Pitch={pitch:7.2f} Roll={roll:7.2f} | "
                f"Gyro=({gx:6.3f},{gy:6.3f},{gz:6.3f})"
            )

            # ---- UDP send of wheel info ----
            now = time.time()
            if now - last_send >= MIN_DT:
                last_send = now
                # Payload format: w1_v w1_w w2_v w2_w
                payload = f"{w1_v} {w1_w} {w2_v} {w2_w}"
                sock.sendto(payload.encode("utf-8"), (UDP_IP, UDP_PORT))
            # Need to calculate robot velocity from calling motor functions.py (need 4 wheels to read tho)    
        except json.JSONDecodeError as e:
            print("JSON error:", e, " line:", repr(line))
        except KeyboardInterrupt:
            print("\nExiting...")
            break
        except Exception as e:
            print("Unexpected error:", e)
            time.sleep(0.1)


if __name__ == "__main__":
    main()
