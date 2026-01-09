#!/usr/bin/env python3

# Turn this script into my zone sensor so i just need to interpret one thing from the payload
import asyncio
import json
import math
import threading
import numpy as np
from rplidar import RPLidar
import websockets
import time
import signal
import socket
stop_event = threading.Event()
# ---------- CONFIG ----------
LIDAR_PORT = "/dev/ttyUSB0"   # adjust if needed
WS_PORT = 8765
MAX_RANGE_METERS = 10.0
DOWNSAMPLE = .1                # take every Nth point to keep payload smaller
# ----------------------------

communication_port = "0.0.0.0"
sender_port = 6065
zone_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

clients = set()
event_loop = None  # asyncio loop used by websocket server

# ----- GLOBAL ZONE SETTER -------
speed_zone_parameter = 1.0
avoid_distances = {
    "close":2.0,
    "middle":3.5,
    "far":4.5
}
avoid_direction = "none"
min_rm = 0
HOLD_SEC = 0.25
lidar_obj = None


async def ws_handler(websocket):
    print("Client connected")
    clients.add(websocket)
    try:
        async for _ in websocket:
            pass
    finally:
        print("Client disconnected")
        clients.discard(websocket)


async def broadcast_scan(angles_rad, ranges_m):
    if not clients:
        return

    payload = {
        "type": "scan",
        "angles": angles_rad,   # list of radians, like your matplotlib script
        "ranges": ranges_m,     # list of meters
        "range_max": MAX_RANGE_METERS,
    }
    msg = json.dumps(payload)
    dead = set()

    for ws in list(clients):
        try:
            await ws.send(msg)
        except Exception as e:
            print("send failed, dropping client:", e)
            dead.add(ws)

    for ws in dead:
        clients.discard(ws)
def speed_check(distance):
        if distance is None:
            return "far"
        elif distance <= avoid_distances["close"]:
            return "close"
        elif distance <= avoid_distances["middle"]:
            return "middle"
        elif distance <= avoid_distances["far"]:
            return "far"
        return "far"

def avoid_obstacles(angles_list, distances_list):
    if not angles_list:
        return 0.0, 0.0

    x_sum = 0.0
    y_sum = 0.0
    w_sum = 0.0

    for a, r in zip(angles_list, distances_list):
        w = max((2.0 - r) / 2.0, 0.0)  # clamp negative
        if w == 0.0:
            continue
        y_sum += w * np.sin(a)
        x_sum += w * np.cos(a)
        w_sum += w

    if w_sum == 0.0:
        return 0.0, 0.0

    return -x_sum / w_sum, -y_sum / w_sum


    # for all points within range 0 - 2.0
    # angles are based on polar coordinates with angles and distances
    # r, theta
    # (tan(angle) * r/rmax)
    # for i in [array]:
        # from center - point
        # vectorx 
        # vectory 
        # normalize vector magnitude
        # summ all vectors to get general direction vector


def lidar_loop():
    """Blocking RPLidar loop running in its own thread."""
    global event_loop
    assert event_loop is not None
    global lidar_obj

    print(f"Connecting to LiDAR on {LIDAR_PORT}...")
    lidar = RPLidar(LIDAR_PORT)
    lidar_obj = lidar
    lidar.stop_motor()
    time.sleep(3)
    lidar.start_motor()
    print("LiDAR connected. Starting scans...")

    try:
        for scan in lidar.iter_scans():
            if stop_event.is_set():
                break
            if not scan:
                continue

            # Same idea as your matplotlib script: use raw angles + distances
            angles_deg = [m[1] for m in scan]
            dists_mm   = [m[2] for m in scan]

            angles_rad = []
            ranges_m   = []
            check_angles_rad = []
            check_ranges_m = []
            min_rm = None
            for i, (a_deg, d_mm) in enumerate(zip(angles_deg, dists_mm)):
                if i % DOWNSAMPLE != 0:
                    continue

                r_m = d_mm / 1000.0
                if r_m <= 0.0 or r_m > MAX_RANGE_METERS:
                    continue
                angles_rad.append(math.radians(a_deg))
                ranges_m.append(r_m)
                if r_m <= avoid_distances["close"]:
                    check_angles_rad.append(math.radians(a_deg))
                    check_ranges_m.append(r_m)
                if r_m <= avoid_distances["far"]:
                    if (min_rm is None) or (r_m < min_rm):
                        min_rm = r_m
                        #print(min_rm)
            x_sum,y_sum = avoid_obstacles(check_angles_rad,check_ranges_m)
            zone_now = speed_check(min_rm)
            now = time.monotonic()
            if zone_now != "far":
                zone_latched = zone_now
                zone_until = now + HOLD_SEC
            elif now > zone_until:
                zone_latched = "far"
            print (zone_now)
            payload = {
                "zone":zone_now,
                "x_sum":x_sum,
                "y_sum":y_sum
            }
            zone_sock.sendto(json.dumps(payload).encode("utf-8"), (communication_port, sender_port))
            if not ranges_m:
                continue
            # schedule websocket send on asyncio loop
            asyncio.run_coroutine_threadsafe(
                broadcast_scan(angles_rad, ranges_m),
                event_loop,
            )
    except Exception as e:
        print("lidar threading error:", repr(e))
    finally:
        print("Stopping LiDAR, closing port...")
        try:
            lidar.stop()
            lidar.stop_motor()
            lidar.disconnect()
        except Exception: pass
        try:
            lidar.stop_motor()
        except Exception: pass
        lidar_obj = None


async def main():
    global event_loop
    event_loop = asyncio.get_running_loop()

    server = await websockets.serve(ws_handler, "0.0.0.0", WS_PORT)
    print(f"WebSocket server listening on ws://0.0.0.0:{WS_PORT}")

    t = threading.Thread(target=lidar_loop, daemon=True)
    t.start()
    def request_shutdown():
        print("nShutdown requested")
        stop_event.set()
        try:
            if lidar_obj is not None:
                lidar_obj.stop()
        except Exception: pass
    for sig in (signal.SIGINT, signal.SIGTERM):
        event_loop.add_signal_handler(sig, request_shutdown)
    try:
        while not stop_event.is_set():
            await asyncio.sleep(0.1)
    finally:
        server.close()
        await server.wait_closed()
        t.join(timeout=2.0)
        print("shutdown complete")
    await asyncio.Future()  # run forever


if __name__ == "__main__":
    try:
        asyncio.run(main())
    except Exception as error:
        print(error)
