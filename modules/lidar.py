import asyncio
import json
import math
import threading
import time
import numpy as np
import websockets
from rplidar import RPLidar
from . import config

class LidarModule:
    def __init__(self, state):
        self.state = state
        self.clients = set()
        self.stop_event = threading.Event()
        self.lidar_obj = None
        self.loop = None
        self.zone_latched = "far"
        self.zone_until = 0.0
        self.HOLD_SEC = 0.25

    async def ws_handler(self, websocket):
        self.clients.add(websocket)
        try:
            await websocket.wait_closed()
        finally:
            self.clients.discard(websocket)

    async def broadcast_scan(self, angles_rad, ranges_m):
        if not self.clients:
            return
        payload = {
            "type": "scan",
            "angles": angles_rad,
            "ranges": ranges_m,
            "range_max": config.LIDAR_MAX_RANGE,
        }
        msg = json.dumps(payload)
        for ws in list(self.clients):
            try:
                await ws.send(msg)
            except Exception:
                pass

    def speed_check(self, distance):
        if distance is None:
            return "far"
        dists = config.LIDAR_AVOID_DISTANCES
        if distance <= dists["close"]:
            return "close"
        elif distance <= dists["middle"]:
            return "middle"
        elif distance <= dists["far"]:
            return "far"
        return "far"

    # This is the main function to check
    def avoid_obstacles(self, angles_list, distances_list):
        if not angles_list:
            return 0.0, 0.0
        x_sum = 0.0
        y_sum = 0.0
        w_sum = 0.0
        for a, r in zip(angles_list, distances_list):
            w = max((2.0 - r) / 2.0, 0.0)
            if w == 0.0:
                continue
            y_sum += w * np.sin(a)
            x_sum += w * np.cos(a)
            w_sum += w
        if w_sum == 0.0:
            return 0.0, 0.0
        return -x_sum / w_sum, -y_sum / w_sum

    def lidar_loop(self):
        print(f"[LIDAR] Connecting to {config.LIDAR_SERIAL_PORT}...")
        try:
            lidar = RPLidar(config.LIDAR_SERIAL_PORT)
            self.lidar_obj = lidar
            lidar.stop_motor()
            time.sleep(1)
            lidar.start_motor()
            print("[LIDAR] Connected. Scanning...")

            for scan in lidar.iter_scans():
                if self.stop_event.is_set():
                    break
                if not scan:
                    continue

                angles_deg = [m[1] for m in scan]
                dists_mm = [m[2] for m in scan]
                angles_rad = []
                ranges_m = []
                check_angles_rad = []
                check_ranges_m = []
                min_rm = None

                for i, (a_deg, d_mm) in enumerate(zip(angles_deg, dists_mm)):
                    #if i % config.LIDAR_DOWNSAMPLE != 0:
                    #    continue
                    r_m = d_mm / 1000.0
                    if r_m <= 0.0 or r_m > config.LIDAR_MAX_RANGE:
                        continue
                    
                    angles_rad.append(math.radians(a_deg))
                    ranges_m.append(r_m)
                    # If distances less than or equal to close get's apppended to check_angles_rad and check_angles_m
                    if r_m <= config.LIDAR_AVOID_DISTANCES["close"]:
                        check_angles_rad.append(math.radians(a_deg))
                        check_ranges_m.append(r_m)

                    # If distance less than far is detected -> minimum r_m, ignore for now
                    #if r_m <= config.LIDAR_AVOID_DISTANCES["far"]:
                    #    if (min_rm is None) or (r_m < min_rm):
                    #        min_rm = r_m

                x_sum, y_sum = self.avoid_obstacles(check_angles_rad, check_ranges_m)
                #zone_now = self.speed_check(min_rm)
                
                self.state.lidar_close = {
                    "angles": check_angles_rad,
                    "ranges": check_ranges_m
                }

                # Update State directly
                #self.state.zone["zone"] = zone_now
                #self.state.zone["x_sum"] = x_sum
                #self.state.zone["y_sum"] = y_sum

                if ranges_m and self.loop:
                    asyncio.run_coroutine_threadsafe(
                        self.broadcast_scan(angles_rad, ranges_m),
                        self.loop
                    )
        except Exception as e:
            print(f"[LIDAR] Error: {e}")
        finally:
            print("[LIDAR] Stopping hardware...")
            try:
                if self.lidar_obj:
                    self.lidar_obj.stop()
                    self.lidar_obj.stop_motor()
                    self.lidar_obj.disconnect()
            except Exception:
                pass

    async def run(self):
        self.loop = asyncio.get_running_loop()
        print(f"[LIDAR] Starting WebSocket server on {config.LIDAR_WS_PORT}")
        
        t = threading.Thread(target=self.lidar_loop, daemon=True)
        t.start()
        
        async with websockets.serve(self.ws_handler, "0.0.0.0", config.LIDAR_WS_PORT):
            try:
                await asyncio.Future()  # Run forever until cancelled
            except asyncio.CancelledError:
                pass
            finally:
                self.stop_event.set()
                t.join(timeout=2.0)