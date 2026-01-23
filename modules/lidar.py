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

        # ---- RATE METRICS ----
        self._rate_lock = threading.Lock()
        self._rx_scans = 0          # how many scans we received from iter_scans()
        self._rx_points = 0         # total points received from iter_scans()
        self._tx_scans = 0          # how many scans we attempted to broadcast (state/ws)
        self._t0 = time.monotonic() # window start time
        # ----------------------

    async def ws_handler(self, websocket):
        self.clients.add(websocket)
        try:
            await websocket.wait_closed()
        finally:
            self.clients.discard(websocket)

    async def broadcast_scan(self, angles_rad, ranges_m, intensity):
        if not self.clients:
            return
        payload = {
            "type": "scan",
            "angles": angles_rad,
            "ranges": ranges_m,
            "intensity": intensity,
            "range_max": config.LIDAR_MAX_RANGE,
        }
        msg = json.dumps(payload)
        for ws in list(self.clients):
            try:
                await ws.send(msg)
            except Exception:
                pass

    def _rate_tick(self):
        """Print intake/broadcast rates every ~1s."""
        now = time.monotonic()
        with self._rate_lock:
            dt = now - self._t0
            if dt < 1.0:
                return

            rx_scans = self._rx_scans
            rx_points = self._rx_points
            tx_scans = self._tx_scans

            rx_hz = rx_scans / dt
            tx_hz = tx_scans / dt
            pts_per_scan = (rx_points / rx_scans) if rx_scans > 0 else 0.0
            pts_per_sec = rx_points / dt
            '''
            print(
                f"[LIDAR RATE] intake={rx_hz:.2f} scans/s | "
                f"points={pts_per_sec:.0f} pts/s | "
                f"avg_pts/scan={pts_per_scan:.0f} | "
                f"broadcast_attempt={tx_hz:.2f} scans/s | "
                f"clients={len(self.clients)}"
            )
            '''
            # reset window
            self._rx_scans = 0
            self._rx_points = 0
            self._tx_scans = 0
            self._t0 = now

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
        while not self.stop_event.is_set():
            try:
                print(f"[LIDAR] Connecting to {config.LIDAR_SERIAL_PORT}...")
                lidar = RPLidar(config.LIDAR_SERIAL_PORT)
                self.lidar_obj = lidar
                lidar.start_motor()
                print("[LIDAR] Connected. Scanning...")

                # Reset rate window when we start scanning
                with self._rate_lock:
                    self._rx_scans = 0
                    self._rx_points = 0
                    self._tx_scans = 0
                    self._t0 = time.monotonic()

                for scan in lidar.iter_scans():
                    if self.stop_event.is_set():
                        break
                    if not scan:
                        continue

                    # ---- intake counters (from hardware/driver) ----
                    with self._rate_lock:
                        self._rx_scans += 1
                        self._rx_points += len(scan)
                    # -----------------------------------------------

                    intensity = [m[0] for m in scan]
                    angles_deg = [m[1] for m in scan]
                    dists_mm = [m[2] for m in scan]
                    angles_rad = []
                    ranges_m = []
                    check_angles_rad = []
                    check_ranges_m = []

                    for a_deg, d_mm in zip(angles_deg, dists_mm):
                        r_m = d_mm / 1000.0
                        if r_m <= 0.0 or r_m > config.LIDAR_MAX_RANGE:
                            continue

                        a_rad = math.radians(a_deg)
                        angles_rad.append(a_rad)
                        ranges_m.append(r_m)
                        if r_m <= config.LIDAR_AVOID_DISTANCES["close"]:
                            check_angles_rad.append(a_rad)
                            check_ranges_m.append(r_m)

                    x_sum, y_sum = self.avoid_obstacles(check_angles_rad, check_ranges_m)

                    self.state.lidar_close = {"angles": check_angles_rad, "ranges": check_ranges_m}

                    if ranges_m and self.loop:
                        # ---- broadcast attempt counter ----
                        with self._rate_lock:
                            self._tx_scans += 1
                        # -----------------------------------

                        # Websocket broadcast
                        asyncio.run_coroutine_threadsafe(
                            self.broadcast_scan(angles_rad, ranges_m, intensity),
                            self.loop
                        )

                        # Update state with the lidar payload for the broadcaster
                        self.state.lidar_payload = {
                            "type": "scan",
                            "angles": angles_rad,
                            "ranges": ranges_m,
                            "intensity": intensity,
                            "range_max": config.LIDAR_MAX_RANGE,
                        }

                    # Print rates roughly once per second
                    self._rate_tick()

            except Exception as e:
                print(f"[LIDAR] Error: {e}. Retrying in 5 seconds...")
                if self.lidar_obj:
                    try:
                        self.lidar_obj.stop()
                        self.lidar_obj.stop_motor()
                        self.lidar_obj.disconnect()
                    except Exception as disconnect_e:
                        print(f"[LIDAR] Error during disconnect: {disconnect_e}")
                self.lidar_obj = None
                time.sleep(5)

        print("[LIDAR] Stopping hardware...")
        if self.lidar_obj:
            try:
                self.lidar_obj.stop()
                self.lidar_obj.stop_motor()
                self.lidar_obj.disconnect()
            except Exception as e:
                print(f"[LIDAR] Error during final stop: {e}")

    async def run(self):
        self.loop = asyncio.get_running_loop()
        print(f"[LIDAR] Starting WebSocket server on {config.LIDAR_WS_PORT}")

        t = threading.Thread(target=self.lidar_loop, daemon=True)
        t.start()

        async with websockets.serve(self.ws_handler, "0.0.0.0", config.LIDAR_WS_PORT):
            try:
                await asyncio.Future()
            except asyncio.CancelledError:
                pass
            finally:
                self.stop_event.set()
                t.join(timeout=2.0)
