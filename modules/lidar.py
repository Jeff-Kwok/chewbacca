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

    async def broadcast_scan(self, angles_rad, ranges_m, intensity, clusters=None, yaw=None):
        if not self.clients:
            return

        # FIX: allow intensity=None and handle any array-like safely
        if intensity is None:
            intensity_out = []
        else:
            intensity_out = intensity.tolist() if hasattr(intensity, "tolist") else list(intensity)

        # NOTE: keep clusters safe (avoid numpy truthiness issues)
        payload = {
            "type": "scan",
            "angles": angles_rad.tolist() if hasattr(angles_rad, "tolist") else list(angles_rad),
            "ranges": ranges_m.tolist() if hasattr(ranges_m, "tolist") else list(ranges_m),
            "intensity": intensity_out,
            "range_max": config.LIDAR_MAX_RANGE,
            "clusters": clusters if clusters is not None else [],   # [[[a1,a0],[r1,r0]], ...]
            "yaw": float(yaw) if yaw is not None else None,
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

    # ---------------- GAP FILLER (added) ----------------
    def angle_diff(self, a, b):
        """Wrap-safe signed difference a-b in radians, in [-pi, pi]."""
        return (a - b + math.pi) % (2.0 * math.pi) - math.pi

    def fill_angle_gaps(self, angles_rad, ranges_m,
                        angle_step_deg=0.25,
                        max_fill_gap_deg=15.0, # gap between two angles no greater than 10
                        dist_gap_m=1.0): # Distance between two points
        """
        Returns:
        filled_a, filled_r: with inserted points for moderate gaps (same as before)
        clusters: stitched segments as endpoints only: [[[a_end,a_start],[r_end,r_start]], ...]
        """

        if (angles_rad is None) or (ranges_m is None) or (len(angles_rad) < 2) or (len(angles_rad) != len(ranges_m)):
            return angles_rad, ranges_m, []

        a = np.asarray(angles_rad, dtype=float)
        r = np.asarray(ranges_m, dtype=float)

        # IMPORTANT: sort angles AND carry ranges with them (you computed order before, but didn't apply)
        order = np.argsort(a)
        a = a[order]
        r = r[order]

        angle_step = math.radians(angle_step_deg)
        max_fill_gap = math.radians(max_fill_gap_deg)

        filled_a = [float(a[0])]
        filled_r = [float(r[0])]

        clusters = []

        # ---- stitched-run state (endpoints only) ----
        run_active = False
        run_a_start = float(a[0])
        run_r_start = float(r[0])
        run_a_end   = float(a[0])
        run_r_end   = float(r[0])
        # -------------------------------------------

        def finalize_run():
            nonlocal run_active, run_a_start, run_r_start, run_a_end, run_r_end
            if run_active:
                # store as [[a_end,a_start],[r_end,r_start]]
                clusters.append([[run_a_end, run_a_start], [run_r_end, run_r_start]])
            run_active = False

        for i in range(1, len(a)):
            a0, r0 = float(a[i-1]), float(r[i-1])
            a1, r1 = float(a[i]),   float(r[i])

            da = self.angle_diff(a1, a0)
            abs_da = abs(da)
            dr = r1 - r0

            stitchable = (abs_da <= max_fill_gap) and (abs(dr) < dist_gap_m)

            # We still fill if the gap is > step but not huge and ranges match
            if abs_da > angle_step and abs_da <= max_fill_gap and abs(dr) < dist_gap_m:
                n = int(abs_da // angle_step)
                for k in range(1, n + 1):
                    t = k / (n + 1)
                    aa = a0 + da * t
                    rr = r0 + dr * t
                    filled_a.append(float(aa))
                    filled_r.append(float(rr))

            # Always append the real sample
            filled_a.append(a1)
            filled_r.append(r1)

            # ---- run stitching logic ----
            if stitchable:
                if not run_active:
                    # start a new run at the previous point
                    run_active = True
                    run_a_start = a0
                    run_r_start = r0
                # extend run to current point
                run_a_end = a1
                run_r_end = r1
            else:
                # continuity broke -> finalize any existing run
                finalize_run()
            # -----------------------------

        finalize_run()
        return filled_a, filled_r, clusters

        # ----------------------------------------------------

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

                    scan_np = np.asarray(scan, dtype=np.float32)   # shape (N,3): [intensity, angle_deg, dist_mm]
                    intensity_all = scan_np[:, 0]
                    angles_deg_all = scan_np[:, 1]
                    dists_mm_all = scan_np[:, 2]

                    ranges_m_all = dists_mm_all * 0.001
                    mask = (ranges_m_all > 0.0) & (ranges_m_all <= config.LIDAR_MAX_RANGE)

                    ranges_m = ranges_m_all[mask]
                    angles_rad = np.deg2rad(angles_deg_all[mask])

                    # IMPORTANT: intensity must match mask length to avoid descriptor length mismatch
                    intensity = intensity_all[mask]

                    # --------- GAP FILL (added, minimal intrusion) ---------
                    yaw = np.deg2rad(float(self.state.stm["yaw"]))
                    angles = (np.asarray(angles_rad, dtype=float) + yaw) % (2*np.pi)
                    angles_out = (angles % (2*np.pi)).tolist()

                    angles_filled, ranges_filled, clusters = self.fill_angle_gaps(
                        angles_out,
                        ranges_m,
                        angle_step_deg=0.5,      # insert every 0.5°
                        max_fill_gap_deg=10.0,   # don't fill if gap is huge
                        dist_gap_m=0.5
                    )
                    # -------------------------------------------------------

                    self.state.lidar_close = {
                        "angles": angles_filled,
                        "ranges": ranges_filled,
                        "clusters": clusters,
                        "yaw": yaw
                    }

                    # FIX: numpy array truthiness => use size (and avoid intensity-length mismatch vs filled arrays)
                    if (hasattr(ranges_m, "size") and ranges_m.size > 0) and self.loop:
                        with self._rate_lock:
                            self._tx_scans += 1

                        asyncio.run_coroutine_threadsafe(
                            self.broadcast_scan(
                                np.asarray(angles_filled, dtype=float),
                                np.asarray(ranges_filled, dtype=float),
                                None,  # FIX: filled points don't have matching intensity; avoid mismatch
                                clusters=clusters,
                                yaw=yaw
                            ),
                            self.loop
                        )

                    # keep your original payload (raw masked arrays)
                    self.state.lidar_payload = {
                        "type": "scan",
                        "angles": angles_rad.tolist(),
                        "ranges": ranges_m.tolist(),
                        "range_max": float(config.LIDAR_MAX_RANGE),
                    }

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
