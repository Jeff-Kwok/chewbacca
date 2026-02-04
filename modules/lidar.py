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
        self._rx_scans = 0
        self._rx_points = 0
        self._tx_scans = 0
        self._t0 = time.monotonic()
        # ----------------------

        # ---- NEW: single-slot "latest scan" for WS (avoid scheduling per scan) ----
        self._ws_lock = threading.Lock()
        self._ws_latest = None  # dict payload ready for json.dumps
        # ------------------------------------------------------------------------

    async def ws_handler(self, websocket):
        self.clients.add(websocket)
        try:
            await websocket.wait_closed()
        finally:
            self.clients.discard(websocket)

    # -------------------- MINIMAL CHANGE: "send dict" helper --------------------
    async def _broadcast_payload(self, payload: dict):
        if not self.clients:
            return
        msg = json.dumps(payload)
        for ws in list(self.clients):
            try:
                await ws.send(msg)
            except Exception:
                pass
    # --------------------------------------------------------------------------

    # -------------------- MINIMAL CHANGE: sender task --------------------------
    async def _ws_sender_loop(self):
        """
        Sends the latest filled scan at a fixed rate.
        This replaces per-scan asyncio.run_coroutine_threadsafe overhead.
        """
        hz = float(getattr(config, "LIDAR_WS_HZ", 20.0))  # optional config, default 20Hz
        dt = 1.0 / max(1.0, hz)

        while not self.stop_event.is_set():
            if self.clients:
                payload = None
                with self._ws_lock:
                    payload = self._ws_latest
                    self._ws_latest = None  # "consume" (optional; keeps bandwidth down)

                if payload is not None:
                    with self._rate_lock:
                        self._tx_scans += 1
                    await self._broadcast_payload(payload)

            await asyncio.sleep(dt)
    # --------------------------------------------------------------------------

    def _rate_tick(self):
        now = time.monotonic()
        with self._rate_lock:
            dt = now - self._t0
            if dt < 1.0:
                return
            # (kept; printing disabled)
            self._rx_scans = 0
            self._rx_points = 0
            self._tx_scans = 0
            self._t0 = now

    # ---------------- GAP FILLER ----------------
    def angle_diff(self, a, b):
        return (a - b + math.pi) % (2.0 * math.pi) - math.pi

    def fill_angle_gaps(self, angles_rad, ranges_m,
                        angle_step_deg=0.25,
                        max_fill_gap_deg=15.0,
                        dist_gap_m=1.0):
        """
        Returns:
          filled_a (np.ndarray), filled_r (np.ndarray), clusters (list)
        MINIMAL CHANGE: returns numpy arrays (not python lists) to avoid list<->np churn.
        """
        if (angles_rad is None) or (ranges_m is None):
            return np.asarray([], dtype=np.float32), np.asarray([], dtype=np.float32), []
        if len(angles_rad) < 2 or len(angles_rad) != len(ranges_m):
            a = np.asarray(angles_rad, dtype=np.float32)
            r = np.asarray(ranges_m, dtype=np.float32)
            return a, r, []

        a = np.asarray(angles_rad, dtype=np.float32)
        r = np.asarray(ranges_m, dtype=np.float32)

        order = np.argsort(a)
        a = a[order]
        r = r[order]

        angle_step = math.radians(angle_step_deg)
        max_fill_gap = math.radians(max_fill_gap_deg)

        filled_a = [float(a[0])]
        filled_r = [float(r[0])]

        clusters = []

        run_active = False
        run_a_start = float(a[0])
        run_r_start = float(r[0])
        run_a_end   = float(a[0])
        run_r_end   = float(r[0])

        def finalize_run():
            nonlocal run_active, run_a_start, run_r_start, run_a_end, run_r_end
            if run_active:
                clusters.append([[run_a_end, run_a_start], [run_r_end, run_r_start]])
            run_active = False

        for i in range(1, len(a)):
            a0, r0 = float(a[i - 1]), float(r[i - 1])
            a1, r1 = float(a[i]),     float(r[i])

            da = self.angle_diff(a1, a0)
            abs_da = abs(da)
            dr = r1 - r0

            stitchable = (abs_da <= max_fill_gap) and (abs(dr) < dist_gap_m)

            if abs_da > angle_step and abs_da <= max_fill_gap and abs(dr) < dist_gap_m:
                n = int(abs_da // angle_step)
                # cap to prevent spikes (still "same behavior" for normal gaps)
                if n > 20:
                    n = 20
                for k in range(1, n + 1):
                    t = k / (n + 1)
                    filled_a.append(a0 + da * t)
                    filled_r.append(r0 + dr * t)

            filled_a.append(a1)
            filled_r.append(r1)

            if stitchable:
                if not run_active:
                    run_active = True
                    run_a_start = a0
                    run_r_start = r0
                run_a_end = a1
                run_r_end = r1
            else:
                finalize_run()

        finalize_run()

        return (
            np.asarray(filled_a, dtype=np.float32),
            np.asarray(filled_r, dtype=np.float32),
            clusters
        )

    # --------------------------------------------

    def lidar_loop(self):
        two_pi = 2.0 * math.pi

        while not self.stop_event.is_set():
            try:
                print(f"[LIDAR] Connecting to {config.LIDAR_SERIAL_PORT}...")
                lidar = RPLidar(config.LIDAR_SERIAL_PORT)
                self.lidar_obj = lidar
                lidar.start_motor()
                print("[LIDAR] Connected. Scanning...")

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

                    with self._rate_lock:
                        self._rx_scans += 1
                        self._rx_points += len(scan)

                    scan_np = np.asarray(scan, dtype=np.float32)  # (N,3): [intensity, angle_deg, dist_mm]
                    #intensity_all = scan_np[:, 0]
                    angles_deg_all = scan_np[:, 1]
                    dists_mm_all = scan_np[:, 2]

                    ranges_m_all = dists_mm_all * 0.001
                    mask = (ranges_m_all > 0.0) & (ranges_m_all <= float(config.LIDAR_MAX_RANGE))

                    ranges_m = ranges_m_all[mask]
                    angles_rad = np.deg2rad(angles_deg_all[mask]).astype(np.float32, copy=False)

                    # raw payload kept for ROS path (you said this is required)
                    self.state.lidar_payload = {
                        "type": "scan",
                        "angles": angles_rad.tolist(),   # RAW (no yaw, no fill)
                        "ranges": ranges_m.tolist(),
                        "range_max": float(config.LIDAR_MAX_RANGE),
                    }

                    # ------ filled + yaw-adjusted for safety + websocket ------
                    yaw = math.radians(float(self.state.stm["yaw"]))  # scalar
                    angles_world = angles_rad + yaw
                    angles_world = np.mod(angles_world, two_pi).astype(np.float32, copy=False)

                    angles_filled, ranges_filled, clusters = self.fill_angle_gaps(
                        angles_world,
                        ranges_m,
                        angle_step_deg=0.5,
                        max_fill_gap_deg=10.0,
                        dist_gap_m=0.5
                    )

                    # state for safety module (filled, yaw-adjusted)
                    self.state.lidar_close = {
                        "angles": angles_filled.tolist(),
                        "ranges": ranges_filled.tolist(),
                        "clusters": clusters,
                        "yaw": yaw
                    }
                    '''

                    # websocket payload (filled only)
                    if angles_filled.size > 0:
                        ws_payload = {
                            "type": "scan",
                            "angles": angles_filled.tolist(),
                            "ranges": ranges_filled.tolist(),
                            #"intensity": [],  # filled points don't match intensity
                            "range_max": float(config.LIDAR_MAX_RANGE),
                            "clusters": clusters,
                            "yaw": float(yaw),
                        }
                        with self._ws_lock:
                            self._ws_latest = ws_payload
                    '''
                    # ----------------------------------------------------------

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

        # start lidar hardware thread
        t = threading.Thread(target=self.lidar_loop, daemon=True)
        t.start()

        # start ws sender task (NEW, minimal)
        #sender_task = asyncio.create_task(self._ws_sender_loop())
        '''
        async with websockets.serve(self.ws_handler, "0.0.0.0", config.LIDAR_WS_PORT):
            try:
                await asyncio.Future()
            except asyncio.CancelledError:
                pass
            finally:
                self.stop_event.set()
                sender_task.cancel()
                try:
                    await sender_task
                except Exception:
                    pass
                t.join(timeout=2.0)
        '''
