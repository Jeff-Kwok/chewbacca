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

        payload = {
            "type": "scan",
            "angles": angles_rad,     # the angles you actually want to plot (you are using angles_filled)
            "ranges": ranges_m,
            "intensity": intensity,
            "range_max": config.LIDAR_MAX_RANGE,
            "clusters": clusters or [],   # [[[a1,a0],[r1,r0]], ...]
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

        if not angles_rad or len(angles_rad) != len(ranges_m) or len(angles_rad) < 2:
            return angles_rad, ranges_m, []

        order = np.argsort(np.asarray(angles_rad))
        a = np.asarray(angles_rad, dtype=float)[order]
        r = np.asarray(ranges_m, dtype=float)[order]

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
                # You may also want to start "single point" runs (usually no)
                # run_active stays False
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

                    # -----------------------------------------------
                    intensity = [m[0] for m in scan] # Raw Intensities
                    angles_deg = [m[1] for m in scan] # Raw Angles degree 0,360
                    dists_mm = [m[2] for m in scan] # Raw Distances in millimeter
                    angles_rad = []
                    angles_ros = []
                    ranges_m = []
                    check_angles_rad = []
                    check_ranges_m = []
                    # -----------------------------------------------

                    for a_deg, d_mm in zip(angles_deg, dists_mm):
                        r_m = d_mm / 1000.0 # Converting for each distance
                        if r_m <= 0.0 or r_m > config.LIDAR_MAX_RANGE:
                            continue # Ends current iteration of the loop -> If we have a value <0 or greater than max range we skip
                            # According to RPLIDAR it's 0 when the measurement is invalid.

                        a_rad = (math.radians(a_deg)) # The angle comes as a degree so we convert it to rads
                        angles_rad.append(a_rad) # We append radian angle to angles_rad
                        ranges_m.append(r_m) # We append the associated distance in meters

                    # --------- GAP FILL (added, minimal intrusion) ---------
                    # Fill missing intermediate angles/ranges for small gaps.
                    yaw = np.deg2rad(float(self.state.stm["yaw"]))
                    angles = (np.asarray(angles_rad, dtype=float) + yaw) % (2*np.pi)
                    angles_out = ((angles) % (2*np.pi)).tolist()
                    angles_filled, ranges_filled,clusters = self.fill_angle_gaps(
                        angles_out,
                        ranges_m,
                        angle_step_deg=0.125,      # insert every 0.5°
                        max_fill_gap_deg=15.0,   # don't fill if gap is huge
                        dist_gap_m=0.675
                    )
                    # -------------------------------------------------------

                    # The result of this loop are 2 sets of arrays where the values exclude non-conforming values.
                    # The first set of arrays is indexed by a subset of angles that are nonzero in distance.
                    # The second set of arrays is indexed by a subset of angles that are <= 1.0 meters in distance.

                    # This is the dictionary that holds a set of arrays that are indexed by <= 1.0 meters in distance. 
                    self.state.lidar_close = {"angles": angles_filled, "ranges": ranges_filled, "clusters":clusters, "yaw":yaw} # Using angles filled

                    if ranges_m and self.loop: # If ranges_m is not empty
                        # ---- broadcast attempt counter ----
                        with self._rate_lock:
                            self._tx_scans += 1
                        # -----------------------------------

                        asyncio.run_coroutine_threadsafe(
                            self.broadcast_scan(angles_filled, ranges_filled, intensity, clusters=clusters, yaw=yaw),
                            self.loop
                        )


                        # Update state with the lidar payload for the broadcaster
                        self.state.lidar_payload = {
                            "type": "scan",
                            "angles": angles_rad, # using rad regular without any changes
                            "ranges": ranges_m,
                            "intensity": intensity,
                            "range_max": config.LIDAR_MAX_RANGE,
                        }
                    #print(clusters)
                    # Print rates roughly once per second
                    #self._rate_tick()

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
