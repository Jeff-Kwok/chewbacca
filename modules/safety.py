import asyncio
import numpy as np
import math
import json
import websockets

class SafetyModule:
    def __init__(self, state, ws_port=8766):
        self.state = state
        self.look_distance = 5.0

        self.fov_rad = np.deg2rad(90)
        self.classify_thresh = 0.08
        self.classify_min_points = 5

        # websocket
        self.ws_port = ws_port
        self.clients = set()

    # ----------------- WS helpers -----------------  
    async def ws_handler(self, websocket):
        self.clients.add(websocket)
        try:
            await websocket.wait_closed()
        finally:
            self.clients.discard(websocket)

    async def broadcast_safety(self, payload: dict):
        if not self.clients:
            return
        msg = json.dumps(payload)
        for ws in list(self.clients):
            try:
                await ws.send(msg)
            except Exception:
                pass

    # ----------------- Angle helpers -----------------
    def angle_wrap(self, a: float) -> float:
        return (a + np.pi) % (2 * np.pi) - np.pi

    def angle_diff(self, a: float, b: float) -> float:
        return self.angle_wrap(a - b)

    def angle_in_interval(self, theta: float, a0: float, a1: float) -> bool:
        theta = self.angle_wrap(theta)
        a0 = self.angle_wrap(a0)
        a1 = self.angle_wrap(a1)

        if a0 <= a1:
            return a0 <= theta <= a1
        else:
            return theta >= a0 or theta <= a1
    def polar_point_distance(self,a1, r1, a2, r2):
        dtheta = self.angle_diff(a2, a1)
        return np.sqrt(r1*r1 + r2*r2 - 2*r1*r2*np.cos(dtheta))
    # ----------------- Payload builders -----------------
    def rectangular_payload(self, cluster, out_thickness, in_thickness, a_lo, r_lo, m):
        """
        Build a polar "thick band" around the fitted line between a0 and a1.

        cluster: [[a1,a0],[r1,r0]] (endpoints only define angular span)
        out_thickness: >=0 (farther than line)
        in_thickness:  >=0 (closer than line)
        a_lo, r_lo, m: fitted line baseline: r_hat(a) = r_lo + m*angle_diff(a, a_lo)
        returns 4 corners (a,r) in order.
        """
        a1, a0 = float(cluster[0][0]), float(cluster[0][1])

        # Ensure consistent direction from a0 -> a1 along shortest arc
        da01 = self.angle_diff(a1, a0)
        if da01 < 0:
            # swap to make forward progress positive
            a0, a1 = a1, a0

        # Baseline ranges from fitted line at each endpoint angle
        t0 = self.angle_diff(a0, a_lo)
        t1 = self.angle_diff(a1, a_lo)
        r_base0 = float(r_lo + m * t0)
        r_base1 = float(r_lo + m * t1)

        # Apply thickness around the baseline (this is the key)
        r_near0 = max(0.0, r_base0 - in_thickness)
        r_near1 = max(0.0, r_base1 - in_thickness)
        r_far0  = r_base0 + out_thickness
        r_far1  = r_base1 + out_thickness

        # Guard against inversion
        if r_near0 > r_far0:
            r_near0, r_far0 = r_far0, r_near0
        if r_near1 > r_far1:
            r_near1, r_far1 = r_far1, r_near1

        return [
            (a0, r_near0),
            (a0, r_far0),
            (a1, r_far1),
            (a1, r_near1),
        ]



    def line_payload(self, cluster):
        return [float(cluster[0][0]), float(cluster[1][0])], [float(cluster[0][1]), float(cluster[1][1])]

    def polar_to_xy(self, a, r):
        return np.array([r * np.cos(a), r * np.sin(a)], dtype=float)
    def circular_payload(self, cluster):
        a1, a0 = float(cluster[0][0]), float(cluster[0][1])
        r1, r0 = float(cluster[1][0]), float(cluster[1][1])

        da = self.angle_diff(a1, a0)
        mid_angle = (a0 + 0.5 * da) % (2 * np.pi)
        mid_dist = 0.5 * (r0 + r1)
        return [float(mid_angle), float(mid_dist)]

    def rect_xy_from_polar_cluster_bestfit(self,a_seg, r_seg):
        """
        Build a true XY rectangle (parallel edges) containing all points in a cluster,
        using a best-fit line (total least squares via PCA).

        Inputs:
        a_seg: 1D array-like of angles (rad)
        r_seg: 1D array-like of ranges (m)

        Output:
        rect_xy: list of 4 corners [[x,y], ...] in meters (CCW order),
                or None if not enough points.
        """
        a_seg = np.asarray(a_seg, dtype=float)
        r_seg = np.asarray(r_seg, dtype=float)
        if a_seg.size < 2 or r_seg.size < 2:
            return None

        # Convert polar -> XY
        x = r_seg * np.cos(a_seg)
        y = r_seg * np.sin(a_seg)
        pts = np.column_stack((x, y))  # (N,2)

        # Mean center
        mu = pts.mean(axis=0)
        X = pts - mu

        # PCA (total least squares line of best fit):
        # principal eigenvector of covariance gives direction of best-fit line
        C = (X.T @ X) / max(1, X.shape[0])
        w, V = np.linalg.eigh(C)                 # eigenvalues asc
        v = V[:, np.argmax(w)]                  # direction along the line
        v = v / (np.linalg.norm(v) + 1e-12)     # unit
        n = np.array([-v[1], v[0]], dtype=float)  # unit normal

        # Project points onto v and n to get extents
        s = X @ v   # along-line coordinates
        d = X @ n   # normal coordinates

        s_min, s_max = float(s.min()), float(s.max())
        d_min, d_max = float(d.min()), float(d.max())

        # Endpoints of the baseline segment along v through mu
        b0 = mu + v * s_min
        b1 = mu + v * s_max

        # Rectangle corners (CCW)
        c0 = b0 + n * d_max
        c1 = b1 + n * d_max
        c2 = b1 + n * d_min
        c3 = b0 + n * d_min

        return [c0.tolist(), c1.tolist(), c2.tolist(), c3.tolist()]


    # ----------------- Classification -----------------
    def classify_cluster(self, cluster, angles, ranges, thresh=0.2, min_points=5):
        a_hi, a_lo = float(cluster[0][0]), float(cluster[0][1])
        r_hi, r_lo = float(cluster[1][0]), float(cluster[1][1])

        angles = np.asarray(angles, dtype=float)
        ranges = np.asarray(ranges, dtype=float)

        in_window = np.array([self.angle_in_interval(a, a_lo, a_hi) for a in angles], dtype=bool)
        a_seg = angles[in_window]
        r_seg = ranges[in_window]

        # guard: no points
        if a_seg.size == 0:
            return {"label": "degenerate", "n": 0}

        # too few points => treat as circle-ish
        if a_seg.size < min_points:
            return {"label": "circle", "n": int(a_seg.size), "circle": self.circular_payload(cluster)}

        da = self.angle_diff(a_hi, a_lo)
        if abs(da) < 1e-6:
            return {"label": "degenerate", "n": int(a_seg.size)}

        t = np.array([self.angle_diff(a, a_lo) for a in a_seg], dtype=float)
        m = (r_hi - r_lo) / da
        r_hat = r_lo + m * t

        resid = r_seg - r_hat
        max_pos = float(np.max(resid))
        max_neg = float(np.min(resid))
        i_pos = int(np.argmax(resid))
        i_neg = int(np.argmin(resid))

        has_pos = max_pos > thresh
        has_neg = max_neg < -thresh

        if has_pos or has_neg:
            rect_xy = self.rect_xy_from_polar_cluster_bestfit(a_seg, r_seg)
            return {"label": "rectangle", "rect_xy": rect_xy, "n": int(a_seg.size)}

        # else line-ish: measure endpoint separation in meters (true Euclidean)
        p1, p2 = self.line_payload(cluster)   # [a,r], [a,r]
        abs_distance = float(self.polar_point_distance(p1[0], p1[1], p2[0], p2[1]))

        if abs_distance > 0.45:
            return {
                "label": "line",
                "n": int(a_seg.size),
                "max_abs": float(np.max(np.abs(resid))),
                "line": [p1, p2],   # [[a,r],[a,r]]
                "length": abs_distance,
            }

        # short segment => treat as circle-ish
        return {
            "label": "circle",
            "n": int(a_seg.size),
            "max_abs": float(np.max(np.abs(resid))),
            "circle": self.circular_payload(cluster),  # [a,r]
            "length": abs_distance,
        }


    # ----------------- Main loop -----------------
    async def run(self):
        print(f"[SAFETY] WebSocket server on port {self.ws_port}")
        async with websockets.serve(self.ws_handler, "0.0.0.0", self.ws_port):
            print("[SAFETY] Listening for close obstacles and controller inputs...")

            while True:
                lidar_data = self.state.lidar_close or {}
                angles = np.asarray(lidar_data.get("angles", []), dtype=float)
                ranges = np.asarray(lidar_data.get("ranges", []), dtype=float)
                clusters = lidar_data.get("clusters", []) or []
                yaw = lidar_data.get("yaw", None)  # IMPORTANT: this is the SAME yaw used in lidar transform

                # read joystick/command inputs
                lx = ly = 0.0
                if self.state.robot_current == 1:
                    lx = float(self.state.axes.get("LX", 0.0))
                    ly = float(self.state.axes.get("LY", 0.0))
                elif self.state.robot_current == 2:
                    lx = float(self.state.command_vector.get("LX", 0.0))
                    ly = float(self.state.command_vector.get("LY", 0.0))
                elif self.state.robot_current == 3:
                    lx = float(self.state.command_vector.get("LX", 0.0))
                    ly = float(self.state.command_vector.get("LY", 0.0))

                # If no yaw yet, just publish empty safety (keeps UI alive)
                if yaw is None:
                    safety_payload = {
                        "type": "safety",
                        "intended_vector": None,
                        "intended_hits": [],
                        "n_clusters": len(clusters),
                    }
                    self.state.safety = safety_payload
                    await self.broadcast_safety(safety_payload)
                    await asyncio.sleep(0.05)
                    continue

                yaw = float(yaw) % (2 * np.pi)

                # joystick direction in robot frame
                # NOTE: choose this so "forward stick" becomes the direction you want in your frame.
                # If your forward is LY, this +pi/2 might be right for your mapping.
                joy_theta = (np.arctan2(lx, ly)) % (2 * np.pi)

                # MATCH LIDAR FRAME:
                # lidar uses: angles_out = (pi - (angles_rad + yaw)) % 2pi
                intended_angle = ((joy_theta + yaw)) % (2 * np.pi)

                intended_magnitude = float(np.hypot(lx, ly))
                intended_look = intended_magnitude * float(self.look_distance)
                intended_directional_vector = [float(intended_angle), float(intended_look)]

                intended_hits = []
                for cl in clusters:
                    a1, a0 = float(cl[0][0]), float(cl[0][1])
                    r1, r0 = float(cl[1][0]), float(cl[1][1])

                    if not self.angle_in_interval(intended_angle, a0, a1):
                        continue
                    if r0 > intended_look:
                        continue

                    info = self.classify_cluster(
                        cl, angles, ranges,
                        thresh=self.classify_thresh,
                        min_points=self.classify_min_points
                    )
                    intended_hits.append({"cluster": cl, "info": info})

                safety_payload = {
                    "type": "safety",
                    "intended_vector": intended_directional_vector,
                    "intended_hits": intended_hits,
                    "n_clusters": len(clusters),
                }

                self.state.safety = safety_payload
                #print(self.state.safety)
                await self.broadcast_safety(safety_payload)

                await asyncio.sleep(0.05)  # ~20 Hz
