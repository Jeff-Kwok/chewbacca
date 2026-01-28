import asyncio
import numpy as np
import math
import json
import websockets


class SafetyModule:
    """
    SAFETY MODULE LOGIC FLOW (high level)

    1) Read lidar_close payload:
         angles[], ranges[], clusters[], yaw
       - angles/ranges are already in the SAME "plot frame" you use in HTML (cos/sin).
       - clusters are tiny segments: [[a1,a0],[r1,r0]]

    2) Read driver intent (lx, ly):
       - compute intended_angle (in the SAME frame as lidar angles)
       - compute intended_magnitude and intended_look distance

    3) Filter obstacles along the intended ray slice:
       - keep only clusters whose angular span contains intended_angle
       - keep only clusters within intended_look
       - classify each into: rectangle / line / circle and attach geometry payloads

    4) Compute avoidance output vector:
       - pick the "first obstacle hit" (minimum positive projection along intended direction)
       - compute how close it is to the intended infinite line (perp distance)
       - compute a side-step magnitude based on danger and how soon you hit it
       - CHOOSE LEFT vs RIGHT by "free-space scoring" (try both sides, pick larger clearance cone)
       - output avoid_vector = forward_keep*intended + mag_side*(normal)

    5) Broadcast everything over WS:
       - intended_vector, intended_hits, avoid_vector, chosen_obstacle debug
    """

    def __init__(self, state, ws_port=8766):
        self.state = state

        # --- How far ahead the look-ahead ray extends when stick magnitude is 1.0 ---
        self.look_distance = 2.5

        # --- Cluster classification parameters ---
        self.classify_thresh = 0.3
        self.classify_min_points = 5

        # --- Avoidance tuning ---
        self.safety_radius = 0.35     # desired clearance from intended line (m)
        self.side_gain = 1.3          # how hard to sidestep
        self.forward_keep = 1.0       # keep forward component
        self.inflate_rect = 0.03      # inflate rectangle thickness slightly (m)

        # --- Free-space side choice tuning ---
        self.cone_half_angle = np.deg2rad(12.0)  # +/-12° cone for clearance scoring
        self.side_test_k = 0.8                   # how sideways the test direction is
        self.side_tie_eps = 0.15                 # if clearances too close, fall back to geometric side

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
        """Wrap to [-pi, pi)."""
        return (a + np.pi) % (2 * np.pi) - np.pi

    def angle_diff(self, a: float, b: float) -> float:
        """Shortest signed difference a-b in [-pi, pi)."""
        return self.angle_wrap(a - b)

    def angle_in_interval(self, theta: float, a0: float, a1: float) -> bool:
        """
        True if theta is in [a0,a1] with wraparound support.
        """
        theta = self.angle_wrap(theta)
        a0 = self.angle_wrap(a0)
        a1 = self.angle_wrap(a1)
        if a0 <= a1:
            return a0 <= theta <= a1
        return theta >= a0 or theta <= a1
    def world_to_body(self, v_xy, yaw):
        """
        Rotate vector from world/plot frame into robot/body frame.
        yaw: robot heading (rad) in the SAME world/plot frame.
        """
        vx, vy = float(v_xy[0]), float(v_xy[1])
        cy = math.cos(yaw)
        sy = math.sin(yaw)
        # R(-yaw) = [[cos, sin],[-sin, cos]]
        bx =  cy * vx + sy * vy
        by = -sy * vx + cy * vy
        return [bx, by]

    # ----------------- Basic geometry helpers (no external dependencies) -----------------
    def _unit(self, v):
        n = float(np.linalg.norm(v))
        if n < 1e-9:
            return np.array([0.0, 0.0], dtype=float)
        return v / n

    def _ray_dir(self, theta):
        return np.array([math.cos(theta), math.sin(theta)], dtype=float)

    def _cross2(self, a, b):
        return float(a[0] * b[1] - a[1] * b[0])

    def _signed_dist_to_intended_line(self, p_xy, u_hat):
        """
        Signed perpendicular distance from point p to the infinite line through origin along u_hat.
        Positive => point is to the "left" of u_hat (right-hand rule).
        """
        return self._cross2(u_hat, p_xy)

    def _along_intended(self, p_xy, u_hat):
        """Projection distance along intended direction."""
        return float(np.dot(p_xy, u_hat))

    # ----------------- Polar utilities -----------------
    def polar_point_distance(self, a1, r1, a2, r2):
        """True euclidean distance between two polar points."""
        dtheta = self.angle_diff(a2, a1)
        return np.sqrt(r1 * r1 + r2 * r2 - 2 * r1 * r2 * np.cos(dtheta))

    def line_payload(self, cluster):
        """cluster: [[a1,a0],[r1,r0]] -> returns endpoints [a,r], [a,r]."""
        return [float(cluster[0][0]), float(cluster[1][0])], [float(cluster[0][1]), float(cluster[1][1])]

    def circular_payload(self, cluster):
        """Simple circle-ish representation: midpoint (a,r)."""
        a1, a0 = float(cluster[0][0]), float(cluster[0][1])
        r1, r0 = float(cluster[1][0]), float(cluster[1][1])
        da = self.angle_diff(a1, a0)
        mid_angle = (a0 + 0.5 * da) % (2 * np.pi)
        mid_dist = 0.5 * (r0 + r1)
        return [float(mid_angle), float(mid_dist)]

    # ----------------- Rectangle from best-fit (PCA TLS) -----------------
    def rect_xy_from_polar_cluster_bestfit(self, a_seg, r_seg):
        """
        Build a true XY rectangle (parallel edges) containing all points in a cluster,
        using a best-fit line (total least squares via PCA).
        Output is rect_xy = [[x,y], ...] in meters.
        """
        a_seg = np.asarray(a_seg, dtype=float)
        r_seg = np.asarray(r_seg, dtype=float)
        if a_seg.size < 2 or r_seg.size < 2:
            return None

        # Convert polar -> XY points
        x = r_seg * np.cos(a_seg)
        y = r_seg * np.sin(a_seg)
        pts = np.column_stack((x, y))  # (N,2)

        # Mean center + PCA direction
        mu = pts.mean(axis=0)
        X = pts - mu

        C = (X.T @ X) / max(1, X.shape[0])
        w, V = np.linalg.eigh(C)
        v = V[:, np.argmax(w)]
        v = v / (np.linalg.norm(v) + 1e-12)
        n = np.array([-v[1], v[0]], dtype=float)

        # Extents
        s = X @ v
        d = X @ n
        s_min, s_max = float(s.min()), float(s.max())
        d_min, d_max = float(d.min()), float(d.max())

        # inflate to contain points robustly
        d_min -= self.inflate_rect
        d_max += self.inflate_rect

        b0 = mu + v * s_min
        b1 = mu + v * s_max

        c0 = b0 + n * d_max
        c1 = b1 + n * d_max
        c2 = b1 + n * d_min
        c3 = b0 + n * d_min

        return [c0.tolist(), c1.tolist(), c2.tolist(), c3.tolist()]

    # ----------------- Cluster classification -----------------
    def classify_cluster(self, cluster, angles, ranges):
        """
        Returns dict like:
          {"label":"rectangle","rect_xy":[...]}
          {"label":"line","line":[[a,r],[a,r]]}
          {"label":"circle","circle":[a,r]}
        """
        a_hi, a_lo = float(cluster[0][0]), float(cluster[0][1])
        r_hi, r_lo = float(cluster[1][0]), float(cluster[1][1])

        angles = np.asarray(angles, dtype=float)
        ranges = np.asarray(ranges, dtype=float)

        # Wrap-safe window selection
        in_window = np.array([self.angle_in_interval(a, a_lo, a_hi) for a in angles], dtype=bool)
        a_seg = angles[in_window]
        r_seg = ranges[in_window]

        # guard: no points
        if a_seg.size == 0:
            return {"label": "degenerate", "n": 0}

        # too few points => circle-ish
        if a_seg.size < self.classify_min_points:
            return {"label": "circle", "n": int(a_seg.size), "circle": self.circular_payload(cluster)}

        # fit a simple line in (theta,r) to estimate residual band
        da = self.angle_diff(a_hi, a_lo)
        if abs(da) < 1e-6:
            return {"label": "degenerate", "n": int(a_seg.size)}

        t = np.array([self.angle_diff(a, a_lo) for a in a_seg], dtype=float)
        m = (r_hi - r_lo) / da
        r_hat = r_lo + m * t
        resid = r_seg - r_hat

        max_pos = float(np.max(resid))
        max_neg = float(np.min(resid))

        # rectangle-ish if total thickness is large
        if (max_pos + abs(max_neg)) > self.classify_thresh:
            rect_xy = self.rect_xy_from_polar_cluster_bestfit(a_seg, r_seg)
            return {"label": "rectangle", "rect_xy": rect_xy, "n": int(a_seg.size)}

        # otherwise line-ish: check endpoint separation
        p1, p2 = self.line_payload(cluster)
        length = float(self.polar_point_distance(p1[0], p1[1], p2[0], p2[1]))
        if length > 0.45:
            return {"label": "line", "line": [p1, p2], "length": length, "n": int(a_seg.size)}

        # short => circle-ish
        return {"label": "circle", "circle": self.circular_payload(cluster), "length": length, "n": int(a_seg.size)}

    # ----------------- Free-space scoring for left vs right -----------------
    def _cone_clearance(self, angles, ranges, theta, cone_half_angle, max_r=6.0):
        """
        Clearance score for steering direction theta.
        Larger is better. Uses 10th percentile range inside cone.
        """
        if angles.size == 0:
            return max_r

        # wrap-safe angular distance
        d = np.array([abs(self.angle_diff(a, theta)) for a in angles], dtype=float)
        mask = d <= cone_half_angle
        if not np.any(mask):
            return max_r

        r = np.asarray(ranges, dtype=float)[mask]
        r = r[np.isfinite(r)]
        if r.size == 0:
            return max_r

        r = r[(r > 0.02) & (r <= max_r)]
        if r.size == 0:
            return max_r

        return float(np.percentile(r, 10))

    def _pick_side_by_free_space(self, angles, ranges, intended_theta, n_left, k):
        """
        Try both candidate directions (left/right) and choose side with more clearance.
        Returns side_sign (+1=left, -1=right) and debug dict.
        """
        u = self._unit(self._ray_dir(float(intended_theta)))

        wL = self._unit(u + k * n_left)
        wR = self._unit(u - k * n_left)

        thetaL = math.atan2(wL[1], wL[0]) % (2 * np.pi)
        thetaR = math.atan2(wR[1], wR[0]) % (2 * np.pi)

        cL = self._cone_clearance(angles, ranges, thetaL, self.cone_half_angle, max_r=6.0)
        cR = self._cone_clearance(angles, ranges, thetaR, self.cone_half_angle, max_r=6.0)

        side_sign = +1.0 if cL >= cR else -1.0
        dbg = {"thetaL": thetaL, "thetaR": thetaR, "clearL": cL, "clearR": cR}
        return side_sign, dbg

    # ----------------- Representative obstacle points for "first hit" -----------------
    def _obstacle_points_xy(self, info):
        """
        Return representative points in XY for:
          rectangle -> corners
          line      -> endpoints
          circle    -> center point
        """
        if not info:
            return None

        if info.get("rect_xy"):
            pts = np.asarray(info["rect_xy"], dtype=float)
            if pts.ndim == 2 and pts.shape[1] == 2 and pts.shape[0] >= 3:
                return pts

        if info.get("line"):
            p1, p2 = info["line"][0], info["line"][1]
            return np.array([
                [float(p1[1]) * math.cos(float(p1[0])), float(p1[1]) * math.sin(float(p1[0]))],
                [float(p2[1]) * math.cos(float(p2[0])), float(p2[1]) * math.sin(float(p2[0]))],
            ], dtype=float)

        if info.get("circle"):
            a, r = float(info["circle"][0]), float(info["circle"][1])
            return np.array([[r * math.cos(a), r * math.sin(a)]], dtype=float)

        return None

    # ----------------- Avoidance computation (core output) -----------------
    def compute_avoid_output(self, intended_angle, intended_magnitude, intended_hits, angles, ranges):
        """
        OUTPUT LOGIC FLOW

        A) Build intended direction u_hat from intended_angle.
        B) Choose "first obstacle hit":
           - for each hit obstacle:
             - get its representative points in XY
             - find the point with smallest abs distance to intended line
             - compute along-distance for that point
           - choose the obstacle with MIN positive along-distance

        C) Compute danger:
           - d = abs(perp distance)
           - danger = clamp((safety_radius - d)/safety_radius)

        D) Choose left vs right:
           - geometric side = sign of signed_dist (closest point)
           - free-space side = score left and right cones using lidar points
           - if free-space difference is tiny => use geometric tie-break

        E) Compute avoid vector:
           v_out = forward_keep * v_intended + mag_side * n_avoid
           cap to intended magnitude
        """
        intended_magnitude = float(intended_magnitude)
        if intended_magnitude <= 1e-6:
            return [0.0, 0.0], None

        u_hat = self._unit(self._ray_dir(float(intended_angle)))
        v_intended = u_hat * intended_magnitude

        if not intended_hits:
            return [float(v_intended[0]), float(v_intended[1])], None

        # look distance scales with stick magnitude (so if stick is small, you look shorter)
        look_dist = max(1e-6, float(self.look_distance) * intended_magnitude)

        # ---- A) choose the first obstacle hit ----
        best = None
        for h in intended_hits:
            info = (h or {}).get("info", {}) or {}
            pts = self._obstacle_points_xy(info)
            if pts is None or len(pts) == 0:
                continue

            # find the representative point closest to intended line (and in front)
            best_p = None
            best_absd = None
            best_sd = None
            best_along = None

            for p in pts:
                along = self._along_intended(p, u_hat)
                if along <= 0.0 or along > look_dist:
                    continue

                sd = self._signed_dist_to_intended_line(p, u_hat)
                absd = abs(sd)

                if best_absd is None or absd < best_absd:
                    best_absd = absd
                    best_sd = sd
                    best_along = along
                    best_p = p

            if best_p is None:
                continue

            if best is None or best_along < best["along"]:
                best = {
                    "hit": h,
                    "info": info,
                    "p_closest": np.array(best_p, dtype=float),
                    "signed_dist": float(best_sd),
                    "abs_dist": float(best_absd),
                    "along": float(best_along),
                }

        if best is None:
            return [float(v_intended[0]), float(v_intended[1])], None

        # ---- B) danger based on how close you are to the line ----
        sr = max(1e-6, float(self.safety_radius))
        d = float(best["abs_dist"])
        danger = max(0.0, min(1.0, (sr - d) / sr))

        # stronger if obstacle is closer along direction
        along_factor = max(0.0, min(1.0, (look_dist - best["along"]) / look_dist))

        # ---- C) choose left vs right ----
        n_left = np.array([-u_hat[1], u_hat[0]], dtype=float)

        # geometric preference (fallback)
        side_geom = -1.0 if best["signed_dist"] > 0.0 else 1.0  # obstacle left => push right

        # free-space preference
        side_fs, fs_dbg = self._pick_side_by_free_space(
            angles=np.asarray(angles, dtype=float),
            ranges=np.asarray(ranges, dtype=float),
            intended_theta=float(intended_angle),
            n_left=n_left,
            k=float(self.side_test_k),
        )

        # tie-break: if both sides are similarly open, use geometric
        if abs(fs_dbg["clearL"] - fs_dbg["clearR"]) < self.side_tie_eps:
            side_sign = side_geom
            side_reason = "geom_tie"
        else:
            side_sign = side_fs
            side_reason = "free_space"

        n_avoid = n_left * float(side_sign)

        # ---- D) compute side magnitude + output ----
        mag_side = self.side_gain * danger * (0.35 + 0.65 * along_factor) * intended_magnitude

        v_out = (self.forward_keep * v_intended) + (mag_side * n_avoid)

        # cap magnitude so we don't overdrive
        out_norm = float(np.linalg.norm(v_out))
        if out_norm > 1e-6 and out_norm > intended_magnitude:
            v_out = v_out * (intended_magnitude / out_norm)

        # ---- E) debug payload for UI ----
        pts_all = self._obstacle_points_xy(best["info"])
        mid_xy = None
        if pts_all is not None and len(pts_all) > 0:
            mid = np.mean(np.asarray(pts_all, dtype=float), axis=0)
            mid_xy = [float(mid[0]), float(mid[1])]

        chosen = {
            "along": float(best["along"]),
            "abs_dist": float(best["abs_dist"]),
            "signed_dist": float(best["signed_dist"]),
            "closest_point_xy": [float(best["p_closest"][0]), float(best["p_closest"][1])],
            "midpoint_xy": mid_xy,
            "danger": float(danger),
            "side": "left" if side_sign > 0 else "right",
            "side_reason": side_reason,
            "free_space": fs_dbg,  # includes clearL/clearR and thetaL/thetaR
        }

        return [float(v_out[0]), float(v_out[1])], chosen

    # ----------------- Main loop -----------------
    async def run(self):
        print(f"[SAFETY] WebSocket server on port {self.ws_port}")
        async with websockets.serve(self.ws_handler, "0.0.0.0", self.ws_port):
            print("[SAFETY] Listening for close obstacles and controller inputs...")

            while True:
                # 1) READ LIDAR PAYLOAD
                lidar_data = self.state.lidar_close or {}
                angles = np.asarray(lidar_data.get("angles", []), dtype=float)
                ranges = np.asarray(lidar_data.get("ranges", []), dtype=float)
                clusters = lidar_data.get("clusters", []) or []
                yaw = lidar_data.get("yaw", None)

                # 2) READ DRIVER INTENT (lx, ly)
                lx = ly = 0.0
                if self.state.robot_current == 1:
                    lx = float(self.state.axes.get("LX", 0.0))
                    ly = float(self.state.axes.get("LY", 0.0))
                elif self.state.robot_current in (2, 3):
                    lx = float(self.state.command_vector.get("LX", 0.0))
                    ly = float(self.state.command_vector.get("LY", 0.0))

                # If yaw isn't ready, still broadcast to keep UI alive
                if yaw is None:
                    safety_payload = {
                        "type": "safety",
                        "intended_vector": None,
                        "intended_hits": [],
                        "n_clusters": len(clusters),
                        "avoid_vector": None,
                        "chosen_obstacle": None,
                    }
                    self.state.safety = safety_payload
                    await self.broadcast_safety(safety_payload)
                    await asyncio.sleep(0.05)
                    continue

                yaw = float(yaw) % (2 * np.pi)

                # 3) COMPUTE INTENDED DIRECTION (MUST MATCH YOUR LIDAR FRAME)
                # Your current mapping:
                joy_theta = (np.arctan2(lx, ly)) % (2 * np.pi)

                # IMPORTANT:
                # If you are still using the lidar flip:
                # angles_out = (pi - (angles_rad + yaw)) % 2pi
                # then intended_angle should be:
                # intended_angle = (np.pi - (joy_theta + yaw)) % (2*np.pi)
                #
                # If you are not flipping, keep this:
                intended_angle = (joy_theta + yaw) % (2 * np.pi)

                intended_magnitude = float(np.hypot(lx, ly))
                intended_look = min(self.look_distance,intended_magnitude * float(self.look_distance))
                intended_directional_vector = [float(intended_angle), float(intended_look)]

                # 4) FILTER + CLASSIFY OBSTACLES IN THE INTENDED SLICE
                intended_hits = []
                for cl in clusters:
                    a1, a0 = float(cl[0][0]), float(cl[0][1])
                    r1, r0 = float(cl[1][0]), float(cl[1][1])

                    # keep clusters that overlap the intended ray angle
                    if not self.angle_in_interval(intended_angle, a0, a1):
                        continue

                    # keep clusters within the intended look-ahead length
                    if r0 > intended_look:
                        continue

                    info = self.classify_cluster(cl, angles, ranges)
                    intended_hits.append({"cluster": cl, "info": info})

                # 5) COMPUTE AVOID OUTPUT (PREFERRED SIDE IS FREE SPACE)
                avoid_xy, chosen = self.compute_avoid_output(
                    intended_angle=intended_angle,
                    intended_magnitude=intended_magnitude,
                    intended_hits=intended_hits,
                    angles=angles,
                    ranges=ranges,
                )

                # 6) BROADCAST
                safety_payload = {
                    "type": "safety",
                    "intended_vector": intended_directional_vector,
                    "intended_hits": intended_hits,
                    "n_clusters": len(clusters),
                    "avoid_vector": avoid_xy,      # [vx, vy] in XY frame
                    "chosen_obstacle": chosen,     # debug for UI
                }

                avoid_body = self.world_to_body(avoid_xy, yaw)
                print(f"avoid_xy: {avoid_xy} | avoid_body: {avoid_body} | intended_vector: {intended_directional_vector} \n chosen: {chosen}")

                self.state.safe_axes["LX"] = float(avoid_body[1])
                self.state.safe_axes["LY"] = float(avoid_body[0])

                self.state.safety = safety_payload
                await self.broadcast_safety(safety_payload)
                await asyncio.sleep(0.05)  # ~20 Hz
