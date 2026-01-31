#!/usr/bin/env python3
import asyncio
import json
import math
import numpy as np
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

    3) Pre-filter cluster merge pass (BEFORE classification):
       - connect clusters that are "nearby" in angle+distance
       - If endpoint-to-endpoint angle diff < 1 deg AND endpoint polar distance < merge_dist_tol
         then union/merge them (transitively).

    4) Filter obstacles:
       - include ALL clusters that overlap a "visible arc" of ±visible_arc_deg centered on intended_angle
       - also include ANY cluster with points within zone_radius meters

    5) For each included hit: classify + compute closest point ON geometry to intended ray.

    6) Angle number-line bins debug:
       - 0..2pi index, 4 bins of 90 degrees each
       - populate each bin using GEOMETRY-derived points (NOT raw lidar points)
         - rectangle: sample MANY points along the closest rectangle edge (segment) to the robot
         - line: endpoints by default
         - circle: center point

       - include joystick axes (lx,ly) on the number line as an "intent" point.

    NOTE:
      - Orthonormal projection visualization was removed (not used for control + can look wrong
        when intended ray sits between obstacles).
    """

    def __init__(self, state, ws_port=8766):
        self.state = state

        # --- How far ahead the look-ahead ray extends when stick magnitude is 1.0 ---
        self.look_distance = 5.0

        # --- include all clusters within this radius (meters), regardless of intended angle ---
        self.zone_radius = 1.5

        # --- NEW: "visible arc" filter around intended ray ---
        self.visible_arc_deg = 25.0  # ±30° around intended_angle

        # --- Cluster merge params (pre-classification) ---
        self.merge_angle_tol_deg = 3.5     # degrees
        self.merge_dist_tol = 0.25         # meters (polar point distance tolerance)

        # --- Cluster classification parameters ---
        self.classify_thresh = 0.12
        self.classify_min_distance = 0.2

        # --- Rectangle inflation (classification geometry) ---
        self.inflate_rect = 0.1

        # ---- Fit/residual visualization ----
        self.max_fit_points = 120

        # ---- Closest-point parameters ----
        self.closest_inflate = 0.20     # inflate obstacles when doing circle math
        self.circle_radius_est = 0.24   # meters (approx obstacle thickness for "circle-ish" clusters)

        # ---- Number-line sampling density ----
        self.dense_rect_points = 20     # NEW: many points along closest rectangle edge
        self.dense_line_points = 20      # keep 0 to NOT densify lines by default (endpoints only)

        # ---- NEW: Repulsion zone parameters (5.0 -> 1.5 -> 0.0) ----
        # We keep zone_radius as the "hard" radius (1.5m) for include_by_zone.
        self.zone_outer_radius = 1.2      # meters (soft influence begins)
        self.zone_inner_radius = 0.60      # meters (hard influence boundary)

        # Repulsion scale factors (negative = repel)
        # 5.0 -> 1.5 : contributes toward -1.0
        # 1.5 -> 0.0 : contributes toward -2.0
        self.repulse_outer_gain = 0.75
        self.repulse_inner_gain = 1.5

        # ---- Free-space interval requirements ----
        self.min_free_interval_deg = 20.0
        self.clear_radius = 1.0  # "no points closer than this" constraint

        # ---- Steering clamp (how much we can limit intended angle) ----
        self.max_angle_correction = math.radians(35.0)

        # --- Potential field gains ---
        self.repulsion_gain = 1.0
        self.midpoint_gain = 0.6

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

    def angle_wrap_2pi(self, a: float) -> float:
        """Wrap to [0, 2pi)."""
        return float(a % (2.0 * np.pi))

    def angle_diff(self, a: float, b: float) -> float:
        """Shortest signed difference a-b in [-pi, pi)."""
        return self.angle_wrap(a - b)

    def angle_in_interval(self, theta: float, a0: float, a1: float, padding: float) -> bool:
        """True if theta is in [a0,a1] with wraparound support."""
        theta = self.angle_wrap(theta)
        padding = np.deg2rad(padding)
        a0 = self.angle_wrap(a0)
        a1 = self.angle_wrap(a1)
        obstacle_arc = (a1 - a0) % (2 * np.pi)
        intended_arc = (theta - a0) % (2 * np.pi)
        return intended_arc <= (obstacle_arc + padding) or intended_arc >= (2 * np.pi - padding)

    def midpoint_vector(self, mid_angle):
        """
        Convert free-interval midpoint angle into a local unit vector.
        """
        return (
            math.sin(mid_angle) * self.midpoint_gain,
            math.cos(mid_angle) * self.midpoint_gain,
        )

    def cluster_overlaps_visible_arc(self, cl, intended_angle, arc_deg):
        """
        True if cluster overlaps the arc [intended_angle-arc, intended_angle+arc].
        Uses endpoints + midpoint check (works well for tiny segments).
        """
        a1, a0 = float(cl[0][0]), float(cl[0][1])
        # midpoint angle along the cluster span
        da = float(self.angle_diff(a1, a0))
        a_mid = (a0 + 0.5 * da) % (2.0 * np.pi)

        arc = math.radians(float(arc_deg))
        ia = float(intended_angle)

        # check endpoints + midpoint against the arc window
        for a in (a0, a1, a_mid):
            if abs(float(self.angle_diff(a, ia))) <= arc:
                return True

        # also check if intended_angle lies inside the cluster span when padded by arc
        if self.angle_in_interval(ia, a0, a1, padding=float(arc_deg)):
            return True

        return False

    # ----------------- Basic geometry helpers -----------------
    def _unit(self, v):
        n = float(np.linalg.norm(v))
        if n < 1e-12:
            return np.array([0.0, 0.0], dtype=float)
        return np.asarray(v, dtype=float) / n

    def _ray_dir(self, theta):
        return np.array([math.cos(theta), math.sin(theta)], dtype=float)

    def _cross2(self, a, b):
        return float(a[0] * b[1] - a[1] * b[0])

    def _along(self, p_xy, u_hat):
        return float(np.dot(p_xy, u_hat))

    def _signed_dist_to_line_through_origin(self, p_xy, u_hat):
        """Signed perpendicular distance to line through origin along u_hat."""
        return self._cross2(u_hat, p_xy)

    def _xy_to_polar(self, p_xy):
        x, y = float(p_xy[0]), float(p_xy[1])
        a = (math.atan2(y, x)) % (2 * math.pi)
        r = math.hypot(x, y)
        return a, r

    def _polar_to_xy(self, a, r):
        return np.array([float(r) * math.cos(float(a)), float(r) * math.sin(float(a))], dtype=float)

    # ----------------- Polar utilities -----------------
    def polar_point_distance(self, a1, r1, a2, r2):
        dtheta = self.angle_diff(a2, a1)
        return math.sqrt(r1 * r1 + r2 * r2 - 2 * r1 * r2 * math.cos(dtheta))

    def line_payload(self, cluster):
        return [float(cluster[0][0]), float(cluster[1][0])], [float(cluster[0][1]), float(cluster[1][1])]

    def circular_payload(self, cluster):
        a1, a0 = float(cluster[0][0]), float(cluster[0][1])
        r1, r0 = float(cluster[1][0]), float(cluster[1][1])
        da = self.angle_diff(a1, a0)
        mid_angle = (a0 + 0.5 * da) % (2 * np.pi)
        mid_dist = 0.5 * (r0 + r1)
        return [float(mid_angle), float(mid_dist)]

    # ----------------- Repulsion helpers -----------------
    def bin_index_for_angle(self, a, n_bins):
        a = self.angle_wrap_2pi(float(a))
        bi = int((a / (2.0 * np.pi)) * int(n_bins))
        return max(0, min(int(n_bins) - 1, bi))

    def repulsion_weight(self, r):
        """
        5.0 -> 1.5 : "block" band (weaker)
        1.5 -> 0.0 : "active repel" band (stronger)
        Returns a positive weight magnitude.
        """
        r = float(r)
        if r >= self.zone_outer_radius:
            return 0.0

        # Inner band: 0..1.5
        if r < self.zone_inner_radius:
            # map r: 1.5 -> 0.0 as t: 0 -> 1
            t = (self.zone_inner_radius - r) / max(1e-6, self.zone_inner_radius)
            # stronger as you get closer
            return abs(float(self.repulse_inner_gain))+(t) 

        # Outer band: 1.5..5.0
        t = (self.zone_outer_radius - r) / max(1e-6, (self.zone_outer_radius - self.zone_inner_radius))
        return abs(float(self.repulse_outer_gain)) * max(0.0, min(1.0, t))

    def repulsion_against_intended(self, hits, intended_angle,lx,ly):
        inner_a = []
        outer_a = []
        repulse_x = 0
        repulse_dx = 0
        repulse_y = 0
        repulse_dy = 0
        all_a = []
        for h in hits:
            info = h.get("info", {}) or {}
            for p in h.get("geom_pts",[]):
                if p["r"] <= self.zone_outer_radius:
                    outer_a.append((float(p["a"]), float(p["r"])))
                if p["r"] <= self.zone_inner_radius:
                    inner_a.append(((float(p["a"]), float(p["r"]))))

        tol = np.deg2rad(1.5)
        if abs(lx)>= 0.2 or abs(ly) >=0.2:
            if any(abs(self.angle_diff(a, intended_angle)) <= tol for a, r in outer_a):
                # Looks for candidates that intake the function and combs for angles close to intended direction
                cands = [(i, a, r) for i, (a, r) in enumerate(outer_a)
                        if abs(self.angle_diff(a, intended_angle)) <= tol]
                if not cands:
                    idx = None
                else:
                    # if cands is not empty we comb through candidates to minimize the closest distance
                    idx, a_best, r_best = min(cands, key=lambda t: t[2])  # minimize r
                    #print(a_best)
                    repulse_x = -np.cos(a_best) * self.repulsion_weight(r_best)
                    repulse_y = -np.sin(a_best) * self.repulsion_weight(r_best)
        for a,r in inner_a:
            #print(a)
            repulse_dx += -np.cos(a)# * self.repulsion_weight(r)/count
            repulse_dy += -np.sin(a)# * self.repulsion_weight(r)/count
        mag = np.hypot(repulse_dx,repulse_dy)
        if mag > 1e-6:
            repulse_dx /=mag
            repulse_dy /=mag
            strength = min(1.0,np.sqrt(len(inner_a)/2.5))
            repulse_x += repulse_dx * strength
            repulse_y += repulse_dy * strength
        return repulse_x,repulse_y

    def largest_free_interval_in_bin(self, angle_bins, intended_angle, n_bins, *, min_span_deg=20.0, clear_r=1.0):
        """
        Finds the largest angular interval within the intended bin such that:
          - It spans at least min_span_deg
          - There are NO geometry points with r < clear_r inside that interval

        Returns dict:
          {"ok":bool, "bin_i":int, "a0":..., "a1":..., "span":..., "mid":...,
           "span_deg":..., "mid_deg":..., "blocked_count":int}
        """
        n_bins = int(n_bins) if int(n_bins) > 0 else 10
        ia = self.angle_wrap_2pi(float(intended_angle))
        bi = self.bin_index_for_angle(ia, n_bins)

        b = angle_bins[bi]
        a0 = float(b["a0"])
        a1 = float(b["a1"])

        blocked = []
        for p in (b.get("points", []) or []):
            if p.get("label") == "intent":
                continue
            r = float(p.get("r", 999.0))
            if r < float(clear_r):
                blocked.append(self.angle_wrap_2pi(float(p.get("a", 0.0))))

        blocked = [a for a in blocked if (a >= a0 and a < a1)]

        if not blocked:
            span = a1 - a0
            mid = a0 + 0.5 * span
            ok = span >= math.radians(float(min_span_deg))
            return {
                "ok": bool(ok),
                "bin_i": int(bi),
                "a0": float(a0), "a1": float(a1),
                "span": float(span),
                "mid": float(self.angle_wrap_2pi(mid)),
                "span_deg": float(np.degrees(span)),
                "mid_deg": float(np.degrees(self.angle_wrap(mid))),
                "blocked_count": 0,
            }

        blocked.sort()

        best = None
        gaps = []
        gaps.append((a0, blocked[0]))
        for i in range(len(blocked) - 1):
            gaps.append((blocked[i], blocked[i + 1]))
        gaps.append((blocked[-1], a1))

        for (g0, g1) in gaps:
            span = float(g1 - g0)
            if best is None or span > best["span"]:
                best = {"a0": float(g0), "a1": float(g1), "span": float(span)}

        if best is None:
            return {
                "ok": False,
                "bin_i": int(bi),
                "a0": float(a0), "a1": float(a1),
                "span": 0.0,
                "mid": float(self.angle_wrap_2pi(ia)),
                "span_deg": 0.0,
                "mid_deg": float(np.degrees(self.angle_wrap(ia))),
                "blocked_count": int(len(blocked)),
            }

        min_span = math.radians(float(min_span_deg))
        ok = best["span"] >= min_span
        mid = best["a0"] + 0.5 * best["span"]

        return {
            "ok": bool(ok),
            "bin_i": int(bi),
            "a0": float(best["a0"]), "a1": float(best["a1"]),
            "span": float(best["span"]),
            "mid": float(self.angle_wrap_2pi(mid)),
            "span_deg": float(np.degrees(best["span"])),
            "mid_deg": float(np.degrees(self.angle_wrap(mid))),
            "blocked_count": int(len(blocked)),
        }

    # ----------------- Cluster endpoint helpers (for merging) -----------------
    def _cluster_endpoints_polar(self, cl):
        a1, a0 = float(cl[0][0]), float(cl[0][1])
        r1, r0 = float(cl[1][0]), float(cl[1][1])
        return [(a0, r0), (a1, r1)]

    def _cluster_unwrapped_span(self, cl):
        a1, a0 = float(cl[0][0]), float(cl[0][1])
        da = float(self.angle_diff(a1, a0))
        a_lo = self.angle_wrap_2pi(a0)
        a_hi = a_lo + da
        if a_hi < a_lo:
            a_hi += 2.0 * np.pi
        return float(a_lo), float(a_hi)

    def _clusters_should_merge(self, c1, c2):
        ang_tol = math.radians(float(self.merge_angle_tol_deg))
        dist_tol = float(self.merge_dist_tol)

        e1 = self._cluster_endpoints_polar(c1)
        e2 = self._cluster_endpoints_polar(c2)

        best = None
        for (aA, rA) in e1:
            for (aB, rB) in e2:
                dA = abs(float(self.angle_diff(aA, aB)))
                if dA > ang_tol:
                    continue
                dP = float(self.polar_point_distance(aA, rA, aB, rB))
                if best is None or dP < best:
                    best = dP

        return (best is not None) and (best <= dist_tol)

    def _merge_cluster_group(self, clusters_group):
        endpoints = []
        for cl in clusters_group:
            endpoints.extend(self._cluster_endpoints_polar(cl))

        a_ref = self.angle_wrap_2pi(endpoints[0][0])

        unwrapped = []
        for (a, r) in endpoints:
            a2 = self.angle_wrap_2pi(a)
            while a2 - a_ref > math.pi:
                a2 -= 2.0 * math.pi
            while a2 - a_ref < -math.pi:
                a2 += 2.0 * math.pi
            unwrapped.append((float(a2), float(r)))

        lo = min(unwrapped, key=lambda ar: ar[0])
        hi = max(unwrapped, key=lambda ar: ar[0])

        a_lo = self.angle_wrap_2pi(lo[0])
        a_hi = self.angle_wrap_2pi(hi[0])
        r_lo = float(lo[1])
        r_hi = float(hi[1])

        return [[float(a_hi), float(a_lo)], [float(r_hi), float(r_lo)]]

    def merge_nearby_clusters(self, clusters):
        clusters = list(clusters or [])
        n = len(clusters)
        if n <= 1:
            return clusters

        parent = list(range(n))

        def find(i):
            while parent[i] != i:
                parent[i] = parent[parent[i]]
                i = parent[i]
            return i

        def union(i, j):
            ri, rj = find(i), find(j)
            if ri != rj:
                parent[rj] = ri

        for i in range(n):
            for j in range(i + 1, n):
                if self._clusters_should_merge(clusters[i], clusters[j]):
                    union(i, j)

        groups = {}
        for i in range(n):
            r = find(i)
            groups.setdefault(r, []).append(clusters[i])

        merged = []
        for _, grp in groups.items():
            merged.append(grp[0] if len(grp) == 1 else self._merge_cluster_group(grp))

        merged.sort(key=lambda cl: self._cluster_unwrapped_span(cl)[0])
        return merged

    # ----------------- Rectangle from best-fit (PCA TLS) -----------------
    def rect_xy_from_polar_cluster_bestfit(self, a_seg, r_seg):
        a_seg = np.asarray(a_seg, dtype=float)
        r_seg = np.asarray(r_seg, dtype=float)
        if a_seg.size < 2 or r_seg.size < 2:
            return None

        x = r_seg * np.cos(a_seg)
        y = r_seg * np.sin(a_seg)
        pts = np.column_stack((x, y))

        mu = pts.mean(axis=0)
        X = pts - mu

        C = (X.T @ X) / max(1, X.shape[0])
        w, V = np.linalg.eigh(C)
        v = V[:, np.argmax(w)]
        v = v / (np.linalg.norm(v) + 1e-12)
        n = np.array([-v[1], v[0]], dtype=float)

        s = X @ v
        d = X @ n
        s_min, s_max = float(s.min()), float(s.max())
        d_min, d_max = float(d.min()), float(d.max())

        d_min -= self.inflate_rect
        d_max += self.inflate_rect

        b0 = mu + v * s_min
        b1 = mu + v * s_max

        c0 = b0 + n * d_max
        c1 = b1 + n * d_max
        c2 = b1 + n * d_min
        c3 = b0 + n * d_min

        return [c0.tolist(), c1.tolist(), c2.tolist(), c3.tolist()]

    # ----------------- Fit debug downsampling -----------------
    def _downsample(self, arr, max_n):
        arr = np.asarray(arr)
        if arr.size <= max_n:
            return arr
        step = max(1, int(arr.size // max_n))
        return arr[::step]

    # ----------------- Cluster classification -----------------
    def classify_cluster(self, cluster, angles, ranges):
        a_hi, a_lo = float(cluster[0][0]), float(cluster[0][1])
        r_hi, r_lo = float(cluster[1][0]), float(cluster[1][1])

        angles = np.asarray(angles, dtype=float)
        ranges = np.asarray(ranges, dtype=float)

        in_window = np.array([self.angle_in_interval(a, a_lo, a_hi, padding=0) for a in angles], dtype=bool)
        a_seg = angles[in_window]
        r_seg = ranges[in_window]

        da = float(self.angle_diff(a_hi, a_lo))
        if (a_seg.size <= 5) or (abs(da) < 1e-3):
            return {"label": "degenerate", "n": int(a_seg.size), "reason": "no_points_or_da"}

        if not np.any(r_seg <= self.look_distance):
            return {"label": "degenerate", "n": int(a_seg.size), "reason": "out_of_range"}

        try:
            x = np.array([self.angle_diff(a, a_lo) for a in a_seg], dtype=float)
            slope = (r_hi - r_lo) / da
            r_hat = r_lo + slope * x
            resid = r_seg - r_hat
        except Exception:
            return {"label": "degenerate", "n": int(a_seg.size), "reason": "fit_failed"}
        '''
        fit_dbg = {
            "a_lo": float(a_lo), "a_hi": float(a_hi),
            "r_lo": float(r_lo), "r_hi": float(r_hi),
            "da": float(da),
            "a_seg": self._downsample(a_seg, self.max_fit_points).tolist(),
            "r_seg": self._downsample(r_seg, self.max_fit_points).tolist(),
            "x": self._downsample(x, self.max_fit_points).tolist(),
            "r_hat": self._downsample(r_hat, self.max_fit_points).tolist(),
            "resid": self._downsample(resid, self.max_fit_points).tolist(),
            "resid_max": float(np.max(resid)),
            "resid_min": float(np.min(resid)),
        }
        '''

        max_pos = float(np.max(resid))
        max_neg = float(np.min(resid))

        p1, p2 = self.line_payload(cluster)
        length = float(self.polar_point_distance(p1[0], p1[1], p2[0], p2[1]))

        if (max_pos + abs(max_neg)) > self.classify_thresh and length >= self.classify_min_distance:
            rect_xy = self.rect_xy_from_polar_cluster_bestfit(a_seg, r_seg)
            return {"label": "rectangle", "rect_xy": rect_xy, "n": int(a_seg.size)}#, "fit": fit_dbg}

        if length >= self.classify_min_distance:
            return {"label": "line", "line": [p1, p2], "length": length, "n": int(a_seg.size)}#, "fit": fit_dbg}

        return {"label": "circle", "circle": self.circular_payload(cluster), "n": int(a_seg.size)}#, "fit": fit_dbg}

    # ============================================================
    #  Zone include check
    # ============================================================
    def cluster_min_range(self, cl, angles, ranges):
        a1, a0 = float(cl[0][0]), float(cl[0][1])

        angles = np.asarray(angles, dtype=float)
        ranges = np.asarray(ranges, dtype=float)

        in_window = np.array([self.angle_in_interval(a, a0, a1, padding=0) for a in angles], dtype=bool)
        r_seg = ranges[in_window]
        if r_seg.size > 0:
            return float(np.min(r_seg))

        r_hi, r_lo = float(cl[1][0]), float(cl[1][1])
        return float(min(r_hi, r_lo))

    # ============================================================
    #  Geometry -> points for "angle number-line" bins
    # ============================================================
    def _closest_point_on_segment_to_origin(self, a_xy, b_xy):
        a = np.asarray(a_xy, float)
        b = np.asarray(b_xy, float)
        v = b - a
        vv = float(np.dot(v, v))
        if vv < 1e-12:
            p = a
            return p, float(np.linalg.norm(p))
        t = float(np.dot(-a, v) / vv)
        t = max(0.0, min(1.0, t))
        p = a + t * v
        return p, float(np.linalg.norm(p))

    def _sample_segment_points(self, A, B, n):
        n = int(n)
        n = max(2, min(400, n))
        pts = []
        for i in range(n):
            t = 0.0 if n <= 1 else (i / (n - 1))
            P = (1.0 - t) * A + t * B
            pts.append(P)
        return pts

    def geometry_points_for_bins(self, info):
        """
        Return geometry-derived polar points:
            [{"a": angle, "r": range, "src": "..."} ...]

        NEW:
          - rectangle: sample MANY points along the closest edge to origin (dense)
          - line: endpoints only (unless dense_line_points > 0)
          - circle: center point
        """
        lab = (info or {}).get("label", "")
        out = []

        if lab == "rectangle" and info.get("rect_xy"):
            pts = np.asarray(info["rect_xy"], float)
            best_edge = None
            best_d = None

            # choose closest edge to origin
            for i in range(len(pts)):
                A = pts[i]
                B = pts[(i + 1) % len(pts)]
                _p_cl, d = self._closest_point_on_segment_to_origin(A, B)
                if best_d is None or d < best_d:
                    best_d = d
                    best_edge = (A, B)

            if best_edge is not None:
                A, B = best_edge
                # DENSE sampling along the closest edge
                for P in self._sample_segment_points(A, B, self.dense_rect_points):
                    aP, rP = self._xy_to_polar(P)
                    out.append({"a": float(aP), "r": float(rP), "src": "rect_edge_dense"})
            return out

        if lab == "line" and info.get("line"):
            p1, p2 = info["line"][0], info["line"][1]
            A = self._polar_to_xy(float(p1[0]), float(p1[1]))
            B = self._polar_to_xy(float(p2[0]), float(p2[1]))

            if int(self.dense_line_points) > 2:
                for P in self._sample_segment_points(A, B, int(self.dense_line_points)):
                    aP, rP = self._xy_to_polar(P)
                    out.append({"a": float(aP), "r": float(rP), "src": "line_dense"})
                return out

            aA, rA = self._xy_to_polar(A)
            aB, rB = self._xy_to_polar(B)
            out.append({"a": float(aA), "r": float(rA), "src": "line_A"})
            out.append({"a": float(aB), "r": float(rB), "src": "line_B"})
            return out

        if lab == "circle" and info.get("circle"):
            a, r = float(info["circle"][0]), float(info["circle"][1])
            out.append({"a": float(a), "r": float(r), "src": "circle_center"})
            return out

        return out

    def build_angle_bins(self, hits, *, intended_angle, lx, ly, look_distance, n_bins=10):
        """
        0..2pi number line, 4 bins of 90° each.
        Populated from GEOMETRY points.

        Also includes an "intent" point for joystick input.
        """
        n_bins = int(n_bins) if int(n_bins) > 0 else 10
        edges = np.linspace(0.0, 2.0 * np.pi, n_bins + 1)
        bins = [{"bin_i": i, "a0": float(edges[i]), "a1": float(edges[i + 1]), "points": []} for i in range(n_bins)]

        for h in (hits or []):
            if not h:
                continue
            info = (h or {}).get("info", {}) or {}
            hid = (h or {}).get("hit_id", None)
            lab = info.get("label", "")

            pts = h.get("geom_pts",[])
            if pts is None:
                pts = self.geometry_points_for_bins(info)  # fallback (should be rare)
            #print(pts)
            for p in pts:
                a = self.angle_wrap_2pi(p["a"])
                bi = int((a / (2.0 * np.pi)) * n_bins)
                bi = max(0, min(n_bins - 1, bi))
                bins[bi]["points"].append({
                    "hit_id": hid,
                    "label": lab,
                    "a": float(a),
                    "r": float(p["r"]),
                    "src": p.get("src", ""),
                    "include_by": (h or {}).get("include_by", {}),
                })

        # intent point
        mag = float(np.hypot(float(lx), float(ly)))
        mag = max(0.0, min(1.0, mag))
        r_intent = float(mag * float(look_distance))

        a_intent = self.angle_wrap_2pi(float(intended_angle))
        bi = int((a_intent / (2.0 * np.pi)) * n_bins)
        bi = max(0, min(n_bins - 1, bi))
        bins[bi]["points"].append({
            "hit_id": 0,
            "label": "intent",
            "a": float(a_intent),
            "r": float(r_intent),
            "src": "axes_LX_LY",
            "lx": float(lx),
            "ly": float(ly),
            "mag": float(mag),
        })

        return bins

    # ----------------- Joy frame <-> angle helpers -----------------
    def joy_to_angle(self, lx, ly):
        """NOTE: your convention is atan2(lx, ly) (swapped), keep it consistent."""
        return float(np.arctan2(float(lx), float(ly)) % (2.0 * np.pi))

    def angle_to_joy(self, joy_theta, mag):
        """
        Inverse of atan2(lx, ly) = theta:
          lx = sin(theta)*mag
          ly = cos(theta)*mag
        """
        m = float(max(0.0, min(1.0, mag)))
        return float(math.sin(float(joy_theta)) * m), float(math.cos(float(joy_theta)) * m)

    # ============================================================
    #  Main loop
    # ============================================================
    async def run(self):
        print(f"[SAFETY] WebSocket server on port {self.ws_port}")
        async with websockets.serve(self.ws_handler, "0.0.0.0", self.ws_port):
            print("[SAFETY] Listening for close obstacles and controller inputs...")
            lx = ly = 0.0
            hit_id_counter = 1

            while True:
                # 1) READ LIDAR PAYLOAD
                lidar_data = self.state.lidar_close or {}
                angles = np.asarray(lidar_data.get("angles", []), dtype=float)
                ranges = np.asarray(lidar_data.get("ranges", []), dtype=float)
                clusters_raw = lidar_data.get("clusters", []) or []
                yaw = float(lidar_data.get("yaw", 0.0)) % (2 * np.pi)

                # 2) READ DRIVER INTENT (lx, ly)
                if getattr(self.state, "robot_current", 0) == 1:
                    lx = float(getattr(self.state, "axes", {}).get("LX", 0.0))
                    ly = float(getattr(self.state, "axes", {}).get("LY", 0.0))
                if getattr(self.state, "robot_current", 0) >= 2:
                    lx = float(getattr(self.state, "command_vector", {}).get("LX", 0.0))
                    ly = float(getattr(self.state, "command_vector", {}).get("LY", 0.0))

                # 3) COMPUTE INTENDED DIRECTION (MUST MATCH YOUR LIDAR FRAME)
                joy_theta = self.joy_to_angle(lx, ly)
                intended_angle = (joy_theta + yaw) % (2 * np.pi)
                intended_magnitude = float(np.hypot(lx, ly))
                intended_vector = [float(intended_angle), float(self.look_distance)]

                # 4) Merge nearby clusters BEFORE filtering/classification
                clusters = self.merge_nearby_clusters(clusters_raw)

                # 5) FILTER + CLASSIFY OBSTACLES
                hits = []
                for cl in clusters:
                    rmin = self.cluster_min_range(cl, angles, ranges)

                    include_by_zone = (rmin <= float(self.zone_radius))
                    include_by_visible_arc = self.cluster_overlaps_visible_arc(
                        cl, intended_angle, arc_deg=float(self.visible_arc_deg)
                    )

                    if not (include_by_visible_arc or include_by_zone):
                        continue

                    info = self.classify_cluster(cl, angles, ranges)
                    if info.get("label") == "degenerate":
                        continue
                    geom_pts = self.geometry_points_for_bins(info)
                    hit = {
                        "hit_id": int(hit_id_counter),
                        "cluster": cl,
                        "info": info,
                        "geom_pts": geom_pts,
                        "r_min": float(rmin),
                        "include_by": {
                            "visible_arc": bool(include_by_visible_arc),
                            "zone": bool(include_by_zone),
                        },
                    }
                    hit_id_counter += 1
                    hits.append(hit)

                # 6) Build 0..2pi "number-line" bins using GEOMETRY points
                n_bins = 10
                angle_bins = self.build_angle_bins(
                    hits,
                    intended_angle=float(intended_angle),
                    lx=float(lx),
                    ly=float(ly),
                    look_distance=float(self.look_distance),
                    n_bins=n_bins
                )

                # 6.5) Find free interval in intended bin (still used for midpoint bias)
                free = self.largest_free_interval_in_bin(
                    angle_bins,
                    intended_angle=float(intended_angle),
                    n_bins=n_bins,
                    min_span_deg=float(self.min_free_interval_deg),
                    clear_r=float(self.clear_radius),
                )

                #rint(repulse_x,repulse_y)
                # repulse_x, repulse_y are WORLD frame (unit)
                # yaw is robot heading in world
                repulse_wx, repulse_wy = self.repulsion_against_intended(hits, intended_angle, lx, ly)
                s = np.cos(yaw)
                c = np.sin(yaw)
                repulse_wx_corrected = (c*repulse_wx*-1 + s*repulse_wy)  
                repulse_wy_corrected = (s*repulse_wx + c*repulse_wy)
                
                # controller input  = repulse_wx + lx
                # 3) WORLD free-space vector (only if ok)
                free_wx = 0.0
                free_wy = 0.0
                if free.get("ok") and free.get("span", 0.0) > 0.25:
                    #span_norm = np.clip(free["span_deg"] / 45.0, 0.0, 1.0)  # 0..1
                    #w = np.log1p(9.0 * span_norm) / np.log1p(9.0)          # 0..1
                    # free["mid"] is a WORLD angle -> use cos/sin
                    #print(f"w: {w:.2f}")
                    #free["mid"] = (free["mid"] +np.pi/2) % (2*np.pi)
                    #print(self.angle_diff(intended_angle,free["mid"]))
                    free_wx = ((intended_angle - free["mid"]))%(2*np.pi)
                    free_wdx = s*np.cos(free_wx) + -c*np.sin(free_wx)
                    free_wdy = c*np.sin(free_wx) + s*np.sin(free_wx)
                    #print(free_wdx,free_wdy)
                #print(f"free: {free['mid']:.2f} yaw: {yaw:.2f} intended: {intended_angle:.2f}") # We can take free_wx as our rtational vector
                # 4) Combine in WORLD (this is the correct place)
                kR = 1.15   # repulsion strength
                kF = 0.32   # free-space strength (tune)
                # 5) WORLD -> ROBOT for controller axes
                lx_corr =  repulse_wx_corrected * kR + lx*.60 -free_wdy*0.32 
                ly_corr =  repulse_wy_corrected * kR + ly *.60
                # 6) Your convention angle (robot frame)
                corrected_angle = self.angle_wrap(np.arctan2(ly_corr, lx_corr))
                #print(
                    #f"repulse_w ({repulse_wx:.2f},{repulse_wy:.2f}) "
                    #f"repulse_w_corrected ({repulse_wx_corrected:.2f},{repulse_wy_corrected:.2f},{free_wx:.2f} ) "
                    #f"yaw: {yaw:.2f} | cos: {c:.2f} sin: {s:.2f} angle: {np.arctan2(repulse_wy,repulse_wx):.2f}"
                #)
                self.state.safe_axes["LX"] = lx_corr
                self.state.safe_axes["LY"] = ly_corr
                self.state.safe_axes["W"] = free_wx 
                                #self.state.safe_axes["W"] = free_wx *-.625
                payload = {
                    "type": "safety",
                    "yaw": float(yaw),

                    # intent (raw + corrected)
                    "axes": {"LX": float(lx), "LY": float(ly)},
                   # "axes_corrected": {"LX": float(corrected_lx), "LY": float(corrected_ly)},

                    "intended_angle": float(intended_angle),
                    "intended_vector": intended_vector,
                    "intended_magnitude": float(intended_magnitude),

                    # correction debug
                    "intended_bin_i": int(self.bin_index_for_angle(intended_angle, n_bins)),
                    #"has_repulsion_pts": bool(has_repulsion_pts),
                    "free_interval": free,
                    #"corrected_angle": float(corrected_angle),
                    "max_angle_correction_deg": float(np.degrees(self.max_angle_correction)),

                    "visible_arc_deg": float(self.visible_arc_deg),
                    "zone_radius": float(self.zone_radius),
                    "zone_outer_radius": float(self.zone_outer_radius),
                    "zone_inner_radius": float(self.zone_inner_radius),

                    # cluster merge debug
                    "cluster_merge": {
                        "merge_angle_tol_deg": float(self.merge_angle_tol_deg),
                        "merge_dist_tol": float(self.merge_dist_tol),
                        "n_clusters_raw": int(len(clusters_raw)),
                        "n_clusters_merged": int(len(clusters)),
                    },

                    # hits + bins
                    "intended_hits": hits,
                    "angle_bins": angle_bins,
                }


                await self.broadcast_safety(payload)
                await asyncio.sleep(0.10)  # ~20 Hz
