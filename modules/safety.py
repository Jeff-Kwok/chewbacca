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
       - compute intended_magnitude and intended_look distance

    3) Filter obstacles along the intended ray slice:
       - keep only clusters whose angular span contains intended_angle
       - keep only clusters within intended_look
       - classify each into: rectangle / line / circle and attach geometry payloads

    4) For each hit: compute the closest point ON THE GEOMETRY to:
       - (A) the intended ray (so we don't pick the far edge when ray goes through a rectangle)
       - (B) and also record closest-to-robot if you want it
       In practice we choose the point with MIN along-distance on the ray that intersects geometry.
       If ray doesn’t intersect geometry, fallback to closest-to-ray distance.

    5) Build an ORTHOGONAL line through the chosen closest point and pick midpoint of largest
       free-space interval along that orthogonal line, using intervals blocked by rectangles/circles/lines.

    6) Broadcast everything over WS:
       - intended_vector, intended_hits (with info.fit + closest + free_space)
    """

    def __init__(self, state, ws_port=8766):
        self.state = state

        # --- How far ahead the look-ahead ray extends when stick magnitude is 1.0 ---
        self.look_distance = 2.5

        # --- Cluster classification parameters ---
        self.classify_thresh = 0.12
        self.classify_min_distance = 0.2

        # --- Rectangle inflation (classification geometry) ---
        self.inflate_rect = 0.05

        # ---- Fit/residual visualization ----
        self.max_fit_points = 120

        # ---- Closest-point / free-space parameters ----
        self.closest_inflate = 0.06     # inflate obstacles when computing intersections/intervals
        self.free_span_deg = 60.0       # conceptually, but we use a true XY orthogonal line
        self.free_max_step = 2.0        # meters left/right on orthogonal line to search
        self.circle_radius_est = 0.24   # meters (approx obstacle thickness for "circle-ish" clusters)

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

    def angle_in_interval(self, theta: float, a0: float, a1: float, padding: float) -> bool:
        """True if theta is in [a0,a1] with wraparound support."""
        theta = self.angle_wrap(theta)
        padding = np.deg2rad(padding)
        a0 = self.angle_wrap(a0)
        a1 = self.angle_wrap(a1)
        obstacle_arc = (a1 - a0) % (2*np.pi)
        intended_arc = (theta - a0) % (2*np.pi)
        return intended_arc <= (obstacle_arc + padding) or intended_arc >= (2*np.pi - padding)

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
        """
        Returns dict like:
          {"label":"rectangle","rect_xy":[...], "fit": {...}}
          {"label":"line","line":[[a,r],[a,r]], "fit": {...}}
          {"label":"circle","circle":[a,r], "fit": {...}}
        """
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

        max_pos = float(np.max(resid))
        max_neg = float(np.min(resid))

        p1, p2 = self.line_payload(cluster)
        length = float(self.polar_point_distance(p1[0], p1[1], p2[0], p2[1]))

        # Rectangle if thick + long enough
        if (max_pos + abs(max_neg)) > self.classify_thresh and length >= self.classify_min_distance:
            rect_xy = self.rect_xy_from_polar_cluster_bestfit(a_seg, r_seg)
            return {"label": "rectangle", "rect_xy": rect_xy, "n": int(a_seg.size), "fit": fit_dbg}

        # Line if long enough
        if length >= self.classify_min_distance:
            return {"label": "line", "line": [p1, p2], "length": length, "n": int(a_seg.size), "fit": fit_dbg}

        # Otherwise circle-ish
        return {"label": "circle", "circle": self.circular_payload(cluster), "n": int(a_seg.size), "fit": fit_dbg}

    # ============================================================
    #  Closest point ON geometry to the intended RAY
    # ============================================================
    def _ray_segment_intersection(self, u_hat, a_xy, b_xy):
        """
        Ray from origin: p(t)=t*u_hat, t>=0
        Segment: a + s*(b-a), s in [0,1]
        Return smallest positive t if intersects, else None.
        """
        u = np.asarray(u_hat, float)
        a = np.asarray(a_xy, float)
        b = np.asarray(b_xy, float)
        v = b - a

        denom = self._cross2(u, v)
        if abs(denom) < 1e-12:
            return None

        # Solve t*u = a + s*v
        t = self._cross2(a, v) / denom
        s = self._cross2(a, u) / denom
        if t >= 0.0 and 0.0 <= s <= 1.0:
            return float(t)
        return None

    def _closest_point_on_segment_to_ray(self, u_hat, a_xy, b_xy):
        """
        Fallback: choose point on segment minimizing distance to ray (line through origin along u_hat),
        but preferring points in front (t>=0).
        Returns dict with closest point and (t, abs_dist).
        """
        u = self._unit(u_hat)
        a = np.asarray(a_xy, float)
        b = np.asarray(b_xy, float)
        v = b - a
        vv = float(np.dot(v, v))
        if vv < 1e-12:
            p = a
            t = self._along(p, u)
            absd = abs(self._signed_dist_to_line_through_origin(p, u))
            return {"p": p, "t": float(t), "absd": float(absd)}

        # sample endpoints + projection of origin-line onto segment via minimal approach (cheap)
        candidates = [a, b]

        # add point on segment closest to the infinite line through origin in direction u
        # approximate by minimizing |cross(u, a + s v)| (2D). Do a small analytic solve:
        # cross(u, a + s v) = cross(u,a) + s*cross(u,v)
        cu_a = self._cross2(u, a)
        cu_v = self._cross2(u, v)
        if abs(cu_v) > 1e-12:
            s0 = -cu_a / cu_v
            s0 = max(0.0, min(1.0, float(s0)))
            candidates.append(a + s0 * v)

        best = None
        for p in candidates:
            t = self._along(p, u)
            if t < 0.0:
                continue
            absd = abs(self._signed_dist_to_line_through_origin(p, u))
            if best is None or absd < best["absd"] or (abs(absd - best["absd"]) < 1e-6 and t < best["t"]):
                best = {"p": p, "t": float(t), "absd": float(absd)}
        if best is None:
            # everything behind -> pick endpoint with largest t (least behind) for stability
            p = a if self._along(a, u) > self._along(b, u) else b
            t = self._along(p, u)
            absd = abs(self._signed_dist_to_line_through_origin(p, u))
            return {"p": p, "t": float(t), "absd": float(absd)}
        return best

    def closest_point_to_intended_ray(self, info, intended_angle):
        """
        Returns:
          {
            "closest_point_xy":[x,y],
            "closest_point_polar":[a,r],
            "t_hit": t,           # along intended ray (>=0) if meaningful
            "abs_dist": d         # perp dist to ray line (for fallback)
          }
        Priority:
          1) If ray intersects geometry => choose smallest t intersection (closest edge facing robot)
          2) else fallback to closest-to-ray distance
        """
        u = self._unit(self._ray_dir(float(intended_angle)))
        lab = info.get("label", "")

        # ---- rectangle: intersect ray with polygon edges, take smallest t ----
        if lab == "rectangle" and info.get("rect_xy"):
            pts = np.asarray(info["rect_xy"], float)
            best_t = None
            for i in range(len(pts)):
                a = pts[i]
                b = pts[(i + 1) % len(pts)]
                t = self._ray_segment_intersection(u, a, b)
                if t is not None and (best_t is None or t < best_t):
                    best_t = t
            if best_t is not None:
                p = best_t * u
                a_p, r_p = self._xy_to_polar(p)
                return {
                    "closest_point_xy": [float(p[0]), float(p[1])],
                    "closest_point_polar": [float(a_p), float(r_p)],
                    "t_hit": float(best_t),
                    "abs_dist": 0.0
                }

            # fallback: choose closest point on rectangle edges to ray
            best = None
            for i in range(len(pts)):
                a = pts[i]
                b = pts[(i + 1) % len(pts)]
                cand = self._closest_point_on_segment_to_ray(u, a, b)
                if best is None or cand["absd"] < best["absd"] or (abs(cand["absd"] - best["absd"]) < 1e-6 and cand["t"] < best["t"]):
                    best = cand
            p = best["p"]
            a_p, r_p = self._xy_to_polar(p)
            return {
                "closest_point_xy": [float(p[0]), float(p[1])],
                "closest_point_polar": [float(a_p), float(r_p)],
                "t_hit": float(best["t"]),
                "abs_dist": float(best["absd"])
            }

        # ---- circle-ish: intersect ray with disk around center ----
        if lab == "circle" and info.get("circle"):
            a_c, r_c = float(info["circle"][0]), float(info["circle"][1])
            c = np.array([r_c * math.cos(a_c), r_c * math.sin(a_c)], dtype=float)
            rad = float(info.get("radius_est", self.circle_radius_est)) + float(self.closest_inflate)

            # Solve |t*u - c|^2 = rad^2
            # t^2 - 2 t (u·c) + |c|^2 - r^2 = 0
            uc = float(np.dot(u, c))
            cc = float(np.dot(c, c))
            disc = uc*uc - (cc - rad*rad)
            if disc >= 0.0:
                sdisc = math.sqrt(max(0.0, disc))
                t1 = uc - sdisc
                t2 = uc + sdisc
                # smallest positive t
                t_candidates = [t for t in [t1, t2] if t >= 0.0]
                if t_candidates:
                    t = min(t_candidates)
                    p = t * u
                    a_p, r_p = self._xy_to_polar(p)
                    return {
                        "closest_point_xy": [float(p[0]), float(p[1])],
                        "closest_point_polar": [float(a_p), float(r_p)],
                        "t_hit": float(t),
                        "abs_dist": 0.0
                    }

            # fallback: closest point on circle to ray direction line
            # project center onto ray: t0=uc (can be negative)
            t0 = max(0.0, uc)
            p0 = t0 * u
            v = c - p0
            nv = float(np.linalg.norm(v))
            if nv < 1e-9:
                # ray goes through center; closest is at t0 - rad (front)
                t = max(0.0, t0 - rad)
                p = t * u
            else:
                # move from center toward ray by radius
                p = c - (rad / nv) * v
                # ensure in front
                if float(np.dot(p, u)) < 0.0:
                    p = c + (rad / nv) * v
            a_p, r_p = self._xy_to_polar(p)
            t = float(np.dot(p, u))
            d = abs(self._signed_dist_to_line_through_origin(p, u))
            return {
                "closest_point_xy": [float(p[0]), float(p[1])],
                "closest_point_polar": [float(a_p), float(r_p)],
                "t_hit": float(t),
                "abs_dist": float(d)
            }

        # ---- line segment obstacle: just treat as segment ----
        if lab == "line" and info.get("line"):
            p1, p2 = info["line"][0], info["line"][1]
            a_xy = np.array([float(p1[1]) * math.cos(float(p1[0])), float(p1[1]) * math.sin(float(p1[0]))], dtype=float)
            b_xy = np.array([float(p2[1]) * math.cos(float(p2[0])), float(p2[1]) * math.sin(float(p2[0]))], dtype=float)

            t = self._ray_segment_intersection(u, a_xy, b_xy)
            if t is not None:
                p = t * u
                a_p, r_p = self._xy_to_polar(p)
                return {
                    "closest_point_xy": [float(p[0]), float(p[1])],
                    "closest_point_polar": [float(a_p), float(r_p)],
                    "t_hit": float(t),
                    "abs_dist": 0.0
                }

            cand = self._closest_point_on_segment_to_ray(u, a_xy, b_xy)
            p = cand["p"]
            a_p, r_p = self._xy_to_polar(p)
            return {
                "closest_point_xy": [float(p[0]), float(p[1])],
                "closest_point_polar": [float(a_p), float(r_p)],
                "t_hit": float(cand["t"]),
                "abs_dist": float(cand["absd"])
            }

        return None

    # ============================================================
    #  Free-space midpoint on orthogonal line through closest point
    # ============================================================
    def _merge_intervals(self, intervals):
        if not intervals:
            return []
        intervals = sorted(intervals, key=lambda t: t[0])
        out = [list(intervals[0])]
        for a, b in intervals[1:]:
            if a <= out[-1][1]:
                out[-1][1] = max(out[-1][1], b)
            else:
                out.append([a, b])
        return [(x[0], x[1]) for x in out]

    def _subtract_intervals(self, base, blocks):
        L, R = base
        free = []
        cur = L
        for a, b in blocks:
            if b <= cur:
                continue
            if a > cur:
                free.append((cur, min(a, R)))
            cur = max(cur, b)
            if cur >= R:
                break
        if cur < R:
            free.append((cur, R))
        return [(a, b) for a, b in free if (b - a) > 1e-4]

    def _occupied_interval_circle_on_line(self, p0, n_hat, c_xy, radius, inflate=0.0):
        p0 = np.asarray(p0, float)
        n = self._unit(np.asarray(n_hat, float))
        c = np.asarray(c_xy, float)
        r = float(radius) + float(inflate)

        dvec = c - p0
        s_c = float(np.dot(dvec, n))
        perp = dvec - s_c * n
        d = float(np.linalg.norm(perp))
        if d >= r:
            return None
        ds = math.sqrt(max(0.0, r*r - d*d))
        return (s_c - ds, s_c + ds)

    def _occupied_interval_segment_on_line(self, p0, n_hat, a_xy, b_xy, inflate=0.0):
        n = self._unit(np.asarray(n_hat, float))
        p0 = np.asarray(p0, float)
        a = np.asarray(a_xy, float)
        b = np.asarray(b_xy, float)

        v = b - a
        denom = self._cross2(n, v)
        if abs(denom) < 1e-12:
            sa = float(np.dot(a - p0, n))
            sb = float(np.dot(b - p0, n))
            lo, hi = (sa, sb) if sa <= sb else (sb, sa)
            return (lo - inflate, hi + inflate)

        s = self._cross2((a - p0), v) / denom
        t = self._cross2((a - p0), n) / denom
        if 0.0 <= t <= 1.0:
            return (float(s - inflate), float(s + inflate))
        return None

    def _occupied_interval_rect_on_line(self, p0, n_hat, rect_xy, inflate=0.0):
        pts = np.asarray(rect_xy, float)
        if pts.ndim != 2 or pts.shape[1] != 2 or pts.shape[0] < 3:
            return None

        n = self._unit(np.asarray(n_hat, float))
        p0 = np.asarray(p0, float)
        hits_s = []
        for i in range(len(pts)):
            a = pts[i]
            b = pts[(i + 1) % len(pts)]
            v = b - a
            denom = self._cross2(n, v)
            if abs(denom) < 1e-12:
                continue
            s = self._cross2((a - p0), v) / denom
            t = self._cross2((a - p0), n) / denom
            if 0.0 <= t <= 1.0:
                hits_s.append(float(s))

        if len(hits_s) >= 2:
            return (min(hits_s) - inflate, max(hits_s) + inflate)

        # fallback projection
        proj = [float(np.dot(p - p0, n)) for p in pts]
        return (min(proj) - inflate, max(proj) + inflate)

    def free_space_midpoint_on_orthogonal(self, intended_angle, closest_point_xy, obstacles):
        u_hat = self._unit(self._ray_dir(float(intended_angle)))
        n_hat = np.array([-u_hat[1], u_hat[0]], dtype=float)
        p0 = np.asarray(closest_point_xy, float)

        base = (-float(self.free_max_step), float(self.free_max_step))
        blocks = []

        for h in obstacles:
            info = (h or {}).get("info", {}) or {}
            lab = info.get("label", "")

            if lab == "rectangle" and info.get("rect_xy"):
                iv = self._occupied_interval_rect_on_line(p0, n_hat, info["rect_xy"], inflate=self.closest_inflate)
                if iv: blocks.append(iv)

            elif lab == "circle" and info.get("circle"):
                a, r = float(info["circle"][0]), float(info["circle"][1])
                c_xy = np.array([r * math.cos(a), r * math.sin(a)], dtype=float)
                rad = float(info.get("radius_est", self.circle_radius_est))
                iv = self._occupied_interval_circle_on_line(p0, n_hat, c_xy, rad, inflate=self.closest_inflate)
                if iv: blocks.append(iv)

            elif lab == "line" and info.get("line"):
                p1, p2 = info["line"][0], info["line"][1]
                a_xy = np.array([float(p1[1]) * math.cos(float(p1[0])),
                                 float(p1[1]) * math.sin(float(p1[0]))], dtype=float)
                b_xy = np.array([float(p2[1]) * math.cos(float(p2[0])),
                                 float(p2[1]) * math.sin(float(p2[0]))], dtype=float)
                iv = self._occupied_interval_segment_on_line(p0, n_hat, a_xy, b_xy, inflate=self.closest_inflate)
                if iv: blocks.append(iv)

        # clip + merge
        clipped = []
        for a, b in blocks:
            a2 = max(base[0], float(a))
            b2 = min(base[1], float(b))
            if b2 > a2:
                clipped.append((a2, b2))
        merged = self._merge_intervals(clipped)

        free = self._subtract_intervals(base, merged)
        if not free:
            return None

        best = max(free, key=lambda ab: (ab[1] - ab[0]))
        s_mid = 0.5 * (best[0] + best[1])
        p_mid = p0 + s_mid * self._unit(n_hat)
        a_mid, r_mid = self._xy_to_polar(p_mid)

        return {
            "best_free": [float(best[0]), float(best[1])],
            "s_mid": float(s_mid),
            "midpoint_xy": [float(p_mid[0]), float(p_mid[1])],
            "midpoint_polar": [float(a_mid), float(r_mid)],
        }
    def clamp(self, v, lo=-1.0, hi=1.0):
        return max(lo, min(hi, float(v)))

    def world_to_body_xy(self, p_xy, yaw):
        """
        Rotate a point/vector from world/plot frame into robot/body frame.
        yaw is robot heading in world/plot frame.
        """
        x, y = float(p_xy[0]), float(p_xy[1])
        cy = math.cos(yaw)
        sy = math.sin(yaw)
        # R(-yaw) * [x,y]
        bx =  cy * x + sy * y
        by = -sy * x + cy * y
        return [bx, by]

    def free_midpoint_to_axes(self, free_mid_xy, yaw, mag=1.0):
        """
        free_mid_xy: [x,y] in world frame.
        Returns (LX, LY) in body/joystick convention:
        LY forward, LX right
        """
        # 1) direction in world (origin -> point)
        v_world = np.array([float(free_mid_xy[0]), float(free_mid_xy[1])], dtype=float)
        n = float(np.linalg.norm(v_world))
        if n < 1e-9:
            return 0.0, 0.0

        v_world /= n

        # 2) rotate into body frame so "forward" is consistent
        bx, by = self.world_to_body_xy(v_world, yaw)

        # 3) map body -> joystick
        # body x = forward, body y = left (based on your world_to_body)
        # you want LY forward, LX right:
        LY = bx
        LX = -by  # because body +y is left, so right is negative

        # 4) scale + clamp
        LX = self.clamp(LX * mag)
        LY = self.clamp(LY * mag)
        return LX, LY

    # ============================================================
    #  Main loop
    # ============================================================
    async def run(self):
        print(f"[SAFETY] WebSocket server on port {self.ws_port}")
        async with websockets.serve(self.ws_handler, "0.0.0.0", self.ws_port):
            print("[SAFETY] Listening for close obstacles and controller inputs...")
            lx = ly = 0.0
            while True:
                # 1) READ LIDAR PAYLOAD
                lidar_data = self.state.lidar_close or {}
                angles = np.asarray(lidar_data.get("angles", []), dtype=float)
                ranges = np.asarray(lidar_data.get("ranges", []), dtype=float)
                clusters = lidar_data.get("clusters", []) or []
                yaw = float(lidar_data.get("yaw", 0.0)) % (2 * np.pi)

                # 2) READ DRIVER INTENT (lx, ly)
                if getattr(self.state, "robot_current", 0) == 1:
                    lx = float(getattr(self.state, "axes", {}).get("LX", 0.0))
                    ly = float(getattr(self.state, "axes", {}).get("LY", 0.0))

                # 3) COMPUTE INTENDED DIRECTION (MUST MATCH YOUR LIDAR FRAME)
                joy_theta = (np.arctan2(lx, ly)) % (2 * np.pi)
                intended_angle = (joy_theta + yaw) % (2 * np.pi)
                intended_magnitude = float(np.hypot(lx, ly))
                intended_look = min(self.look_distance, intended_magnitude * float(self.look_distance))
                intended_vector = [float(intended_angle), float(intended_look)]

                # 4) FILTER + CLASSIFY OBSTACLES IN THE INTENDED SLICE
                intended_hits = []
                for cl in clusters:
                    a1, a0 = float(cl[0][0]), float(cl[0][1])
                    if not self.angle_in_interval(intended_angle, a0, a1, padding=25):
                        continue

                    info = self.classify_cluster(cl, angles, ranges)
                    if info.get("label") == "degenerate":
                        continue

                    hit = {"cluster": cl, "info": info}

                    # ---- closest point (for HTML red dot) ----
                    closest = self.closest_point_to_intended_ray(info, intended_angle)
                    if closest:
                        hit["closest"] = closest
                        # Also print polar for ease of interpretation
                        ap, rp = closest["closest_point_polar"]
                        print(f"[CLOSEST] label={info.get('label')}  a={ap:.3f}  r={rp:.3f}  t={closest.get('t_hit',0):.3f}")

                    intended_hits.append(hit)

                # 5) Choose the "active" hit by smallest positive t_hit (ray intersection wins)
                active_idx = None
                active_t = None
                for i, h in enumerate(intended_hits):
                    c = h.get("closest")
                    if not c:
                        continue
                    t = float(c.get("t_hit", 1e9))
                    if t >= 0.0 and (active_t is None or t < active_t):
                        active_t = t
                        active_idx = i

                # 6) Free-space midpoint on orthogonal line through active closest point
                if active_idx is not None:
                    p0 = intended_hits[active_idx]["closest"]["closest_point_xy"]
                    fs = self.free_space_midpoint_on_orthogonal(intended_angle, p0, intended_hits)
                    if fs:
                        intended_hits[active_idx]["free_space"] = fs

                        # ---- NEW: convert free midpoint to joystick axes ----
                        mid_xy = fs["midpoint_xy"]              # [x,y] in world frame
                        LX, LY = self.free_midpoint_to_axes(mid_xy, yaw, mag=intended_magnitude)

                        # publish to your control layer (choose ONE approach)
                        self.state.safe_axes["LX"] = float(LX*-1*.80)
                        self.state.safe_axes["LY"] = float(LY*.80)

                        # optional debug print
                        a_mid, r_mid = fs["midpoint_polar"]
                        print(f"[FREE->AXES] mid(a={a_mid:.3f}, r={r_mid:.3f}) -> LX={LX:.3f} LY={LY:.3f}")

                # 7) BROADCAST what the HTML expects
                payload = {
                    "type": "safety",
                    "yaw": float(yaw),
                    "intended_vector": intended_vector,
                    "intended_hits": intended_hits,
                    "n_clusters": int(len(clusters)),
                }
                await self.broadcast_safety(payload)

                await asyncio.sleep(0.05)  # ~20 Hz
