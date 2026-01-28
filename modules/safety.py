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
        self.classify_thresh = 0.2
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

    # ----------------- Payload builders -----------------
    def rectangular_payload(self, payload, cluster):
        a1, a0 = float(cluster[0][0]), float(cluster[0][1])
        r1, r0 = float(cluster[1][0]), float(cluster[1][1])
        a_pos, r_pos, a_neg, r_neg = map(float, payload)

        def closest_extreme_to(a_end):
            d_pos = abs(self.angle_diff(a_pos, a_end))
            d_neg = abs(self.angle_diff(a_neg, a_end))
            return r_pos if d_pos <= d_neg else r_neg

        r_far0 = max(closest_extreme_to(a0), r0)
        r_far1 = max(closest_extreme_to(a1), r1)

        return [(a0, r0), (a0, r_far0), (a1, r_far1), (a1, r1)]

    def line_payload(self, cluster):
        return [float(cluster[0][0]), float(cluster[1][0])], [float(cluster[0][1]), float(cluster[1][1])]

    def circular_payload(self, cluster):
        a1, a0 = float(cluster[0][0]), float(cluster[0][1])
        r1, r0 = float(cluster[1][0]), float(cluster[1][1])

        da = self.angle_diff(a1, a0)
        mid_angle = (a0 + 0.5 * da) % (2 * np.pi)
        mid_dist = 0.5 * (r0 + r1)
        return [float(mid_angle), float(mid_dist)]

    # ----------------- Classification -----------------
    def classify_cluster(self, cluster, angles, ranges, thresh=0.2, min_points=5):
        a_hi, a_lo = float(cluster[0][0]), float(cluster[0][1])
        r_hi, r_lo = float(cluster[1][0]), float(cluster[1][1])

        angles = np.asarray(angles, dtype=float)
        ranges = np.asarray(ranges, dtype=float)

        in_window = np.array([self.angle_in_interval(a, a_lo, a_hi) for a in angles], dtype=bool)
        a_seg = angles[in_window]
        r_seg = ranges[in_window]

        if a_seg.size < min_points:
            return {"label": "Circle", "n": int(a_seg.size), "circle": self.circular_payload(cluster)}

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
            payload = [float(a_seg[i_pos]), float(r_seg[i_pos]),
                       float(a_seg[i_neg]), float(r_seg[i_neg])]
            corners = self.rectangular_payload(payload, cluster)
            return {
                "label": "rectangle_like",
                "n": int(a_seg.size),
                "max_pos": max_pos,
                "max_neg": max_neg,
                "payload": payload,
                "corners": corners,
            }

        p1, p2 = self.line_payload(cluster)
        return {
            "label": "line_like",
            "n": int(a_seg.size),
            "max_abs": float(np.max(np.abs(resid))),
            "line": (p1, p2),
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
                print(self.state.safety)
                await self.broadcast_safety(safety_payload)

                await asyncio.sleep(0.05)  # ~20 Hz
