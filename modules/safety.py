import asyncio
from . import config
import numpy as np
import math

class SafetyModule:
    def __init__(self, state):
        self.state = state

    def angle_wrap(self, a):
        return (a + np.pi) % (2 * np.pi) - np.pi

    def angle_diff(self, a, b):
        return self.angle_wrap(a - b)

    def rectangular_payload(self, payload, cluster):
        a1, a0 = float(cluster[0][0]), float(cluster[0][1])
        r1, r0 = float(cluster[1][0]), float(cluster[1][1])

        a_pos, r_pos, a_neg, r_neg = map(float, payload)

        # For each endpoint, choose the extreme point closest in angle
        # (keep angle+range paired!)
        def closest_extreme_to(a_end):
            d_pos = abs(self.angle_diff(a_pos, a_end))
            d_neg = abs(self.angle_diff(a_neg, a_end))
            return r_pos if d_pos <= d_neg else r_neg

        r_far0 = closest_extreme_to(a0)
        r_far1 = closest_extreme_to(a1)

        # Optional: force "far" to be at least the baseline (avoid inverted rectangles)
        r_far0 = max(r_far0, r0)
        r_far1 = max(r_far1, r1)

        # 4 corners in polar (a, r)
        corners = [
            (a0, r0),      # near at a0
            (a0, r_far0),  # far  at a0
            (a1, r_far1),  # far  at a1
            (a1, r1),      # near at a1
        ]
        return corners
        # Plot these points provided by cluster

    def line_payload(self, cluster):
        point1 = [cluster[0][0], cluster[1][0]]
        point2 = [cluster[0][1], cluster[1][1]]
        return point1, point2

        # Cluster payload is unchanged so line stays as a line

    def circular_payload(self, cluster):
        point1 = [float(cluster[0][0]), float(cluster[1][0])]
        point2 = [float(cluster[0][1]), float(cluster[1][1])]

        # Midpoint in angle, wrap-safe
        # (simple: go halfway along shortest angular diff)
        da = self.angle_diff(point1[0], point2[0])
        mid_angle = (point2[0] + 0.5 * da) % (2 * np.pi)

        # Midpoint in distance using linear interpolation between endpoints
        mid_distance = 0.5 * (point1[1] + point2[1])

        mid_point = [mid_angle, mid_distance]
        return mid_point

        # The cluster payload is singular so we just need to make

    def classify_cluster_line_vs_rectangle(self, cluster, angles, ranges, thresh, min_points):
        """
        cluster: [[a1,a0],[r1,r0]]  (two endpoints)
        angles, ranges: arrays of all scan points (radians, meters), same length
        thresh: deviation threshold in meters (e.g. 0.2)
        Returns dict with residual stats + a label.
        """

        # unpack endpoints (two points)
        a_hi, a_lo = float(cluster[0][0]), float(cluster[0][1])
        r_hi, r_lo = float(cluster[1][0]), float(cluster[1][1])

        # make sure we have increasing angle bounds
        a_min = min(a_lo, a_hi)
        a_max = max(a_lo, a_hi)

        angles = np.asarray(angles, dtype=float)
        ranges = np.asarray(ranges, dtype=float)

        # select interior points by angle window
        in_window = (angles >= a_min) & (angles <= a_max)
        a_seg = angles[in_window]
        r_seg = ranges[in_window]

        if a_seg.size < min_points:
            circle_pt = self.circular_payload(cluster)
            return {"label": "Circle", "n": int(a_seg.size), "circle": circle_pt}

        # avoid divide-by-zero if endpoints have same angle
        da = (a_hi - a_lo)
        if abs(da) < 1e-6:
            return {"label": "degenerate", "n": int(a_seg.size)}

        # line model in (theta, r): r_hat(theta) = r_lo + m*(theta - a_lo)
        m = (r_hi - r_lo) / da
        r_hat = r_lo + m * (a_seg - a_lo)

        # residuals: positive means measured is farther than line; negative means closer
        resid = r_seg - r_hat

        # strongest deviations
        max_pos = float(np.max(resid))
        max_neg = float(np.min(resid))

        has_pos = max_pos > thresh
        has_neg = max_neg < -thresh

        # optionally return the angles where extremes occur
        i_pos = int(np.argmax(resid))
        i_neg = int(np.argmin(resid))

        if has_pos or has_neg:
            payload = [
                float(a_seg[i_pos]), float(r_seg[i_pos]),
                float(a_seg[i_neg]), float(r_seg[i_neg])
            ]
            corners = self.rectangular_payload(payload, cluster)
            return {
                "label": "rectangle_like",
                "n": int(a_seg.size),
                "max_pos": max_pos,
                "max_neg": max_neg,
                "payload": payload,
                "corners": corners,
            }
        else:
            p1, p2 = self.line_payload(cluster)
            return {
                "label": "line_like",
                "n": int(a_seg.size),
                "max_abs": float(np.max(np.abs(resid))),
                "line": (p1, p2),
            }

    async def run(self):
        print("[SAFETY] Listening for close obstacles and controller inputs...")
        self.clusters = []
        self.angle_gap = np.deg2rad(10)
        self.angle_step = np.deg2rad(0.5)    # fill resolution: add points every 0.5 degrees
        self.dist_gap = 0.5
        self.resultant_x = 0
        self.resultant_y = 0
        yaw = self.state.stm["yaw"]

        avoidance_val = 25
        while True:
            # Receive Lidar data for the close zone
            # These are populated by LidarModule based on config.LIDAR_AVOID_DISTANCES["close"]
            # This is a dictionary that holds two arrays indexed by points where their distance is Non-Zero and less than the LIDAR-MAX-RANGE.
            lidar_data = self.state.lidar_close

            # Rotating the Lidar points 90 degrees while keeping the range [0, 2pi]
            # Distance that are within ["middle"]
            angles = np.asarray(lidar_data.get("angles", []), dtype=float)
            ranges = np.asarray(lidar_data.get("ranges", []), dtype=float)
            clusters = lidar_data.get("clusters", [])

            # Clustering and segmentation
            # in the form [[a1,a0],[r1,r0]]
            for cl in clusters:
                info = self.classify_cluster_line_vs_rectangle(
                    cl, angles, ranges, thresh=0.2, min_points=5
                )
            


                # (optional) do something with info here, e.g. draw corners/lines/circle
            # For a cluster if the distance is greater than .5 it forms a line
            # For a cluster if the distance is greater than .5 but it deviates it's a rectangle
            # For a cluster less than .5 it's considred a circle
            await asyncio.sleep(0.05) # Run at ~20Hz
