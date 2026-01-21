#!/usr/bin/env python3
"""
Standalone RPLidar A1 scan-matching (2D scan-to-scan ICP) "odometry" script.

- Reads scans from RPLidar A1
- Converts each scan to 2D point cloud (meters)
- Runs ICP to align current scan -> previous scan
- Accumulates pose (x, y, yaw) in a local "world" frame
- Prints pose and match quality continuously

Dependencies:
  pip3 install rplidar numpy

Notes:
- This is scan-to-scan LiDAR odometry (will drift over time)
- Works best indoors with structure (walls/edges), moderate speed, low slip
"""

import math
import time
import numpy as np
from rplidar import RPLidar

# ----------------------------
# USER CONFIG
# ----------------------------
PORT = "/dev/ttyUSB0"   # change if needed

MIN_RANGE_M = 0.15
MAX_RANGE_M = 6.0

DOWNSAMPLE_STRIDE = 3      # take every Nth point
MAX_POINTS = 220           # cap points for speed

ICP_ITERS = 12
MAX_CORR_DIST = 0.35       # meters, correspondence rejection threshold
MIN_MATCH_RATIO = 0.20     # reject if too few correspondences

PRINT_HZ = 10.0            # status print rate


# ----------------------------
# SE(2) helpers
# ----------------------------
def pose_to_T(x, y, yaw):
    c = math.cos(yaw)
    s = math.sin(yaw)
    return np.array([[c, -s, x],
                     [s,  c, y],
                     [0,  0, 1]], dtype=np.float64)

def T_to_pose(T):
    x = float(T[0, 2])
    y = float(T[1, 2])
    yaw = math.atan2(float(T[1, 0]), float(T[0, 0]))
    return x, y, yaw

def transform_points(T, pts):
    # pts Nx2
    R = T[:2, :2]
    t = T[:2, 2]
    return (pts @ R.T) + t


# ----------------------------
# ICP core
# ----------------------------
def best_fit_transform_2d(A, B):
    """Compute rigid 2D transform T mapping A -> B (least squares) given correspondences."""
    if A.shape[0] < 3:
        return np.eye(3)

    cA = A.mean(axis=0)
    cB = B.mean(axis=0)

    AA = A - cA
    BB = B - cB

    H = AA.T @ BB
    U, _, Vt = np.linalg.svd(H)
    R = Vt.T @ U.T

    # ensure det(R)=+1
    if np.linalg.det(R) < 0:
        Vt[1, :] *= -1
        R = Vt.T @ U.T

    t = cB - (R @ cA)

    T = np.eye(3)
    T[:2, :2] = R
    T[:2, 2] = t
    return T

def nearest_neighbors_bruteforce(src, dst):
    """For each src point, find nearest dst point (brute-force)."""
    diffs = src[:, None, :] - dst[None, :, :]
    d2 = np.sum(diffs * diffs, axis=2)
    idx = np.argmin(d2, axis=1)
    dists = np.sqrt(d2[np.arange(d2.shape[0]), idx])
    return idx, dists

def icp_2d(src_pts, dst_pts, init_T=None, iters=12, max_corr_dist=0.35):
    """
    Align src_pts to dst_pts. Returns (T, fitness, rmse, used)
    where T maps src -> dst.
    """
    if init_T is None:
        T = np.eye(3)
    else:
        T = init_T.copy()

    if src_pts.shape[0] < 20 or dst_pts.shape[0] < 20:
        return T, 0.0, float("inf"), 0

    last_rmse = None
    for _ in range(iters):
        src_tf = transform_points(T, src_pts)

        idx, dists = nearest_neighbors_bruteforce(src_tf, dst_pts)

        mask = dists < max_corr_dist
        used = int(mask.sum())
        if used < 10:
            return T, 0.0, float("inf"), used

        A = src_tf[mask]
        B = dst_pts[idx[mask]]

        dT = best_fit_transform_2d(A, B)
        T = dT @ T

        rmse = float(np.sqrt(np.mean((A - B) ** 2)))
        if last_rmse is not None and abs(last_rmse - rmse) < 1e-4:
            break
        last_rmse = rmse

    # final metrics
    src_tf = transform_points(T, src_pts)
    idx, dists = nearest_neighbors_bruteforce(src_tf, dst_pts)
    mask = dists < max_corr_dist
    used = int(mask.sum())
    fitness = float(used) / float(src_pts.shape[0] + 1e-9)
    rmse = float(np.sqrt(np.mean(dists[mask] ** 2))) if used > 0 else float("inf")
    return T, fitness, rmse, used


# ----------------------------
# Scan -> points
# ----------------------------
def scan_to_points(scan):
    """
    scan: list of tuples (quality, angle_deg, distance_mm)
    returns Nx2 points in meters in LiDAR frame.
    """
    pts = []
    for q, ang_deg, dist_mm in scan:
        if dist_mm <= 0:
            continue
        r = dist_mm / 1000.0
        if r < MIN_RANGE_M or r > MAX_RANGE_M:
            continue
        a = math.radians(ang_deg)
        pts.append((r * math.cos(a), r * math.sin(a)))

    if not pts:
        return np.zeros((0, 2), dtype=np.float64)

    pts = np.asarray(pts, dtype=np.float64)

    if DOWNSAMPLE_STRIDE > 1 and pts.shape[0] > DOWNSAMPLE_STRIDE:
        pts = pts[::DOWNSAMPLE_STRIDE].copy()

    if pts.shape[0] > MAX_POINTS:
        idx = np.linspace(0, pts.shape[0] - 1, MAX_POINTS).astype(int)
        pts = pts[idx].copy()

    return pts


# ----------------------------
# Main
# ----------------------------
def main():
    lidar = RPLidar(PORT)
    print("[LIDAR] Info:", lidar.get_info())
    print("[LIDAR] Health:", lidar.get_health())
    print("[LIDAR] Starting scan...")

    prev_pts = None
    T_world = np.eye(3)

    print_period = 1.0 / PRINT_HZ
    last_print = 0.0

    try:
        for scan in lidar.iter_scans(max_buf_meas=1000):
            pts = scan_to_points(scan)
            if pts.shape[0] < 30:
                continue

            if prev_pts is None:
                prev_pts = pts
                continue

            # Align current -> previous (scan-to-scan)
            T_rel, fitness, rmse, used = icp_2d(
                src_pts=pts,
                dst_pts=prev_pts,
                init_T=np.eye(3),
                iters=ICP_ITERS,
                max_corr_dist=MAX_CORR_DIST
            )

            # Reject bad matches
            if fitness < MIN_MATCH_RATIO or not np.isfinite(rmse):
                prev_pts = pts
                continue

            # T_rel maps current->previous, invert for previous->current
            T_prev_to_curr = np.linalg.inv(T_rel)

            # accumulate pose
            T_world = T_world @ T_prev_to_curr

            now = time.monotonic()
            if now - last_print >= print_period:
                last_print = now
                x, y, yaw = T_to_pose(T_world)
                print(f"[POSE] x={x:+.3f} m  y={y:+.3f} m  yaw={yaw:+.3f} rad  "
                      f"fitness={fitness:.2f} rmse={rmse:.3f} used={used}")

            prev_pts = pts

    except KeyboardInterrupt:
        print("\n[EXIT] Ctrl-C")
    finally:
        print("[LIDAR] Stopping...")
        try:
            lidar.stop()
        except Exception:
            pass
        try:
            lidar.stop_motor()
        except Exception:
            pass
        try:
            lidar.disconnect()
        except Exception:
            pass


if __name__ == "__main__":
    main()
