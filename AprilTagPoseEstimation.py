#!/usr/bin/env python3
import cv2
import numpy as np
from dt_apriltags import Detector

detector = Detector(
    searchpath=['/usr/local/lib'],  # optional; can omit
    families='tagCircle49h12',
    nthreads=4,
    quad_decimate=1.0,
    quad_sigma=0.0,
    refine_edges=1,
    decode_sharpening=0.25,
    debug=0
)

# --------- CONFIG YOU MUST SET (CALIBRATION) ----------
# Tag size in METERS (0.16 = 16cm)
TAG_SIZE_M = 0.166

# Replace these with your calibrated intrinsics (pixels)
# If you don't have them yet, these placeholders will "work" but pose will be wrong-scale/biased.
FX_L, FY_L = 304.34, 304.34
FX_R, FY_R = 300.0, 300.0
# ------------------------------------------------------

def split_stereo_side_by_side(frame):
    """Split a side-by-side stereo frame into (left, right)."""
    h, w = frame.shape[:2]
    mid = w // 2
    left = frame[:, :mid]
    right = frame[:, mid:]
    return left, right, mid

def detect_with_pose(detector, img_bgr_or_gray, fx, fy, tag_size_m):
    """
    Returns dict:
      {tag_id: {"center":(cx,cy), "R":3x3, "t":3x1, "err":float, "det":Detection}}
    """
    if img_bgr_or_gray.ndim == 3:
        gray = cv2.cvtColor(img_bgr_or_gray, cv2.COLOR_BGR2GRAY)
    else:
        gray = img_bgr_or_gray

    h, w = gray.shape[:2]
    cx = w / 2.0
    cy = h / 2.0
    camera_params = (fx, fy, cx, cy)

    detections = detector.detect(
        gray,
        estimate_tag_pose=True,
        camera_params=camera_params,
        tag_size=tag_size_m
    )

    out = {}
    for d in detections:
        tag_id = int(d.tag_id)
        out[tag_id] = {
            "center": (float(d.center[0]), float(d.center[1])),
            "R": np.array(d.pose_R, dtype=np.float64) if d.pose_R is not None else None,
            "t": np.array(d.pose_t, dtype=np.float64) if d.pose_t is not None else None,
            "err": float(d.pose_err) if d.pose_err is not None else None,
            "det": d,
            "corners": np.array(d.corners, dtype=np.float64),  # <--- ADD THIS
        }
    return out

def match_by_id(left_map, right_map):
    """Return dict: {tag_id: (left_info, right_info)} for ids seen in both."""
    matched = {}
    for tag_id, L in left_map.items():
        if tag_id in right_map:
            matched[tag_id] = (L, right_map[tag_id])
    return matched

def fmt_R(R):
    return (
        f"[{R[0,0]: .3f}, {R[0,1]: .3f}, {R[0,2]: .3f}]\n"
        f"[{R[1,0]: .3f}, {R[1,1]: .3f}, {R[1,2]: .3f}]\n"
        f"[{R[2,0]: .3f}, {R[2,1]: .3f}, {R[2,2]: .3f}]"
    )

def main_packed_stereo():
    cap = cv2.VideoCapture(0)  # or your GStreamer pipeline

    while True:
        ok, frame = cap.read()
        if not ok:
            break

        # Split packed stereo
        gray_packed = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        left, right, mid = split_stereo_side_by_side(gray_packed)

        # Detect + pose per camera half
        left_info  = detect_with_pose(detector, left,  FX_L, FY_L, TAG_SIZE_M)
        right_info = detect_with_pose(detector, right, FX_R, FY_R, TAG_SIZE_M)

        matched = match_by_id(left_info, right_info)

        print(f"Left detections: {len(left_info)} | Right detections: {len(right_info)}")
        print(f"Matched (both cameras): {len(matched)}")

        # Draw + print
        for tag_id, (L, R) in matched.items():
            (xL, yL) = L["center"]
            (xR, yR) = R["center"]
            corners_L = L["corners"]
            corners_R = R["corners"]

            # Draw on the ORIGINAL packed BGR frame
            cv2.circle(frame, (int(xL), int(yL)), 4, (0, 0, 255), 1)              # left half
            cv2.circle(frame, (int(xR) + mid, int(yR)), 4, (0, 0, 255), 1)        # right half shifted by mid

            print(f"\nTag {tag_id}:")
            print(f"  Centers: L=({xL:.2f},{yL:.2f})  R=({xR:.2f},{yR:.2f})")

            # Left pose
            if L["R"] is not None and L["t"] is not None:
                t = L["t"].reshape(-1)
                print(f"  Left pose:")
                print(f"    t = [{t[0]: .3f}, {t[1]: .3f}, {t[2]: .3f}]  meters")
                #print(f"    R =\n{fmt_R(L['R'])}")
                #print(f"    err = {L['err']:.4f}")
            for (x, y) in corners_L:
                cv2.circle(frame, (int(x), int(y)), 3, (0, 255, 0), -1)

            else:
                print("  Left pose: (not available)")

            # Right pose
            if R["R"] is not None and R["t"] is not None:
                t = R["t"].reshape(-1)
                print(f"  Right pose:")
                print(f"    t = [{t[0]: .3f}, {t[1]: .3f}, {t[2]: .3f}]  meters")
                #print(f"    R =\n{fmt_R(R['R'])}")
                #print(f"    err = {R['err']:.4f}")
                for (x, y) in corners_R:
                    cv2.circle(frame, (int(x+320), int(y)), 3, (0, 255, 0), -1)
            else:
                print("  Right pose: (not available)")

        cv2.imshow("packed stereo (left|right)", frame)

        key = cv2.waitKey(1) & 0xFF
        if key == ord('q'):
            break

    cap.release()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    main_packed_stereo()
