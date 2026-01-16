#!/usr/bin/env python3
import cv2
import numpy as np
from dt_apriltags import Detector

detector = Detector(
    families='tagStandard41h12',
    nthreads=4,
    quad_decimate=1.0,
    quad_sigma=0.0,
    refine_edges=1,
    decode_sharpening=0.25,
    debug=0
)

TAG_SIZE_M = 0.102

# Replace with calibrated intrinsics for each camera if you have them.
FX_L, FY_L = 210.0, 210.0
FX_R, FY_R = 210, 210.0

# If you have calibrated principal points, set them here.
# If None, we'll fall back to w/2,h/2 for each half-image.
CX_L, CY_L = None, None
CX_R, CY_R = None, None


def split_stereo_side_by_side(frame_gray):
    h, w = frame_gray.shape[:2]
    mid = w // 2
    left = frame_gray[:, :mid]
    right = frame_gray[:, mid:]
    return left, right, mid


def detect_pose(gray_img, fx, fy, tag_size_m, cx=None, cy=None):
    """
    Returns: (detections, camera_params)
    Each detection d may have: d.pose_R, d.pose_t, d.pose_err, d.corners, d.center, d.tag_id
    """
    h, w = gray_img.shape[:2]
    if cx is None: cx = w / 2.0
    if cy is None: cy = h / 2.0
    camera_params = (fx, fy, cx, cy)

    dets = detector.detect(
        gray_img,
        estimate_tag_pose=True,
        camera_params=camera_params,
        tag_size=tag_size_m
    )
    return dets, camera_params


def draw_detection(img_bgr, d, x_offset=0):
    """Draw center + corners on a BGR image, with optional x offset (for right half)."""
    cx, cy = float(d.center[0]), float(d.center[1])
    cv2.circle(img_bgr, (int(cx) + x_offset, int(cy)), 4, (0, 0, 255), -1)

    corners = np.array(d.corners, dtype=np.float32)
    for (x, y) in corners:
        cv2.circle(img_bgr, (int(x) + x_offset, int(y)), 3, (0, 255, 0), -1)


def print_detection_pose(label, d):
    tag_id = int(d.tag_id)
    print(f"{label} Tag {tag_id}: decision_margin={d.decision_margin:.2f}, hamming={d.hamming}")

    if d.pose_t is None or d.pose_R is None:
        print("  pose: None")
        return

    t = np.array(d.pose_t, dtype=np.float64).reshape(-1)
    err = float(d.pose_err) if d.pose_err is not None else None
    print(f"  t = [{t[0]: .3f}, {t[1]: .3f}, {t[2]: .3f}] m  err={err}")

def stereo_depth_from_centers(xL,yL,xR,yR,fx=(310.00),baseline_m=0.065):
    disparity = xL - xR
    if disparity <= 0:
        return None
    z = (fx*baseline_m)/disparity
    return z
def main():
    cap = cv2.VideoCapture(0)
    xL, yL,xR,yR = 0,0,0,0

    while True:
        ok, frame_bgr = cap.read()
        if not ok:
            break

        gray_packed = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2GRAY)
        left_gray, right_gray, mid = split_stereo_side_by_side(gray_packed)

        # Pose from LEFT
        dets_L, camL = detect_pose(left_gray, FX_L, FY_L, TAG_SIZE_M, CX_L, CY_L)

        # Pose from RIGHT (separate function call)
        dets_R, camR = detect_pose(right_gray, FX_R, FY_R, TAG_SIZE_M, CX_R, CY_R)

        print(f"\nLEFT dets: {len(dets_L)} cam={camL} | RIGHT dets: {len(dets_R)} cam={camR} | tag_size={TAG_SIZE_M}")

        # Draw + print left detections
        for d in dets_L:
            draw_detection(frame_bgr, d, x_offset=0)
            print_detection_pose("LEFT", d)
            xL =d.center[0]
            yL = d.center[1]

        # Draw + print right detections (shift drawings by mid)
        for d in dets_R:
            draw_detection(frame_bgr, d, x_offset=mid)
            print_detection_pose("RIGHT", d)
            xR = d.center[0]
            yR = d.center[1]
        if all(v is not None for v in (xL, yL, xR, yR)):
            print(stereo_depth_from_centers(xL, yL, xR, yR))

        cv2.imshow("packed stereo (pose L and R independently)", frame_bgr)

        if (cv2.waitKey(1) & 0xFF) == ord('q'):
            break

    cap.release()
    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
