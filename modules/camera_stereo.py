#!/usr/bin/env python3
import os
import cv2
import time
import glob
from ultralytics import YOLO
import math
import numpy as np

# UDP SOCKET SENDING
import json
import asyncio
import socket

# ---------------- CONFIG ----------------
IMAGE_RES = (640, 480)

# Reconnect behavior
OPEN_RETRY_SEC      = 1.0     # wait between open attempts
REOPEN_BACKOFF_SEC  = 0.5     # small pause after releasing camera
MAX_CONSEC_FAIL     = 10      # reopen after this many failed reads
FIND_DEVICE_EVERY_N = 1       # re-scan devices each reconnect (1 = always)

# YOLO behavior
MODEL_PATH = "models/yolo11l-pose.pt"
print(MODEL_PATH)
IMGSZ      = 512
CONF       = 0.35
# ----------------------------------------
SEND_PORT = 5005
SEND_IP = "127.0.0.1"
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
def build_vision_payload(cx, cy, cls, z, angle):
    return {
        "center": [float(cx), float(cy)],
        "id": f"object_{cls}",
        "z": float(z) if z is not None else 0.0,
        "angle": float(angle) if angle is not None else 0.0,
        "ts": time.time()
    }


def find_camera_device(prefer_substr: str | None = None) -> str | None:
    """
    Returns a stable camera device path. Prefers /dev/v4l/by-id.
    If prefer_substr is provided, it will pick the first matching path.
    """
    # Stable names first
    by_id = sorted(glob.glob("/dev/v4l/by-id/*"))
    if prefer_substr:
        by_id = [p for p in by_id if prefer_substr in os.path.basename(p)]
    if by_id:
        return by_id[0]

    # Fallback to by-path (also stable-ish)
    by_path = sorted(glob.glob("/dev/v4l/by-path/*"))
    if prefer_substr:
        by_path = [p for p in by_path if prefer_substr in os.path.basename(p)]
    if by_path:
        return by_path[0]

    # Last resort: /dev/video*
    videos = sorted(glob.glob("/dev/video*"))
    return videos[0] if videos else None

def open_camera(device: str, width: int, height: int) -> cv2.VideoCapture | None:
    """
    Opens a camera device path with V4L2 and applies basic settings.
    """
    cap = cv2.VideoCapture(device, cv2.CAP_V4L2)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH,  width)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
    cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)  # reduce latency; not always honored

    if not cap.isOpened():
        cap.release()
        return None

    return cap

def stereo_depth(cx_left,cx_right,fx,baseline):
    d = cx_left - cx_right
    if d<= 0:
        return None
    return(fx*baseline/d)

def process_obb(r, tag=""):
    obb = r.obb
    out = []

    if obb is None or len(obb) == 0:
        return out

    xywhr = obb.xywhr.cpu().numpy()           # (N, 5)
    polys = obb.xyxyxyxy.cpu().numpy()        # (N, 8)
    confs = obb.conf.cpu().numpy()            # (N,)
    clss  = obb.cls.cpu().numpy().astype(int) # (N,)

    for i in range(len(xywhr)):
        cx, cy, w, h, ang = xywhr[i]
        poly = polys[i].reshape(4, 2)

        out.append({
            "cx": cx,
            "cy": cy,
            "w": w,
            "h": h,
            "ang": ang,
            "poly": poly,
            "conf": confs[i],
            "cls": clss[i],
        })

        print(
            f"[{tag}] cx={cx:.1f} cy={cy:.1f} "
            f"w={w:.1f} h={h:.1f} ang={ang:.3f} "
            f"conf={confs[i]:.2f} cls={clss[i]}"
        )

    return out
    
def process_pose(r, tag=""):
    out = []
    if r.keypoints is None or len(r.keypoints) == 0:
        return out

    kpts = r.keypoints.xy.cpu().numpy()      # (N, K, 2)
    conf = None
    if hasattr(r.keypoints, "conf") and r.keypoints.conf is not None:
        conf = r.keypoints.conf.cpu().numpy()  # (N, K)
    clss = r.boxes.cls.cpu().numpy().astype(int) if r.boxes is not None else np.zeros(kpts.shape[0], dtype=int)

    # YOLO pose keypoint indices (COCO): 5=L shoulder, 6=R shoulder, 11=L hip, 12=R hip
    LSH, RSH, LHIP, RHIP = 5, 6, 11, 12

    for i in range(kpts.shape[0]):
        pts = kpts[i]  # (K,2)

        # bbox from keypoints (more stable than r.boxes if you want)
        xs = pts[:, 0]; ys = pts[:, 1]
        x1, y1, x2, y2 = xs.min(), ys.min(), xs.max(), ys.max()
        cx, cy = (x1 + x2) * 0.5, (y1 + y2) * 0.5
        w, h = (x2 - x1), (y2 - y1)

        def valid(idx):
            if conf is None:  # no conf available, assume valid if finite
                return np.isfinite(pts[idx]).all()
            return conf[i, idx] > 0.3

        # Orientation vector preference:
        # 1) shoulder line: from left shoulder -> right shoulder
        # 2) hip line
        # 3) torso: mid-hip -> mid-shoulder
        ang = None

        if valid(LSH) and valid(RSH):
            vx, vy = pts[RSH][0] - pts[LSH][0], pts[RSH][1] - pts[LSH][1]
            ang = math.atan2(vy, vx)  # radians in image plane
        elif valid(LHIP) and valid(RHIP):
            vx, vy = pts[RHIP][0] - pts[LHIP][0], pts[RHIP][1] - pts[LHIP][1]
            ang = math.atan2(vy, vx)
        elif (valid(LSH) or valid(RSH)) and (valid(LHIP) or valid(RHIP)):
            sh = (pts[LSH] + pts[RSH]) * 0.5 if (valid(LSH) and valid(RSH)) else (pts[LSH] if valid(LSH) else pts[RSH])
            hp = (pts[LHIP] + pts[RHIP]) * 0.5 if (valid(LHIP) and valid(RHIP)) else (pts[LHIP] if valid(LHIP) else pts[RHIP])
            vx, vy = sh[0] - hp[0], sh[1] - hp[1]
            ang = math.atan2(vy, vx)

        out.append({
            "cx": cx, "cy": cy, "w": w, "h": h,
            "ang": ang,           # orientation in image plane (radians) or None
            "kpts": pts,          # all keypoints
            "kp_conf": None if conf is None else conf[i],
            "cls": int(clss[i]),
        })

        print(f"[{tag}] cx={cx:.1f} cy={cy:.1f} w={w:.1f} h={h:.1f} ang={None if ang is None else f'{ang:.3f}'}")

    return out

def draw_pose_debug(frame, objs, color=(0,255,0)):
    for o in objs:
        cx, cy = int(o["cx"]), int(o["cy"])

        # draw center
        cv2.circle(frame, (cx, cy), 4, (0,0,255), -1)

        # draw orientation arrow if available
        if o["ang"] is not None:
            L = 50
            x2 = int(cx + L * np.cos(o["ang"]))
            y2 = int(cy + L * np.sin(o["ang"]))
            cv2.arrowedLine(frame, (cx, cy), (x2, y2), color, 2)

        # draw keypoints
        for (x, y) in o["kpts"]:
            if np.isfinite(x) and np.isfinite(y):
                cv2.circle(frame, (int(x), int(y)), 2, color, -1)

def main():
    # (Optional) reduce CPU thread thrash
    cv2.setNumThreads(1)
    os.environ.setdefault("OMP_NUM_THREADS", "1")
    os.environ.setdefault("MKL_NUM_THREADS", "1")

    model = YOLO(MODEL_PATH)

    cap = None
    device = None
    consec_fail = 0

    last_t = time.time()
    fps = 0.0
    f = 700*(1/2.3) # pixels
    B = 60 # mm
    d = 0 
    print("Press 'q' or ESC to quit")
    cLx, cLy, cxR,cyR = 0,0,0,0
    cx = 0
    cy = 0 
    while True:
        # Ensure camera is open
        if cap is None:
            device = find_camera_device()
            if device is None:
                print(f"[CAM] no camera device found. retrying in {OPEN_RETRY_SEC}s...")
                time.sleep(OPEN_RETRY_SEC)
                continue

            cap = open_camera(device, IMAGE_RES[0], IMAGE_RES[1])
            if cap is None:
                print(f"[CAM] open failed for {device}. retrying in {OPEN_RETRY_SEC}s...")
                time.sleep(OPEN_RETRY_SEC)
                continue

            print(f"[CAM] opened {device}")
            consec_fail = 0

        # Read frame
        ret, frame = cap.read()

        # Handle transient failures
        if (not ret) or (frame is None):
            consec_fail += 1
            time.sleep(0.02)

            if consec_fail >= MAX_CONSEC_FAIL:
                print("[CAM] read failing repeatedly -> reopening camera")
                try:
                    cap.release()
                except Exception:
                    pass
                cap = None
                consec_fail = 0
                time.sleep(REOPEN_BACKOFF_SEC)
            continue

        consec_fail = 0

        # ----- YOLO inference -----
        # Use source=frame to avoid "source missing" warnings
# ----- YOLO inference -----
        h_full, w_full = frame.shape[:2]
        left  = frame[:, :w_full//2].copy()
        right = frame[:, w_full//2:].copy()

        results = model.predict(source=[left, right], imgsz=IMGSZ, conf=CONF, verbose=False)
        rL, rR = results[0], results[1]

        objsL = process_pose(rL, "L")
        objsR = process_pose(rR, "R")
        draw_pose_debug(left,  objsL, (0,255,0))
        draw_pose_debug(right, objsR, (255,0,0))
        if objsL and objsR:
            try: 
                z=float(stereo_depth(objsL[0]["cx"],objsR[0]["cx"],f,B))
                angle = np.arctan((320/2-objsL[0]["cx"])/f*3)
                payload = build_vision_payload(objsL[0]["cx"], objsL[0]["cy"], objsL[0]["cls"], z, angle)
                packet = json.dumps(payload).encode('utf-8')
                if packet:
                    sock.sendto(packet,(SEND_IP,SEND_PORT))
                    print(payload)
            except Exception as e:
                print(e)
        else:
            None
        # ----- FPS -----
        now = time.time()
        dt = now - last_t
        last_t = now
        if dt > 0:
            fps = 0.9 * fps + 0.1 * (1.0 / dt)

        cv2.putText(
            left,
            f"FPS: {fps:.1f}",
            (10, left.shape[0] - 10),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.7,
            (0, 255, 0),
            2,
        )

        vis = np.hstack([left, right])
        cv2.imshow("YOLO Pose Stereo", vis)

        key = cv2.waitKey(1) & 0xFF
        if key == ord("q") or key == 27:
            break

    if cap is not None:
        cap.release()
    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
