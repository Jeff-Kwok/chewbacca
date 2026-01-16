#!/usr/bin/env python3
import os
import sys
import cv2
import time
import glob
from ultralytics import YOLO
from dt_apriltags import Detector
import math
import numpy as np
import gc
import torch

# UDP SOCKET SENDING
import json
import socket

# Allow importing from parent directory if run as script
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))

try:
    from . import config
    from .state import RobotState
except ImportError:
    import config
    from state import RobotState


def build_vision_payload(cx, cy, cls, z, angle, corners=None):
    payload = {
        "center": [float(cx), float(cy)],
        "id": f"object_{cls}",
        "z": float(z) if z is not None else 0.0,
        "angle": float(angle) if angle is not None else 0.0,
        "ts": time.time()
    }
    # Optional: include tag corners if provided
    if corners is not None:
        payload["corners"] = [[float(x), float(y)] for (x, y) in corners]
    return payload


class CameraStereo:
    def __init__(self):
        self.state = RobotState()
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

        # Control socket to receive mode switches
        self.ctrl_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.ctrl_sock.bind(("0.0.0.0", config.CAMERA_CONTROL_PORT))
        self.ctrl_sock.setblocking(False)

        # Reduce CPU thread thrash (important when mixing OpenCV + native libs)
        cv2.setNumThreads(1)
        os.environ.setdefault("OMP_NUM_THREADS", "1")
        os.environ.setdefault("MKL_NUM_THREADS", "1")
        os.environ.setdefault("OPENBLAS_NUM_THREADS", "1")
        os.environ.setdefault("NUMEXPR_NUM_THREADS", "1")

        self.model = None
        self.at_detector = None
        self.at_detector_bad = False   # if we detect crashes, we can re-init once

        self.cap = None
        self.device = None
        self.consec_fail = 0

        # Create the detector ONCE (don’t create/destroy repeatedly)
        self._init_apriltag_detector()

    # ---------- CAMERA ----------
    def find_camera_device(self, prefer_substr: str | None = None) -> str | None:
        by_id = sorted(glob.glob("/dev/v4l/by-id/*"))
        if prefer_substr:
            by_id = [p for p in by_id if prefer_substr in os.path.basename(p)]
        if by_id:
            return by_id[0]

        by_path = sorted(glob.glob("/dev/v4l/by-path/*"))
        if prefer_substr:
            by_path = [p for p in by_path if prefer_substr in os.path.basename(p)]
        if by_path:
            return by_path[0]

        videos = sorted(glob.glob("/dev/video*"))
        return videos[0] if videos else None

    def open_camera(self, device: str, width: int, height: int) -> cv2.VideoCapture | None:
        cap = cv2.VideoCapture(device, cv2.CAP_V4L2)
        cap.set(cv2.CAP_PROP_FRAME_WIDTH,  width)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
        cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)

        if not cap.isOpened():
            cap.release()
            return None
        return cap

    # ---------- STEREO ----------
    def stereo_depth(self, cx_left, cx_right, fx, baseline):
        d = cx_left - cx_right
        if d <= 0:
            return None
        return (fx * baseline / d)

    # ---------- APRILTAG ----------
    def _init_apriltag_detector(self):
        """
        Create one Detector instance and keep it for the life of the process.
        This avoids native heap crashes from repeated init/free cycles.
        """
        try:
            if self.at_detector is not None:
                # do NOT del in a tight loop; but on init we can safely release
                self.at_detector = None
                gc.collect()
                time.sleep(0.05)

            # Use your family (you had tag16h5)
            self.at_detector = Detector(
                families='tag16h5',
                nthreads=1,              # keep it 1 to avoid racey native alloc issues
                quad_decimate=1.0,
                quad_sigma=0.8,
                refine_edges=1,
                decode_sharpening=0.25,
                debug=0
            )
            self.at_detector_bad = False
            print("[CAM] AprilTag Detector initialized (persistent).")
        except Exception as e:
            print(f"[CAM] AprilTag Detector init failed: {e}")
            self.at_detector = None
            self.at_detector_bad = True

    def detect_with_pose(self, detector, img_bgr_or_gray, fx, fy, tag_size_m):
        """
        Returns dict:
          {tag_id: {"center":(cx,cy), "R":3x3, "t":3x1, "err":float, "corners":(4,2)}}
        """

        if detector is None:
            return {}

        # Ensure grayscale uint8 and contiguous memory (VERY important for native code stability)
        if img_bgr_or_gray.ndim == 3:
            gray = cv2.cvtColor(img_bgr_or_gray, cv2.COLOR_BGR2GRAY)
        else:
            gray = img_bgr_or_gray

        gray = np.ascontiguousarray(gray, dtype=np.uint8)

        h, w = gray.shape[:2]
        cx = w / 2.0
        cy = h / 2.0
        camera_params = (float(fx), float(fy), float(cx), float(cy))

        # Run detection (native)
        detections = detector.detect(
            gray,
            estimate_tag_pose=True,
            camera_params=camera_params,
            tag_size=float(tag_size_m)
        )

        out = {}
        for d in detections:
            tag_id = int(d.tag_id)

            corners = np.array(d.corners, dtype=np.float32) if getattr(d, "corners", None) is not None else None

            out[tag_id] = {
                "center": (float(d.center[0]), float(d.center[1])),
                "R": np.array(d.pose_R, dtype=np.float64) if getattr(d, "pose_R", None) is not None else None,
                "t": np.array(d.pose_t, dtype=np.float64) if getattr(d, "pose_t", None) is not None else None,
                "err": float(d.pose_err) if getattr(d, "pose_err", None) is not None else None,
                "corners": corners,  # 4x2
            }
        return out

    def match_by_id(self, left_map, right_map):
        matched = {}
        for tag_id, L in left_map.items():
            if tag_id in right_map:
                matched[tag_id] = (L, right_map[tag_id])
        return matched

    def draw_tag_debug(self, frame, info, color=(0, 255, 0)):
        """
        Draw center + polygon corners for apriltags.
        """
        for tag_id, d in info.items():
            cx, cy = d["center"]
            cv2.circle(frame, (int(cx), int(cy)), 4, (0, 0, 255), -1)
            cv2.putText(frame, f"id:{tag_id}", (int(cx)+6, int(cy)+6),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)

            corners = d.get("corners", None)
            if corners is not None and corners.shape == (4, 2):
                pts = corners.astype(np.int32)
                cv2.polylines(frame, [pts], isClosed=True, color=color, thickness=2)
                for i, (x, y) in enumerate(pts):
                    cv2.circle(frame, (int(x), int(y)), 3, (255, 0, 0), -1)
                    cv2.putText(frame, str(i), (int(x)+4, int(y)-4),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 0), 1)

    # ---------- YOLO POSE ----------
    def process_pose(self, r, tag=""):
        out = []
        if r.keypoints is None or len(r.keypoints) == 0:
            return out

        kpts = r.keypoints.xy.cpu().numpy()
        conf = None
        if hasattr(r.keypoints, "conf") and r.keypoints.conf is not None:
            conf = r.keypoints.conf.cpu().numpy()
        clss = r.boxes.cls.cpu().numpy().astype(int) if r.boxes is not None else np.zeros(kpts.shape[0], dtype=int)

        LSH, RSH, LHIP, RHIP = 5, 6, 11, 12

        for i in range(kpts.shape[0]):
            pts = kpts[i]

            xs = pts[:, 0]; ys = pts[:, 1]
            x1, y1, x2, y2 = xs.min(), ys.min(), xs.max(), ys.max()
            cx, cy = (x1 + x2) * 0.5, (y1 + y2) * 0.5
            w, h = (x2 - x1), (y2 - y1)

            def valid(idx):
                if conf is None:
                    return np.isfinite(pts[idx]).all()
                return conf[i, idx] > 0.3

            ang = None
            if valid(LSH) and valid(RSH):
                vx, vy = pts[RSH][0] - pts[LSH][0], pts[RSH][1] - pts[LSH][1]
                ang = math.atan2(vy, vx)
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
                "ang": ang,
                "kpts": pts,
                "kp_conf": None if conf is None else conf[i],
                "cls": int(clss[i]),
            })

            print(f"[{tag}] cx={cx:.1f} cy={cy:.1f} w={w:.1f} h={h:.1f} ang={None if ang is None else f'{ang:.3f}'}")

        return out

    def draw_pose_debug(self, frame, objs, color=(0,255,0)):
        for o in objs:
            cx, cy = int(o["cx"]), int(o["cy"])
            cv2.circle(frame, (cx, cy), 4, (0,0,255), -1)

            if o["ang"] is not None:
                L = 50
                x2 = int(cx + L * np.cos(o["ang"]))
                y2 = int(cy + L * np.sin(o["ang"]))
                cv2.arrowedLine(frame, (cx, cy), (x2, y2), color, 2)

            for (x, y) in o["kpts"]:
                if np.isfinite(x) and np.isfinite(y):
                    cv2.circle(frame, (int(x), int(y)), 2, color, -1)

    # ---------- MODES ----------
    def _run_hunting(self, left, right, f, B):
        if self.model is None:
            print("[CAM] Initializing YOLO model...")
            self.model = YOLO(config.MODEL_PATH)

        results = self.model.predict(source=[left, right], imgsz=config.IMGSZ, conf=config.CONF, verbose=False)
        rL, rR = results[0], results[1]

        objsL = self.process_pose(rL, "L")
        objsR = self.process_pose(rR, "R")
        self.draw_pose_debug(left,  objsL, (0,255,0))
        self.draw_pose_debug(right, objsR, (255,0,0))

        if objsL and objsR:
            try:
                z = float(self.stereo_depth(objsL[0]["cx"], objsR[0]["cx"], f, B))
                angle = np.arctan((320/2 - objsL[0]["cx"]) / f * 3)
                payload = build_vision_payload(objsL[0]["cx"], objsL[0]["cy"], objsL[0]["cls"], z, angle)
                packet = json.dumps(payload).encode('utf-8')
                if packet:
                    self.sock.sendto(packet, (config.SELF_SEND_IP, config.CAMERA_PORT))
                    print(payload)
            except Exception as e:
                print(e)

    def _run_tagging(self, left, right, f, B):
        # Do NOT destroy/recreate detector in loop. Keep persistent.

        try:
            left_info  = self.detect_with_pose(self.at_detector, left,  f, f, config.TAG_SIZE_M)
            right_info = self.detect_with_pose(self.at_detector, right, f, f, config.TAG_SIZE_M)
        except Exception as e:
            # If native code throws, rebuild detector ONCE (not per-frame)
            print(f"[CAM] AprilTag detect exception: {e}")
            if not self.at_detector_bad:
                print("[CAM] Reinitializing AprilTag detector once due to error...")
                self.at_detector_bad = True
                self._init_apriltag_detector()
            return

        matched = self.match_by_id(left_info, right_info)

        # Draw debug
        self.draw_tag_debug(left, left_info, color=(0, 255, 0))
        self.draw_tag_debug(right, right_info, color=(0, 255, 0))

        # Send one matched tag
        for tag_id, (L, R) in matched.items():
            cxL, cyL = L["center"]

            if L["t"] is not None:
                t = L["t"].reshape(-1)
                z_m = float(t[2])
                z_mm = z_m * 1000.0

                angle = math.atan2(-float(t[0]), float(t[2]))

                # include corners in payload (from left tag)
                corners = None
                if L.get("corners", None) is not None and L["corners"].shape == (4, 2):
                    corners = L["corners"]

                payload = build_vision_payload(cxL, cyL, tag_id, z_mm, angle, corners=corners)
                packet = json.dumps(payload).encode('utf-8')
                if packet:
                    self.sock.sendto(packet, (config.SELF_SEND_IP, config.CAMERA_PORT))
                    print(f"[TAG] ID={tag_id} z={z_mm:.1f}mm ang={angle:.3f} corners={corners.tolist() if corners is not None else None}")
                break

        cv2.putText(left, "MODE: TAGGING", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)

    # ---------- MAIN LOOP ----------
    def run(self):
        last_t = time.time()
        fps = 0.0
        f = config.STEREO_FOCAL_LENGTH
        B = config.STEREO_BASELINE

        print("Press 'q' or ESC to quit")

        while True:
            # Check for mode updates from controller
            try:
                data, _ = self.ctrl_sock.recvfrom(1024)
                msg = json.loads(data.decode('utf-8'))
                if "mode" in msg:
                    self.state.camera_mode = msg["mode"]
                    print(f"[CAM] Mode switched to: {self.state.camera_mode}")
            except BlockingIOError:
                pass
            except Exception:
                pass

            # Ensure camera is open
            if self.cap is None:
                self.device = self.find_camera_device()
                if self.device is None:
                    print(f"[CAM] no camera device found. retrying in {config.OPEN_RETRY_SEC}s...")
                    time.sleep(config.OPEN_RETRY_SEC)
                    continue

                self.cap = self.open_camera(self.device, config.IMAGE_RES[0], config.IMAGE_RES[1])
                if self.cap is None:
                    print(f"[CAM] open failed for {self.device}. retrying in {config.OPEN_RETRY_SEC}s...")
                    time.sleep(config.OPEN_RETRY_SEC)
                    continue

                print(f"[CAM] opened {self.device}")
                self.consec_fail = 0

            ret, frame = self.cap.read()

            if (not ret) or (frame is None):
                self.consec_fail += 1
                time.sleep(0.02)

                if self.consec_fail >= config.MAX_CONSEC_FAIL:
                    print("[CAM] read failing repeatedly -> reopening camera")
                    try:
                        self.cap.release()
                    except Exception:
                        pass
                    self.cap = None
                    self.consec_fail = 0
                    time.sleep(config.REOPEN_BACKOFF_SEC)
                continue

            self.consec_fail = 0

            h_full, w_full = frame.shape[:2]
            left  = frame[:, :w_full//2].copy()
            right = frame[:, w_full//2:].copy()

            if self.state.camera_mode == "hunting":
                self._run_hunting(left, right, f, B)
            elif self.state.camera_mode == "tagging":
                # If you REALLY must free YOLO GPU memory, do it only once when mode switches,
                # but frequent create/destroy also causes instability. Keeping it is safer.
                self._run_tagging(left, right, f, B)

            # FPS
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

        if self.cap is not None:
            self.cap.release()
        cv2.destroyAllWindows()


if __name__ == "__main__":
    cs = CameraStereo()
    cs.run()
