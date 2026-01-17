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
    def __init__(self, state):
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
        self.last_t = time.time()
        self.fps = 0.0
        self.f = config.STEREO_FOCAL_LENGTH
        self.B = config.STEREO_BASELINE
        self.state = state

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

    def release_camera(self):
        if self.cap is not None:
            self.cap.release()
        self.cap = None

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

            # Parameters from AprilTagPoseEstimation.py for identical detection scheme
            self.at_detector = Detector(
                families='tagStandard41h12',
                nthreads=4,
                quad_decimate=1.0,
                quad_sigma=0.0,
                refine_edges=1,
                decode_sharpening=0.25,
                debug=0
            )
            self.at_detector_bad = False
            print("[CAM] AprilTag Detector initialized (persistent, AprilTagPoseEstimation scheme).")
        except Exception as e:
            print(f"[CAM] AprilTag Detector init failed: {e}")
            self.at_detector = None
            self.at_detector_bad = True

    def detect_with_pose(self, detector, img_bgr_or_gray, fx, fy, tag_size_m):
        if detector is None:
            return {}

        if img_bgr_or_gray.ndim == 3:
            gray = cv2.cvtColor(img_bgr_or_gray, cv2.COLOR_BGR2GRAY)
        else:
            gray = img_bgr_or_gray

        gray = np.ascontiguousarray(gray, dtype=np.uint8)

        h, w = gray.shape[:2]
        cx = w / 2.0
        cy = h / 2.0
        camera_params = (float(fx), float(fy), float(cx), float(cy))

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
                "corners": corners,
            }
        return out

    def match_by_id(self, left_map, right_map):
        matched = {}
        for tag_id, L in left_map.items():
            if tag_id in right_map:
                matched[tag_id] = (L, right_map[tag_id])
        return matched

    def draw_tag_debug(self, frame, info, color=(0, 255, 0)):
        for tag_id, d in info.items():
            cx, cy = d["center"]
            cv2.circle(frame, (int(cx), int(cy)), 4, (0, 0, 255), -1)
            cv2.putText(frame, f"id:{tag_id}", (int(cx)+6, int(cy)+6),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
            corners = d.get("corners", None)
            if corners is not None and corners.shape == (4, 2):
                pts = corners.astype(np.int32)
                cv2.polylines(frame, [pts], isClosed=True, color=color, thickness=2)

    # ---------- YOLO POSE ----------
    def process_pose(self, r, tag=""):
        out = []
        if r.keypoints is None or len(r.keypoints) == 0:
            return out
        kpts = r.keypoints.xy.cpu().numpy()
        conf = r.keypoints.conf.cpu().numpy() if hasattr(r.keypoints, "conf") and r.keypoints.conf is not None else None
        clss = r.boxes.cls.cpu().numpy().astype(int) if r.boxes is not None else np.zeros(kpts.shape[0], dtype=int)
        LSH, RSH, LHIP, RHIP = 5, 6, 11, 12
        for i in range(kpts.shape[0]):
            pts = kpts[i]
            xs, ys = pts[:, 0], pts[:, 1]
            cx, cy = (xs.min() + xs.max()) / 2, (ys.min() + ys.max()) / 2
            def valid(idx):
                return conf[i, idx] > getattr(config, 'YOLO_KPT_CONF', 0.3) if conf is not None else np.isfinite(pts[idx]).all()
            ang = None
            if valid(LSH) and valid(RSH): ang = math.atan2(pts[RSH][1] - pts[LSH][1], pts[RSH][0] - pts[LSH][0])
            out.append({"cx": cx, "cy": cy, "ang": ang, "cls": int(clss[i])})
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

    # ---------- MODES ----------
    def _run_hunting(self, left, right):
        if self.model is None:
            print("[CAM] Initializing YOLO model...")
            self.model = YOLO(config.MODEL_PATH)
        results = self.model.predict(source=[left, right], imgsz=config.IMGSZ, conf=config.CONF, verbose=getattr(config, 'YOLO_VERBOSE', False))
        rL, rR = results
        objsL, objsR = self.process_pose(rL, "L"), self.process_pose(rR, "R")
        self.draw_pose_debug(left, objsL), self.draw_pose_debug(right, objsR)
        if objsL and objsR:
            try:
                z = float(self.stereo_depth(objsL[0]["cx"], objsR[0]["cx"], self.f, self.B))
                angle = np.arctan((config.CAMERA_CENTER_X / 2 - objsL[0]["cx"]) / self.f * 3)
                return build_vision_payload(objsL[0]["cx"], objsL[0]["cy"], objsL[0]["cls"], z, angle)
            except Exception as e:
                print(e)
        return None

    def _run_tagging(self, left, right):
        try:
            left_info  = self.detect_with_pose(self.at_detector, left,  self.f, self.f, config.TAG_SIZE_M)
            right_info = self.detect_with_pose(self.at_detector, right, self.f, self.f, config.TAG_SIZE_M)
        except Exception as e:
            print(f"[CAM] AprilTag detect exception: {e}")
            if not self.at_detector_bad:
                print("[CAM] Reinitializing AprilTag detector once due to error...")
                self.at_detector_bad = True
                self._init_apriltag_detector()
            return None

        
        matched = self.match_by_id(left_info, right_info)
        self.draw_tag_debug(left, left_info), self.draw_tag_debug(right, right_info)
        
        for tag_id, (L, R) in matched.items():
            # Has all information of the seen id use matched
            cx_left = L["center"][0]
            cx_right = R["center"][0]
            # Calculate depth from stereo disparity
            z_m = self.stereo_depth(cx_left, cx_right, self.f, self.B)

            if z_m is not None:
                z_mm = z_m * 1000.0  # Convert to mm
                
                # We can still get angle from the monocular pose of the left camera
                angle = None
                if L["t"] is not None:
                    t = L["t"].reshape(-1)
                    angle = math.atan2(-float(t[0]), float(t[2]))

                corners = L.get("corners")
                return build_vision_payload(L["center"][0], L["center"][1], tag_id,t[2], angle, corners)
        return None

    def _run_idle(self, left, right):
        cv2.putText(left, "MODE: REST", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 0), 2)
        return None

    # ---------- MAIN LOOP ----------
    def process_frame(self, camera_mode):
        if self.cap is None:
            self.device = self.find_camera_device()
            if self.device is None:
                print(f"[CAM] no camera device found. retrying in {config.OPEN_RETRY_SEC}s...")
                time.sleep(config.OPEN_RETRY_SEC)
                return None, None
            self.cap = self.open_camera(self.device, config.IMAGE_RES[0], config.IMAGE_RES[1])
            if self.cap is None:
                print(f"[CAM] open failed for {self.device}. retrying in {config.OPEN_RETRY_SEC}s...")
                time.sleep(config.OPEN_RETRY_SEC)
                return None, None
            print(f"[CAM] opened {self.device}")
            self.consec_fail = 0

        ret, frame = self.cap.read()

        if not ret or frame is None:
            self.consec_fail += 1
            if self.consec_fail >= config.MAX_CONSEC_FAIL:
                print("[CAM] read failing repeatedly -> reopening camera")
                self.release_camera()
                time.sleep(config.REOPEN_BACKOFF_SEC)
            else:
                time.sleep(0.02)
            return None, None
        
        self.consec_fail = 0
        
        h_full, w_full = frame.shape[:2]
        left  = frame[:, :w_full//2].copy()
        right = frame[:, w_full//2:].copy()

        payload = None
        if camera_mode == "Yolo":
            payload = self._run_hunting(left, right)
        elif camera_mode == "AprilTag":
            payload = self._run_tagging(left, right)
        else: # Rest
            payload = self._run_idle(left, right)

        now = time.time()
        dt = now - self.last_t
        self.last_t = now
        if dt > 0: self.fps = 0.9 * self.fps + 0.1 * (1.0 / dt)

        cv2.putText(left, f"FPS: {self.fps:.1f}", (10, left.shape[0] - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
        
        vis = np.hstack([left, right])
        return vis, payload