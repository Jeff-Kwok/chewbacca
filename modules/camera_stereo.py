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
import subprocess

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
    if corners is not None:
        payload["corners"] = [[float(x), float(y)] for (x, y) in corners]
    return payload


class CameraStereo:
    """
    UPDATED (per request):
      - ONLY runs YOLO when camera_mode == "Yolo"
      - ONLY runs AprilTag when camera_mode == "AprilTag"
      - REST/other modes do NOT run either pipeline
      - Pipelines are lazy-initialized and (optionally) torn down on mode switches
      - Keeps your naming conventions and existing structure; changes are minimal and targeted
    """
    def __init__(self, state):
        cv2.setNumThreads(1)
        os.environ.setdefault("OMP_NUM_THREADS", "1")
        os.environ.setdefault("MKL_NUM_THREADS", "1")
        os.environ.setdefault("OPENBLAS_NUM_THREADS", "1")
        os.environ.setdefault("NUMEXPR_NUM_THREADS", "1")

        try:
            torch.set_num_threads(1)
            torch.set_num_interop_threads(1)
        except Exception:
            pass

        self.state = state

        # Camera
        self.cap = None
        self.device = None
        self.consec_fail = 0
        self.last_t = time.time()
        self.fps = 0.0

        # Stereo params
        self.f = config.STEREO_FOCAL_LENGTH
        self.B = config.STEREO_BASELINE

        # Pipelines (LAZY INIT)
        self.model = None
        self.at_detector = None
        self.at_detector_bad = False

        # Track last mode so we can unload inactive pipeline
        self._last_mode = None

        # Model path
        default_engine = "yolo11n-pose.engine"
        model_path = getattr(config, "MODEL_PATH", default_engine) or default_engine
        self._resolved_model_path = self._resolve_model_path(model_path)

        # YOLO settings
        self._yolo_imgsz = 320
        self._yolo_conf = float(getattr(config, "CONF", 0.50))
        self._yolo_kpt_conf = float(getattr(config, "YOLO_KPT_CONF", 0.30))
        self._yolo_max_det = int(getattr(config, "YOLO_MAX_DET", 2))

        self._last_yolo_z = None
        self._last_yolo_angle = None

    def _resolve_model_path(self, model_path: str) -> str:
        if not model_path:
            return model_path

        if os.path.isabs(model_path) and os.path.exists(model_path):
            return model_path

        try:
            here = os.path.dirname(os.path.abspath(__file__))
            cand = os.path.join(here, model_path)
            if os.path.exists(cand):
                return cand
        except Exception:
            pass

        return model_path

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

    def _force_v4l2_mode(self, device: str, width: int, height: int, fps: int, pixfmt: str):
        """
        Force mode via v4l2-ctl (OpenCV sometimes ignores FPS/pixfmt).
        """
        try:
            subprocess.run(
                ["v4l2-ctl", "-d", device, f"--set-fmt-video=width={width},height={height},pixelformat={pixfmt}"],
                check=False, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL
            )
            subprocess.run(
                ["v4l2-ctl", "-d", device, f"--set-parm={fps}"],
                check=False, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL
            )
        except Exception:
            pass

    def open_camera(self, device: str, width: int, height: int) -> cv2.VideoCapture | None:
        """
        Match your fast standalone path: MJPG 640x240 @ 120.
        """
        self._force_v4l2_mode(device=device, width=640, height=240, fps=120, pixfmt="MJPG")

        cap = cv2.VideoCapture(device, cv2.CAP_V4L2)
        if not cap.isOpened():
            cap.release()
            return None

        cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 240)
        cap.set(cv2.CAP_PROP_FPS, 120)
        cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)

        w = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
        h = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
        fps = cap.get(cv2.CAP_PROP_FPS)
        fourcc = int(cap.get(cv2.CAP_PROP_FOURCC))
        fourcc_str = "".join([chr((fourcc >> 8*i) & 0xFF) for i in range(4)])
        print(f"[CAM] negotiated: {w}x{h} fps:{fps} fourcc:{fourcc_str}")

        return cap

    def release_camera(self):
        if self.cap is not None:
            self.cap.release()
        self.cap = None

    # ---------- PIPELINE LIFECYCLE ----------
    def _ensure_mode(self, camera_mode: str):
        """
        Ensure ONLY the pipeline for the active mode exists.
        Prevents AprilTag from running/competing while YOLO is active (and vice versa).
        """
        if camera_mode == self._last_mode:
            return

        # Switching modes: drop anything not needed
        if camera_mode == "Yolo":
            # drop AprilTag
            if self.at_detector is not None:
                self.at_detector = None
                gc.collect()
            self.at_detector_bad = False

        elif camera_mode == "AprilTag":
            # drop YOLO
            if self.model is not None:
                self.model = None
                gc.collect()

        else:
            # Rest/other: drop both
            if self.model is not None:
                self.model = None
            if self.at_detector is not None:
                self.at_detector = None
            self.at_detector_bad = False
            gc.collect()

        self._last_mode = camera_mode

    # ---------- STEREO ----------
    def stereo_depth(self, cx_left, cx_right, fx, baseline):
        d = cx_left - cx_right
        if d <= 0:
            return None
        return (fx * baseline / d)

    # ---------- APRILTAG ----------
    def _init_apriltag_detector(self):
        """
        LAZY: only called when camera_mode == "AprilTag"
        """
        try:
            if self.at_detector is not None:
                self.at_detector = None
                gc.collect()
                time.sleep(0.05)

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
            print("[CAM] AprilTag Detector initialized (lazy).")
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
        return {tid: (L, right_map[tid]) for tid, L in left_map.items() if tid in right_map}

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

    # ---------- YOLO DRAW ----------
    def draw_pose_kpts(self, frame, r, det_i: int = 0, kpt_thr=None):
        if kpt_thr is None:
            kpt_thr = self._yolo_kpt_conf

        if r is None or r.keypoints is None or len(r.keypoints) == 0:
            return
        if det_i < 0 or det_i >= len(r.keypoints):
            return

        pts = r.keypoints.xy[det_i].cpu().numpy()
        conf = None
        if hasattr(r.keypoints, "conf") and r.keypoints.conf is not None:
            conf = r.keypoints.conf[det_i].cpu().numpy()

        def ok(i):
            return True if conf is None else (conf[i] >= kpt_thr)

        for i in range(pts.shape[0]):
            x, y = pts[i]
            if not np.isfinite(x) or not np.isfinite(y) or not ok(i):
                continue
            cv2.circle(frame, (int(x), int(y)), 3, (0, 0, 255), -1)

        if pts.shape[0] >= 17:
            pairs = [
                (5, 6), (5, 7), (7, 9), (6, 8), (8, 10),
                (5, 11), (6, 12), (11, 12),
                (11, 13), (13, 15), (12, 14), (14, 16),
                (0, 1), (0, 2), (1, 3), (2, 4)
            ]
            for a, b in pairs:
                if not (ok(a) and ok(b)):
                    continue
                xa, ya = pts[a]
                xb, yb = pts[b]
                if not (np.isfinite(xa) and np.isfinite(ya) and np.isfinite(xb) and np.isfinite(yb)):
                    continue
                cv2.line(frame, (int(xa), int(ya)), (int(xb), int(yb)), (0, 255, 255), 2)

    def _overlay_text(self, frame, lines, org=(10, 30), line_h=24):
        x, y = org
        for s in lines:
            cv2.putText(frame, s, (x, y), cv2.FONT_HERSHEY_SIMPLEX, 0.65, (255, 255, 0), 2)
            y += line_h

    # ---------- MODES ----------
    def _run_hunting_batch2(self, left, right):
        """
        YOLO MODE:
          - Lazy-init YOLO only when mode is active
          - Single batched call: predict([left, right])
        """
        if self.model is None:
            print("[CAM] Initializing YOLO model...")
            self.model = YOLO(self._resolved_model_path)
            print(f"[CAM] YOLO model loaded: {self._resolved_model_path}")

            dummy = np.zeros((self._yolo_imgsz, self._yolo_imgsz, 3), dtype=np.uint8)
            _ = self.model.predict([dummy, dummy],
                                   imgsz=self._yolo_imgsz,
                                   conf=self._yolo_conf,
                                   device=0,
                                   max_det=self._yolo_max_det,
                                   verbose=False)

        results = self.model.predict(
            source=[left, right],
            imgsz=self._yolo_imgsz,
            conf=self._yolo_conf,
            device=0,
            max_det=self._yolo_max_det,
            verbose=False
        )

        rL = results[0] if len(results) > 0 else None
        rR = results[1] if len(results) > 1 else None

        #self.draw_pose_kpts(left, rL, det_i=0, kpt_thr=self._yolo_kpt_conf)
        #self.draw_pose_kpts(right, rR, det_i=0, kpt_thr=self._yolo_kpt_conf)

        def center_from_result(r):
            if r is None or r.keypoints is None or len(r.keypoints) == 0:
                return None
            pts = r.keypoints.xy[0].cpu().numpy()
            xs, ys = pts[:, 0], pts[:, 1]
            if not np.isfinite(xs).any() or not np.isfinite(ys).any():
                return None
            cx = float(np.nanmin(xs) + np.nanmax(xs)) / 2.0
            cy = float(np.nanmin(ys) + np.nanmax(ys)) / 2.0
            return cx, cy

        cL = center_from_result(rL)
        cR = center_from_result(rR)

        z = None
        angle = None
        if cL is not None and cR is not None:
            cxL, cyL = cL
            cxR, cyR = cR
            z = self.stereo_depth(cxL, cxR, self.f, self.B)

            w_half = left.shape[1]
            angle = float(np.arctan(((w_half / 2.0) - cxL) / self.f))

        self._last_yolo_z = z
        self._last_yolo_angle = angle

        self._overlay_text(left, [
            "MODE: YOLO (batch=2 L|R)",
            f"z: {z:.2f} m" if z is not None else "z: --",
            f"ang: {angle:+.3f} rad" if angle is not None else "ang: --",
        ], org=(10, 30))
        '''
        self._overlay_text(left, [
            "MODE: YOLO (batch=2)",
            f"kpt_thr: {self._yolo_kpt_conf:.2f}",
            f"conf: {self._yolo_conf:.2f}",
        ], org=(10, 60))
        '''

        if cL is not None and cR is not None and (z is not None) and (angle is not None):
            return build_vision_payload(cL[0], cL[1], 0, z, angle)
        return None

    def _run_tagging(self, left, right):
        """
        AprilTag MODE:
          - Lazy-init Detector only when mode is active
          - ONLY runs in AprilTag mode (never in Yolo mode)
        """
        if self.at_detector is None and not self.at_detector_bad:
            self._init_apriltag_detector()

        try:
            left_info = self.detect_with_pose(self.at_detector, left, self.f, self.f, config.TAG_SIZE_M)
            right_info = self.detect_with_pose(self.at_detector, right, self.f, self.f, config.TAG_SIZE_M)
        except Exception as e:
            print(f"[CAM] AprilTag detect exception: {e}")
            if not self.at_detector_bad:
                print("[CAM] Reinitializing AprilTag detector once due to error...")
                self.at_detector_bad = True
                self._init_apriltag_detector()
            return None

        matched = self.match_by_id(left_info, right_info)
        self.draw_tag_debug(left, left_info)
        self.draw_tag_debug(right, right_info)

        for tag_id, (L, R) in matched.items():
            cx_left = L["center"][0]
            cx_right = R["center"][0]
            z_m = self.stereo_depth(cx_left, cx_right, self.f, self.B)
            if z_m is not None:
                angle = None
                t = None
                if L["t"] is not None:
                    t = L["t"].reshape(-1)
                    angle = math.atan2(-float(t[0]), float(t[2]))
                corners = L.get("corners")
                z_out = float(t[2]) if (t is not None and len(t) >= 3) else float(z_m)
                return build_vision_payload(L["center"][0], L["center"][1], tag_id, z_out, angle, corners)
        return None

    def _run_idle(self, left, right):
        cv2.putText(left, "MODE: REST", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 0), 2)
        return None

    # ---------- MAIN LOOP ----------
    def process_frame(self, camera_mode):
        # Ensure only the right pipeline is alive for this mode
        self._ensure_mode(camera_mode)
        t0 = time. perf_counter()
        if self.cap is None:
            self.device = self.find_camera_device()
            if self.device is None:
                print(f"[CAM] no camera device found. retrying in {config.OPEN_RETRY_SEC}s...")
                time.sleep(config.OPEN_RETRY_SEC)
                return None, None

            self.cap = self.open_camera(self.device, 640, 240)
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
                time.sleep(0.005)
            return None, None

        self.consec_fail = 0

        h_full, w_full = frame.shape[:2]
        mid = w_full // 2

        left = frame[:, :mid].copy()
        right = frame[:, mid:].copy()

        payload = None
        if camera_mode == "Yolo":
            payload = self._run_hunting_batch2(left, right)
        elif camera_mode == "AprilTag":
            payload = self._run_tagging(left, right)
        else:
            payload = self._run_idle(left, right)

        now = time.time()
        dt = now - self.last_t
        self.last_t = now
        if dt > 0:
            self.fps = 0.9 * self.fps + 0.1 * (1.0 / dt)

        cv2.putText(left, f"FPS: {self.fps:.1f}", (10, left.shape[0] - 10),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)

        vis = left
        t1 = time.perf_counter()
        #print("process frame:",(t1-t0)*1000)
        return vis, payload
