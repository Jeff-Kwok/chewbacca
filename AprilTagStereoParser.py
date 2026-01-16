import cv2
import numpy as np
from dt_apriltags import Detector
detector = Detector(
    searchpath=['/usr/local/lib'],  # optional; can omit
    families='tag36h11',
    nthreads=4,
    quad_decimate=1.0,
    quad_sigma=0.0,
    refine_edges=1,
    decode_sharpening=0.25,
    debug=0
)
def split_stereo_side_by_side(frame):
    """Split a side-by-side stereo frame into (left, right)."""
    h, w = frame.shape[:2]
    mid = w // 2
    left = frame[:, :mid]
    right = frame[:, mid:]
    return left, right

def detect_centers(detector, img_bgr_or_gray):
    # dt_apriltags wants grayscale
    if img_bgr_or_gray.ndim == 3:
        gray = cv2.cvtColor(img_bgr_or_gray, cv2.COLOR_BGR2GRAY)
    else:
        gray = img_bgr_or_gray

    detections = detector.detect(gray, estimate_tag_pose=False)

    centers = {}
    for d in detections:
        # d is a Detection object (not a dict)
        tag_id = int(d.tag_id)
        cx, cy = float(d.center[0]), float(d.center[1])
        centers[tag_id] = (cx, cy)

    return centers

def match_stereo_centers(left_centers, right_centers):
    """
    Returns dict: {tag_id: ((xL,yL), (xR,yR))} only for ids seen in both.
    """
    matched = {}
    for tag_id, cL in left_centers.items():
        if tag_id in right_centers:
            matched[tag_id] = (cL, right_centers[tag_id])
    return matched

def main_packed_stereo():
    # Read packed stereo image
    cap = cv2.VideoCapture(0)  # or GStreamer pipeline
    while True:
        ok, frame = cap.read()
        if not ok:
            break

        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        left, right = split_stereo_side_by_side(gray)

        left_centers  = detect_centers(detector, left)
        right_centers = detect_centers(detector, right)
        matched = match_stereo_centers(left_centers, right_centers)
        # Now you have per-tag (xL,yL,xR,yR) synchronized by construction.
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break


        print(f"Left detections: {len(left_centers)} | Right detections: {len(right_centers)}")
        print(f"Matched (both cameras): {len(matched)}")

        for tag_id, (cL, cR) in matched.items():
            xL, yL = cL
            xR, yR = cR
            print(f"tag {tag_id}: L=({xL:.2f},{yL:.2f})  R=({xR:.2f},{yR:.2f})")
            cv2.circle(frame, (int(xL), int(yL)), 4, (0,0,255), 1)
            cv2.circle(frame, (int(xR)+320, int(yR)), 4, (0,0,255), 1)
        cv2.imshow("frame", frame)

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
    cap.release()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    main_packed_stereo()
