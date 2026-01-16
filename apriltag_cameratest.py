import cv2
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

cap = cv2.VideoCapture(0)

while True:
    ret, frame = cap.read()
    if not ret:
        break

    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

    # If you know your camera intrinsics, pass them for pose estimation:
    # tags = detector.detect(gray, estimate_tag_pose=True,
    #                        camera_params=(fx, fy, cx, cy),
    #                        tag_size=0.162)  # meters
    tags = detector.detect(gray, estimate_tag_pose=False)

    for t in tags:
        # t.tag_id, t.center, t.corners are commonly available
        print("id:", t.tag_id, "center:", t.center)

    cv2.imshow("frame", frame)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()
