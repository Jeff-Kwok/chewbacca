# Network Configuration
LISTEN_IP = "0.0.0.0"
LISTEN_PORT = 5456    # controller UDP
CAMERA_PORT = 5005    # camera UDP
CAMERA_CONTROL_PORT = 5006 # camera control UDP
LIDAR_PORT = 6065
SEND_PORT = 7607
SEND_IP = "192.168.30.227"

# Hardware Configuration
STM_SERIAL_PORT = "/dev/ttyACM0"
STM_BAUD = 115200

MECANUM_PINS = {
    "nrr": [4, 5, 10],
    "nrl": [1, 0, 8],
    "nfr": [6, 7, 11],
    "nfl": [3, 2, 9],
}
# ---------------- CONFIG ----------------
IMAGE_RES = (640, 480)

# Reconnect behavior
OPEN_RETRY_SEC      = 1.0     # wait between open attempts
REOPEN_BACKOFF_SEC  = 0.5     # small pause after releasing camera
MAX_CONSEC_FAIL     = 10      # reopen after this many failed reads
FIND_DEVICE_EVERY_N = 1       # re-scan devices each reconnect (1 = always)

# YOLO behavior
MODEL_PATH = "models/yolo11m-pose.pt"
print(MODEL_PATH)
IMGSZ      = 512
TAG_SIZE_M = 0.166
CONF       = 0.35
STEREO_FOCAL_LENGTH = 700*(1/2.3) # pixels
STEREO_BASELINE = 60 # mm
YOLO_KPT_CONF = 0.6
YOLO_VERBOSE = False
# ----------------------------------------
SELF_SEND_IP = "127.0.0.1"

# Control Parameters
JOY_TIMEOUT = 0.5
CONTROL_HZ = 100.0
CAMERA_CENTER_X = 320
CAMERA_TOL = 40.0
CAMERA_TIMEOUT = 0.3

# Lidar Configuration
LIDAR_SERIAL_PORT = "/dev/ttyUSB0"
LIDAR_WS_PORT = 8765
LIDAR_MAX_RANGE = 12.0 # Meters
LIDAR_DOWNSAMPLE = 0 # Not being used 
LIDAR_AVOID_DISTANCES = {"close": 1.5, "middle": 2.5, "far": 3.5}

# apriltag
AT_FAMILIES = 'tagStandard41h12'
AT_NTHREADS = 1
AT_QUAD_DECIMATE = 1.0
AT_QUAD_SIGMA = 0.8
AT_REFINE_EDGES = 1
AT_DECODE_SHARPENING = 0.25
AT_DEBUG = 0
