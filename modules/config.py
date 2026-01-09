# Network Configuration
LISTEN_IP = "0.0.0.0"
LISTEN_PORT = 5456    # controller UDP
CAMERA_PORT = 5005    # camera UDP
LIDAR_PORT = 6065
SEND_PORT = 7607
SEND_IP = "192.168.0.227"

# Hardware Configuration
STM_SERIAL_PORT = "/dev/ttyACM0"
STM_BAUD = 115200

MECANUM_PINS = {
    "nrr": [4, 5, 10],
    "nrl": [1, 0, 8],
    "nfr": [6, 7, 11],
    "nfl": [3, 2, 9],
}

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
