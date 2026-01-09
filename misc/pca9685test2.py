import time
import busio
import board
from adafruit_motor import servo
from adafruit_pca9685 import PCA9685
# receiver.py
import socket
from select import select
import json
import numpy as np
# Initialize I2C
i2c = busio.I2C(board.SCL, board.SDA)
pca = PCA9685(i2c, address=0x40)
pca.frequency = 50

# Change channel if needed
servo0 = servo.Servo(
    pca.channels[0],
    min_pulse=500,
    max_pulse=2500
)
HOST = "0.0.0.0"
camera_PORT = 5005
controller_PORT = 5456

sock_camera = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock_camera.bind((HOST, camera_PORT))

sock_controller = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock_controller.bind((HOST, controller_PORT))

sockets = [sock_camera, sock_controller]

print(f"[receiver] Listening on {HOST}:{camera_PORT} (camera)")
print(f"[receiver] Listening on {HOST}:{controller_PORT} (controller)")

while True:
    # Block until at least one socket has data
    readable, _, _ = select(sockets, [], [])

    for s in readable:
        data, addr = s.recvfrom(1024)
        if s is sock_camera:
            #print(f"[CAMERA] from {addr}: {data.decode('utf-8', errors='ignore')}")
            payload = json.loads(data.decode("utf-8"))
        # Extract midx, midy
            midx, midy = payload["center"]
            print(midx)

            servo0.angle = np.interp(midx, [0, 640], [0, 180])
        elif s is sock_controller:
            print(f"[CTRL]   from {addr}: {data.decode('utf-8', errors='ignore')}")

