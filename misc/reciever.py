# receiver.py
import socket
from select import select

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
            print(f"[CAMERA] from {addr}: {data.decode('utf-8', errors='ignore')}")
        elif s is sock_controller:
            print(f"[CTRL]   from {addr}: {data.decode('utf-8', errors='ignore')}")
