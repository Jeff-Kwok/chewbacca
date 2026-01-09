# sender.py
import socket
import time

TARGET_HOST = "127.0.0.1"  # same net namespace = localhost works
TARGET_PORT = 5005

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

for i in range(10):
    msg = f"hello from sidecar #{i}"
    sock.sendto(msg.encode("utf-8"), (TARGET_HOST, TARGET_PORT))
    print(f"[sender] Sent: {msg}")
    time.sleep(1.0)
