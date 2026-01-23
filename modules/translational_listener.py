import asyncio
import json
import socket
import time

class TranslationListener:
    def __init__(self, ip, port, state):
        self.ip = ip
        self.port = port
        self.state = state
        self.sock = None

        # Optional: create these on state if you haven't already
        if not hasattr(self.state, "tf_pose"):
            self.state.tf_pose = None
        if not hasattr(self.state, "tf_map_pose"):
            self.state.tf_map_pose = {"x": 0.0, "y": 0.0, "yaw": 0.0}
        if not hasattr(self.state, "tf_last_time"):
            self.state.tf_last_time = 0.0

    def _update_state(self, msg: dict):
        """Parse incoming message and update state."""
        self.state.tf_pose = msg
        self.state.tf_last_time = time.time()
        # Prefer map pose
        if isinstance(msg, dict) and "map" in msg and isinstance(msg["map"], dict):
            m = msg["map"]
            x = float(m.get("x", 0.0))
            y = float(m.get("y", 0.0))
            yaw = float(m.get("yaw", 0.0))
            self.state.tf_map_pose["x"] = x
            self.state.tf_map_pose["y"] = y
            self.state.tf_map_pose["yaw"] = yaw
            return

        # Fallback to odom pose (if that's all you send)
        if isinstance(msg, dict) and "odom" in msg and isinstance(msg["odom"], dict):
            o = msg["odom"]
            x = round(float(o.get("x", 0.0)),4)
            y = round(float(o.get("y", 0.0)),4)
            yaw = round(float(o.get("yaw", 0.0)),4)
            self.state.tf_map_pose["x"] = x
            self.state.tf_map_pose["y"] = y
            self.state.tf_map_pose["yaw"] = yaw

    async def run(self):
        print(f"[TF-UDP] Binding {self.ip}:{self.port}...")
        try:
            self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            self.sock.bind((self.ip, self.port))
            self.sock.setblocking(True)
        except Exception as e:
            print(f"[TF-UDP] Failed to bind UDP: {e}")
            return

        while True:
            #print("trying")
            try:
                # recvfrom is blocking, so run it in a thread (like your serial)
                data, addr = await asyncio.to_thread(self.sock.recvfrom, 65535)
                #print(data)
                if not data:
                    continue

                s = data.decode("utf-8", errors="ignore").strip()
                if not s:
                    continue

                # Sender uses JSON + "\n" so .strip() is fine.
                # If multiple lines ever appear, try line-by-line.
                try:
                    msg = json.loads(s)
                except json.JSONDecodeError:
                    msg = None
                    for line in s.splitlines():
                        line = line.strip()
                        if not line:
                            continue
                        try:
                            msg = json.loads(line)
                            break
                        except json.JSONDecodeError:
                            continue
                    if msg is None:
                        continue

                self._update_state(msg)

            except Exception as e:
                print(f"[TF-UDP] Error: {e}")
                await asyncio.sleep(0.05)
