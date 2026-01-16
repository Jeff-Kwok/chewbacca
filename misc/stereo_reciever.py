import asyncio
import json
from . import config

class StereoReceiver:
    def __init__(self, sock, state, motors):
        self.sock = sock
        self.state = state
        self.motors = motors

    async def run(self):
        print("[STEREO] Listening for JSON stereo data...")
        loop = asyncio.get_running_loop()
        
        while True:
            try:
                data = await asyncio.wait_for(
                    loop.sock_recv(self.sock, 4096),
                    timeout=config.CAMERA_TIMEOUT
                )
                try:
                    msg = json.loads(data.decode("utf-8"))
                    z = msg.get("z")
                    angle = msg.get("angle")
                    
                    if z is not None and angle is not None:
                        self.state.hunted["distance"] = z
                        self.state.hunted["angle"] = angle
                    else:
                        self.state.hunted["distance"] = 0.0
                        self.state.hunted["angle"] = 0.0
                        
                    #obj_id = msg.get("id", "unknown")
                    #print(f"ID={obj_id} | Z={self.state.hunted['distance']:.2f} | Angle={self.state.hunted['angle']:.2f}")
                    
                except json.JSONDecodeError:
                    print("[STEREO] Invalid JSON")
            except asyncio.TimeoutError:
                self.state.hunted["distance"] = 0.0
                self.state.hunted["angle"] = 0.0
            except Exception as e:
                print(f"[STEREO] Error: {e}")
                await asyncio.sleep(0.1)
