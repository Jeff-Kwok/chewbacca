import asyncio
import socket
import serial
import json
import time

class STMModule:
    def __init__(self, port, baud, state,socket,SEND_PORT,SEND_ID):
        self.port = port
        self.baud = baud
        self.state = state
        self.socket = socket
        self.SEND_PORT = SEND_PORT
        self.SEND_ID = SEND_ID

    async def run(self):
        print(f"[STM] Opening {self.port}...")
        try:
            ser = serial.Serial(self.port, self.baud, timeout=1)
            time.sleep(2.0)
        except Exception as e:
            print(f"[STM] Failed to open serial: {e}")
            return

        while True:
            try:
                line = await asyncio.to_thread(ser.readline)
                if not line:
                    continue
                line = line.decode("utf-8", errors="ignore").strip()
                if not line or not line.startswith("{"):
                    continue
                
                data = json.loads(line)
                # Update state
                for k, v in data.items():
                    if k in self.state.stm:
                        self.state.stm[k] = v
                #print(f"[STM] Data: {data}")
                try:
                    message = json.dumps(data).encode('utf-8')
                    self.socket.sendto(message, (self.SEND_ID, self.SEND_PORT))
                except Exception as e:
                    print("[STM] UDP Send Error:", e)
            except Exception as e:
                print(f"[STM] Error: {e}")
                await asyncio.sleep(0.1)