import asyncio
import time
from .utils import parse_payload

class ControllerModule:
    def __init__(self, sock, state):
        self.sock = sock
        self.state = state
        self._last_dpad_up = 0
        self.last_button3 = 0
        self.last_button4 = 0

    def state_flip(self):
        self.state.mode = "camera" if self.state.mode == "controller" else "controller"
        print(f"[STATE] Switched to: {self.state.mode}")

    def select_frame(self, button1, button2, button3, button4):
        # DPAD_LEFT increments
        if button1 == 1:
            self.state.select_state += 1
            print("select_state:", self.state.select_state)

        # DPAD_RIGHT decrements
        if button2 == 1:
            self.state.select_state -= 1
            print("select_state:", self.state.select_state)

        if self.state.select_state < 1:
            self.state.select_state = 1

        # X button toggles tracker
        if button3 == 1 and self.last_button3 == 0:
            self.state.tracker ^= 1
            print(f"tracker: {self.state.tracker}")
        self.last_button3 = button3
        
        # Y button toggles hunter
        if button4 == 1 and self.last_button4 == 0:
            self.state.hunter ^= 1
            print(f"hunter: {self.state.hunter}")
        self.last_button4 = button4

    async def run(self):
        loop = asyncio.get_running_loop()
        print(f"[CTRL UDP] Listening...")

        while True:
            data = await loop.sock_recv(self.sock, 1024)
            msg = data.decode("ascii", errors="ignore")

            axes, triggers, buttons = parse_payload(msg)
            if axes is None:
                continue

            self.state.axes = axes
            self.state.triggers = triggers
            self.state.buttons = buttons
            self.state.last_joy_time = time.monotonic()

            dpad_up = buttons["DPAD_UP"]
            if dpad_up == 1 and self._last_dpad_up == 0:
                self.state_flip()
            self._last_dpad_up = dpad_up

            self.select_frame(buttons["DPAD_LEFT"], buttons["DPAD_RIGHT"], buttons["X"], buttons["Y"])