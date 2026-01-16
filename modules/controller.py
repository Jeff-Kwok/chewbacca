import asyncio
import time
import socket
import json
from . import config
from .utils import parse_payload

class ControllerModule:
    def __init__(self, sock, state):
        self.sock = sock
        self.state = state
        self.last_button3 = 0 
        self.last_lb = 0
        self.last_rb = 0
        self.last_mode = 0
        self._last_dpad_down = 0

    def camera_mode_flip(self):
        self.state.camera_current = (self.state.camera_current + 1) % len(self.state.camera_modes)
        new_mode = self.state.camera_modes[self.state.camera_current]
        print(f"[CAMERA] Switched to: {new_mode}")

    def select_frame(self, leftTrigger, rightTrigger, toggle):
        # Mode shifting from ["Rest","Manual","Follower","Tag"]
        if leftTrigger == 1:
            self.state.robot_current -= 1
            self.state.robot_current = max(0, self.state.robot_current)
            print(self.state.robot_modes[self.state.robot_current])
        if rightTrigger == 1:
            self.state.robot_current += 1
            self.state.robot_current = min(self.state.robot_current, 3)
            print(self.state.robot_modes[self.state.robot_current])
        # X button toggles tracker

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

            dpad_down = buttons["DPAD_DOWN"]
            if dpad_down == 1 and self._last_dpad_down == 0:
                self.camera_mode_flip()
            self._last_dpad_down = dpad_down

            # Robot Mode Cycler -> To go to new modes press RB and LB, toggle will always set to be off when transitioning to a new mode
            lb = buttons["LB"]
            rb = buttons["RB"]
            lb_pressed = (lb == 1 and self.last_lb == 0)
            rb_pressed = (rb == 1 and self.last_rb == 0)

            if lb_pressed:
                new_idx = self.state.robot_current - 1
                if new_idx >= 0:
                    self.state.robot_current = new_idx
                    self.state.toggle = 0
                    print(self.state.robot_modes[self.state.robot_current])

            if rb_pressed:
                new_idx = self.state.robot_current + 1
                if new_idx < len(self.state.robot_modes):
                    self.state.robot_current = new_idx
                    self.state.toggle = 0
                    print(self.state.robot_modes[self.state.robot_current])


            self.last_lb = lb
            self.last_rb = rb

            toggle = buttons["A"]
            if toggle == 1 and self.last_button3 == 0:
                self.state.toggle ^= 1
                print(f"Toggle: {self.state.toggle}")
            self.last_button3 = toggle
           # self.select_frame(buttons["LB"],buttons["RB"],buttons["A"])