import asyncio
import time
from . import config
from . import behavior
import numpy as np
class ControlLoop:
    def __init__(self, state, motors):
        self.state = state
        self.motors = motors

        self.behavior_map = {
            "Rest": behavior.Rest(),
            "Manual": behavior.Manual(),
            "Follower": behavior.Follower(),
            "Tag": behavior.Tag(),
        }

    async def run(self):
        dt_nominal = 1.0 / config.CONTROL_HZ
        last_time = time.monotonic()
        print(f"[CONTROL] Starting loop at {config.CONTROL_HZ} Hz")

        while True:
            now = time.monotonic()
            dt = now - last_time
            if dt <= 0.0: dt = dt_nominal
            last_time = now
            
            await asyncio.sleep(max(0.0, dt_nominal - (time.monotonic() - now)))

            age = now - self.state.last_joy_time
            if age > config.JOY_TIMEOUT:
                continue

            if self.state.triggers["RT"] > 0:
                self.motors.brake_all_motors(message_toggle=False)
                continue
            
            # Behavior Scripts
            robot_mode_current = self.state.robot_modes[self.state.robot_current]
            b = self.behavior_map.get(robot_mode_current)
            if b is None:
                self.motors.brake_all_motors(message_toggle=False)
                continue

            if self.state.toggle == 0:
                b.stop(self, dt)
                self._entered_mode = None          # reset enter gating when not hunting
            else:
                if self._entered_mode != robot_mode_current:
                    if hasattr(b, "on_enter"):
                        b.on_enter(self)
                    self._entered_mode = robot_mode_current
                b.hunt(self, dt)

                