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

        # --- NEW: mode cache + brake gate (reduces repeated work / I/O spam) ---
        self._entered_mode = None
        self._last_mode_name = None
        self._last_behavior = None
        self._last_brake = 0
        # ---------------------------------------------------------------------

    async def run(self):
        dt_nominal = 1.0 / config.CONTROL_HZ
        next_tick = time.monotonic() + dt_nominal
        last_time = time.monotonic()
        print(f"[CONTROL] Starting loop at {config.CONTROL_HZ} Hz")

        while True:
            now = time.monotonic()
            dt = now - last_time
            if dt <= 0.0:
                dt = dt_nominal
            last_time = now

            # Sleep to next tick (less drift, fewer extra monotonic calls)
            delay = next_tick - now
            if delay > 0:
                await asyncio.sleep(delay)
            else:
                # if we're behind, don't sleep; just resync
                await asyncio.sleep(0)

            next_tick += dt_nominal

            age = now - self.state.last_joy_time
            if age > config.JOY_TIMEOUT:
                # NEW: don't hammer CPU if joystick is stale
                await asyncio.sleep(dt_nominal)
                continue

            # NEW: gate repeated brake calls while RT held (avoid spamming I/O)
            if self.state.triggers["RT"] > 0:
                if self._last_brake == 0:
                    self.motors.brake_all_motors(message_toggle=False)
                    self._last_brake = 1
                continue
            else:
                self._last_brake = 0

            # Behavior Scripts (NEW: cache behavior lookup)
            robot_mode_current = self.state.robot_modes[self.state.robot_current]

            if robot_mode_current != self._last_mode_name:
                self._last_mode_name = robot_mode_current
                self._last_behavior = self.behavior_map.get(robot_mode_current)

            b = self._last_behavior
            if b is None:
                self.motors.brake_all_motors(message_toggle=False)
                continue

            if self.state.toggle == 0:
                b.stop(self, dt)
                self._entered_mode = None  # reset enter gating when not hunting
            else:
                if self._entered_mode != robot_mode_current:
                    if hasattr(b, "on_enter"):
                        b.on_enter(self)
                    self._entered_mode = robot_mode_current
                b.hunt(self, dt)
