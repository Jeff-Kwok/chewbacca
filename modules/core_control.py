import asyncio
import time
from . import config
import numpy as np
class ControlLoop:
    def __init__(self, state, motors):
        self.state = state
        self.motors = motors

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

            if self.state.mode != "controller":
                if self.state.triggers["RT"] > 0:
                    self.motors.brake_all_motors(message_toggle=False)
                continue

            if self.state.triggers["RT"] > 0:
                self.motors.brake_all_motors(message_toggle=False)
                continue

            # Drive motors using SafetyModule output
            x = self.state.safe_axes["LX"]
            y = self.state.safe_axes["LY"]
            rx = self.state.axes["RX"]*-1
            ry = self.state.axes["RY"]
            # STM32 Yaw Reading
            yaw = (np.deg2rad(self.state.stm["yaw"])-np.pi/2) % (2*np.pi)
            # Predicted angle reading from controller 
            angle = (np.arctan2(ry,rx)-np.pi/2) % (2*np.pi)
            # w,val needs to match
            w,val = self.motors.calc_robot_yaw(angle,yaw)

            #print(f"yaw: {yaw:.2f} angle: {angle:.2f}:.2f val: {val:.2f}:.2f w: {w:.2f}\n")
            w_fl, w_fr, w_rl, w_rr = self.motors.calc_norm_vector(x, y, w)
            self.motors.drive_all_wheels({
                "nfr": w_fr, "nfl": w_fl, "nrr": w_rr, "nrl": w_rl,
            })
