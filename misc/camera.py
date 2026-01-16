import asyncio
import json
import time
from .utils import calc_rot_speed
from . import config

class CameraModule:
    def __init__(self, sock, state, motors):
        self.sock = sock
        self.state = state
        self.motors = motors
        self.active_objects = {}

    async def run(self):
        loop = asyncio.get_running_loop()
        print(f"[CAMERA] Listening...")
        omega_max = self.motors.mecanum_configuration["max_rad/s"]

        while True:
            try:
                data = await asyncio.wait_for(
                    loop.sock_recv(self.sock, 4096),
                    timeout=config.CAMERA_TIMEOUT,
                )
            except asyncio.TimeoutError:
                if self.state.mode == "camera":
                    self.motors.brake_all_motors(message_toggle=False)
                continue

            try:
                payload = json.loads(data.decode("utf-8"))
            except json.JSONDecodeError:
                print("[CAMERA] bad JSON payload")
                continue

            try:
                now_time = time.monotonic()
                midx, midy = payload["center"]
                label = payload["id"]
                id_num = int(label.split("_")[-1])
                
                self.active_objects[label] = now_time
                cutoff = now_time - config.CAMERA_TIMEOUT
                for oid, t_last in list(self.active_objects.items()):
                    if t_last < cutoff:
                        del self.active_objects[oid]
                current_count = len(self.active_objects)

                if self.state.select_state > current_count:
                    self.state.select_state = current_count
                elif self.state.select_state < 1:
                    self.state.select_state = 1
            except (KeyError, ValueError, TypeError):
                continue

            if self.state.triggers["RT"] > 0:
                self.motors.brake_all_motors(message_toggle=False)
                continue

            if self.state.mode == "camera" and self.state.tracker == 1:
                if self.state.select_state != id_num:
                    continue
                
                print(f"[CAMERA] center: {midx},{midy} label={label} sel={self.state.select_state}")
                distance = config.CAMERA_CENTER_X - midx
                if abs(distance) <= config.CAMERA_TOL:
                    print("[CAMERA] Target centered.")
                    self.motors.brake_all_motors(message_toggle=False)
                    continue

                speed_cmd = calc_rot_speed(midx, config.CAMERA_TOL, config.CAMERA_CENTER_X, 320.0, omega_max)
                rotate_command = -(speed_cmd/omega_max)*3.14/4.0
                
                checkx = 0
                checky = 0.5
                if abs(self.state.zone["x_sum"]) >= 0.12:
                    checkx = -self.state.zone["x_sum"]
                if abs(self.state.zone["y_sum"]) >= 0.12:
                    checky = -self.state.zone["y_sum"]

                if self.state.hunter == 1:
                    FL, FR, RL, RR = self.motors.calc_norm_vector(checkx, checky, rotate_command)
                else:
                    FL, FR, RL, RR = self.motors.calc_norm_vector(0, 0, rotate_command)
                
                self.motors.drive_all_wheels({
                    "nfr": FR, "nfl": FL, "nrr": RR, "nrl": RL,
                })
            else:
                self.motors.brake_all_motors(message_toggle=False)