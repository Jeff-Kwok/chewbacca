import asyncio
import serial
import json
import time

class STMModule:
    def __init__(self, port, baud, state,motors):
        self.port = port
        self.baud = baud
        self.state = state
        self.motors = motors
    def calculate_all_values(self,data):
        # Calculating ODOM
        w1_w = data.get("w1_w")
        w2_w = data.get("w2_w")
        w3_w = data.get("w3_w")
        w4_w = data.get("w4_w")   
        longitudinal_velocity, lateral_velocity, angular_velocity, roe, velocity_vector \
        = self.motors.calc_robot_velocity(w1_w, w2_w, w3_w, w4_w)

        self.state.stm_odom_payload["twist_x"] = longitudinal_velocity * 0.01
        self.state.stm_odom_payload["twist_y"] = lateral_velocity * 0.01 
        self.state.stm_odom_payload["angular_z"] = angular_velocity * 0.01

        # Calculating IMU
        angular_velocity_z = data.get("gz")
        linear_acceleration_x = data.get("lax")
        linear_acceleration_y = data.get("lay")
        magnetic_x = data.get("mx")
        magnetic_y = data.get("my")
        magnetic_z = data.get("mz")
        quaternion_x = data.get("qx")
        quaternion_y = data.get("qy")
        quaternion_z = data.get("qz")
        self.state.stm_imu_payload["angular_velocity_z"] = angular_velocity_z
        self.state.stm_imu_payload["linear_acceleration_x"] = linear_acceleration_x
        self.state.stm_imu_payload["linear_acceleration_y"] = linear_acceleration_y
        self.state.stm_imu_payload["magnetic_x"] = magnetic_x
        self.state.stm_imu_payload["magnetic_y"] = magnetic_y
        self.state.stm_imu_payload["magnetic_z"] = magnetic_z
        self.state.stm_imu_payload["quaternion_x"] = quaternion_x
        self.state.stm_imu_payload["quaternion_y"] = quaternion_y
        self.state.stm_imu_payload["quaternion_z"] = quaternion_z
        # Calculating IMU 
        return

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
                #print(data.get("w1_ticks"),data.get("w2_ticks"),data.get("w3_ticks"),data.get("w4_ticks"))
                #print(data.get("w1_w"),data.get("w2_w"),data.get("w3_w"),data.get("w4_w"))
                #print(data.get("lax"),data.get("lay"),data.get("laz"))
                #print(data.get("qx"),data.get("qy"),data.get("qz"))
                self.calculate_all_values(data)
            except Exception as e:
                print(f"[STM] Error: {e}")
                await asyncio.sleep(0.1)