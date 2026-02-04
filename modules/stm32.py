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
        w1_w = data.get("w1_w") # rear left
        w2_w = data.get("w2_w") # forward left
        w3_w = data.get("w3_w") # # forward right
        w4_w = data.get("w4_w") # rear right
        gz = data.get("gz")
        #print(data.get("w1_ticks"),data.get("w2_ticks"),data.get("w3_ticks"),data.get("w4_ticks"))
        #print(f"w1_w: {w1_w:.2f} | w2_w: {w2_w:.2f} | w3_w: {w3_w:.2f} | w4_w: {w4_w:.2f}")
        yaw = data.get("yaw")   
        longitudinal_velocity, lateral_velocity, angular_velocity = self.motors.calc_robot_velocity(w1_w, w2_w, w3_w, w4_w)
        #self.state.stm_odom_payload["pose_orientation_z"] = yaw 
        # Flipping lateral directions here because postiive reads to the right normally, but our positive is to the left based on stick convention and ros2 slam
        self.state.stm_odom_payload["twist_x"] = longitudinal_velocity 
        self.state.stm_odom_payload["twist_y"] = lateral_velocity 
        #print(
         #   f"lateral_velocity: {lateral_velocity:.2f} | longitudinal_velocity: {longitudinal_velocity:.2f} | angular_velocity: {angular_velocity:.2f} | gz: {gz:.2f} | yaw: {yaw:.2f}"
          #  )
        self.state.stm_odom_payload["angular_z"] = angular_velocity

        # Calculating IMU
        self.state.stm_imu_payload["quaternion_w"] = data.get("qw", 1.0)
        self.state.stm_imu_payload["quaternion_x"] = data.get("qx", 0.0)
        self.state.stm_imu_payload["quaternion_y"] = data.get("qy", 0.0)
        self.state.stm_imu_payload["quaternion_z"] = data.get("qz", 0.0)

        # Gyro (your Arduino comment says rad/s; BNO055 VECTOR_GYROSCOPE in Adafruit lib is rad/s)
        self.state.stm_imu_payload["angular_velocity_x"] = data.get("gx", 0.0)
        self.state.stm_imu_payload["angular_velocity_y"] = data.get("gy", 0.0)
        self.state.stm_imu_payload["angular_velocity_z"] = data.get("gz", 0.0)

        # Linear acceleration (gravity removed) -> best choice for IMU msg if you enable remove_gravity in EKF
        self.state.stm_imu_payload["linear_acceleration_x"] = data.get("lax", 0.0)
        self.state.stm_imu_payload["linear_acceleration_y"] = data.get("lay", 0.0)
        self.state.stm_imu_payload["linear_acceleration_z"] = data.get("laz", 0.0)

        # Magnetometer (microtesla)
        self.state.stm_imu_payload["magnetic_x"] = data.get("mx", 0.0)
        self.state.stm_imu_payload["magnetic_y"] = data.get("my", 0.0)
        self.state.stm_imu_payload["magnetic_z"] = data.get("mz", 0.0)
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
                #print(f"[STM] Error: {e}")
                await asyncio.sleep(0.025)