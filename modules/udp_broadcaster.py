import asyncio
import socket
import json
from . import config

class UdpBroadcaster:
    def __init__(self, state,motors):
        self.state = state
        self.motors = motors
        #
        self.lidar_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.stm_odom_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.stm_imu_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.stm_generic_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        # Declaring port
        self.lidar_address = (config.UDP_BROADCASTER_IP, config.UDP_BROADCASTER_PORT_LIDAR)
        self.stm_odom_address = (config.UDP_BROADCASTER_IP, config.UDP_BROADCASTER_PORT_STM_ODOM)
        self.stm_imu_address = (config.UDP_BROADCASTER_IP, config.UDP_BROADCASTER_PORT_STM_IMU)
        self.stm_generic_address = (config.SEND_IP, config.SEND_PORT)

    async def run(self):
        print("[UDP Broadcaster] Starting...")
        while True:
            try:
                if self.state.lidar_payload:
                    message = json.dumps(self.state.lidar_payload).encode('utf-8')
                    self.lidar_socket.sendto(message, self.lidar_address)

                if self.state.stm_odom_payload:
                    message = json.dumps(self.state.stm_odom_payload).encode('utf-8')
                    self.stm_odom_socket.sendto(message, self.stm_odom_address)
                
                if self.state.stm_imu_payload:
                    message = json.dumps(self.state.stm_imu_payload).encode('utf-8')
                    self.stm_imu_socket.sendto(message, self.stm_imu_address)
                #'''
                if self.state.stm:
                    message = json.dumps(self.state.stm).encode('utf-8')
                    self.stm_generic_socket.sendto(message, self.stm_generic_address)
                #'''
                await asyncio.sleep(1.0 / config.UDP_BROADCASTER_HZ)
            except Exception as e:
                print(f"[UDP Broadcaster] Error: {e}")
                await asyncio.sleep(1)
