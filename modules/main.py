import asyncio
import socket
import sys
import os

# Ensure we can import motorfunctions from parent directory
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))

from motorfunctions import MotorFunctions
from .state import RobotState
from . import config
from .controller import ControllerModule
from .stereo_reciever import StereoReceiver
from .stm32 import STMModule
from .lidar import LidarModule
from .core_control import ControlLoop
from .safety import SafetyModule

async def main():
    motors = MotorFunctions()
    motors.set_all_direction_pins(config.MECANUM_PINS)
    
    state = RobotState()
    
    sock_ctrl = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock_ctrl.bind((config.LISTEN_IP, config.LISTEN_PORT))
    sock_ctrl.setblocking(False)
    
    sock_cam = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock_cam.bind((config.LISTEN_IP, config.CAMERA_PORT))
    sock_cam.setblocking(False)

    sock_desktop = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

    ctrl_mod = ControllerModule(sock_ctrl, state)
    stereo_mod = StereoReceiver(sock_cam, state, motors)
    stm_mod = STMModule(config.STM_SERIAL_PORT, config.STM_BAUD, state,sock_desktop,config.SEND_PORT,config.SEND_IP)
    lidar_mod = LidarModule(state)
    core_mod = ControlLoop(state, motors)
    safety_mod = SafetyModule(state)
    
    await asyncio.gather(
        ctrl_mod.run(),
        stereo_mod.run(),
        stm_mod.run(),
        lidar_mod.run(),
        core_mod.run(),
        safety_mod.run()
    )

if __name__ == "__main__":
    try:
        asyncio.run(main())
    except KeyboardInterrupt:
        print("\nExiting.")