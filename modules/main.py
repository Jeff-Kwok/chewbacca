import asyncio
import socket
import sys
import os
import cv2

# Ensure we can import from parent directory
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))

from motorfunctions import MotorFunctions
from .state import RobotState
from . import config
from .controller import ControllerModule
from .camera_stereo import CameraStereo
from .stm32 import STMModule
from .lidar import LidarModule
from .core_control import ControlLoop
from .safety import SafetyModule

async def camera_loop(state: RobotState):
    """
    Main loop for camera processing. Runs in a separate thread to avoid blocking asyncio.
    """
    print("[CAM] Starting camera loop...")
    cam = CameraStereo(RobotState())
    
    loop = asyncio.get_running_loop()

    while True:
        # Get current camera mode from state
        mode = state.camera_modes[state.camera_current]

        # Run processing in a thread
        vis_frame, payload = await loop.run_in_executor(
            None,  # Use default executor (a thread pool)
            cam.process_frame,
            mode
        )

        if payload:
            if mode == "Yolo":
                z = payload.get("z")
                angle = payload.get("angle")
                if z is not None and angle is not None:
                    state.hunted["distance"] = z
                    state.hunted["angle"] = angle
                else:
                    state.hunted["distance"] = 0.0
                    state.hunted["angle"] = 0.0
            elif mode == "AprilTag":
                tag_id = payload.get("id")
                x = payload.get("z")
                angle = payload.get("angle")
                if tag_id is not None:
                    state.tag_sequence = {
                        "id": tag_id,
                        "x": x,
                        "angle": angle,
                    }
                else:
                    state.tag_sequence = None
        else:
            state.hunted["distance"] = 0.0
            state.hunted["angle"] = 0.0
            state.tag_sequence = None

        if vis_frame is not None:
            cv2.imshow("Camera", vis_frame)
        
        if cv2.waitKey(1) & 0xFF in [ord('q'), 27]: # ESC
            break
        
        await asyncio.sleep(1 / 100) # Aim for 100 FPS, but processing will be the bottleneck
    
    cam.release_camera()
    cv2.destroyAllWindows()
    print("[CAM] Camera loop stopped.")


async def main():
    motors = MotorFunctions()
    motors.set_all_direction_pins(config.MECANUM_PINS)
    
    state = RobotState()
    
    # Socket for controller
    sock_ctrl = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock_ctrl.bind((config.LISTEN_IP, config.LISTEN_PORT))
    sock_ctrl.setblocking(False)

    # Socket for desktop app
    sock_desktop = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

    # Initialize modules
    ctrl_mod = ControllerModule(sock_ctrl, state)
    stm_mod = STMModule(config.STM_SERIAL_PORT, config.STM_BAUD, state, sock_desktop, config.SEND_PORT, config.SEND_IP)
    lidar_mod = LidarModule(state)
    core_mod = ControlLoop(state, motors)
    safety_mod = SafetyModule(state)
    
    print("Starting all modules...")
    try:
        await asyncio.gather(
            ctrl_mod.run(),
            camera_loop(state),
            core_mod.run(),
            #stm_mod.run(),
            #lidar_mod.run(),
            #safety_mod.run()
        )
    finally:
        # Cleanup
        sock_ctrl.close()
        sock_desktop.close()
        # Any other cleanup needed by modules
        print("All modules stopped.")

if __name__ == "__main__":
    try:
        asyncio.run(main())
    except KeyboardInterrupt:
        print("\nExiting.")
