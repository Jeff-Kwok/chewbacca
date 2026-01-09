class RobotState:
    def __init__(self):
        # STM32 Sensor Data
        self.stm = {
            "t_ms": None, "w1_ticks": 0, "w2_ticks": 0,
            "w1_w": 0.0, "w1_v": 0.0, "w2_w": 0.0, "w2_v": 0.0,
            "w3_w": 0.0, "w3_v": 0.0, "w4_w": 0.0, "w4_v": 0.0,
            "yaw": 0.0, "pitch": 0.0, "roll": 0.0,
            "gx": 0.0, "gy": 0.0, "gz": 0.0,
        }
        # Controller Inputs
        self.axes = {"LX": 0.0, "LY": 0.0, "RX": 0.0, "RY": 0.0}
        self.triggers = {"LT": 0.0, "RT": 0.0}
        self.buttons = {
            "A": 0, "B": 0, "X": 0, "Y": 0,
            "DPAD_UP": 0, "DPAD_DOWN": 0,
            "DPAD_LEFT": 0, "DPAD_RIGHT": 0,
        }
        self.last_joy_time = 0.0
        
        # High Level State
        self.mode = "controller" # "controller" or "camera"
        self.select_state = 1
        self.tracker = 0
        self.hunter = 0
        
        # Lidar Zone
        self.zone = {"zone": "Far", "x_sum": 0.0, "y_sum": 0.0}
        # Lidar Close Data (Points <= close distance)
        self.lidar_close = {"angles": [], "ranges": []}
        # Safe Motor Commands (Calculated by SafetyModule)
        self.safe_axes = {"LX": 0.0, "LY": 0.0,"W":0.0}