class RobotState:
    def __init__(self):
        # STM32 Sensor Data
        self.stm = {
            "t_ms": None, "w1_ticks": 0, "w2_ticks": 0, "w3_ticks": 0, "w4_ticks": 0,
            "w1_w": 0.0, "w1_v": 0.0, "w2_w": 0.0, "w2_v": 0.0,
            "w3_w": 0.0, "w3_v": 0.0, "w4_w": 0.0, "w4_v": 0.0,
            "yaw": 0.0, "pitch": 0.0, "roll": 0.0,
            "gx": 0.0, "gy": 0.0, "gz": 0.0,
            "ax":0.0, "ay": 0.0, "az": 0.0,
            "lax": 0.0, "lay": 0.0, "laz": 0.0,
            "mx": 0.0, "my": 0.0, "mz": 0.0,
            "qx": 0.0, "qy": 0.0, "qz": 0.0, "qw": 0.0
        }
        # Controller Inputs
        self.axes = {"LX": 0.0, "LY": 0.0, "RX": 0.0, "RY": 0.0}
        self.triggers = {"LT": 0.0, "RT": 0.0}
        self.buttons = {
            "A": 0, "B": 0, "X": 0, "Y": 0,
            "DPAD_UP": 0, "DPAD_DOWN": 0,
            "DPAD_LEFT": 0, "DPAD_RIGHT": 0,
            "LB": 0, "RB" :0,
        }
        self.last_joy_time = 0.0
        
        # High Level State
        self.robot_modes = ["Rest","Manual","Follower","Tag"]
        self.robot_current = 0
        self.toggle = 0

        self.camera_modes = ["Rest","Yolo","AprilTag"] # "hunting" or "tagging"
        self.camera_current = 0
        self.hunted = {"angle": 0.0, "distance": 0.0}
        
        # April Tag Behavior List
        self.tag_behavior_modes = ["Rest","Docking","Lost","Hunting","Centering"]
        self.tag_behavior_current = 0
        self.tag_behavior_toggle = 0
        self.tag_sequence = []
        self.tag_discovered = {}
        self.tag_explore_sequence = []
        self.tag_current = None


        # Lidar Zone
        self.zone = {"zone": "Far", "x_sum": 0.0, "y_sum": 0.0}
        # Lidar Close Data (Points <= close distance)
        self.lidar_close = {"angles": [], "ranges": []}
        # Lidar Full Scan
        self.lidar_full_scan = {}
        # Lidar Payload for UDP
        self.lidar_payload = {}
        # STM Payload for UDP
        self.stm_odom_payload = {
            "pose_x": 0.0,
            "pose_y": 0.0,
            "pose_z": 0.0,
            "pose_orientation_w":0.0,
            "pose_orientation_x":0.0,
            "pose_orientation_y":0.0,
            "pose_orientation_z":0.0,
            "twist_x":0.0,
            "twist_y":0.0,
            "twist_z":0.0,
            "angular_x":0.0,
            "angular_y":0.0,
            "angular_z":0.0,
        }      
        self.stm_imu_payload = {
            "quaternion_x":0.0,
            "quaternion_y":0.0,
            "quaternion_z":0.0,
            "quaternion_w":0.0,
            "angular_velocity_x":0.0,
            "angular_velocity_y":0.0,
            "angular_velocity_z":0.0,
            "linear_acceleration_x":0.0,
            "linear_acceleration_y":0.0,
            "linear_acceleration_z":0.0,
            "magnetic_x":0.0,
            "magnetic_y":0.0,
            "magnetic_z":0.0,
        }
        # Safe Motor Commands (Calculated by SafetyModule)
        self.safe_axes = {"LX": 0.0, "LY": 0.0,"W":0.0}
        # Translational Coordinates
        self.tf_pose = None
        self.tf_map_pose = {"x": 0.0, "y": 0.0, "yaw": 0.0}
        self.tf_last_time = 0.0
        self.command_vector = {"LX":00, "LY":0.0, "W":0.0}
