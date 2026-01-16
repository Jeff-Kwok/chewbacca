from adafruit_pca9685 import PCA9685
import busio
import board
import numpy as np
#Ⱍ
class MotorFunctions:
    def __init__(self):
        self.i2c = busio.I2C(board.SCL, board.SDA)
        self.address = 0x40
        self.freq = 50
        self.pca = PCA9685(self.i2c, address=self.address)
        self.pca.frequency = self.freq

        self.mecanum_wheels = {
            "nrr": 0.0,
            "nrl": 0.0,
            "nfr": 0.0,
            "nfl": 0.0,
        }

        # [dirA, dirB, pwm]
        self.mecanum_direction_pins = {
            "nrr": [0, 0, 0],
            "nrl": [0, 0, 0],
            "nfr": [0, 0, 0],
            "nfl": [0, 0, 0],
        }

        self.mecanum_configuration = {
            "wheel_diameter":       0.079,
            "roller_radius":        0.0075,
            "wheel_base_length":    0.2,
            "wheel_base_width":     0.18,
            "wheel_positions":      0.4,      # L term
            "velocity_x_max":       1.347,
            "velocity_y_max":       1.347,
            "angular_velocity_max": 7.483,
            "max_rad/s":            34.56,
            "real_max_rad/s":       34.56,
        }

    def set_direction_pins(self, wheel, pinA, pinB, pwm):
        if wheel in self.mecanum_direction_pins:
            self.mecanum_direction_pins[wheel] = [pinA, pinB, pwm]
        else:
            raise ValueError("Invalid wheel name")

    def set_all_direction_pins(self, direction_pins):
        for wheel, pins in direction_pins.items():
            self.set_direction_pins(wheel, pins[0], pins[1], pins[2])
        print("[MotorFunctions] Direction pins:", self.mecanum_direction_pins)
 
    def drive_wheel(self, wheel, speed_rad):
        """
        speed_rad: PID command in rad/s, in [-34.56, 34.56]
        """
        if wheel not in self.mecanum_wheels:
            raise ValueError("Invalid wheel name")
        
        # Clamp to physical limits
        max_w = self.mecanum_configuration["max_rad/s"]
        real_max_w = self.mecanum_configuration["real_max_rad/s"]
        if speed_rad > max_w:
            speed_rad = max_w
        elif speed_rad < -max_w:
            speed_rad = -max_w
        
        self.mecanum_wheels[wheel] = speed_rad
        dirA, dirB, pwm = self.mecanum_direction_pins[wheel]
        
        # Deadband only very close to zero (really "stop")
        if abs(speed_rad) < 0.05:   # tiny value
            # Full stop
            self.pca.channels[dirA].duty_cycle = 0
            self.pca.channels[dirB].duty_cycle = 0
            self.pca.channels[pwm].duty_cycle = 0
            #print(f"Driving {wheel} at speed {speed_rad:.2f} rad/s (stopped)")
            return
        
        # Direction
        if speed_rad > 0:
            self.pca.channels[dirA].duty_cycle = 0xFFFF
            self.pca.channels[dirB].duty_cycle = 0
        else:
            self.pca.channels[dirA].duty_cycle = 0
            self.pca.channels[dirB].duty_cycle = 0xFFFF
        
        # --------- NONLINEAR PWM MAPPING (INLINE) ---------
        # 1) normalize to 0..1
        mag = abs(speed_rad) / real_max_w
        if mag > 1.0:
            mag = 1.0

        # 2) bend the curve with a power (gamma)
        #    gamma > 1.0 = softer at low speeds, stronger near max
        gamma = 3.5          # tweak this (1.5, 2.0, 3.0, etc.)
        mag_shaped = mag ** gamma

        # 3) map to duty range
        min_duty = 4050      # minimum to overcome friction
        max_duty = 65535
        duty = int(min_duty + mag_shaped * (max_duty - min_duty))
        # ----------------------------------------------

        self.pca.channels[pwm].duty_cycle = duty
        
        #print(
            #f"Driving {wheel} at speed {speed_rad:.2f} rad/s "
            #f"(mag={mag:.2f}, shaped={mag_shaped:.2f}, duty={duty})"
        #)

    def drive_all_wheels(self, speeds):
        for wheel, speed in speeds.items():
            self.drive_wheel(wheel, speed)
    # Basic hard coded directional steering based on speed input command
    def drive_all_left(self,speeds):
        self.drive_all_wheels({
            "nrr":  -speeds,
            "nrl":  speeds,
            "nfr":  -speeds,
            "nfl":  speeds,
        })
    def drive_all_right(self,speeds):
        self.drive_all_wheels({
            "nrr": speeds,
            "nrl": -speeds,
            "nfr": speeds,
            "nfl": -speeds,
        })
    
    def drive_all_forward(self,speeds):
        self.drive_all_wheels({
            "nrr": speeds,
            "nrl": speeds,
            "nfr": speeds,
            "nfl": speeds,
        })
    
    def drive_all_backwards(self,speeds):
        self.drive_all_wheels({
            "nrr": -speeds,
            "nrl": -speeds,
            "nfr": -speeds,
            "nfl": -speeds, 
        })
    def drive_robo_left(self,speeds):
        self.drive_all_wheels({
            "nrr": speeds,
            "nrl": -speeds,
            "nfr": speeds,
            "nfl": -speeds,
        })
    def drive_robo_right(self,speeds):
        self.drive_all_wheels({
            "nrr": -speeds,
            "nrl": speeds,
            "nfr" : -speeds,
            "nfl": speeds, 
    })
        
    # Basic hard coded directional steering based on speed input command
    def brake_all_motors(self, message_toggle=False):
        for wheel in self.mecanum_wheels:
            self.mecanum_wheels[wheel] = 0.0
        for ch in range(16):
            self.pca.channels[ch].duty_cycle = 0
        if message_toggle:
            print("All motors braked")

    def calc_norm_vector(self, x, y, w):
        """
        Given joystick inputs x,y,w in [-1,1], compute desired wheel
        angular velocities [rad/s], clamped to ±max_rad/s.
        """
        vx = x * self.mecanum_configuration["velocity_x_max"]
        vy = y * self.mecanum_configuration["velocity_y_max"]
        omega_max = self.mecanum_configuration["max_rad/s"]
        vw = w * self.mecanum_configuration["angular_velocity_max"]
        L = self.mecanum_configuration["wheel_positions"]
        r = self.mecanum_configuration["wheel_diameter"] / 2.0

        # Standard mecanum inverse kinematics → wheel angular velocities
        # We're calculating based on magnitude vector being applied to maximum velocities
        FL = (vy + vx + vw * L) / r
        FR = (vy - vx - vw * L) / r
        RL = (vy - vx + vw * L) / r
        RR = (vy + vx - vw * L) / r

        # Clamp each to ±omega_max
        FL = max(min(FL, omega_max), -omega_max)
        FR = max(min(FR, omega_max), -omega_max)
        RL = max(min(RL, omega_max), -omega_max)
        RR = max(min(RR, omega_max), -omega_max)

        return FL, FR, RL, RR

    def calc_robot_velocity(self, w1, w2, w3, w4):
        """
        Approximate robot-frame velocities from wheel angular velocities.
        NOTE: formulas may need tweaking; dimensions not fully checked.
        """
        r = self.mecanum_configuration["wheel_diameter"] / 2.0

        longitudinal_velocity = (w1 + w2 + w3 + w4) / (r * 4.0)
        lateral_velocity      = (w1 + w2 - w3 - w4) / (r * 4.0)
        angular_velocity      = (w1 - w2 + w3 - w4) / (r * 4.0)

        roe = np.atan2(lateral_velocity, longitudinal_velocity)
        velocity_vector = math.sqrt(
            longitudinal_velocity**2 + lateral_velocity**2
        )

        return longitudinal_velocity, lateral_velocity, angular_velocity, roe, velocity_vector
    # Plugging in radians from 0 2*pi
    def calc_robot_yaw(self,desired,reading):
        w_rate = 0
        value = ((desired - reading) + 2*np.pi) % (2*np.pi)
        if abs(value) >= np.deg2rad(6.5):
            if value < np.pi:
                w_rate = (value/np.pi)*.5
            elif value > np.pi:
                # Subtracting because we're looking from 0 to 3.14 -> already established over threshold so we jus tneed magnitude
                w_rate = ((value-2*np.pi)/np.pi)*.5
        else:
            w_rate = 0
        return w_rate,value       
    def read_device_encoder(self, device):
        # Placeholder for reading device encoder values
        pass


class PID:
    def __init__(self):
        """
        Simple PID controller with configurable gains and limits.
        dt_default should match your update period (e.g. 0.032 s).
        All units in rad/s.
        """
        self.parameters = {
            "kp": 1.1,
            "ki": 0.3,
            "kd": 0.0,
            "output_limits_min": -34.56,
            "output_limits_max":  34.56,
            "integral_limits_min": -float("inf"),
            "integral_limits_max":  float("inf"),
            "dt": 0.032,
        }

        self.kp = self.parameters["kp"]
        self.ki = self.parameters["ki"]
        self.kd = self.parameters["kd"]

        self.dt_default = self.parameters["dt"]

        self.min_output = self.parameters["output_limits_min"]
        self.max_output = self.parameters["output_limits_max"]

        self.min_integral = self.parameters["integral_limits_min"]
        self.max_integral = self.parameters["integral_limits_max"]

        self.integral = 0.0
        self.prev_error = 0.0
        self.first_update = True

    def change_parameters(self, name, val):
        if name not in self.parameters:
            raise ValueError(f"Invalid parameter name: {name}")

        self.parameters[name] = val

        if name == "kp":
            self.kp = val
        elif name == "ki":
            self.ki = val
        elif name == "kd":
            self.kd = val
        elif name == "dt":
            self.dt_default = val
        elif name == "output_limits_min":
            self.min_output = val
        elif name == "output_limits_max":
            self.max_output = val
        elif name == "integral_limits_min":
            self.min_integral = val
        elif name == "integral_limits_max":
            self.max_integral = val

    def reset(self):
        self.integral = 0.0
        self.prev_error = 0.0
        self.first_update = True

    def update(self, setpoint, measurement, dt=None):
        """
        setpoint: desired value (rad/s)
        measurement: current value (rad/s)
        dt: timestep in seconds. If None, uses dt_default.
        """
        if dt is None:
            dt = self.dt_default
        if dt <= 0.0:
            dt = 1e-6

        error = setpoint - measurement

        # P
        P = self.kp * error

        # I with clamping
        self.integral += error * dt
        if self.integral > self.max_integral:
            self.integral = self.max_integral
        elif self.integral < self.min_integral:
            self.integral = self.min_integral
        I = self.ki * self.integral

        # D
        if self.first_update:
            derivative = 0.0
            self.first_update = False
        else:
            derivative = (error - self.prev_error) / dt
        D = self.kd * derivative

        self.prev_error = error

        u = P + I + D

        # clamp
        if u > self.max_output:
            u = self.max_output
        elif u < self.min_output:
            u = self.min_output

        return u
