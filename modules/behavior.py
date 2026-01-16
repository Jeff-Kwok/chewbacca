'''
# The way behaviors should be defined is that all behaviors should have a stop and hunt method
 -> rest: Idling, no commands
 -> Hunting: Actively driving the robot based on some input
 The reason it's seperated 3 times into each category is 
 that the idle behaviors will look different for each mode
 When toggling between modes we always default to a state of toggle = 0 therefore 
 The cameras should be set to rest
'''

# behavior.py
import numpy as np
class BehaviorBase:
    """Common interface."""
    name = "Base"
    def stop(self, ctrl, dt):   # idle/rest
        raise NotImplementedError
    def hunt(self, ctrl, dt):   # active driving
        raise NotImplementedError


class Rest(BehaviorBase):
    name = "Rest"
    def stop(self, ctrl, dt):
        # idle means: no commands / brake
        ctrl.state.camera_current = 0
        ctrl.motors.brake_all_motors(message_toggle=False)
        #print()

    def hunt(self, ctrl, dt):
        # Rest mode doesn't hunt; just stop
        self.stop(ctrl, dt)


class Manual(BehaviorBase):
    name = "Manual"
    def stop(self, ctrl, dt):
        ctrl.state.camera_current = 0
        ctrl.motors.brake_all_motors(message_toggle=False)
    def hunt(self, ctrl, dt):
        # drive based on joystick
        x  = ctrl.state.axes["LX"] * -1
        y  = ctrl.state.axes["LY"]
        rx = ctrl.state.axes["RX"] * -1
        ry = ctrl.state.axes["RY"]
        yaw = (np.deg2rad(ctrl.state.stm["yaw"])-np.pi/2) % (2*np.pi)
        if ctrl.state.triggers["LT"] == 1:
            angle = (np.arctan2(ry,rx)-np.pi/2) % (2*np.pi)
        else:
            angle = 0
        w,val = ctrl.motors.calc_robot_yaw(angle,yaw)
        if abs(w) <= 0.1:
            w = 0 
            #print(f"stick{angle:.2f}")
        w_fl, w_fr, w_rl, w_rr = ctrl.motors.calc_norm_vector(x, y, w)
        print(f"yaw: {yaw:.2f} angle: {angle:.2f}:.2f val: {val:.2f}:.2f w: {w:.2f}\n")
        ctrl.motors.drive_all_wheels({
                "nfr": w_fr, "nfl": w_fl, "nrr": w_rr, "nrl": w_rl,
            })


class Follower(BehaviorBase):
    name = "Follower"
    def stop(self, ctrl, dt):
        ctrl.motors.brake_all_motors(message_toggle=False)
        ctrl.state.camera_current = 0
        #print(ctrl.state.camera_modes[ctrl.state.camera_current])

    def on_enter(self,ctrl):
        ctrl.state.camera_current = 1
        print(ctrl.state.camera_modes[ctrl.state.camera_current])

    def hunt(self, ctrl, dt):
        # follow logic here
        angle = ctrl.state.hunted["angle"] %(2*np.pi)
        distance = 0.9
        yaw = (np.deg2rad(ctrl.state.stm["yaw"])-np.pi/2) % (2*np.pi)
        if distance <= 0.2:
            ctrl.motors.brake_all_motors(message_toggle=False)  # optional
            return
        w,val = ctrl.motors.calc_robot_yaw(0,angle)
        w = w * 2.0
        w = np.sign(w)*(abs(w)**1.2)
        w = np.clip(w/0.8, -1.0,1.0)
        # This math is lowkey stupid asf need to revamp this.
        x = np.clip(np.sin(angle)*distance*-0.5,-1.0,1.0)
        if abs(x) <= 0.2:
            x = 0
        y = np.clip(np.cos(angle)*distance*0.5,-1.0,1.0)
        if abs(y) <= 0.2:
            y = 0
        if abs(w) <= 0.040:
            w = 0 
        w_fl, w_fr, w_rl, w_rr = ctrl.motors.calc_norm_vector(x, y, w)
        print(f"yaw: {yaw:.2f} angle: {angle:.2f}:.2f val: {val:.2f}:.2f w: {w:.2f}\n")
        ctrl.motors.drive_all_wheels({
                "nfr": w_fr, "nfl": w_fl, "nrr": w_rr, "nrl": w_rl,
            })

class Tag(BehaviorBase):
    name = "Tag"
    def stop(self, ctrl, dt):
        ctrl.state.camera_current = 0
        ctrl.motors.brake_all_motors(message_toggle=False)

    def on_enter(self,ctrl):
        ctrl.state.camera_current = 2
        print(ctrl.state.camera_modes[ctrl.state.camera_current])

    def hunt(self, ctrl, dt):
        # tag hunt logic here
        
        pass
