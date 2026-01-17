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
    # Make sure we don't reset our explored tags if we leave this mode
    def __init__(self):
        self.last_seen_tag = None
    def stop(self, ctrl, dt):
        ctrl.state.camera_current = 0
        ctrl.motors.brake_all_motors(message_toggle=False)

    def on_enter(self,ctrl):
        ctrl.state.camera_current = 2
        print(ctrl.state.camera_modes[ctrl.state.camera_current])
    
    def docking(self,ctrl,dt):
        # Where are we right now
        if ctrl.state.tag_discovered is not None:
            home = ctrl.state.tag_discovered[0]
        
        # Move towards home such that robot position should == home[x,y]
        # Calculate vector from (robot[x,y],home[x,y])
        # Movement function should be if robot[x] != home[x], mv wheel_[x_vec,y_vec]
        pass

    def Lost(self):
        # If robot at supposed tag_id[x,y] and tag_id not found, spin in circle to locate tag
        # If spin no success, go back to previous point and idle.
        pass
    
    def hunt(self, ctrl, dt):
        # tag hunt logic here
        now = time.monotonic()
        seq = ctrl.state.tag_sequence
        #print(seq)
        if seq is None:
            self.last_seen_tag = None
            return
        tag_id = seq["id"]
        x = seq["x"]
        angle = seq["angle"]
        # Let's just add our discoveries to the explore sequence -> we'll make it mutable via websocket later
        if tag_id != self.last_seen_tag:
            if tag_id not in ctrl.state.tag_discovered:
                ctrl.state.tag_discovered[tag_id] = [x, angle]
                ctrl.state.tag_explore_sequence.append[tag_id]
                print(f"[TAG] Discovered new tag: {tag_id}")
                print(ctrl.state.tag_discovered)
            self.last_seen_tag = tag_id

        if ctrl.state.tag_behavior_toggle == 1:
            submode = ctrl.state.tag_behavior_modes[ctrl.state.tag_behavior_current]
            # Once autonomy is toggled -> we 
            if submode == "Rest":
                ctrl.motors.brake_all_motors(message_toggle=False)
                return
            if submode == "Hunting":
                if lost:
                    ctrl.state.tag_behavior_current = ctrl.state.tag_behavior_modes.index("Lost")
                    return

            # If visible and aligned/close enough -> go docking or centering
            # Example: if abs(angle) < 0.1 -> Centering; if distance < 0.3 -> Docking
            # (you’ll define those thresholds)
            return

            if submode == "Lost":
                # do a search pattern (spin/scan)
                # if tag reappears -> Hunting
                if not lost:
                    ctrl.state.tag_behavior_current = ctrl.state.tag_behavior_modes.index("Hunting")
                return

            if submode == "Centering":
                # center on tag angle
                return

            if submode == "Docking":
                # drive forward until close
                return

        # If tag_behavior_current == #: -> that will mess up with our hunting sequence if detects a new tag
