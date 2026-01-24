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
import time
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
        #print(ctrl.state.tf_map_pose)
        #print(ctrl.state.tf_pose)


class Manual(BehaviorBase):
    name = "Manual"
    def stop(self, ctrl, dt):
        ctrl.state.camera_current = 0
        ctrl.motors.brake_all_motors(message_toggle=False)
    def hunt(self, ctrl, dt):
        # drive based on joystick
        x  = ctrl.state.safe_axes["LX"]  * -1
        y  = ctrl.state.safe_axes["LY"]
        rx = ctrl.state.axes["RX"] * -1 # negative when sending to calculate with robot yaw function
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
        w_fl, w_fr, w_rl, w_rr = ctrl.motors.calc_norm_vector(x, y, rx*-1)
        #print(f"yaw: {yaw:.2f} angle: {angle:.2f}:.2f val: {val:.2f}:.2f w: {w:.2f}\n")
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
        ctrl.state.tag_behavior_toggle = 0
        ctrl.state.tag_behavior_current = 0
        print(ctrl.state.camera_modes[ctrl.state.camera_current])

    def hunt(self, ctrl, dt):
        # follow logic here
        angle = ctrl.state.hunted["angle"] %(2*np.pi)
        distance = ctrl.state.hunted["distance"]
        yaw = (np.deg2rad(ctrl.state.stm["yaw"])-np.pi/2) % (2*np.pi)
        if distance <= 1.5:
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
        ctrl.state.command_vector["x"] = x * .45
        ctrl.state.command_vector["x"] = y * .45
        w_fl, w_fr, w_rl, w_rr = ctrl.motors.calc_norm_vector(ctrl.state.safe_axes["LX"],ctrl.state.safe_axes["LY"], w*.2)
        print(f"yaw: {yaw:.2f} angle: {angle:.2f}:.2f val: {val:.2f}:.2f w: {w:.2f}\n")
        ctrl.motors.drive_all_wheels({
                "nfr": w_fr, "nfl": w_fl, "nrr": w_rr, "nrl": w_rl,
            })

class Tag(BehaviorBase):
    name = "Tag"
    # Make sure we don't reset our explored tags if we leave this mode
    def __init__(self):
        self.last_seen_tag = None
        self.last_autonomy_toggle = 0
        self.performance_point = 0
        self.sequence_check_condition = False
        self.docking_check_condition = False
        self.point_goal = {
            "x": 0.0,
            "y": 0.0,
            "yaw": 0.0
        }
        self.explore_condition = []
        self.explore_index = 0

    
    def stop(self, ctrl, dt):
        ctrl.state.camera_current = 0
        ctrl.motors.brake_all_motors(message_toggle=False)

    def on_enter(self,ctrl):
        ctrl.state.camera_current = 2
        print(ctrl.state.camera_modes[ctrl.state.camera_current])
    ## On leave
    def enter_autonomy(self, ctrl):
        print("[AUTO] Entering tag autonomy")
        ctrl.motors.brake_all_motors(message_toggle=False)
        self.performance_point = 0
        ctrl.state.tag_behavior_current = ctrl.state.tag_behavior_modes.index("Rest")

    def checking_position_move(self,ctrl,num,move):
        # We can have this condition be one-flipped if it doesn't need to be iterated over
        try:
            sequence = ctrl.state.tag_explore_sequence
            tag_id = sequence[num]
            print(
                f"Our goal tag is: {tag_id}"
            )
            self.point_goal["x"] = ctrl.state.tag_discovered[tag_id]["seen_from"]["x"]
            self.point_goal["y"] = ctrl.state.tag_discovered[tag_id]["seen_from"]["y"]
            self.point_goal["yaw"] = ctrl.state.tag_discovered[tag_id]["seen_from"]["yaw"]
        except Exception as e:
            print(f"Error: Non-retrieval of tag_id -> {e}")
            self.performance_point = 0
            return
        if tag_id is not None:
            try: 
                x, y, yaw = ctrl.motors.calc_robot_desired_pos(ctrl.state.tf_map_pose["x"],ctrl.state.tf_map_pose["y"],self.point_goal["x"],self.point_goal["y"],ctrl.state.tf_map_pose["yaw"],self.point_goal["yaw"])
                if x == 0.0 and y == 0.0 and yaw == 0.0:
                    print(
                        "Arrived at:"
                        f"{self.point_goal['x']}"
                        f"{self.point_goal['y']}"
                        f"{self.point_goal['yaw']}"
                        )
                    self.performance_point = 1
                    return
                if move == True:
                    try:
                        ctrl.state.command_vector["LX"] = y
                        ctrl.state.command_vector["LY"] = x
                        print("Moving")
                        w_fl, w_fr, w_rl, w_rr = ctrl.motors.calc_norm_vector(ctrl.state.safe_axes["LX"], ctrl.state.safe_axes["LY"], yaw*-.25)
                        ctrl.motors.drive_all_wheels({
                                "nfr": w_fr, "nfl": w_fl, "nrr": w_rr, "nrl": w_rl,
                            })
                    except Exception as e:
                        print(
                            f"Error: {e}"
                        )
                print(x,y,yaw)
            except Exception as e:
                self.performance_point = 0
                print(e)
                return
    def checking_docking_pos(self,ctrl):
        self.checking_position_move(ctrl,0,move=False)
        self.performance_point = 0.5
        if self.performance_point == 0:
            self.docking_check_condition = False
        elif self.performance_point == 1:
            self.docking_check_condition = True

    def mode_change_button(self,ctrl,left_state,right_state):
        print(
            f"press dpad left to go: {ctrl.state.tag_behavior_modes[left_state]}\n"
            f"press dpad right to go: {ctrl.state.tag_behavior_modes[right_state]}"
        )
                            # --- Dpad save button ---
        dpad_RIGHT = ctrl.state.buttons.get("DPAD_RIGHT", None)
        dpad_LEFT = ctrl.state.buttons.get("DPAD_LEFT", None)
        if dpad_RIGHT == 1:
            ctrl.state.tag_behavior_current = right_state
        elif dpad_LEFT == 1:
            ctrl.state.tag_behavior_current = left_state
    # DEFINE MOVEMENT FUNCTION -> PULLS FROM MOTOR
    
    def sequence_check(self,ctrl,sequence):
        if len(sequence) >= 2:
            print(
                f"Adequate number of tags held: {ctrl.state.tag_explore_sequence}",
                "Starting autonomy sequence:",
                #f"Docking at: {ctrl.state.tag_explore_sequence[0]}"
            )
            self.performance_point = 1
        elif len(sequence) < 2:
            print(f"Inadequate number of tags held: {ctrl.state.tag_explore_sequence}, need at least 3")
            self.performance_point = 0
            return

    # Completely stops all behavior
    def behavior_roll(self,ctrl):
        # Sends behavior back to rest but perhaps we should do "last behavior"?
        ctrl.motors.brake_all_motors(message_toggle=False)
        ctrl.state.tag_behavior_current = 0
        ctrl.state.tag_behavior_toggle = 0
        self.last_autonomy_toggle = 0
        self.point_goal = {"x":0.0,"y":0.0,"yaw":0.0}
        self.performance_point = 0
        print(
        "Going back to manual tagging mode",
        f"Behavior Mode set to: {ctrl.state.tag_behavior_modes[ctrl.state.tag_behavior_current]}",
        f"Autonomy Toggle: {ctrl.state.tag_behavior_toggle}"
        )

    def hunt(self, ctrl, dt):
            # tag hunt logic here
        now = time.monotonic()
        seq = ctrl.state.tag_sequence

        # --- TF pose (dict) ---
        loq_x = float(ctrl.state.tf_map_pose.get("x", 0.0))
        loq_y = float(ctrl.state.tf_map_pose.get("y", 0.0))
        loq_yaw = float(ctrl.state.tf_map_pose.get("yaw", 0.0))

        # --- Dpad save button ---
        dpad = ctrl.state.buttons.get("DPAD_UP", None)

        # Pick which direction is "save"
        SAVE_DPAD = 1

        # --- edge detect: only trigger once per press ---
        if not hasattr(self, "_prev_dpad"):
            self._prev_dpad = None

        save_pressed = (dpad == SAVE_DPAD) and (self._prev_dpad != SAVE_DPAD)
        self._prev_dpad = dpad

        # No tag seen -> nothing to save
        if seq is None:
            self.last_seen_tag = None
        else:
            # Tag data
            tag_id = seq.get("id")
            tag_x = seq.get("x")
            tag_angle = seq.get("angle")
            if tag_id is None:
                return
            # Track last seen tag (optional)
            if tag_id != getattr(self, "last_seen_tag", None):
                self.last_seen_tag = tag_id

            # --- Only save when button is pressed ---
            if save_pressed:
                # Store "seen-from" location (map frame) + tag observation info
                # If you press again on same tag, this overwrites (updates).
                ctrl.state.tag_discovered[tag_id] = {
                    "seen_from": {"x": loq_x, "y": loq_y, "yaw": loq_yaw},
                    "tag_obs": {"x": tag_x, "angle": tag_angle},
                    "t": now,
                }

                # Add to explore sequence only once (don’t duplicate)
                if tag_id not in ctrl.state.tag_explore_sequence:
                    ctrl.state.tag_explore_sequence.append(tag_id)

                print(f"[TAG] Saved/Updated tag {tag_id} @ map(x={loq_x:.2f}, y={loq_y:.2f}, yaw={loq_yaw:.2f})")
                # Optional: print full dict
                # The definition for phrase
                print(ctrl.state.tag_discovered[tag_id])
                # The whole dictionary
                print(ctrl.state.tag_discovered)
                # The list of phrases
                print(ctrl.state.tag_explore_sequence)



        if self.last_autonomy_toggle == 1 and ctrl.state.tag_behavior_toggle == 0:
            ctrl.state.tag_behavior_current = 0
            print("Exiting Autonomy Mode")
            self.last_autonomy_toggle = 0
        if ctrl.state.tag_behavior_toggle == 1:
            if self.last_autonomy_toggle == 0:
                print("Entering autonomy")
                self.enter_autonomy(ctrl)
                self.last_autonomy_toggle = 1
            else:   
                print("doing nothing")
            # Once autonomy is toggled -> we need to go into routine behavior
            #ctrl.motors.brake_all_motors(message_toggle=False)
            submode = ctrl.state.tag_behavior_modes[ctrl.state.tag_behavior_current]
            # Resting sequence -> should check sequence if haven't checked.
            if submode == "Rest":
                self.checking_docking_pos(ctrl)
                if self.sequence_check_condition == False:
                    try:
                        self.sequence_check(ctrl,ctrl.state.tag_explore_sequence)
                    except Exception as e:
                        print(f"Error: {e}")
                        self.behavior_roll(ctrl)
                        return

                    if self.performance_point == 0:
                        print("Couldn't complete sequence check")
                        self.behavior_roll(ctrl)
                        return

                    if self.performance_point == 1:
                        print("Completed sequence check, moving onto docking")
                        self.sequence_check_condition = True
                        ctrl.state.tag_behavior_current = 1

                if self.sequence_check_condition == True:
                    self.mode_change_button(ctrl,1,3)          


            if submode == "Docking":
                print("Initiating docking")
                self.checking_docking_pos(ctrl)
                if self.docking_check_condition == False:
                    try:
                        self.performance_point = 0.5
                        self.checking_position_move(ctrl,0,move=True)
                    except Exception as e:
                        print(f"Error when docking: {e}")
                        self.behavior_roll(ctrl)
                        return
                    if self.performance_point == 0:
                        # If we are IN docking -> and we can't succeed, go to lost
                        print("Couldn't complete docking")
                        ctrl.state.tag_behavior_current = 2
                        return
                        # If we are IN docking -> and we succeed, set condition to true go back to rest
                    if self.performance_point == 1:
                        print("Robot is docked")
                        self.docking_check_condition = True
        
                elif self.docking_check_condition == True:
                    self.mode_change_button(ctrl,0,3)                   


            # In submode hunting -> As long as we're docked and our sequence has been checked -> we're going to start exploring the sequence
            if submode == "Hunting":
                print(
                    "Initiating Hunting"
                )
                self.checking_docking_pos(ctrl)
                if self.sequence_check_condition == True:
                    # For every tag in sequence we're going to try to go to them, if it any point we encounter an issue, we know we're no longer docked -> and we go to lost
                    if len(self.explore_condition) != len(ctrl.state.tag_explore_sequence):
                        try:
                            self.performance_point = 0.5
                            self.checking_position_move(ctrl,self.explore_index,move=True)
                            if self.performance_point == 0:
                                ctrl.state.tag_behavior_current = 2
                                return
                            if self.performance_point == 1:
                                self.explore_condition.append(ctrl.state.tag_explore_sequence[self.explore_index])
                                self.explore_index += 1
                        except Exception as e:
                            print(f"Error: {e}")
                    else: 
                        print("press dpad right to go docking hunt")
                        print("press dpad left to go sleep")
                                            # --- Dpad save button ---
                        dpad_RIGHT = ctrl.state.buttons.get("DPAD_RIGHT", None)
                        dpad_LEFT = ctrl.state.buttons.get("DPAD_LEFT", None)
                        if dpad_RIGHT == 1:
                            self.explore_index = 0
                            print(self.explore_condition)
                            self.explore_condition = []
                            ctrl.state.tag_behavior_current = 1

                        elif dpad_LEFT == 1:
                            ctrl.state.tag_behavior_current = 0  

            if submode == "Lost":
                # Set both sequence and docking checkst o false and return to rest -> begin docking loop.
                self.sequence_check_condition = False
                self.docking_check_condition = False
                ctrl.state.tag_behavior_current = 0
                print("I'm lost!")

