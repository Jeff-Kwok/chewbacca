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
        w_fl, w_fr, w_rl, w_rr = ctrl.motors.calc_norm_vector(x, y, rx*-.65)
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
        self._prev_submode = None
        self.retry_index = 1

    
    def stop(self, ctrl, dt):
        ctrl.state.camera_current = 0
        ctrl.motors.brake_all_motors(message_toggle=False)
    def on_enter(self,ctrl):
        ctrl.state.camera_current = 2
        print(ctrl.state.camera_modes[ctrl.state.camera_current])
    ## On leave
    def enter_autonomy(self, ctrl):
        print("[AUTO] Switching tag autonomy states | All behaviors are stopped and we will be at rest.")
        ctrl.motors.brake_all_motors(message_toggle=False)
        self.performance_point = 0
        ctrl.state.tag_behavior_current = ctrl.state.tag_behavior_modes.index("Rest")
    def checking_position_move(self,ctrl,num,move,performance_set):
        self.performance_point = performance_set
        try:
            sequence = ctrl.state.tag_explore_sequence
            tag_id = sequence[num]
            print(f"Our goal tag is: {tag_id}")
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
                # If our movement command is non-zero we know that we need to move thus we're not at position[tag_id]
                if x == 0.0 and y == 0.0 and yaw == 0.0:
                    print(
                        "Arrived at:"
                        f"{self.point_goal['x']}"
                        f"{self.point_goal['y']}"
                        f"{self.point_goal['yaw']}"
                        )
                    self.performance_point = 1
                    return
                else:
                    print(f"x: {x:.2f} | y: {y:.2f} | yaw: {yaw:.2f}")
                if move == True:
                    try:
                        # need to fix this for collision avoidance
                        ctrl.state.command_vector["LX"] = y *-1
                        ctrl.state.command_vector["LY"] = x
                        print("Moving")
                        w_fl, w_fr, w_rl, w_rr = ctrl.motors.calc_norm_vector(ctrl.state.safe_axes["LX"], ctrl.state.safe_axes["LY"], yaw*-.25)
                        ctrl.motors.drive_all_wheels({
                                "nfr": w_fr, "nfl": w_fl, "nrr": w_rr, "nrl": w_rl,
                            })
                    except Exception as e:
                        print(f"Error: {e}")
                return x,y,yaw
            except Exception as e:
                self.performance_point = 0
                print(e)
                return
    def checking_docking_pos(self,ctrl):
        # CHECK IF WE ALREADY ARE AT POSITION 0 WITHOUT MOVING
        self.checking_position_move(ctrl,0,move=False,performance_set=0.5)
        if self.performance_point == 0.5:
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
            self._prev_submode = ctrl.state.tag_behavior_current
            ctrl.state.tag_behavior_current = right_state
        elif dpad_LEFT == 1:
            self._prev_submode = ctrl.state.tag_behavior_current
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
        loq_x,loq_y,loq_yaw = float(ctrl.state.tf_map_pose.get("x", 0.0)),float(ctrl.state.tf_map_pose.get("y", 0.0)),float(ctrl.state.tf_map_pose.get("yaw", 0.0))
        # --- Dpad save button ---
        dpad_up, dpad_down = ctrl.state.buttons.get("DPAD_UP", None),ctrl.state.buttons.get("DPAD_DOWN", None)

        # THUMB BAD FOR CONTROLLING ENTRY OF TAG SEQUENCES
        if not hasattr(self, "_prev_dpad_up"):
            self._prev_dpad_up = 0
        if not hasattr(self, "_prev_dpad_down"):
            self._prev_dpad_down = 0
        save_pressed   = (dpad_up   == 1) and (self._prev_dpad_up   != 1)
        delete_pressed = (dpad_down == 1) and (self._prev_dpad_down != 1)
        self._prev_dpad_up   = dpad_up
        self._prev_dpad_down = dpad_down

        # No tag seen -> nothing to save
        if seq is None:
            self.last_seen_tag = None
        else:
            # Tag data
            tag_id,tag_x,tag_angle  = seq.get("id"), seq.get("x"), seq.get("angle")
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
                # The definition for phrase
                print(ctrl.state.tag_discovered[tag_id])
                # The whole dictionary
                print(ctrl.state.tag_discovered)
                # The list of phrases
                print(ctrl.state.tag_explore_sequence)
        
            elif delete_pressed:
                # Remove from discovered dict (if exists)
                if tag_id in ctrl.state.tag_discovered:
                    removed = ctrl.state.tag_discovered.pop(tag_id, None)
                    print(f"[TAG] Removed tag_discovered[{tag_id}] -> {removed is not None}")
                else:
                    print(f"[TAG] Tag {tag_id} not in tag_discovered")

                # Remove from explore sequence (all occurrences, just in case)
                if tag_id in ctrl.state.tag_explore_sequence:
                    ctrl.state.tag_explore_sequence = [t for t in ctrl.state.tag_explore_sequence if t != tag_id]
                    print(f"[TAG] Removed {tag_id} from tag_explore_sequence")
                else:
                    print(f"[TAG] Tag {tag_id} not in tag_explore_sequence")

                # Optional: show current state
                print("[TAG] tag_explore_sequence:", ctrl.state.tag_explore_sequence)
                print("[TAG] tag_discovered keys:", list(ctrl.state.tag_discovered.keys()))

        if self.last_autonomy_toggle == 1 and ctrl.state.tag_behavior_toggle == 0:
            # Setting t
            self.enter_autonomy(ctrl)
            self.last_autonomy_toggle = 0
            print("Exiting Autonomy Mode")
        if ctrl.state.tag_behavior_toggle == 1:
            if self.last_autonomy_toggle == 0:
                self.enter_autonomy(ctrl)
                self.last_autonomy_toggle = 1
                print("Entering autonomy")
            else:   
                #print("Error switching states - you should always get an entering/exit message if not break loop")
                pass
            # Once autonomy is toggled -> we need to go into routine behavior
            submode = ctrl.state.tag_behavior_modes[ctrl.state.tag_behavior_current]

            ###### NO BREAK LOOPS OTHERWISE WE STOP OUR CAMERA LOOP #####

            # RESTING SEQUENCE -> VALIDATE TAG SEQUENCE TO MEET THRESHOLD.
            # ONCE TAG SEQUENCE HAS BEEN VALIDATED -> AUTOMATICALLY GO INTO DOCKING
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
                        self.sequence_check_condition = True
                        self._prev_submode = ctrl.state.tag_behavior_current
                        ctrl.state.tag_behavior_current = 1
                        print("Completed sequence check, moving onto docking")

                #elif self.docking_check_condition == False:
                #    ctrl.state.tag_behavior_current = 1 # If we know sequence check is true and docking is false -> send it into docking | We only idle at docking
                else:
                    self.mode_change_button(ctrl,1,3) # If we know sequence check is true and we aren't docked -> we can send to docking or hunting

            # DOCKING SEQUENCE -> RETURN TO INITIAL TAG
            # 
            if submode == "Docking":
                self.checking_docking_pos(ctrl) # WE CHECK IF WE ARE ALREADY AT DOCKING POSITION
                if self.docking_check_condition == False:
                    try:
                        self.checking_position_move(ctrl,0,move=True,performance_set=0.5)
                    except Exception as e:
                        self.behavior_roll(ctrl)
                        print(f"Error when docking: {e}")
                        return
                    if self.performance_point == 0:
                        # IF we can't succeed in docking go into "LOST"
                        self._prev_submode = ctrl.state.tag_behavior_current
                        ctrl.state.tag_behavior_current = 2
                        print("Couldn't complete docking")
                        return
                        # If we are IN docking -> and we succeed, set condition to true go back to rest
                    if self.performance_point == 1:
                        print("Robot is docked")
                        self.docking_check_condition = True
                        return
                elif self.docking_check_condition == True:
                    self.mode_change_button(ctrl,0,3) # IF WE PICK 0 HERE -> We SHOULD IDLE AT REST


            # In submode hunting -> As long as we're docked and our sequence has been checked -> we're going to start exploring the sequence
            if submode == "Hunting":
                self.checking_docking_pos(ctrl)
                # if self.sequence_check_condition == True:
                # For every tag in sequence we're going to try to go to them, if it any point we encounter an issue, we know we're no longer docked -> and we go to lost
                is_docked = bool(self.docking_check_condition)

                # ----- entry detect: only run once when we ENTER hunting -----
                if self._prev_submode != 3:
                    if is_docked:
                        # entering hunting from dock -> full sweep
                        self.explore_index = 0
                        self.explore_condition = []
                        print("[HUNT] Entered from DOCK: start sweep from 0")
                    else:
                        # entering hunting from elsewhere -> resume last goal
                        self.explore_index = getattr(self, "explore_index", 0)
                        self.explore_condition = getattr(self, "explore_condition", [])
                        print(f"[HUNT] Entered NOT docked: resume at index={self.explore_index}")
                if len(self.explore_condition) != len(ctrl.state.tag_explore_sequence):
                    try:
                        self.checking_position_move(ctrl,self.explore_index,move=True,performance_set=0.5) # Move to current EXPLORE INDEX
                        self._prev_submode = 3
                        if self.performance_point == 0: # If there is some error encountered we automatically go to lost
                            ctrl.state.tag_behavior_current = 2 
                            return
                        if self.performance_point == 1:
                            if ctrl.state.tag_explore_sequence[self.explore_index] not in self.explore_condition:
                                self.explore_condition.append(ctrl.state.tag_explore_sequence[self.explore_index]) # Only increase the EXPLORE INDEX upon hitting a successful venture
                            self.explore_index = min(len(ctrl.state.tag_explore_sequence),(self.explore_index + 1)) # The explore index should always be the last goal set. It is only reset if we docked. Otherwise it will always try to finish the sequence.
                            print(self.explore_condition) # This should be the tags that we've passed by
                            #ctrl.state.tag_behavior_current=4
                            ## Opportunity to make it center the robot to the tag ->
                            return
                    except Exception as e:
                        print(f"Error: {e}")
                        self.behavior_roll(ctrl)
                        return
                else: 
                    self.mode_change_button(ctrl,0,1) # IF WE PICK 0 HERE -> We SHOULD IDLE AT REST. If we've already explored all the tags when entering hunting we can go docking to restart the behavior


                        
            if submode == "Lost":
                # Set both sequence and docking checkst o false and return to rest -> begin docking loop.
                if self._prev_submode == 1:
                    self.sequence_check_condition = False
                    self._prev_submode = ctrl.state.tag_behavior_current
                    ctrl.state.tag_behavior_current = 0
                    return
                if self._prev_submode == 3:
                    if self.retry_index < (len(self.explore_condition)-1) and self.retry_index < self.explore_index: # Coming from hunting and we haven't retried at all
                        try:
                            self.explore_index -= 1 
                            self._prev_submode = ctrl.state.tag_behavior_current
                            ctrl.state.tag_behavior_current = 3
                            self.retry_index += 1
                        except Exception as e:
                            print(f"Error: {e}")
                            self.behavior_roll(ctrl)
                            return
                    else: 
                        self.retry_index = 1
                        self._prev_submode = ctrl.state.tag_behavior_current
                        ctrl.state.tag_behavior_current = 0
                        print("Couldn't make it back")
                print("I'm lost!")
'''
            if submode == "Centering":
                # It goes into centering mode when it correctly arrives at tags location -> tries to make sure the tag pose estimation and distance meets some threshhold
                if self._prev_submode == 3:
                    #x,angle= ctrl.state.tag_sequence.get("x"),ctrl.state.tag_sequence.get("angle")
                    try:
                        if ctrl.state.tag_sequence is not None and ctrl.state.tag_sequence.get("id") == ctrl.state.tag_explore_sequence[self.explore_index]:
                            center = ctrl.state.tag_sequence.get("center") # -> Center x,y from left camera of whatever tag we see rn
                            val = 160.0 - center[0]
                            if val >= 10:
                                ctrl.motors.drive_all_left(.35)
                            elif val <= 10:
                                ctrl.motors.drive_all_right(.35)
                            print(center[0])
                        else:
                            pass
                    except Exception as e:
                        print(f"Error: {e}")
                #self._prev_submode = 4
                #ctrl.state.tag_behavior_current = 3
'''

