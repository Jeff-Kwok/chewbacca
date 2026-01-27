import asyncio
from . import config
import numpy as np
import math
class SafetyModule:
    def __init__(self, state):
        self.state = state
    def angle_wrap(self,a):
        return (a + np.pi) % (2 * np.pi)-np.pi
    def angle_diff(self,a,b):
        return(self.angle_wrap(a-b))
    async def run(self):
        print("[SAFETY] Listening for close obstacles and controller inputs...")
        lx = 0
        ly = 0
        count = 0
        scale = 0
        rep_x = 0
        rep_y = 0
        angles_x = 0
        angles_y = 0
        self.clusters = []
        self.angle_gap = np.deg2rad(10)
        self.angle_step = np.deg2rad(0.5)    # fill resolution: add points every 0.5 degrees
        self.dist_gap = 0.5
        self.resultant_x = 0
        self.resultant_y = 0
        avoidance_val = 25
        while True:
            # Receive Lidar data for the close zone
            # These are populated by LidarModule based on config.LIDAR_AVOID_DISTANCES["close"]
            # This is a dictionary that holds two arrays indexed by points where their distance is Non-Zero and less than the LIDAR-MAX-RANGE.
            lidar_data = self.state.lidar_close

            # Rotating the Lidar points 90 degrees while keeping the range [0, 2pi]
            angles = ((lidar_data.get("angles", [])) + np.deg2rad(90)) % (2*np.pi)
            # Distance that are within ["middle"]
            ranges = lidar_data.get("ranges", [])

            # Receive Controller axes
            if self.state.robot_current == 1:
                lx = self.state.axes["LX"] * -1 # Orientation
                ly = self.state.axes["LY"]
            elif self.state.robot_current == 2:
                lx = self.state.command_vector["LX"] * -1 # Orientation
                ly = self.state.command_vector["LY"]
            elif self.state.robot_current == 3:
                lx = self.state.command_vector["LX"] # Orientation
                ly = self.state.command_vector["LY"]
                avoidance_val = 45  
            
            # Clustering and segmentation
            filled_angles = [angles[0]]
            filled_ranges = [ranges[0]]

            for i in range(1, len(angles)):
                a0, r0 = angles[i-1], ranges[i-1]
                a1, r1 = angles[i],   ranges[i]

                da = self.angle_diff(a1, a0)     # signed, wrap-safe
                abs_da = abs(da)
                dr = r1 - r0

                # If the angular gap is large -> segment boundary (don’t fill)
                if abs_da > self.angle_gap:
                    # (your cluster boundary bookkeeping)
                    self.cluster.append(a0)
                    self.cluster.append(a1)

                    filled_angles.append(a1)
                    filled_ranges.append(r1)
                    continue

                # If the gap is moderate and distance is continuous -> fill
                if abs_da > self.angle_step and abs(dr) < self.dist_gap:
                    n = int(abs_da // self.angle_step)   # number of interior samples
                    for k in range(1, n + 1):
                        t = k / (n + 1)             # 0..1
                        a = a0 + da * t             # interpolate angle along shortest wrap-safe path
                        r = r0 + dr * t             # linear interpolate range
                        filled_angles.append(a)
                        filled_ranges.append(r)

                filled_angles.append(a1)
                filled_ranges.append(r1)

            # Replace originals (effectively “appended”)
            angles = filled_angles
            ranges = filled_ranges










            
            # All points that are too close 
            too_close = np.asarray(close_ranges,dtype=float) <= self.state.LIDAR_AVOID_DISTANCES["close"] # Further filtering the ranges to points that are less than ["close"]
            # This is a boolean array where it considers true/false per index in array based on input array.
            # ---------------------------------------------------
            controller_angle = round((np.arctan2(ly, lx)),3) % (2*np.pi)
            avoidance_arc = np.deg2rad(avoidance_val)

            angles_x = np.cos(controller_angle)
            angles_y = np.sin(controller_angle)
            # ---------------------------------------------------


            # Remains of all angles within avoidance_arc and associated distances with those angles
            # These are indexed based operations so if there are any errors most likely that the angles are not correctly matching the distances detected.
            # We create a boolean array such that for all close-angles we consider only those within our arc which is indexed by its difference below the threshold.
            in_arc = np.abs(self.angle_diff(close_angles,controller_angle))<=avoidance_arc
            # We then overlap the two boolean arrays. The result is a boolean array that are only true for the indexes that are both close and within the arc of our intended directional vector.
            mask = in_arc & too_close
            count = int(np.sum(mask)) # How many true values that are in both boolean arrays? 
            if count > 0: # If our intended directional vector has no obstacles the count would be 0.
                scale = 1.0/count # Scalar value is calculated by the magnitude of repulsion desired divised by the number of parts.
                rep_x = np.sum(np.cos(close_angles[mask])) * scale
                rep_y = np.sum(np.sin(close_angles[mask])) * scale
                angles_x -= rep_x
                angles_y -= rep_y
            self.resultant_x = angles_x
            self.resultant_y = angles_y

            # Scale by input magnitude to ensure we stop when joystick is released
            # and update the shared state for core_control.py
            input_mag = np.sqrt(lx**2 + ly**2)
            if input_mag < 0.2:
                self.resultant_x = 0.0
                self.resultant_y = 0.0
            else:
                self.resultant_x *= input_mag
                self.resultant_y *= input_mag

            self.state.safe_axes["LX"] = self.resultant_x
            self.state.safe_axes["LY"] = self.resultant_y
            '''
            print(
                f"count:{count} | scale:{scale} |\n" 
                f"LX: {lx:.2f} LY: {ly:.2f} | Controller Angle: {np.rad2deg(controller_angle):.2f} |\n"
                f"rep x:{rep_x:.2f} | rep y:{rep_y:.2f} |\n"
                f"resultant output: LX: {self.resultant_x:.2f} LY: {self.resultant_y:.2f}"
                )
            '''
            await asyncio.sleep(0.05) # Run at ~20Hz