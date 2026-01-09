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
        count = 0
        scale = 0
        rep_x = 0
        rep_y = 0
        angles_x = 0
        angles_y = 0
        self.resultant_x = 0
        self.resultant_y = 0
        while True:
            # Receive Lidar data for the close zone
            # These are populated by LidarModule based on config.LIDAR_AVOID_DISTANCES["close"]
            lidar_data = self.state.lidar_close
            close_angles = ((lidar_data.get("angles", [])) + np.deg2rad(90)) % (2*np.pi)# Orientation and wrap from 0 to 2pi
            close_ranges = lidar_data.get("ranges", [])

            # Receive Controller axes
            lx = self.state.axes["LX"] * -1 # Orientation
            ly = self.state.axes["LY"]

            # All points that are too close 
            too_close = np.asarray(close_ranges,dtype=float) <= 1.2
            avoidance_arc = np.deg2rad(25)
            controller_angle = round((np.arctan2(ly, lx)),3) % (2*np.pi)

            angles_x = np.cos(controller_angle)
            angles_y = np.sin(controller_angle)
            # Remains of all angles within avoidance_arc and associated distances with those angles
            # These are indexed based operations so if there are any errors most likely that the angles are not correctly matching the distances detected.
            in_arc = np.abs(self.angle_diff(close_angles,controller_angle))<=avoidance_arc
            mask = in_arc & too_close
            count = int(np.sum(mask))

            if count > 0:
                scale = 1.0/count
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


'''
            # For each value in the angles detected within close range we want to check that value lies within our controller arc
            count = sum (controller_arc[0] <= val <= controller_arc[1] for val in close_angles)
            for i,val in enumerate(close_angles):
                if controller_arc[0] <= val <= controller_arc[1]:
                    # If our intention vector is within an area of the vector histogram, we use the distance to determine
                    if close_ranges[i] < 0.5:
                        print(angles_y)
                        angles_x -= 1*np.cos(val) * 10/count
                        angles_y -= 1*np.sin(val) * 10/count
                else:
                    continue
            #print(angles_x)
            #print(angles_y)
            #resultant_angle =round(np.arctan2(controller_y,controller_x),3)
            #if resultant_angle < 0:
            #    resultant_angle = resultant_angle + 2*math.pi
            #print(np.rad2deg(resultant_angle))'''