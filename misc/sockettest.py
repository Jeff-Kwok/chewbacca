#!/usr/bin/env python3
import socket
import asyncio
import math
import json
import time
import numpy as np
from motorfunctions import MotorFunctions, PID
motors=MotorFunctions()
motors.set_all_direction_pins({
        # DIR A and DIR B and PWM must be defined such that DRIVE A 0XFFF AND DRIVE B 00000 WHEN FORWARD
        "nrr":[3,2,9],
        "nrl":[0,1,8],
        "nfr":[7,6,11],
        "nfl":[5,4,10]
})
# I2C / PCA9685

LISTEN_IP   = "0.0.0.0"
LISTEN_PORT = 5456    # controller
CAMERA_PORT = 5005    # camera
STM_PORT = 5678

# ---------- PARSER ----------

def parse_payload(msg: str):
    parts = msg.strip().split(",")
    if len(parts) != 14:
        print(f"[WARN] Expected 14 fields, got {len(parts)}: {msg!r}")
        return None, None, None

    try:
        lx  = float(parts[0])
        ly  = float(parts[1])
        rx  = float(parts[2])
        ry  = float(parts[3])
        lt  = float(parts[4])
        rt  = float(parts[5])

        a   = int(parts[6])
        b   = int(parts[7])
        x   = int(parts[8])
        y   = int(parts[9])

        d_up    = int(parts[10])
        d_down  = int(parts[11])
        d_left  = int(parts[12])
        d_right = int(parts[13])
    except ValueError as e:
        print(f"[WARN] Failed to parse payload {msg!r}: {e}")
        return None, None, None

    axes = {"LX": lx, "LY": ly, "RX": rx, "RY": ry}
    triggers = {"LT": lt, "RT": rt}
    buttons = {
        "A": a, "B": b, "X": x, "Y": y,
        "DPAD_UP": d_up, "DPAD_DOWN": d_down,
        "DPAD_LEFT": d_left, "DPAD_RIGHT": d_right,
    }
    return axes, triggers, buttons
# ---------- STATE ----------

select_state = 1
state = "controller"
_last_dpad_up = 0  # for edge-detect on toggle button
def state_flip():
    global state
    state = "camera" if state == "controller" else "controller"
    print(f"[STATE] Switched to: {state}")
# ---------- CONTROLLER LOOP ----------
def accel_limit(current, target, max_delta,dt):
    delta = target - current
    max_step = max_delta * dt
    if abs(delta) > max_delta:
        delta = math.copysign(max_delta, delta)
    return current + delta
async def controller_loop(sock):
    global state, _last_dpad_up

    loop = asyncio.get_running_loop()
    print(f"Listening for controller UDP on {LISTEN_IP}:{LISTEN_PORT} ...")

    while True:
        data = await loop.sock_recv(sock, 1024)
        msg = data.decode("ascii", errors="ignore")

        axes, triggers, buttons = parse_payload(msg)
        if axes is None:
            continue

        # Toggle state on DPAD_UP press (edge: 0 -> 1)
        dpad_up = buttons["DPAD_UP"]
        if dpad_up == 1 and _last_dpad_up == 0:
            state_flip()
        _last_dpad_up = dpad_up

        # Always process controller for safety, etc.
        x = axes["LX"]
        y = axes["LY"]
        rotate = axes["RX"]
        clutch = triggers["LT"]
        #fl, fr, rl, rr = mecanum_from_stick(x, y, rotate)
        #nfl, nfr, nrl, nrr = normalize_wheels(fl, fr, rl, rr)

        #print("Current State:", state)
        select_frame(buttons["DPAD_LEFT"], buttons["DPAD_RIGHT"], buttons["X"])

        if state == "controller":
            # Your existing mecanum + PCA9685 logic
            
            print(
                "Axes:", axes,
                "| Triggers:", triggers,
                "| Buttons:", buttons,
            )
            # This gves me individual wheel speeds
            nfl,nfr,nrl,nrr =motors.calc_norm_vector(x, y, rotate)
            # I want robocentrix simple vector velocities.
            # This is target wheel velocities
            print(
                f"  Wheels norm: FL={nfl:+.2f}, FR={nfr:+.2f}, RL={nrl:+.2f}, RR={nrr:+.2f}"
            )


            # We should send decomposed vector velocities in robot frame to pid loop.
            # 
        else:
            # state == "camera" -> you might still want some safety logic here
            # e.g. RT to emergency stop, etc.
            if triggers["RT"] > 0:
                motors.brake_all_motors(message_toggle=False)

# ---------- CAMERA LOOP ----------
tracker = 0
last_button3 = 0
import time
def select_frame(button1,button2,button3):
    global select_state
    global tracker
    global last_button3
    if button1 == 1:
        select_state += 1
        print("select_state:", select_state)
    else:
        pass
    if button2 == 1:
        select_state -=1
        print("select_state:", select_state)
    else:
        pass
    last_button3 = 0
    if button3 == 1 and last_button3 == 0:
        tracker ^= 1
        print(f"tracker: {tracker}")
    last_button3 = button3
CAMERA_CENTER_X = 320          # your nominal center
CAMERA_TOL      = 40.0         # +/- pixels considered "centered"
CAMERA_TIMEOUT  = 0.3          # seconds with no packet -> stop
active_objects = {}
async def camera_loop(sock_camera):
    global state
    loop = asyncio.get_running_loop()
    print(f"Listening for camera UDP on {LISTEN_IP}:{CAMERA_PORT} ...")


    while True:
        try:
            # Wait for a camera packet, but only up to CAMERA_TIMEOUT seconds
            data = await asyncio.wait_for(
                loop.sock_recv(sock_camera, 4096),
                timeout=CAMERA_TIMEOUT,
            )
        except asyncio.TimeoutError:
            # No camera packet in CAMERA_TIMEOUT seconds
            if state == "camera":
                #print("[CAMERA] Timeout - no detections, stopping motors.")
                motors.brake_all_motors(message_toggle=False)
            continue  # go back and wait again


        try:
            payload = json.loads(data.decode("utf-8"))
        except json.JSONDecodeError:
            print("[CAMERA] bad JSON payload")
            continue
        try:
            #global max_objects_seen
            now_time = time.monotonic()
            midx, midy = payload["center"]
            label = payload["id"]
            #print(all_label)
            id_num = int(label.split("_")[-1])
            active_objects[label] = now_time
            cutoff = now_time - CAMERA_TIMEOUT
            # Remove old objects
            for oid, t_last in list(active_objects.items()):
                if t_last < cutoff:
                    del active_objects[oid]
            current_count = len(active_objects)
            global select_state
            if select_state > current_count:
                select_state = current_count
            elif select_state < 1:
                select_state = 1 
            #print(id_num)
        except (KeyError, ValueError, TypeError):
            print("[CAMERA] payload missing/invalid 'center'")
            continue
        if state == "camera" and tracker == 1:
            if select_state != id_num:
                continue
            # If we're roughly centered, stop
            print(f"[CAMERA] center: midx={midx}, midy={midy}, label={label}, count={current_count}, select={select_state} ")
            distance = CAMERA_CENTER_X - midx
            if abs(distance) <= CAMERA_TOL:
                print("[CAMERA] Target centered, stopping motors.")
                motors.brake_all_motors(message_toggle=False)
                continue
                        #print(f"[CAMERA] Adjusting motors based on midx={midx}, distance={distance}")
            vel_scale = int(np.interp(abs(distance), [0, 320], [0,100]))
            # Turn left / right based on sign of distance
            if distance < -CAMERA_TOL:
                # Object is to the RIGHT of center -> rotate right, say
                motors.drive_all_wheels({
                    "nrr":vel_scale,
                    "nrl":-vel_scale,
                    "nfr":vel_scale,
                    "nfl":-vel_scale
                })
            elif distance > CAMERA_TOL:
                # Object is to the LEFT of center -> rotate left
                motors.drive_all_wheels({
                    "nrr":-vel_scale,
                    "nrl":vel_scale,
                    "nfr":-vel_scale,
                    "nfl":vel_scale
                })
        else:
            # Not in camera state; you can ignore or still log if you want
            motors.brake_all_motors(message_toggle=False)
            pass
            #print(f"[CAMERA] (ignored, state={state}) center={midx},{midy}")
# ---------- MAIN ----------
async def main():
    # Create sockets once, pass into tasks
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind((LISTEN_IP, LISTEN_PORT))
    sock.setblocking(False)

    sock_camera = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock_camera.bind((LISTEN_IP, CAMERA_PORT))
    sock_camera.setblocking(False)

    # Launch both loops concurrently
    controller_task = asyncio.create_task(controller_loop(sock))
    camera_task     = asyncio.create_task(camera_loop(sock_camera))

    await asyncio.gather(controller_task, camera_task)

if __name__ == "__main__":
    try:
        asyncio.run(main())
    except KeyboardInterrupt:
        print("\nExiting.")
        motors.brake_all_motors(message_toggle=False)
