#!/usr/bin/env python3
import socket
import asyncio
import math
import json
import time
import numpy as np
import serial
import websockets
from motorfunctions import MotorFunctions, PID

motors = MotorFunctions()


motors.set_all_direction_pins({
    # DIR A and DIR B and PWM must be defined such that
    # DRIVE A 0xFFFF and DRIVE B 0x000 when forward
    #0 -3 = back 
    # 3 -7 = forward
    # 8 =right rear
    # 11 =left rear
    # 10 = left forward
    # 9 = right forward
    "nrr": [4, 5, 10],
    "nrl": [1, 0, 8],
    "nfr": [6, 7, 11    ],
    "nfl": [3, 2, 9],
})

# ---------- NETWORK / PORTS ----------
LISTEN_IP   = "0.0.0.0"
LISTEN_PORT = 5456    # controller UDP
CAMERA_PORT = 5005    # camera UDP
LIDAR_PORT = 6065

# ---------- LIDAR CONFIG ------------



# ---------- STM32 SERIAL CONFIG ----------
STM_SERIAL_PORT = "/dev/ttyACM0"
STM_BAUD        = 115200

# ---------- GLOBAL STM STATE ----------
latest_stm_state = {
    "t_ms":    None,
    "w1_ticks": 0,
    "w2_ticks": 0,
    "w1_w":    0.0,
    "w1_v":    0.0,
    "w2_w":    0.0,
    "w2_v":    0.0,
    "w3_w":    0.0,
    "w3_v":    0.0,
    "w4_w":    0.0,
    "w4_v":    0.0,
    "yaw":     0.0,
    "pitch":   0.0,
    "roll":    0.0,
    "gx":      0.0,
    "gy":      0.0,
    "gz":      0.0,
}

# ---------- GLOBAL CONTROLLER STATE ----------
latest_axes = {"LX": 0.0, "LY": 0.0, "RX": 0.0, "RY": 0.0}
latest_triggers = {"LT": 0.0, "RT": 0.0}
latest_buttons = {
    "A": 0, "B": 0, "X": 0, "Y": 0,
    "DPAD_UP": 0, "DPAD_DOWN": 0,
    "DPAD_LEFT": 0, "DPAD_RIGHT": 0,
}
last_joy_time = 0.0   # time.monotonic of last controller packet

JOY_TIMEOUT = 0.5     # seconds → after this, treat as "no input" and stop

# ---------- HIGH-LEVEL MODE STATE ----------
select_state = 1
state = "controller"
_last_dpad_up = 0  # for edge-detect on toggle button

def state_flip():
    global state
    state = "camera" if state == "controller" else "controller"
    print(f"[STATE] Switched to: {state}")

# ---------- PARSER (CONTROLLER PAYLOAD) ----------

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

# ---------- SELECTION / TRACKER ----------
hunter = 0
tracker = 0
last_button3 = 0
last_button4 = 0
def select_frame(button1, button2, button3,button4):
    """
    button1 = DPAD_LEFT
    button2 = DPAD_RIGHT
    button3 = X (tracker toggle)
    """
    global select_state, tracker, last_button3, hunter

    # DPAD_LEFT increments
    if button1 == 1:
        select_state += 1
        print("select_state:", select_state)

    # DPAD_RIGHT decrements
    if button2 == 1:
        select_state -= 1
        print("select_state:", select_state)

    # Prevent going below 1 even if camera isn't active
    if select_state < 1:
        select_state = 1

    # X button toggles tracker on rising edge
    if button3 == 1 and last_button3 == 0:
        tracker ^= 1
        print(f"tracker: {tracker}")
    if button4 == 1 and last_button4 ==0:
        hunter ^= 1
        print(f"hunter: {hunter}")

    last_button3 = button3

# ---------- UDP LISTENER FOR CONTROLLER (NO PID HERE) ----------

async def controller_udp_listener(sock):
    """
    Just listens for UDP controller packets and updates global
    latest_axes / latest_triggers / latest_buttons / state / selector.
    The PID/control logic runs in a separate fixed-rate loop.
    """
    global latest_axes, latest_triggers, latest_buttons
    global last_joy_time, _last_dpad_up, state

    loop = asyncio.get_running_loop()
    print(f"[CTRL UDP] Listening on {LISTEN_IP}:{LISTEN_PORT} ...")

    while True:
        data = await loop.sock_recv(sock, 1024)
        msg = data.decode("ascii", errors="ignore")

        axes, triggers, buttons = parse_payload(msg)
        if axes is None:
            continue

        # update globals
        latest_axes = axes
        latest_triggers = triggers
        latest_buttons = buttons
        last_joy_time = time.monotonic()

        # Handle mode toggle (DPAD_UP edge)
        dpad_up = buttons["DPAD_UP"]
        if dpad_up == 1 and _last_dpad_up == 0:
            state_flip()
        _last_dpad_up = dpad_up

        # selection + tracker
        select_frame(buttons["DPAD_LEFT"], buttons["DPAD_RIGHT"], buttons["X"], buttons["Y"])

# ---------- FIXED-RATE CONTROL LOOP (RUNS PID EVERY TICK) ----------

async def control_loop():
    """
    Runs at a fixed rate (e.g. 100 Hz) using the latest joystick + STM state
    to compute wheel setpoints and PID outputs, even if no new controller
    packets arrive.
    """
    global latest_axes, latest_triggers, latest_buttons, state

    CONTROL_HZ = 100.0
    dt_nominal = 1.0 / CONTROL_HZ

    last_time = time.monotonic()

    omega_max = motors.mecanum_configuration["max_rad/s"]

    print(f"[CONTROL] Starting control loop at {CONTROL_HZ} Hz")

    while True:
        now = time.monotonic()
        dt = now - last_time
        if dt <= 0.0:
            dt = dt_nominal
        last_time = now

        # Sleep until next tick
        await asyncio.sleep(max(0.0, dt_nominal - (time.monotonic() - now)))

        # Check joystick timeout → stop if no input for a while
        age = now - last_joy_time
        if age > JOY_TIMEOUT:
            # No fresh joystick data → zero setpoint, brake
            #motors.brake_all_motors(message_toggle=False)
            #pid_fr.reset()
            continue

        axes = latest_axes
        triggers = latest_triggers
        buttons = latest_buttons
        stm = latest_stm_state

        # Only drive in "controller" mode
        if state != "controller":
            # Still allow RT as an e-stop
            if triggers["RT"] > 0:
                motors.brake_all_motors(message_toggle=False)
            continue

        x = axes["LX"]
        y = axes["LY"]
        rotate = axes["RX"]

        # Desired wheel angular velocities [rad/s]
        w_fl_des, w_fr_des, w_rl_des, w_rr_des = motors.calc_norm_vector(x, y, rotate)
        print(w_fl_des,w_fr_des,w_rl_des,w_rr_des)
        # Measured wheel speeds (ASSUMPTION: w1_w = FR encoder)
        # w1 = 13 12 -> forward right
        # w2 = 10 11 -> forward left 
        # w3 = 5 6 -> rear right
        # w4 = A0 A1 -> rear left
        w1_w = stm["w1_w"]
        w2_w = stm["w2_w"]
        w3_w = stm["w3_w"]
        w4_w = stm["w4_w"]

        # RT pressed → emergency brake + reset PID
        if triggers["RT"] > 0:
            motors.brake_all_motors(message_toggle=False)
            continue

        '''
        if zone_state["zone"] == "close":
            print(zone_state["zone"])
            w_fr_des = w_fr_des
            w_fl_des = w_fl_des
            w_rr_des = w_rr_des
            w_rl_des = w_rl_des
        '''
        w1_w = stm["w1_w"]
        w2_w = stm["w2_w"]
        w3_w = stm["w3_w"]
        w4_w = stm["w4_w"]
        w1_v = stm["w1_v"]
        w2_v = stm["w2_v"]
        w3_v = stm["w3_v"]
        w4_v = stm["w4_v"]
        #print(w1_w,w2_w,w3_w,w4_w)
        #print(w1_v,w2_v,w3_v,w4_v)
        print(
            f"[CONTROL] mode={state} x={x:+.2f} y={y:+.2f} rot={rotate:+.2f} | \n"
            f"FR_des={w_fr_des:.2f} rad/s, FR_meas={w1_w:.2f}\n"
            f"FL_des={w_fl_des:.2f} rad/s, FL_meas={w2_w:.2f}\n"
            f"RR_des={w_rr_des:.2f} rad/s, RR_meas={w3_w:.2f}\n"
            f"RL_des={w_rl_des:.2f} rad/s, RL_meas={w4_w:.2f}"
        )
        motors.drive_all_wheels({
            "nfr":w_fr_des,
            "nfl":w_fl_des,
            "nrr":w_rr_des,
            "nrl":w_rl_des, 
        })

        # Debug print (you can comment these out once it works)'
        


# ---------- CAMERA LOOP (UNCHANGED LOGIC BUT READS GLOBAL STATE) ----------

def calc_rot_speed(point, tolerance, center, maxdist,
                   omega_max=motors.mecanum_configuration["max_rad/s"]):
    distance = center - point
    vel_scale_norm = distance / maxdist

    sign = 1 if vel_scale_norm >= 0 else -1
    mag  = abs(vel_scale_norm)

    return sign * (mag ** 1.4) * omega_max/1.5

CAMERA_CENTER_X = 320
CAMERA_TOL      = 40.0
CAMERA_TIMEOUT  = 0.3
active_objects  = {}
CURRENT_ROTATION = None
LAST_SEEN = 0.0
LAST_TURNSPEED = 0.0
async def camera_loop(sock_camera):
    global state, select_state, tracker, latest_triggers, hunter
    loop = asyncio.get_running_loop()
    print(f"[CAMERA] Listening on {LISTEN_IP}:{CAMERA_PORT} ...")

    omega_max = motors.mecanum_configuration["max_rad/s"]

    while True:
        try:
            data = await asyncio.wait_for(
                loop.sock_recv(sock_camera, 4096),
                timeout=CAMERA_TIMEOUT,
            )
        except asyncio.TimeoutError:
            if state == "camera":
                motors.brake_all_motors(message_toggle=False)
            continue

        try:
            payload = json.loads(data.decode("utf-8"))
        except json.JSONDecodeError:
            print("[CAMERA] bad JSON payload")
            continue

        try:
            now_time = time.monotonic()
            midx, midy = payload["center"]
            label = payload["id"]

            id_num = int(label.split("_")[-1])
            active_objects[label] = now_time

            cutoff = now_time - CAMERA_TIMEOUT
            for oid, t_last in list(active_objects.items()):
                if t_last < cutoff:
                    del active_objects[oid]
            current_count = len(active_objects)

            if select_state > current_count:
                select_state = current_count
            elif select_state < 1:
                select_state = 1

        except (KeyError, ValueError, TypeError):
            print("[CAMERA] payload missing/invalid 'center'")
            continue

        triggers = latest_triggers

        # E-stop always allowed
        if triggers["RT"] > 0:
            motors.brake_all_motors(message_toggle=False)
            continue
        # EDIT CODE
        if state == "camera" and tracker == 1:
            now = time.monotonic()
            global CURRENT_ROTATION, LAST_SEEN, LAST_TURNSPEED
            if select_state != id_num:
                #  grace spinning
                #if (now - LAST_SEEN) <=0.3 and CURRENT_ROTATION in ("left","right"):
                #    if CURRENT_ROTATION == "left":
                #        motors.drive_all_left(LAST_TURNSPEED)
                #    elif CURRENT_ROTATION == "right":
                #        motors.drive_all_right(LAST_TURNSPEED)
                #else:
                #motors.brake_all_motors(message_toggle=False) # reset after grace ends
                continue
            print(f"[CAMERA] center: midx={midx}, midy={midy}, "
                  f"label={label}, count={current_count}, select={select_state} ")
            distance = CAMERA_CENTER_X - midx
            if abs(distance) <= CAMERA_TOL:
                print("[CAMERA] Target centered, stopping motors.")
                motors.brake_all_motors(message_toggle=False)
                continue

            # 
            speed_cmd = calc_rot_speed(midx,CAMERA_TOL,CAMERA_CENTER_X,320.0)
            rotate_command = -(speed_cmd/omega_max)*3.14/4.0
            if abs(zone_state["x_sum"]) >= 0.12: 
                checkx = -zone_state["x_sum"]
            else:
                checkx = 0
            if abs(zone_state["y_sum"]) >= 0.12:
                checky = -zone_state["y_sum"]
            else:
                checky = 0.5
            if hunter == 1:
                FL, FR, RL, RR = motors.calc_norm_vector(checkx,checky,rotate_command)
            else:
                FL, FR, RL, RR = motors.calc_norm_vector(0,0,rotate_command)
            motors.drive_all_wheels({
                "nfr": FR,
                "nfl": FL,
                "nrr": RR,
                "nrl": RL,
            })
            #LAST_SEEN = now
            #LAST_TURNSPEED = speed_cmd
        else:
            motors.brake_all_motors(message_toggle=False)

current_zone = "far"
LIDAR_WS_PORT = 6065
zone_state = {
    "zone":"Far",
    "x_sum":0.0,
    "y_sum":0.0
}
# ---------- LIDAR SERIAL LOOP ----------
async def zone_udp_loop():
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind(("0.0.0.0", LIDAR_WS_PORT))
    sock.setblocking(False)

    loop = asyncio.get_running_loop()
    print(f"[ZONE] Listening UDP on 0.0.0.0:{LIDAR_WS_PORT} ...")

    while True:
        data = await loop.sock_recv(sock, 2048)
        try:
            msg = json.loads(data.decode("utf-8"))
            zone_state["zone"] = msg.get("zone", zone_state["zone"])
            zone_state["x_sum"] = msg.get("x_sum", zone_state["x_sum"])
            zone_state["y_sum"] = msg.get("y_sum", zone_state["y_sum"])
            print(zone_state)
        except json.JSONDecodeError:
            continue

        if msg.get("type") != "zone":
            continue

# ---------- STM32 SERIAL LOOP ----------
async def stm_loop():
    """
    Continuously read JSON lines from STM32F405 on /dev/ttyACM0 and
    update latest_stm_state so controller/camera can use encoders + IMU.
    """
    global latest_stm_state

    print(f"[STM] Opening {STM_SERIAL_PORT} at {STM_BAUD} baud...")
    ser = serial.Serial(STM_SERIAL_PORT, STM_BAUD, timeout=1)

    time.sleep(2.0)  # let the STM settle

    while True:
        try:
            line = await asyncio.to_thread(ser.readline)
            if not line:
                continue

            line = line.decode("utf-8", errors="ignore").strip()
            if not line or not line.startswith("{"):
                continue

            data = json.loads(line)

            latest_stm_state["t_ms"]     = data.get("t",        latest_stm_state["t_ms"])
            latest_stm_state["w1_ticks"] = data.get("w1_ticks", latest_stm_state["w1_ticks"])
            latest_stm_state["w2_ticks"] = data.get("w2_ticks", latest_stm_state["w2_ticks"])
            latest_stm_state["w1_w"]     = data.get("w1_w",     latest_stm_state["w1_w"])
            latest_stm_state["w1_v"]     = data.get("w1_v",     latest_stm_state["w1_v"])
            latest_stm_state["w2_w"]     = data.get("w2_w",     latest_stm_state["w2_w"])
            latest_stm_state["w2_v"]     = data.get("w2_v",     latest_stm_state["w2_v"])
            latest_stm_state["w3_w"]     = data.get("w3_w",     latest_stm_state["w3_w"])
            latest_stm_state["w3_v"]     = data.get("w3_v",     latest_stm_state["w3_v"])
            latest_stm_state["w4_w"]     = data.get("w4_w",     latest_stm_state["w4_w"])
            latest_stm_state["w4_v"]     = data.get("w4_v",     latest_stm_state["w4_v"])
            # Using YAW
            latest_stm_state["yaw"]      = data.get("yaw",      latest_stm_state["yaw"])
            latest_stm_state["pitch"]    = data.get("pitch",    latest_stm_state["pitch"])
            latest_stm_state["roll"]     = data.get("roll",     latest_stm_state["roll"])
            latest_stm_state["gx"]       = data.get("gx",       latest_stm_state["gx"])
            latest_stm_state["gy"]       = data.get("gy",       latest_stm_state["gy"])
            latest_stm_state["gz"]       = data.get("gz",       latest_stm_state["gz"])
            #print(latest_stm_state["w1_v"],latest_stm_state["w2_v"],latest_stm_state["w3_v"],latest_stm_state["w4_v"])
        except json.JSONDecodeError as e:
            print("[STM] JSON error:", e, " line:", repr(line))
        except Exception as e:
            print("[STM] Unexpected error:", e)
            await asyncio.sleep(0.1)

# ---------- MAIN ----------

async def main():
    # Controller socket
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind((LISTEN_IP, LISTEN_PORT))
    sock.setblocking(False)

    # Camera socket
    sock_camera = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock_camera.bind((LISTEN_IP, CAMERA_PORT))
    sock_camera.setblocking(False)

    ctrl_udp_task = asyncio.create_task(controller_udp_listener(sock))
    control_task  = asyncio.create_task(control_loop())
    camera_task   = asyncio.create_task(camera_loop(sock_camera))
    stm_task      = asyncio.create_task(stm_loop())
    lidar_task = asyncio.create_task(zone_udp_loop())

    await asyncio.gather(ctrl_udp_task, control_task, camera_task, stm_task, lidar_task)

if __name__ == "__main__":
    try:
        asyncio.run(main())
    except KeyboardInterrupt:
        print("\nExiting.")
        motors.brake_all_motors(message_toggle=False)
