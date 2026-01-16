def parse_payload(msg: str):
    parts = msg.strip().split(",")
    if len(parts) != 16:
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
        LB = int(parts[10])
        RB = int(parts[11])
        d_up    = int(parts[12])
        d_down  = int(parts[13])
        d_left  = int(parts[14])
        d_right = int(parts[15])
    except ValueError as e:
        print(f"[WARN] Failed to parse payload {msg!r}: {e}")
        return None, None, None

    axes = {"LX": lx, "LY": ly, "RX": rx, "RY": ry}
    triggers = {"LT": lt, "RT": rt}
    buttons = {
        "A": a, "B": b, "X": x, "Y": y,
        "LB": LB, "RB": RB,
        "DPAD_UP": d_up, "DPAD_DOWN": d_down,
        "DPAD_LEFT": d_left, "DPAD_RIGHT": d_right,
    }
    return axes, triggers, buttons

def calc_rot_speed(point, tolerance, center, maxdist, omega_max):
    distance = center - point
    vel_scale_norm = distance / maxdist
    sign = 1 if vel_scale_norm >= 0 else -1
    mag  = abs(vel_scale_norm)
    return sign * (mag ** 1.4) * omega_max/1.5