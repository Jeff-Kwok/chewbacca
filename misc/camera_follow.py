async def camera_loop(sock_camera):
    global state, select_state, tracker, latest_triggers
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
            if select_state != id_num:
                continue

            print(f"[CAMERA] center: midx={midx}, midy={midy}, "
                  f"label={label}, count={current_count}, select={select_state} ")
            distance = CAMERA_CENTER_X - midx

            if abs(distance) <= CAMERA_TOL:
                print("[CAMERA] Target centered, stopping motors.")
                motors.brake_all_motors(message_toggle=False)
                continue
            vel_scale_norm =min(1.0,max(0.0,abs(distance) / 320.0))**1.2
            speed_cmd = 2.5 + abs((vel_scale_norm * omega_max)*(0.9))  # rad/s

            if distance < -CAMERA_TOL:
                # Object is to RIGHT of center -> rotate right
                motors.drive_all_wheels({
                    "nrr":  -speed_cmd,
                    "nrl": speed_cmd,
                    "nfr":  -speed_cmd,
                    "nfl": speed_cmd,
                })
            elif distance > CAMERA_TOL:
                # Object is to LEFT of center -> rotate left
                motors.drive_all_wheels({
                    "nrr": speed_cmd,
                    "nrl":  -speed_cmd,
                    "nfr": speed_cmd,
                    "nfl": -speed_cmd,
                })
            # If no object detect drive forward - write function
            motors.drive_all_wheels({
                "nrr":  17,
                "nrl":  17,
                "nfr":  17,
                "nfl":  17,
            })
        else:
            motors.brake_all_motors(message_toggle=False)
async def camera_loop(sock_camera):
    global state, select_state, tracker, latest_triggers
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
            if select_state != id_num:
                continue

            print(f"[CAMERA] center: midx={midx}, midy={midy}, "
                  f"label={label}, count={current_count}, select={select_state} ")
            distance = CAMERA_CENTER_X - midx

            if abs(distance) <= CAMERA_TOL:
                print("[CAMERA] Target centered, stopping motors.")
                motors.brake_all_motors(message_toggle=False)
                continue
            vel_scale_norm =min(1.0,max(0.0,abs(distance) / 320.0))**1.2
            speed_cmd = 2.5 + abs((vel_scale_norm * omega_max)*(0.9))  # rad/s

            if distance < -CAMERA_TOL:
                # Object is to RIGHT of center -> rotate right
                motors.drive_all_wheels({
                    "nrr":  -speed_cmd,
                    "nrl": speed_cmd,
                    "nfr":  -speed_cmd,
                    "nfl": speed_cmd,
                })
            elif distance > CAMERA_TOL:
                # Object is to LEFT of center -> rotate left
                motors.drive_all_wheels({
                    "nrr": speed_cmd,
                    "nrl":  -speed_cmd,
                    "nfr": speed_cmd,
                    "nfl": -speed_cmd,
                })
            # If no object detect drive forward - write function
            motors.drive_all_wheels({
                "nrr":  17,
                "nrl":  17,
                "nfr":  17,
                "nfl":  17,
            })
        else:
            motors.brake_all_motors(message_toggle=False)
