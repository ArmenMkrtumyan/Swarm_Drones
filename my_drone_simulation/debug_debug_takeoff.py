from pymavlink import mavutil
import time

MASTER = "udpin:localhost:14551"

APPLY_DEBUG_PARAMS = False
DEBUG_PARAMS = {
    "ARMING_MAGTHRESH": 0,
    "INS_USE2": 0,
}

PRE_ARM_WAIT_SECONDS = 10.0
POST_ARM_MONITOR_SECONDS = 10.0

ACK_RESULTS = {
    0: "ACCEPTED",
    1: "TEMPORARILY_REJECTED",
    2: "DENIED",
    3: "UNSUPPORTED",
    4: "FAILED",
    5: "IN_PROGRESS",
    6: "CANCELLED",
}


def ack_name(result):
    return ACK_RESULTS.get(int(result), f"UNKNOWN({result})")


def wait_heartbeat(master):
    print("Waiting for heartbeat...")
    master.wait_heartbeat()
    print(f"Heartbeat from system={master.target_system} component={master.target_component}")


def wait_command_ack(master, command, timeout=5.0):
    end = time.time() + timeout
    while time.time() < end:
        msg = master.recv_match(type="COMMAND_ACK", blocking=True, timeout=0.5)
        if msg and msg.command == command:
            return msg
    return None


def is_armed(master):
    hb = master.recv_match(type="HEARTBEAT", blocking=True, timeout=0.5)
    if hb is None:
        return False
    return bool(hb.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED)


def wait_armed(master, timeout=5.0):
    end = time.time() + timeout
    while time.time() < end:
        if is_armed(master):
            return True
    return False


def normalize_param_id(param_id):
    if isinstance(param_id, bytes):
        return param_id.decode("utf-8", errors="ignore").rstrip("\x00")
    if isinstance(param_id, str):
        return param_id.rstrip("\x00")
    return str(param_id).rstrip("\x00")


def set_param(master, name, value, timeout=5.0):
    print(f"Setting param {name} = {value}")
    master.mav.param_set_send(
        master.target_system,
        master.target_component,
        name.encode("utf-8"),
        float(value),
        mavutil.mavlink.MAV_PARAM_TYPE_REAL32,
    )

    end = time.time() + timeout
    while time.time() < end:
        msg = master.recv_match(type="PARAM_VALUE", blocking=True, timeout=0.5)
        if not msg:
            continue

        param_id = normalize_param_id(msg.param_id)
        if param_id == name:
            print(f"Confirmed {name} = {msg.param_value}")
            return True

    print(f"WARNING: did not receive PARAM_VALUE confirmation for {name}")
    return False


def set_mode(master, mode_name):
    mode_map = master.mode_mapping()
    if mode_map is None or mode_name not in mode_map:
        raise RuntimeError(
            f"Mode {mode_name} not available. Modes: {list(mode_map.keys()) if mode_map else 'None'}"
        )

    mode_id = mode_map[mode_name]
    master.mav.set_mode_send(
        master.target_system,
        mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
        mode_id,
    )
    print(f"Requested mode: {mode_name}")
    time.sleep(1.5)


def arm(master, force=False):
    param2 = 21196 if force else 0
    print(f"Sending {'FORCE ' if force else ''}ARM command...")

    master.mav.command_long_send(
        master.target_system,
        master.target_component,
        mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
        0,
        1,
        param2,
        0, 0, 0, 0, 0
    )

    ack = wait_command_ack(master, mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, timeout=5.0)
    if ack:
        print(f"Arm ACK: {ack_name(ack.result)} raw={ack}")
    else:
        print("Arm ACK: none")


def request_streams(master):
    try:
        master.mav.request_data_stream_send(
            master.target_system,
            master.target_component,
            mavutil.mavlink.MAV_DATA_STREAM_POSITION,
            10,
            1
        )
        master.mav.request_data_stream_send(
            master.target_system,
            master.target_component,
            mavutil.mavlink.MAV_DATA_STREAM_EXTENDED_STATUS,
            5,
            1
        )
        master.mav.request_data_stream_send(
            master.target_system,
            master.target_component,
            mavutil.mavlink.MAV_DATA_STREAM_EXTRA1,
            10,
            1
        )
    except Exception as e:
        print("Stream request warning:", repr(e))


def monitor(master, duration=5.0, label="monitor"):
    print(f"\n--- {label}: monitoring for {duration:.1f}s ---")
    end = time.time() + duration

    saw_prearm_error = False
    saw_gps_unhealthy = False

    while time.time() < end:
        msg = master.recv_match(blocking=True, timeout=0.5)
        if not msg:
            continue

        mtype = msg.get_type()

        if mtype == "STATUSTEXT":
            text = str(msg.text)
            print(f"STATUSTEXT: {text}")
            low = text.lower()
            if "prearm:" in low or low.startswith("arm:"):
                saw_prearm_error = True
            if "gps 1: not healthy" in low or "gps not healthy" in low:
                saw_gps_unhealthy = True

        elif mtype == "COMMAND_ACK":
            print(f"COMMAND_ACK: command={msg.command} result={ack_name(msg.result)} raw={msg}")

        elif mtype == "HEARTBEAT":
            armed = bool(msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED)
            print(f"HEARTBEAT: armed={armed} mode={msg.custom_mode}")

        elif mtype == "LOCAL_POSITION_NED":
            print(f"LOCAL_POSITION_NED: x={msg.x:.2f} y={msg.y:.2f} z={msg.z:.2f} vz={msg.vz:.2f}")

        elif mtype == "GLOBAL_POSITION_INT":
            print(
                f"GLOBAL_POSITION_INT: rel_alt={msg.relative_alt/1000.0:.2f}m "
                f"vz={msg.vz/100.0:.2f}m/s"
            )

        elif mtype == "GPS_RAW_INT":
            fix_type = int(getattr(msg, "fix_type", 0))
            sats = int(getattr(msg, "satellites_visible", 0))
            print(f"GPS_RAW_INT: fix_type={fix_type} sats={sats}")

    print(f"--- end {label} ---")
    return saw_prearm_error, saw_gps_unhealthy


def main():
    print("Connecting to SITL...")
    master = mavutil.mavlink_connection(MASTER)

    wait_heartbeat(master)
    request_streams(master)

    if APPLY_DEBUG_PARAMS:
        for name, value in DEBUG_PARAMS.items():
            set_param(master, name, value)

    set_mode(master, "STABILIZE")

    print(f"\nWaiting {PRE_ARM_WAIT_SECONDS:.1f}s in STABILIZE before arming...")
    saw_prearm_error, saw_gps_unhealthy = monitor(
        master,
        duration=PRE_ARM_WAIT_SECONDS,
        label="stabilize pre-arm settle"
    )

    if saw_prearm_error or saw_gps_unhealthy:
        print("Aborting before arm because health was not stable in STABILIZE.")
        return

    print("\nAttempting normal arm in STABILIZE...")
    arm(master, force=False)

    monitor(master, duration=POST_ARM_MONITOR_SECONDS, label="post-arm monitor")
    armed = wait_armed(master, timeout=3.0)
    print("Armed after normal arm?", armed)

    if armed:
        print("SUCCESS: vehicle armed normally in STABILIZE.")
    else:
        print("Normal arming failed even in STABILIZE.")


if __name__ == "__main__":
    main()