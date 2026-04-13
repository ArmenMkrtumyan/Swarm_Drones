from pymavlink import mavutil
import time
import math

MASTER = "udpin:localhost:14551"

# Mission from mission_creator.py
HOME_LAT = 40.192
HOME_LON = 44.50446
HOME_ALT_AMSL = 1200.0

WP2_LAT = 40.194682
WP2_LON = 44.504019
WP3_LAT = 40.193507
WP3_LON = 44.503119

MISSION_ALT = 50.0  # relative altitude

# Optional debug/bring-up tweaks
SET_ARMING_MAGTHRESH_ZERO = True
FORCE_ARM_FALLBACK = False   # keep False unless you really need bench-force-arming

MISSION_ACK_TYPES = {
    0: "ACCEPTED",
    1: "ERROR",
    2: "UNSUPPORTED_FRAME",
    3: "UNSUPPORTED",
    4: "NO_SPACE",
    5: "INVALID",
    6: "INVALID_PARAM1",
    7: "INVALID_PARAM2",
    8: "INVALID_PARAM3",
    9: "INVALID_PARAM4",
    10: "INVALID_PARAM5_X",
    11: "INVALID_PARAM6_Y",
    12: "INVALID_PARAM7",
    13: "INVALID_SEQUENCE",
    14: "DENIED",
    15: "OPERATION_CANCELLED",
}

COMMAND_ACK_RESULTS = {
    0: "ACCEPTED",
    1: "TEMPORARILY_REJECTED",
    2: "DENIED",
    3: "UNSUPPORTED",
    4: "FAILED",
    5: "IN_PROGRESS",
    6: "CANCELLED",
}


def mission_ack_name(v):
    return MISSION_ACK_TYPES.get(int(v), f"UNKNOWN({v})")


def command_ack_name(v):
    return COMMAND_ACK_RESULTS.get(int(v), f"UNKNOWN({v})")


def latlon_to_int(deg: float) -> int:
    return int(round(deg * 1e7))


def wait_heartbeat(master):
    print("Waiting for heartbeat...")
    master.wait_heartbeat()
    print(f"Heartbeat from system={master.target_system} component={master.target_component}")


def drain_messages(master, duration=3.0):
    print(f"Reading messages for {duration:.1f}s...")
    end = time.time() + duration
    while time.time() < end:
        msg = master.recv_match(blocking=True, timeout=0.5)
        if not msg:
            continue

        mtype = msg.get_type()
        if mtype == "STATUSTEXT":
            print(f"STATUSTEXT: {msg.text}")
        elif mtype == "COMMAND_ACK":
            print(f"COMMAND_ACK: command={msg.command} result={command_ack_name(msg.result)} raw={msg}")
        elif mtype == "MISSION_ACK":
            print(f"MISSION_ACK: type={mission_ack_name(msg.type)} raw={msg}")
        elif mtype == "MISSION_CURRENT":
            print(f"MISSION_CURRENT: seq={msg.seq}")
        elif mtype == "MISSION_ITEM_REACHED":
            print(f"MISSION_ITEM_REACHED: seq={msg.seq}")
        elif mtype == "HEARTBEAT":
            armed = bool(msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED)
            print(f"HEARTBEAT: armed={armed} mode={msg.custom_mode}")
        elif mtype == "GLOBAL_POSITION_INT":
            print(
                f"GLOBAL_POSITION_INT: lat={msg.lat/1e7:.7f} lon={msg.lon/1e7:.7f} "
                f"rel_alt={msg.relative_alt/1000.0:.2f}m vz={msg.vz/100.0:.2f}m/s"
            )
        elif mtype == "LOCAL_POSITION_NED":
            print(f"LOCAL_POSITION_NED: x={msg.x:.2f} y={msg.y:.2f} z={msg.z:.2f} vz={msg.vz:.2f}")


def wait_command_ack(master, command, timeout=5.0):
    end = time.time() + timeout
    while time.time() < end:
        msg = master.recv_match(type="COMMAND_ACK", blocking=True, timeout=0.5)
        if msg and msg.command == command:
            return msg
    return None


def wait_mission_ack(master, timeout=10.0):
    end = time.time() + timeout
    while time.time() < end:
        msg = master.recv_match(type="MISSION_ACK", blocking=True, timeout=0.5)
        if msg:
            return msg
    return None


def set_param(master, name, value):
    print(f"Setting param {name} = {value}")
    master.mav.param_set_send(
        master.target_system,
        master.target_component,
        name.encode("utf-8"),
        float(value),
        mavutil.mavlink.MAV_PARAM_TYPE_REAL32,
    )
    time.sleep(1.0)


def set_mode(master, mode_name):
    mode_map = master.mode_mapping()
    if mode_name not in mode_map:
        raise RuntimeError(f"Mode {mode_name} not available. Modes: {list(mode_map.keys())}")

    mode_id = mode_map[mode_name]
    master.mav.set_mode_send(
        master.target_system,
        mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
        mode_id,
    )
    print(f"Requested mode: {mode_name}")
    time.sleep(1.0)


def arm(master, force=False):
    param2 = 21196 if force else 0
    print(f"Sending {'FORCE ' if force else ''}ARM command...")
    master.mav.command_long_send(
        master.target_system,
        master.target_component,
        mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
        0,
        1,          # arm
        param2,     # 21196 = force arm
        0, 0, 0, 0, 0
    )
    ack = wait_command_ack(master, mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, timeout=5.0)
    if ack:
        print(f"Arm ACK: {command_ack_name(ack.result)} raw={ack}")
    else:
        print("Arm ACK: none")


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


def send_mission_clear_all(master):
    try:
        master.mav.mission_clear_all_send(
            master.target_system,
            master.target_component,
            mavutil.mavlink.MAV_MISSION_TYPE_MISSION,
        )
    except TypeError:
        master.mav.mission_clear_all_send(
            master.target_system,
            master.target_component,
        )
    print("Sent MISSION_CLEAR_ALL")
    time.sleep(1.0)


def send_mission_count(master, count):
    try:
        master.mav.mission_count_send(
            master.target_system,
            master.target_component,
            count,
            mavutil.mavlink.MAV_MISSION_TYPE_MISSION,
        )
    except TypeError:
        master.mav.mission_count_send(
            master.target_system,
            master.target_component,
            count,
        )
    print(f"Sent MISSION_COUNT: {count}")


def send_mission_item_int(master, item):
    try:
        master.mav.mission_item_int_send(
            master.target_system,
            master.target_component,
            item["seq"],
            item["frame"],
            item["command"],
            item["current"],
            item["autocontinue"],
            item["param1"],
            item["param2"],
            item["param3"],
            item["param4"],
            item["x"],
            item["y"],
            item["z"],
            mavutil.mavlink.MAV_MISSION_TYPE_MISSION,
        )
    except TypeError:
        master.mav.mission_item_int_send(
            master.target_system,
            master.target_component,
            item["seq"],
            item["frame"],
            item["command"],
            item["current"],
            item["autocontinue"],
            item["param1"],
            item["param2"],
            item["param3"],
            item["param4"],
            item["x"],
            item["y"],
            item["z"],
        )


def build_mission():
    GLOBAL_REL = mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT
    MISSION_FRAME = mavutil.mavlink.MAV_FRAME_MISSION

    items = [
        {
            "seq": 0,
            "frame": GLOBAL_REL,
            "command": mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
            "current": 0,
            "autocontinue": 1,
            "param1": 0.0,
            "param2": 0.0,
            "param3": 0.0,
            "param4": 0.0,
            "x": latlon_to_int(HOME_LAT),
            "y": latlon_to_int(HOME_LON),
            "z": float(MISSION_ALT),
        },
        {
            "seq": 1,
            "frame": GLOBAL_REL,
            "command": mavutil.mavlink.MAV_CMD_NAV_WAYPOINT,
            "current": 0,
            "autocontinue": 1,
            "param1": 5.0,   # hold
            "param2": 0.0,
            "param3": 0.0,
            "param4": 0.0,
            "x": latlon_to_int(WP2_LAT),
            "y": latlon_to_int(WP2_LON),
            "z": float(MISSION_ALT),
        },
        {
            "seq": 2,
            "frame": GLOBAL_REL,
            "command": mavutil.mavlink.MAV_CMD_NAV_WAYPOINT,
            "current": 0,
            "autocontinue": 1,
            "param1": 5.0,   # hold
            "param2": 0.0,
            "param3": 0.0,
            "param4": 0.0,
            "x": latlon_to_int(WP3_LAT),
            "y": latlon_to_int(WP3_LON),
            "z": float(MISSION_ALT),
        },
        {
            "seq": 3,
            "frame": MISSION_FRAME,
            "command": mavutil.mavlink.MAV_CMD_NAV_RETURN_TO_LAUNCH,
            "current": 0,
            "autocontinue": 1,
            "param1": 0.0,
            "param2": 0.0,
            "param3": 0.0,
            "param4": 0.0,
            "x": 0,
            "y": 0,
            "z": 0.0,
        },
    ]
    return items


def upload_mission(master, items):
    # clear existing mission and wait for THAT ack
    send_mission_clear_all(master)
    ack = wait_mission_ack(master, timeout=5.0)
    if ack is None or int(ack.type) != mavutil.mavlink.MAV_MISSION_ACCEPTED:
        raise RuntimeError(f"MISSION_CLEAR_ALL failed: {ack}")

    # now start upload
    send_mission_count(master, len(items))

    sent = set()
    while True:
        msg = master.recv_match(
            type=["MISSION_REQUEST_INT", "MISSION_REQUEST", "MISSION_ACK"],
            blocking=True,
            timeout=10.0,
        )
        if msg is None:
            raise RuntimeError("Timed out waiting for mission protocol response")

        mtype = msg.get_type()
        if mtype in ("MISSION_REQUEST_INT", "MISSION_REQUEST"):
            seq = int(msg.seq)
            if seq < 0 or seq >= len(items):
                raise RuntimeError(f"Invalid requested seq {seq}")
            send_mission_item_int(master, items[seq])
            sent.add(seq)
            print(f"Sent mission item {seq}")

        elif mtype == "MISSION_ACK":
            if len(sent) != len(items):
                raise RuntimeError(
                    f"Got MISSION_ACK before all items were requested. sent={sorted(sent)}"
                )
            if int(msg.type) != mavutil.mavlink.MAV_MISSION_ACCEPTED:
                raise RuntimeError(f"Mission upload failed: {msg}")
            break

def set_current_mission_item(master, seq=0):
    master.mav.mission_set_current_send(
        master.target_system,
        master.target_component,
        seq
    )
    print(f"Requested current mission item = {seq}")
    time.sleep(1.0)


def start_mission(master):
    print("Sending MAV_CMD_MISSION_START...")
    master.mav.command_long_send(
        master.target_system,
        master.target_component,
        mavutil.mavlink.MAV_CMD_MISSION_START,
        0,
        0, 0, 0, 0, 0, 0, 0
    )
    ack = wait_command_ack(master, mavutil.mavlink.MAV_CMD_MISSION_START, timeout=5.0)
    if ack:
        print(f"Mission start ACK: {command_ack_name(ack.result)} raw={ack}")
    else:
        print("Mission start ACK: none")


def main():
    print("Connecting to SITL...")
    master = mavutil.mavlink_connection(MASTER)

    wait_heartbeat(master)
    drain_messages(master, duration=2.0)

    if SET_ARMING_MAGTHRESH_ZERO:
        set_param(master, "ARMING_MAGTHRESH", 0)

    items = build_mission()
    print("Mission to upload:")
    for item in items:
        print(item)

    upload_mission(master, items)
    drain_messages(master, duration=2.0)

    set_current_mission_item(master, seq=0)

    set_mode(master, "AUTO")
    drain_messages(master, duration=2.0)

    arm(master, force=False)
    drain_messages(master, duration=4.0)

    armed = wait_armed(master, timeout=5.0)
    print("Armed after normal arm?", armed)

    if not armed and FORCE_ARM_FALLBACK:
        print("Normal arm failed. Trying force arm.")
        arm(master, force=True)
        drain_messages(master, duration=4.0)
        armed = wait_armed(master, timeout=5.0)
        print("Armed after force arm?", armed)

    if not armed:
        print("Vehicle is not armed. Stop here and inspect STATUSTEXT / EKF / GPS health.")
        return

    start_mission(master)
    drain_messages(master, duration=20.0)

    print("Mission script finished monitoring. Vehicle should now be in AUTO mission.")

if __name__ == "__main__":
    main()
