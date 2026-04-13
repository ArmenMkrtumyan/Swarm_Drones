from pymavlink import mavutil
import time

MASTER = "udpin:localhost:14551"
TAKEOFF_ALT = 3.0

# Bench-test fallback
FORCE_ARM_FALLBACK = True
BENCH_THROTTLE = 1200
BENCH_SECONDS = 3.0

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
            print(f"COMMAND_ACK: command={msg.command} result={ack_name(msg.result)} raw={msg}")
        elif mtype == "HEARTBEAT":
            armed = bool(msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED)
            print(f"HEARTBEAT: armed={armed} mode={msg.custom_mode}")
        elif mtype == "LOCAL_POSITION_NED":
            print(f"LOCAL_POSITION_NED: x={msg.x:.2f} y={msg.y:.2f} z={msg.z:.2f} vz={msg.vz:.2f}")
        elif mtype == "GLOBAL_POSITION_INT":
            print(f"GLOBAL_POSITION_INT: rel_alt={msg.relative_alt/1000.0:.2f}m vz={msg.vz/100.0:.2f}m/s")


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
        print(f"Arm ACK: {ack_name(ack.result)} raw={ack}")
    else:
        print("Arm ACK: none")


def takeoff(master, alt):
    print(f"Sending TAKEOFF to {alt:.1f}m...")
    master.mav.command_long_send(
        master.target_system,
        master.target_component,
        mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
        0,
        0, 0, 0, 0,
        0, 0, alt
    )
    ack = wait_command_ack(master, mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, timeout=5.0)
    if ack:
        print(f"Takeoff ACK: {ack_name(ack.result)} raw={ack}")
    else:
        print("Takeoff ACK: none")


def send_rc_override(master, roll=1500, pitch=1500, throttle=1200, yaw=1500):
    master.mav.rc_channels_override_send(
        master.target_system,
        master.target_component,
        roll, pitch, throttle, yaw,
        65535, 65535, 65535, 65535
    )


def clear_rc_override(master):
    master.mav.rc_channels_override_send(
        master.target_system,
        master.target_component,
        0, 0, 0, 0,
        0, 0, 0, 0
    )


def main():
    print("Connecting to SITL...")
    master = mavutil.mavlink_connection(MASTER)

    wait_heartbeat(master)
    drain_messages(master, duration=2.0)

    # Narrow debug tweak: disable only the mag-field strength check while testing.
    set_param(master, "ARMING_MAGTHRESH", 0)

    # First try the normal path.
    set_mode(master, "GUIDED")
    drain_messages(master, duration=2.0)

    arm(master, force=False)
    drain_messages(master, duration=4.0)

    armed = wait_armed(master, timeout=3.0)
    print("Armed after normal arm?", armed)

    if armed:
        takeoff(master, TAKEOFF_ALT)
        drain_messages(master, duration=10.0)
        return

    if not FORCE_ARM_FALLBACK:
        print("Normal arming failed and force-arm fallback is disabled.")
        return

    print("Normal arm failed. Falling back to bench-test force arm.")
    set_mode(master, "STABILIZE")
    drain_messages(master, duration=2.0)

    arm(master, force=True)
    drain_messages(master, duration=4.0)

    armed = wait_armed(master, timeout=3.0)
    print("Armed after force arm?", armed)

    if not armed:
        print("Still not armed. Stop here and inspect STATUSTEXT.")
        return

    print(f"Sending low throttle RC override ({BENCH_THROTTLE}) for {BENCH_SECONDS:.1f}s...")
    start = time.time()
    while time.time() - start < BENCH_SECONDS:
        send_rc_override(master, throttle=BENCH_THROTTLE)
        time.sleep(0.1)

    clear_rc_override(master)
    print("Cleared RC override. Check Isaac console for PWM > 1000 and visible motor response.")
    drain_messages(master, duration=3.0)


if __name__ == "__main__":
    main()
