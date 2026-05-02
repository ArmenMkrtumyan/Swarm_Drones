# Send Json From NVidia to QGroundControl and see what is happening

# NVIDIA

# 1. GPS
# 2. Velocity (acceleration)
# 3. Gyro + accelerometer
# 4. Radio

# SITL

# Motors (actuators)

# PID (proportional integral differential)
# Input - Gyro 
# Output - Actuator response (motor throttles)
# Threshold needs to be given (So that it wont stick to for example 44.7 instead of 45)
# We need to have some coefficient k_p * error(t) + integral(e(t)dt)*k_i 
# For the error(t) function, integral will always increase, so we can use it as SOME_NUMBER

# Lets introduce new function that will gradually get slower as time goes 
# We can use the differential of the function (As error decreases, differential will decrease too)
#  k_p * error(t) + integral(e(t)dt)*k_i + k_d  * d(error)/dt

# Find libraries for which our drones movement will move correctly as we give motor signals 

from pymavlink import mavutil
import time

MASTER = "udpin:localhost:14551"
TAKEOFF_ALT = 3.0
HOLD_SECONDS = 15.0         # how long to actively hold position after reaching altitude
HOLD_SEND_HZ = 5            # position-target refresh rate (GUIDED requires regular updates)

# Bench-test fallback
FORCE_ARM_FALLBACK = True
BENCH_THROTTLE = 1700   # 1200 was below min motor spin -> drone won't lift; 1700 gives real thrust
BENCH_SECONDS = 5.0

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


EKF_POS_HORIZ_ABS = 16   # EKF_STATUS_INFO_FLAGS: absolute horizontal position OK
EKF_POS_VERT_ABS  = 32   #                        absolute vertical position OK
EKF_PRED_POS_HORIZ_ABS = 512
EKF_GPS_GLITCHING = 32768

# Human-readable decoding for debug output
_EKF_FLAG_NAMES = [
    (1,     "ATTITUDE"),
    (2,     "VEL_HORIZ"),
    (4,     "VEL_VERT"),
    (8,     "POS_HORIZ_REL"),
    (16,    "POS_HORIZ_ABS"),
    (32,    "POS_VERT_ABS"),
    (64,    "POS_VERT_AGL"),
    (128,   "CONST_POS_MODE"),
    (256,   "PRED_POS_HORIZ_REL"),
    (512,   "PRED_POS_HORIZ_ABS"),
    (1024,  "UNINITIALIZED"),
    (32768, "GPS_GLITCHING"),
]


def _decode_ekf_flags(flags):
    return [name for bit, name in _EKF_FLAG_NAMES if flags & bit]


GPS_FIX_NAMES = {0: "NO_GPS", 1: "NO_FIX", 2: "2D", 3: "3D", 4: "DGPS", 5: "RTK_FLOAT", 6: "RTK_FIXED"}


def _request_message_interval(master, message_id, hz):
    """Tell ArduPilot to stream a given message ID at `hz` Hz."""
    interval_us = int(1_000_000 / hz)
    master.mav.command_long_send(
        master.target_system,
        master.target_component,
        mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL,
        0,
        message_id,
        interval_us,
        0, 0, 0, 0, 0,
    )


def wait_for_ekf_ready(master, timeout=30.0, poll_hz=2.0):
    """Poll EKF_STATUS_REPORT.flags until bit POS_HORIZ_ABS is set, which is
    what GUIDED's "Need Position Estimate" gate checks. Also polls GPS_RAW_INT
    so we can see whether SITL's simulated GPS is even reporting a fix — if
    the EKF stays in CONST_POS_MODE, that almost always means GPS fusion is
    being blocked by one of the EK3_GPS_CHECK bits.
    """
    _request_message_interval(master, 193, poll_hz)  # EKF_STATUS_REPORT
    _request_message_interval(master, 24, 1.0)       # GPS_RAW_INT (1 Hz is plenty)

    print(f"Waiting up to {timeout:.1f}s for EKF3 POS_HORIZ_ABS...")
    end = time.time() + timeout
    last_print = 0.0
    last_flags = None
    last_gps = None  # (fix_type, sats)
    while time.time() < end:
        msg = master.recv_match(type=["EKF_STATUS_REPORT", "GPS_RAW_INT"],
                                blocking=True, timeout=0.5)
        if msg is None:
            continue
        mtype = msg.get_type()
        now = time.time()
        if mtype == "GPS_RAW_INT":
            last_gps = (int(msg.fix_type), int(msg.satellites_visible))
            continue
        # EKF_STATUS_REPORT
        flags = int(msg.flags)
        if flags != last_flags or now - last_print >= 2.0:
            names = _decode_ekf_flags(flags)
            gps_str = ""
            if last_gps is not None:
                fix, sats = last_gps
                gps_str = f"  gps={GPS_FIX_NAMES.get(fix, fix)}/sats={sats}"
            print(f"  [ekf] flags=0x{flags:04x}  active={names}{gps_str}")
            last_print = now
            last_flags = flags
        if flags & EKF_POS_HORIZ_ABS and not (flags & EKF_GPS_GLITCHING):
            print(f"  [ekf] POS_HORIZ_ABS set — ready to arm")
            return True
    # Timed out. Report last state for diagnosis.
    if last_flags is None:
        print("  [ekf] no EKF_STATUS_REPORT received — check SET_MESSAGE_INTERVAL support")
        return False
    missing = []
    if not (last_flags & EKF_POS_HORIZ_ABS):
        missing.append("POS_HORIZ_ABS")
    if last_flags & 128:  # CONST_POS_MODE
        missing.append("(stuck in CONST_POS_MODE — GPS fusion blocked)")
    gps_str = ""
    if last_gps is not None:
        fix, sats = last_gps
        gps_str = f"  gps={GPS_FIX_NAMES.get(fix, fix)}/sats={sats}"
    print(f"  [ekf] timed out; last flags=0x{last_flags:04x}  missing={missing}{gps_str}")
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


def send_position_target(master, x, y, z, yaw=0.0):
    """Position-only GUIDED target in LOCAL_NED. z negative = up."""
    # type_mask: ignore velocity (bits 3-5), accel (6-8), force (9), yaw (10), yaw_rate (11).
    # Bits 0-2 (position) = 0 (use). = 0b0000111111111000 = 4088
    type_mask = 0b0000111111111000
    master.mav.set_position_target_local_ned_send(
        0,
        master.target_system,
        master.target_component,
        mavutil.mavlink.MAV_FRAME_LOCAL_NED,
        type_mask,
        x, y, z,
        0, 0, 0,
        0, 0, 0,
        yaw, 0.0,
    )


def takeoff_and_hold(master, alt, hold_s=15.0, climb_timeout=20.0, rate_hz=5):
    """Fire NAV_TAKEOFF, let ArduCopter's takeoff state machine run undisturbed,
    THEN start streaming position targets once we've reached the target altitude.

    Why two phases (and not stream-from-t=0):
      ArduCopter's GUIDED mode runs a dedicated takeoff submode (SubMode::TakeOff)
      after MAV_CMD_NAV_TAKEOFF. That submode owns the altitude target and ramps
      throttle past the 90 % breakout that calls set_land_complete(false). If we
      send SET_POSITION_TARGET_LOCAL_NED *during* the climb, ArduCopter's
      set_pos_NED_m calls pos_control_start(), which unconditionally overwrites
      guided_mode = SubMode::Pos — evicting takeoff before it can finish ramping.
      pos_control_run then sees ap.land_complete still true and routes through
      make_safe_ground_handling, clamping motors at MOT_SPIN_ARM (PWM ~1100). The
      drone never lifts. (See ArduCopter takeoff.cpp:18-48, mode_guided.cpp:367,
      mode.cpp:611-617, takeoff.cpp:113-180.)

      GUID_TIMEOUT only fires while GUIDED has no active controller target;
      during takeoff, the takeoff state machine IS the active target — no
      stream needed. After takeoff completes, then we stream for position hold.

    Returns True if target altitude was reached, False on timeout.
    """
    takeoff(master, alt)

    interval = 1.0 / rate_hz
    start = time.time()
    last_report = 0.0
    reached = False
    reached_t = None

    print(f"Phase 1 (climb): waiting for ArduCopter takeoff state to lift drone "
          f"to {alt:.1f}m. NOT streaming position targets (would override SubMode::TakeOff).")

    while not reached:
        now = time.time()
        elapsed = now - start
        if elapsed >= climb_timeout:
            print(f"Climb timeout after {climb_timeout:.0f}s — target alt never reached.")
            return False

        # Drain telemetry. We don't send anything — let takeoff state run.
        msg = master.recv_match(blocking=True, timeout=0.2)
        if msg is None:
            continue
        mtype = msg.get_type()
        if mtype == "GLOBAL_POSITION_INT":
            alt_now = msg.relative_alt / 1000.0
            if alt_now >= 0.95 * alt:
                reached = True
                reached_t = now
                print(f"Reached target alt ({alt_now:.2f}m); switching to streamed "
                      f"position hold for {hold_s:.1f}s...")
            elif now - last_report >= 1.0:
                print(f"[climb] t={elapsed:5.1f}s rel_alt={alt_now:.2f}m "
                      f"vz={msg.vz/100.0:+.2f}m/s")
                last_report = now
        elif mtype == "STATUSTEXT":
            print(f"STATUSTEXT: {msg.text}")

    # Phase 2: NOW stream position-target for hold. Submode switches to Pos,
    # which is what we want for stationary hover at altitude.
    last_report = 0.0
    while True:
        now = time.time()
        if (now - reached_t) >= hold_s:
            print(f"Hold complete ({hold_s:.1f}s).")
            return True

        send_position_target(master, 0.0, 0.0, -alt)

        for _ in range(20):
            msg = master.recv_match(blocking=False)
            if msg is None:
                break
            mtype = msg.get_type()
            if mtype == "GLOBAL_POSITION_INT":
                alt_now = msg.relative_alt / 1000.0
                if now - last_report >= 1.0:
                    print(f"[hold]  t={now-start:5.1f}s rel_alt={alt_now:.2f}m "
                          f"vz={msg.vz/100.0:+.2f}m/s")
                    last_report = now
            elif mtype == "STATUSTEXT":
                print(f"STATUSTEXT: {msg.text}")

        time.sleep(interval)


def land_and_wait(master, timeout=20.0):
    """Switch to LAND mode and watch telemetry until disarm or timeout."""
    print("Sending LAND...")
    set_mode(master, "LAND")
    end = time.time() + timeout
    last_report = 0.0
    while time.time() < end:
        for _ in range(20):
            msg = master.recv_match(blocking=False)
            if msg is None:
                break
            mtype = msg.get_type()
            if mtype == "GLOBAL_POSITION_INT":
                now = time.time()
                if now - last_report >= 1.0:
                    print(f"[land] rel_alt={msg.relative_alt/1000.0:.2f}m "
                          f"vz={msg.vz/100.0:+.2f}m/s")
                    last_report = now
            elif mtype == "STATUSTEXT":
                print(f"STATUSTEXT: {msg.text}")
                if "Disarm" in msg.text or "disarm" in msg.text.lower():
                    print("Drone disarmed — landing complete.")
                    return
        time.sleep(0.1)
    print("Land watch ended (timeout).")


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

    # Physical drone is QuadX (motors at diagonals). ArduPilot defaults can land on PLUS
    # from a stale EEPROM — force X so the mixer matches the bridge's motor layout.
    set_param(master, "FRAME_CLASS", 1)   # 1 = Quad
    set_param(master, "FRAME_TYPE", 1)    # 1 = X

    # Narrow debug tweak: disable only the mag-field strength check while testing.
    set_param(master, "ARMING_MAGTHRESH", 0)

    # First try the normal path. Wait for EKF3 to report "using GPS" before
    # the arm attempt — otherwise GUIDED arm fails with "Need Position Estimate"
    # because the EKF's origin isn't set yet.
    set_mode(master, "GUIDED")
    wait_for_ekf_ready(master, timeout=30.0)

    arm(master, force=False)
    drain_messages(master, duration=4.0)

    armed = wait_armed(master, timeout=3.0)
    print("Armed after normal arm?", armed)

    if armed:
        # Stream position targets continuously from t=0 — this keeps GUIDED alive
        # throughout the climb so ArduPilot doesn't throttle down near the target.
        takeoff_and_hold(master, TAKEOFF_ALT, hold_s=HOLD_SECONDS, rate_hz=HOLD_SEND_HZ)
        # Graceful landing at the end
        land_and_wait(master, timeout=20.0)
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
