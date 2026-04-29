from pymavlink import mavutil
import time

MASTER = "udpin:0.0.0.0:14551"
TAKEOFF_ALT = 3.0

def wait_command_ack(master, command, timeout=5):
    start = time.time()
    while time.time() - start < timeout:
        msg = master.recv_match(type="COMMAND_ACK", blocking=True, timeout=1)
        if msg and msg.command == command:
            return msg
    return None

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

def arm(master):
    master.mav.command_long_send(
        master.target_system,
        master.target_component,
        mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
        0,
        1, 0, 0, 0, 0, 0, 0
    )
    ack = wait_command_ack(master, mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, timeout=5)
    print("Arm ACK:", ack)
    master.motors_armed_wait()
    print("Motors armed")

def takeoff(master, alt):
    master.mav.command_long_send(
        master.target_system,
        master.target_component,
        mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
        0,
        0, 0, 0, 0,
        0, 0, alt
    )
    ack = wait_command_ack(master, mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, timeout=5)
    print("Takeoff ACK:", ack)

def main():
    print("Connecting to SITL...")
    master = mavutil.mavlink_connection(MASTER)
    master.wait_heartbeat()
    print(f"Heartbeat from system={master.target_system} component={master.target_component}")

    set_mode(master, "GUIDED")
    time.sleep(1)

    arm(master)
    time.sleep(2)

    takeoff(master, TAKEOFF_ALT)
    print("Takeoff command sent")

if __name__ == "__main__":
    main()
