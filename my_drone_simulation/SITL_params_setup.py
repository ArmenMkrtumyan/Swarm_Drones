from pymavlink import mavutil
import time

MASTER = "udpin:localhost:14551"

PARAMS_TO_SET = {
    "SCHED_LOOP_RATE": 300,
    "INS_USE2": 0,
    "ARMING_MAGTHRESH": 0,
}

def wait_heartbeat(master):
    print("Waiting for heartbeat...")
    master.wait_heartbeat()
    print(f"Heartbeat from system={master.target_system} component={master.target_component}")

def set_param(master, name, value, timeout=5.0):
    print(f"Setting {name} = {value}")
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

        param_id = msg.param_id.decode("utf-8", errors="ignore").rstrip("\x00")
        if param_id == name:
            print(f"Confirmed {name} = {msg.param_value}")
            return True

    print(f"WARNING: no PARAM_VALUE confirmation for {name}")
    return False

def main():
    print("Connecting to SITL...")
    master = mavutil.mavlink_connection(MASTER)
    wait_heartbeat(master)

    for name, value in PARAMS_TO_SET.items():
        set_param(master, name, value)

    print("\nDone.")
    print("Now fully restart SITL before the next test.")

if __name__ == "__main__":
    main()
