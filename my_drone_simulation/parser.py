"""
RL-Ready MAVLink Telemetry Interface
-------------------------------------

- Maintains internal state dictionary
- Updates state from MAVLink messages
- Provides get_state() for RL agent
- Controlled logging frequency
"""

from pymavlink import mavutil
import time
import math
import threading


class RLTekemetry:

    def __init__(self, connection_string='udp:127.0.0.1:14550'):

        # Connect to MAVLink stream
        self.master = mavutil.mavlink_connection(connection_string)

        print("Waiting for heartbeat...")
        self.master.wait_heartbeat()
        print("Connected.\n")

        # Shared state dictionary (this is what RL will read)
        self.state = {
            # Orientation
            "roll": 0.0,
            "pitch": 0.0,
            "yaw": 0.0,
            "roll_rate": 0.0,
            "pitch_rate": 0.0,
            "yaw_rate": 0.0,

            # Local position (NED)
            "x": 0.0,
            "y": 0.0,
            "z": 0.0,

            # Velocity
            "vx": 0.0,
            "vy": 0.0,
            "vz": 0.0,
            "speed": 0.0,

            # IMU
            "ax": 0.0,
            "ay": 0.0,
            "az": 0.0,

            # System
            "armed": False,
            "mode": "UNKNOWN",
            "battery": 0.0,
        }

        # Thread lock for safe state reading
        self.lock = threading.Lock()

        # Start background listener thread
        self.running = True
        self.thread = threading.Thread(target=self._listener)
        self.thread.daemon = True
        self.thread.start()


    # ==========================================================
    # Background listener thread
    # ==========================================================
    def _listener(self):

        while self.running:

            msg = self.master.recv_match(blocking=True)
            if not msg:
                continue

            msg_type = msg.get_type()

            with self.lock:

                # ------------------------
                # ATTITUDE
                # ------------------------
                if msg_type == "ATTITUDE":
                    self.state["roll"] = math.degrees(msg.roll)
                    self.state["pitch"] = math.degrees(msg.pitch)
                    self.state["yaw"] = (math.degrees(msg.yaw) + 360) % 360

                    self.state["roll_rate"] = msg.rollspeed
                    self.state["pitch_rate"] = msg.pitchspeed
                    self.state["yaw_rate"] = msg.yawspeed

                # ------------------------
                # LOCAL POSITION
                # ------------------------
                elif msg_type == "LOCAL_POSITION_NED":
                    self.state["x"] = msg.x
                    self.state["y"] = msg.y
                    self.state["z"] = msg.z

                    self.state["vx"] = msg.vx
                    self.state["vy"] = msg.vy
                    self.state["vz"] = msg.vz

                    self.state["speed"] = math.sqrt(
                        msg.vx**2 + msg.vy**2 + msg.vz**2
                    )

                # ------------------------
                # RAW IMU (Acceleration)
                # ------------------------
                elif msg_type == "RAW_IMU":
                    self.state["ax"] = msg.xacc
                    self.state["ay"] = msg.yacc
                    self.state["az"] = msg.zacc

                # ------------------------
                # BATTERY
                # ------------------------
                elif msg_type == "BATTERY_STATUS":
                    raw = msg.voltages[0]
                    if raw != 65535:
                        self.state["battery"] = raw / 1000

                # ------------------------
                # MODE + ARM STATUS
                # ------------------------
                elif msg_type == "HEARTBEAT":
                    self.state["mode"] = mavutil.mode_string_v10(msg)
                    self.state["armed"] = self.master.motors_armed()


    # ==========================================================
    # RL Interface
    # ==========================================================
    def get_state(self):
        """Return a copy of current state (safe for RL)."""
        with self.lock:
            return self.state.copy()

    def stop(self):
        self.running = False


# ==============================================================
# Example Usage
# ==============================================================

if __name__ == "__main__":

    telemetry = RLTekemetry()

    # Log at 10Hz (not every MAVLink packet)
    while True:
        state = telemetry.get_state()

        print(f"Mode: {state['mode']} | Armed: {state['armed']}")
        print(f"Pos: ({state['x']:.2f}, {state['y']:.2f}, {state['z']:.2f})")
        print(f"Vel: {state['speed']:.2f} m/s")
        print(f"Att: Roll {state['roll']:.2f}  Pitch {state['pitch']:.2f}  Yaw {state['yaw']:.2f}")
        print(f"Battery: {state['battery']:.2f} V")
        print("-" * 60)

        time.sleep(0.1)
