"""JSONL flight-log reader for the Isaac<->SITL bridge.

The bridge (Swarm_Drones/lab/bridge/Nvidia_SITL_connecter.py) writes
one JSON object per line. Three sources:

  src=bridge       lifecycle events: bridge_setup_started, motor_model_calibrated,
                   home_locked, shutdown, errors. Always have `event` key.
  src=isaac->sitl  state telemetry @50 Hz max (STATE_LOG_PERIOD_S=0.02):
                   gyro_frd, accel_frd, pos_ned, vel_ned, rpy, home_locked,
                   payload_kg, thrust_total_N, optional wind_world.
  src=sitl->isaac  PWM packets from ArduCopter: pwm[4]. (magic / frame_first
                   / addr captured once in the `first_sitl_packet` event.)

Coordinate conventions (from the bridge):
  pos_ned, vel_ned: NED frame anchored at home-lock, +D = down (so altitude
                   above home = -pos_ned[2]).
  gyro_frd, accel_frd: body FRD (forward-right-down) — what ArduPilot expects.
  rpy: [roll, pitch, yaw] radians.
"""
from __future__ import annotations

import json
from dataclasses import dataclass, field
from pathlib import Path
from typing import Iterable, Iterator


@dataclass
class FlightLog:
    """Parsed flight log split by source. Lists are time-ordered."""
    path: Path
    events: list[dict] = field(default_factory=list)         # src=bridge
    states: list[dict] = field(default_factory=list)         # src=isaac->sitl
    sitl_packets: list[dict] = field(default_factory=list)   # src=sitl->isaac

    @property
    def duration_s(self) -> float:
        if not self.states:
            return 0.0
        return float(self.states[-1]["t"] - self.states[0]["t"])

    def find_event(self, name: str) -> dict | None:
        for ev in self.events:
            if ev.get("event") == name:
                return ev
        return None

    def home_lock_time(self) -> float | None:
        """Wall-clock t at which home_locked first became True in the state stream."""
        for s in self.states:
            if s.get("home_locked"):
                return float(s["t"])
        return None


def iter_jsonl(path: str | Path) -> Iterator[dict]:
    p = Path(path)
    with p.open("r", encoding="utf-8") as f:
        for line_num, line in enumerate(f, start=1):
            line = line.strip()
            if not line:
                continue
            try:
                yield json.loads(line)
            except json.JSONDecodeError as e:
                # Bridge writes line-buffered; partial last line is the only
                # realistic corruption. Surface other failures.
                raise ValueError(f"{p}:{line_num} not JSON: {e}") from e


def load(path: str | Path) -> FlightLog:
    log = FlightLog(path=Path(path))
    for entry in iter_jsonl(path):
        src = entry.get("src")
        if src == "bridge":
            log.events.append(entry)
        elif src == "isaac->sitl":
            log.states.append(entry)
        elif src == "sitl->isaac":
            log.sitl_packets.append(entry)
        # other sources silently ignored — forward compatible
    return log


def states_in_window(states: Iterable[dict], t0: float, t1: float) -> list[dict]:
    return [s for s in states if t0 <= s["t"] <= t1]
