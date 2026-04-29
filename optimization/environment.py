"""
Continuous-action point-mass swarm coverage environment.

Designed so the same control logic (Potential Fields, Consensus, MARL)
can later be ported to Isaac Sim with minimal changes:
    - Actions are 2D acceleration commands (not grid moves).
    - max_speed and max_accel mirror real quadcopter limits.
    - Sensing is range-limited per drone (sensor_radius).
    - Coverage is tracked on a discrete grid for objective evaluation.
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field
from typing import Optional

import numpy as np

from maze import FREE, WALL, random_free_positions


# STEEReoCAM Nano camera spec (per `optimization/docs/camera_specs/`):
#     HFOV = 54°, VFOV = 49.5°, DFOV = 77.9°
#     Stereo depth range = 0.95 – 8 m (beyond 8 m, depth quality degrades)
#     Sensor: 2× OV2311, 100 mm baseline, 1600×1300 per eye, 30 fps stereo
# Used by sensor_radius_from_altitude() below; if you change the camera,
# update these constants in lock-step with the docs.
STEEREOCAM_HFOV_DEG = 54.0
STEEREOCAM_VFOV_DEG = 49.5
STEEREOCAM_DEPTH_MIN_M = 0.95
STEEREOCAM_DEPTH_MAX_M = 8.0


def sensor_radius_from_altitude(
    altitude_m: float,
    hfov_deg: float = STEEREOCAM_HFOV_DEG,
    vfov_deg: float = STEEREOCAM_VFOV_DEG,
    shape: str = "inscribed",
) -> float:
    """
    Convert a real-world camera FOV + altitude into a disc-equivalent
    sensor_radius in meters. The camera footprint is rectangular; we
    approximate it with a disc using one of three conventions:

        - "inscribed": disc fits inside the rectangle. CONSERVATIVE — every
          cell inside the disc is guaranteed inside the camera's actual view.
          Radius = altitude × min(tan(HFOV/2), tan(VFOV/2)).
        - "equivalent_area": disc has the same area as the rectangle. A
          balanced choice for "how much ground is in view per snapshot."
          Radius = altitude × √(4 · tan(HFOV/2) · tan(VFOV/2) / π).
        - "diagonal": disc covers the rectangle's corners. LIBERAL — some
          cells inside the disc are outside the actual FOV (in the corners).
          Radius = altitude × √(tan²(HFOV/2) + tan²(VFOV/2)).

    Defaults match the project's STEEReoCAM Nano (see docs/camera_specs/
    for the datasheet). Pass altitude in meters, get radius in meters; the
    caller is responsible for dividing by `meters_per_cell` to get cells.

    Tip: real F450 swarm coverage missions typically run at 5–16 m AGL.
    Beyond ~8 m the stereo depth quality drops below the camera spec, but
    visual coverage (which is all our model tracks) still works.
    """
    hw = altitude_m * math.tan(math.radians(hfov_deg / 2))
    hh = altitude_m * math.tan(math.radians(vfov_deg / 2))
    if shape == "inscribed":
        return min(hw, hh)
    if shape == "equivalent_area":
        return math.sqrt(4 * hw * hh / math.pi)
    if shape == "diagonal":
        return math.hypot(hw, hh)
    raise ValueError(f"unknown shape {shape!r}; "
                     f"expected one of: inscribed, equivalent_area, diagonal")


@dataclass
class SimConfig:
    """
    Environment-level settings (apply to the whole sim, not any one drone).

    `meters_per_cell` is the world-scale knob: how many real meters one grid
    cell represents. Default 5.0 is derived from the STEEReoCAM Nano camera
    footprint at the implicit `flight_altitude_m` below (inscribed-disc
    convention). The defaults reproduce:

        sensor_radius = 1.5 cells × 5 m/cell = 7.5 m
            ≈ inscribed disc of STEEReoCAM (HFOV 54°, VFOV 49.5°)
              at h = 7.5 / tan(49.5°/2) ≈ 16.3 m AGL
        max_speed     = 1.5 cells/s × 5      = 7.5 m/s  (realistic F450 cruise)
        max_accel     = 2.5 cells/s² × 5     = 12.5 m/s² ≈ 1.3 g
        21×21 map                            = 105 m × 105 m

    To rescale for a different altitude, use sensor_radius_from_altitude()
    above and adjust SimConfig.meters_per_cell + DroneConfig.sensor_radius
    in lock-step. Note that 16 m AGL is above the STEEReoCAM's 0.95–8 m
    stereo-depth ceiling — the camera sees that far visually, but stereo
    depth degrades. For depth-reliable coverage, fly at h ≤ 8 m.
    """
    step_seconds: float = 0.1       # how much sim time one env.step() advances
    meters_per_cell: float = 5.0    # cell ↔ meter conversion (see class docstring)
    flight_altitude_m: float = 16.3 # informational: AGL altitude assumed by sensor_radius
                                    # default. NOT used in physics — sensor_radius is
                                    # what the model uses. Stored here so it's discoverable
                                    # in the panel / printouts.


@dataclass
class DroneConfig:
    """
    Per-drone vehicle properties. The current `CoverageEnv` applies one shared
    `DroneConfig` to every drone (homogeneous fleet). To support heterogeneous
    swarms later, accept a list of `DroneConfig` instead.

    `sensor_radius` is the disc-equivalent ground footprint of the downward
    camera (STEEReoCAM Nano on this drone — see docs/f450-reference.md and
    docs/camera_specs/ for the datasheet). At the default 5 m/cell + 1.5 cells,
    the footprint disc is 7.5 m, derived from the inscribed disc of the
    camera's 54° × 49.5° rectangle at ~16 m AGL. To rescale for different
    altitudes use environment.sensor_radius_from_altitude().
    """
    sensor_radius: float = 1.5      # cells (= 7.5 m at meters_per_cell=5; STEEReoCAM
                                    # inscribed disc at ~16 m AGL — see SimConfig)
    max_speed: float = 1.0          # cells / second (≈ 5 m/s at meters_per_cell=5)
    max_accel: float = 2.0          # cells / second^2 (≈ 10 m/s² at meters_per_cell=5)
    drone_radius: float = 0.05      # cells (≈ 0.25 m: Hawk's Work F450 half-width); collision only
    mass_kg: float = 1.9            # Hawk's Work F450 + Jetson Nano + STEEReoCAM + 3S LiPo.
                                    # Informational only — power model uses back-calculated
                                    # hover_power_w rather than deriving from m·g and rotor area.


@dataclass
class BatteryConfig:
    """
    Hawk's Work F450 reference build (https://www.hawks-work.com/pages/f450-drone):
        - Frame: F450 450 mm wheelbase (clone of DJI Flame Wheel F450)
        - Motors: 4× A2212 920 KV
        - ESCs: 4× 20 A brushless
        - Props: 9450 self-tightening
        - Battery: 11.1 V 3S LiPo, 4200 mAh, 25 C, XT60
        - Hover power: ~165 W (back-calculated from F450-class flight times)

    Per-step instantaneous power:  P = hover_power_w + motion_coeff_w_per_v2 * |v|^2
    Cutoff: drone is "depleted" once current_voltage_v(battery_j) <= min_voltage_v.
    """
    voltage_v: float = 11.1             # nominal pack voltage (11.1 V = 3S LiPo: 3 × 3.7 V)
    capacity_mah: float = 4200.0        # F450 reference pack
    hover_power_w: float = 165.0        # back-calculated from F450 flight times — see README
    # Quadratic profile-power adder, calibrated so that at v = 15 m/s = 3 cells/s
    # (real F450 max), total power ≈ 280 W (matches published measurements:
    # ~70% over hover). k_meters = 115 W / 15² ≈ 0.51 W·s²/m²; converting via
    # k_cells = k_meters × meters_per_cell² = 0.51 × 25 = 12.75 → round to 13.
    # The v² form (rather than v³) is justified for our speed regime, where
    # blade-profile drag dominates parasitic airframe drag — see README sources.
    motion_coeff_w_per_v2: float = 13.0
    min_voltage_v: float = 10.0        # cutoff: 10 V / 3 cells ≈ 3.33 V/cell (cell-damage threshold)
    cell_full_voltage_v: float = 4.2   # per-cell maximum at 100% charge (LiPo standard)
    cell_dead_voltage_v: float = 3.0   # per-cell at 0% remaining (anchor for linear V(E))

    @property
    def n_cells(self) -> int:
        # 3.7 V is the LiPo per-cell nominal voltage. Pack voltage / 3.7 gives the
        # cell count: 11.1 V → 3 (3S), 14.8 V → 4 (4S), 22.2 V → 6 (6S).
        return max(1, round(self.voltage_v / 3.7))

    @property
    def initial_energy_j(self) -> float:
        # Energy in joules from voltage × charge (where charge in Coulombs = mAh × 3.6).
        # E_J = V · (mAh / 1000) · 3600 = V · mAh · 3.6.  See README "unit conversions".
        return self.voltage_v * (self.capacity_mah / 1000.0) * 3600.0

    def current_voltage_v(self, energy_j: float) -> float:
        """
        Linear V(E) approximation: voltage drops linearly from V_full at full charge
        to V_dead at zero charge. NOT physically accurate (real LiPo has a flat
        plateau and a sharp end-of-discharge knee) — used only to map the user's
        `min_voltage_v` cutoff to an energy threshold. See README "Voltage cutoff".
        """
        v_full = self.cell_full_voltage_v * self.n_cells
        v_dead = self.cell_dead_voltage_v * self.n_cells
        frac = max(0.0, energy_j / self.initial_energy_j) if self.initial_energy_j > 0 else 0.0
        return v_dead + (v_full - v_dead) * frac

    @property
    def cutoff_energy_j(self) -> float:
        """
        Energy threshold at which current_voltage_v == min_voltage_v.
        Inverse of the linear V(E):  frac = (V_min - V_dead) / (V_full - V_dead),
        then cutoff_energy_j = frac × initial_energy_j.
        For F450 defaults: frac = (10 - 9) / (12.6 - 9) = 0.278 → 27.8% remaining.
        """
        v_full = self.cell_full_voltage_v * self.n_cells
        v_dead = self.cell_dead_voltage_v * self.n_cells
        v_range = v_full - v_dead
        if v_range <= 0:
            return 0.0
        frac = max(0.0, (self.min_voltage_v - v_dead) / v_range)
        return min(1.0, frac) * self.initial_energy_j


@dataclass
class Drone:
    pos: np.ndarray                 # shape (2,), float, in cell units (x, y)
    vel: np.ndarray = field(default_factory=lambda: np.zeros(2))
    battery_j: float = 0.0          # set by env on reset


class CoverageEnv:
    """
    State container plus stepping logic. Not a Gym env; trivial to wrap later.
    """

    def __init__(
        self,
        grid: np.ndarray,
        n_drones: int,
        sim: Optional[SimConfig] = None,
        drone: Optional[DroneConfig] = None,
        battery: Optional[BatteryConfig] = None,
    ) -> None:
        self.grid = grid.astype(np.int8, copy=True)
        self.n_drones = n_drones
        self.sim_cfg = sim or SimConfig()
        self.drone_cfg = drone or DroneConfig()
        self.battery_cfg = battery or BatteryConfig()

        self.h, self.w = self.grid.shape
        self.covered = np.zeros_like(self.grid, dtype=bool)
        self.covered[self.grid == WALL] = True  # walls do not count as coverable
        # Per-drone coverage masks: True wherever this specific drone has been
        # within sensor_radius of a free cell. Walls stay False here so each
        # drone's fraction is over coverable cells only. Sum across drones can
        # exceed the global mask — the gap measures overlap.
        self.drone_covered = np.zeros((n_drones, self.h, self.w), dtype=bool)
        # Entry-count state. `entry_count[i, y, x]` = number of times drone i has
        # transitioned from outside-its-disc to inside-its-disc on cell (y, x).
        # `prev_footprint[i]` is last step's disc, used to diff for new entries.
        # Hovering does not inflate counts (cells stay inside the disc, no transition).
        self.entry_count = np.zeros((n_drones, self.h, self.w), dtype=np.int32)
        self.prev_footprint = np.zeros((n_drones, self.h, self.w), dtype=bool)
        # `first_visitor[y, x]` = drone index that first claimed the cell, -1 if
        # none yet. Ties on the same step go to the lower index (deterministic).
        # Used by the renderer to paint per-drone territory in distinct colors.
        self.first_visitor = np.full((self.h, self.w), -1, dtype=np.int8)

        self.drones: list[Drone] = []
        self.time_seconds: float = 0.0
        self.step_count: int = 0

    # -------- lifecycle --------

    def reset(self, seed: Optional[int] = None) -> None:
        positions = random_free_positions(self.grid, self.n_drones, seed=seed)
        full = self.battery_cfg.initial_energy_j
        self.drones = [Drone(pos=p.copy(), battery_j=full) for p in positions]
        self.covered[:] = False
        self.covered[self.grid == WALL] = True
        self.drone_covered[:] = False
        self.entry_count[:] = 0
        self.prev_footprint[:] = False
        self.first_visitor[:] = -1
        self.time_seconds = 0.0
        self.step_count = 0
        self._update_coverage()

    # -------- stepping --------

    def step(self, actions: np.ndarray) -> None:
        """
        actions: shape (n_drones, 2), interpreted as acceleration commands.
        Clipped to max_accel; resulting velocity clipped to max_speed.

        If `self.is_done()` is already True (100% coverage reached on a previous
        step), this becomes a no-op: drones are forced to zero velocity, no
        battery drain, no clock advance. Callers should check `is_done()` and
        break out of their loop instead of relying on this safety net.
        """
        if self.is_done():
            for d in self.drones:
                d.vel = np.zeros(2, dtype=np.float64)
            return

        actions = np.asarray(actions, dtype=np.float64).reshape(self.n_drones, 2)
        a_norm = np.linalg.norm(actions, axis=1, keepdims=True)
        scale = np.minimum(1.0, self.drone_cfg.max_accel / np.maximum(a_norm, 1e-9))
        accel = actions * scale

        step_seconds = self.sim_cfg.step_seconds
        cutoff_j = self.battery_cfg.cutoff_energy_j

        for i, drone in enumerate(self.drones):
            # Voltage cutoff: depleted drones freeze in place and stop draining.
            if drone.battery_j <= cutoff_j:
                drone.vel = np.zeros(2, dtype=np.float64)
                continue

            new_vel = drone.vel + accel[i] * step_seconds
            v_norm = np.linalg.norm(new_vel)
            if v_norm > self.drone_cfg.max_speed:
                new_vel *= self.drone_cfg.max_speed / v_norm

            new_pos = drone.pos + new_vel * step_seconds
            new_pos, new_vel = self._resolve_walls(drone.pos, new_pos, new_vel)
            drone.pos = new_pos
            drone.vel = new_vel

            # Battery drain: hover power plus a quadratic-in-speed motion adder.
            speed_sq = float(new_vel @ new_vel)
            power_w = self.battery_cfg.hover_power_w + self.battery_cfg.motion_coeff_w_per_v2 * speed_sq
            drone.battery_j = max(cutoff_j, drone.battery_j - power_w * step_seconds)

        self._update_coverage()
        self.time_seconds += step_seconds
        self.step_count += 1

    # -------- collision (axis-separated, simple) --------

    def _resolve_walls(
        self,
        old_pos: np.ndarray,
        new_pos: np.ndarray,
        vel: np.ndarray,
    ) -> tuple[np.ndarray, np.ndarray]:
        r = self.drone_cfg.drone_radius
        out_pos = old_pos.copy()
        out_vel = vel.copy()

        # x-axis move
        cand_x = np.array([new_pos[0], old_pos[1]])
        if not self._collides(cand_x, r):
            out_pos[0] = new_pos[0]
        else:
            out_vel[0] = 0.0

        # y-axis move (using updated x)
        cand_y = np.array([out_pos[0], new_pos[1]])
        if not self._collides(cand_y, r):
            out_pos[1] = new_pos[1]
        else:
            out_vel[1] = 0.0

        return out_pos, out_vel

    def _collides(self, pos: np.ndarray, r: float) -> bool:
        x, y = pos
        x0 = int(np.floor(x - r))
        x1 = int(np.floor(x + r))
        y0 = int(np.floor(y - r))
        y1 = int(np.floor(y + r))
        for cy in range(y0, y1 + 1):
            for cx in range(x0, x1 + 1):
                if cy < 0 or cy >= self.h or cx < 0 or cx >= self.w:
                    return True
                if self.grid[cy, cx] == WALL:
                    return True
        return False

    # -------- coverage --------

    def _update_coverage(self) -> None:
        r = self.drone_cfg.sensor_radius
        ys = np.arange(self.h)[:, None] + 0.5
        xs = np.arange(self.w)[None, :] + 0.5
        free_mask = self.grid == FREE

        for i, drone in enumerate(self.drones):
            dx = xs - drone.pos[0]
            dy = ys - drone.pos[1]
            footprint = ((dx * dx + dy * dy) <= r * r) & free_mask

            # Entry events: cells newly inside the disc since last step. Hovering
            # generates 0 entries (cells stay inside the disc → no transition).
            entries = footprint & ~self.prev_footprint[i]
            self.entry_count[i] += entries.astype(np.int32)

            # First-visitor claim: lowest drone index wins ties on the same step
            # because we iterate i = 0..n-1 and only fill cells still unclaimed.
            unclaimed = (self.first_visitor == -1) & entries
            self.first_visitor[unclaimed] = i

            self.prev_footprint[i] = footprint
            self.covered |= footprint
            self.drone_covered[i] |= footprint
        self.covered[self.grid == WALL] = True

    def coverage_fraction(self) -> float:
        free_mask = self.grid == FREE
        n_free = int(free_mask.sum())
        if n_free == 0:
            return 1.0
        return float((self.covered & free_mask).sum()) / n_free

    def is_done(self) -> bool:
        """
        Mission-success terminal condition: every coverable cell has been seen.
        """
        return self.coverage_fraction() >= 1.0

    def all_depleted(self) -> bool:
        """
        Mission-failure terminal condition: every drone has hit the voltage
        cutoff and frozen in place. No further movement or coverage gain is
        possible — keep stepping is pointless.
        """
        if not self.drones:
            return False
        cutoff = self.battery_cfg.cutoff_energy_j
        return all(d.battery_j <= cutoff for d in self.drones)

    def is_terminal(self) -> bool:
        """
        Combined terminal condition for driver loops: stop when either the
        mission is complete (100% coverage) or impossible (all drones depleted).
        Drivers should use this — `is_done()` alone misses the depleted case
        and would loop forever once batteries die.
        """
        return self.is_done() or self.all_depleted()

    def drone_coverage_fraction(self, drone_idx: int) -> float:
        """
        Fraction of free cells this specific drone has personally covered.
        Sum across drones can exceed `coverage_fraction()` due to overlap
        (multiple drones passing through the same area).
        """
        free_mask = self.grid == FREE
        n_free = int(free_mask.sum())
        if n_free == 0:
            return 1.0
        return float(self.drone_covered[drone_idx].sum()) / n_free

    def drone_coverage_m2(self, drone_idx: int) -> float:
        """Square meters of free territory drone `drone_idx` has personally covered."""
        cell_area_m2 = self.sim_cfg.meters_per_cell ** 2
        return float(self.drone_covered[drone_idx].sum()) * cell_area_m2

    def overlap_cells_m2(self) -> float:
        """
        Spatial-snapshot complement, in **square meters**: total area of free
        territory that has been touched by ≥ 2 drones at some point.
        Monotone; bounded by the total free area of the map.
        """
        cell_area_m2 = self.sim_cfg.meters_per_cell ** 2
        multi_covered = int((self.drone_covered.sum(axis=0) >= 2).sum())
        return multi_covered * cell_area_m2

    # -------- entry-count metrics (visit-level, not just set membership) --------

    def total_visits(self, drone_idx: int) -> int:
        """
        Total cell-entry events by this drone over the whole run. A "visit" is
        one transition from outside-disc to inside-disc for a single free cell.
        Hovering does not inflate this — cells already in the disc don't count
        again until the drone leaves and returns.
        """
        return int(self.entry_count[drone_idx].sum())

    def unique_cells_visited(self, drone_idx: int) -> int:
        """Distinct free cells this drone has ever entered."""
        return int((self.entry_count[drone_idx] > 0).sum())

    def self_revisits(self, drone_idx: int) -> int:
        """
        Times this drone re-entered a cell it had already visited. Equals
        ``total_visits(i) - unique_cells_visited(i)`` — every entry past the
        first one for a given cell is a self-revisit.
        """
        return self.total_visits(drone_idx) - self.unique_cells_visited(drone_idx)

    def cross_overlap_visits(self, drone_idx: int) -> int:
        """
        This drone's entries into cells some OTHER drone has visited at least
        once during the run. Symmetric across the full episode (we don't ask
        who entered first); each entry event by drone i into a cell that any
        other drone has touched contributes 1.

        Counted per-entry, not per-cell: if drone i enters a shared cell three
        times, that's 3 cross-overlap visits, not 1.
        """
        other_visited = np.zeros((self.h, self.w), dtype=bool)
        for j in range(self.n_drones):
            if j != drone_idx:
                other_visited |= self.entry_count[j] > 0
        return int(self.entry_count[drone_idx, other_visited].sum())

    def wasted_visits_total(self) -> int:
        """
        Swarm-wide count of entries that were not unique discoveries: every
        entry past the first one for any given cell, summed across all drones.
        Equals ``Σᵢ total_visits(i) − (free cells visited by anyone)``. Captures
        both self-revisits and cross-drone re-coverage in a single number.
        """
        total_entries = int(self.entry_count.sum())
        free_mask = self.grid == FREE
        unique_global = int((self.covered & free_mask).sum())
        return total_entries - unique_global

    # -------- observations (for future RL/PF use) --------

    def positions(self) -> np.ndarray:
        return np.stack([d.pos for d in self.drones])

    def velocities(self) -> np.ndarray:
        return np.stack([d.vel for d in self.drones])

    # -------- battery --------

    def battery_state(self, drone_idx: int = 0) -> dict:
        """Snapshot of one drone's energy budget in physical units."""
        cfg = self.battery_cfg
        d = self.drones[drone_idx]
        initial_j = cfg.initial_energy_j
        used_j = initial_j - d.battery_j
        # 1 mAh @ V volts = V × 3.6 J  (since 1 mAh = 3.6 C, and J = V·C).
        # So mAh_per_J is the inverse: divide joules by (V × 3.6) to get mAh.
        mah_per_j = 1.0 / (cfg.voltage_v * 3.6)
        return {
            "voltage_v_nominal": cfg.voltage_v,
            "voltage_v_current": cfg.current_voltage_v(d.battery_j),
            "min_voltage_v": cfg.min_voltage_v,
            "capacity_mah": cfg.capacity_mah,
            "initial_energy_j": initial_j,
            "remaining_energy_j": d.battery_j,
            "used_energy_j": used_j,
            "used_mah": used_j * mah_per_j,
            "remaining_mah": cfg.capacity_mah - used_j * mah_per_j,
            "percent_remaining": 100.0 * d.battery_j / initial_j if initial_j > 0 else 0.0,
            "depleted": d.battery_j <= cfg.cutoff_energy_j,
        }
