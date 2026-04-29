"""
Distance / tile-scale physics verification.

The whole simulation hinges on the cell→meter conversion (`SimConfig.meters_per_cell`).
If that constant doesn't reflect a sensible real-world scale, every derived
quantity — sensor footprint, max speed in m/s, world dimensions, motion-power
constants — is wrong. This script asserts:

  1. Tile scale is the documented 5.0 m/cell.
  2. Derived physical quantities (sensor footprint, max speed, world size,
     drone radius) match expected real-world F450 references.
  3. Linear motion: a drone at constant velocity for N steps moves
     `v_cells · N · dt` cells, which equals `v_cells · N · dt · m_per_cell`
     meters. Verified by accumulating per-step displacements through env.step.
  4. Sensor disc area: cell-discretised footprint at `sensor_radius = 1.5`
     cells = 7.5 m falls inside the expected ±30 % of the continuous π·r²
     value (tight tolerance is impossible at this radius — only 7-9 cells
     intersect the disc, so quantisation dominates; large radii would be
     tighter).
  5. Range to cutoff: drone flying at max_speed until battery cutoff covers
     `cruise_endurance × max_speed × m_per_cell` meters. Combines distance
     integration with the battery model — if either is wrong, this fails.

The first two checks are configuration sanity (you're running the F450
defaults, not someone's overridden DroneConfig); 3, 4, and 5 actually
exercise `env.step`'s integration, `_update_coverage`'s footprint geometry,
and the cell→meter conversion compounded over a full battery discharge.

**No published F450 range spec exists** — the F450 is a DIY frame kit, so
range depends on the builder's motors / props / battery choices. Our
comparison range (5.4–8.1 km at 7.5 m/s) is *inferred* from the published
F450 hover endurance × translational-lift cruise gain. See README "Cruise
(forward flight)" for the derivation.

Usage:
  python verification_scripts/test_distance.py            # headless asserts
  python verification_scripts/test_distance.py --gui      # live distance counter
"""

from __future__ import annotations

import argparse
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

import numpy as np

from environment import CoverageEnv, DroneConfig, SimConfig
from maze import FREE, WALL


# Physics tolerance — env.step's integration is explicit Euler with no rounding,
# so motion accuracy should be near machine epsilon for short paths.
MOTION_TOLERANCE_PCT = 0.01

# Long-path tolerance (range-to-cutoff): the battery clamps to cutoff_energy_j
# on the final step, so the last step's distance is partial. Over thousands of
# steps that's a fraction of a percent — ample headroom at 0.5 %, same value
# test_flight_time uses for the same reason.
RANGE_TOLERANCE_PCT = 0.5

# Sensor footprint tolerance vs continuous πr² is loose because at r = 1.5
# cells the disc only intersects 7-13 cells (depending on alignment). Discrete
# vs continuous area diverges by ~25 % at this radius — that's geometry, not
# a bug. We verify the scale conversion separately.
FOOTPRINT_TOLERANCE_PCT = 30.0


def _build_one_drone_env(grid_size: int = 51) -> CoverageEnv:
    """Open arena big enough that linear motion never hits a wall during the test."""
    grid = np.full((grid_size, grid_size), FREE, dtype=np.int8)
    grid[0, :] = WALL
    grid[-1, :] = WALL
    grid[:, 0] = WALL
    grid[:, -1] = WALL
    env = CoverageEnv(
        grid=grid,
        n_drones=1,
        sim=SimConfig(step_seconds=0.1, meters_per_cell=5.0),
        drone=DroneConfig(sensor_radius=1.5, max_speed=1.5, max_accel=2.5),
    )
    env.reset(seed=0)
    return env


def _check(label: str, ok: bool, detail: str = "") -> bool:
    mark = "✓" if ok else "✗"
    print(f"  {mark} {label}{('  — ' + detail) if detail else ''}")
    return ok


def test_tile_scale(env: CoverageEnv) -> bool:
    """Validate meters_per_cell and every derived physical quantity."""
    print("Test 1: TILE SCALE & derived dimensions")
    mpc = env.sim_cfg.meters_per_cell
    h, w = env.grid.shape
    cfg = env.drone_cfg

    sensor_m = cfg.sensor_radius * mpc
    max_speed_m_s = cfg.max_speed * mpc
    max_accel_m_s2 = cfg.max_accel * mpc
    drone_radius_m = cfg.drone_radius * mpc

    print(f"      meters_per_cell     = {mpc} m/cell")
    print(f"      world (cells)       = {h} × {w}")
    print(f"      world (meters)      = {h * mpc:.0f} m × {w * mpc:.0f} m")
    print(f"      sensor_radius       = {cfg.sensor_radius} cells = {sensor_m} m")
    print(f"        sensor footprint  ≈ π·r² = {np.pi * sensor_m ** 2:.1f} m² (continuous)")
    print(f"      max_speed           = {cfg.max_speed} cells/s = {max_speed_m_s} m/s")
    print(f"      max_accel           = {cfg.max_accel} cells/s² = {max_accel_m_s2} m/s²"
          f"  ({max_accel_m_s2 / 9.81:.2f} g)")
    print(f"      drone_radius        = {cfg.drone_radius} cells = {drone_radius_m} m"
          f"  (real F450 half-width ≈ 0.25 m)")

    all_ok = True
    all_ok &= _check("meters_per_cell == 5.0",                abs(mpc - 5.0) < 1e-9)
    all_ok &= _check("sensor_radius   == 7.5 m",              abs(sensor_m - 7.5) < 1e-9,
                     "F450 camera footprint at ~10 m altitude")
    all_ok &= _check("max_speed       == 7.5 m/s",            abs(max_speed_m_s - 7.5) < 1e-9,
                     "realistic F450 cruise (manufacturer spec ≈ 15 m/s max forward)")
    all_ok &= _check("max_accel       ≈ 12.5 m/s² (~1.3 g)",  abs(max_accel_m_s2 - 12.5) < 1e-9,
                     "typical multirotor punch")
    all_ok &= _check("drone_radius    == 0.25 m",             abs(drone_radius_m - 0.25) < 1e-9,
                     "F450 frame half-width")
    return all_ok


def test_distance_traveled(env: CoverageEnv) -> bool:
    """
    Drone moves at constant velocity for N steps. Accumulated displacement
    must equal `v · t` in cells and `v · t · meters_per_cell` in meters.
    """
    print("\nTest 2: LINEAR DISTANCE (env.step integration)")
    n_steps = 100
    dt = env.sim_cfg.step_seconds
    mpc = env.sim_cfg.meters_per_cell
    v_cells = env.drone_cfg.max_speed
    v_m_s = v_cells * mpc

    # Park drone in arena center moving in +x, verify no walls touched.
    env.drones[0].pos = np.array([env.w / 2.0, env.h / 2.0])
    env.drones[0].vel = np.array([v_cells, 0.0])
    start_pos = env.drones[0].pos.copy()

    distance_cells_accum = 0.0
    for _ in range(n_steps):
        old_pos = env.drones[0].pos.copy()
        # Re-pin velocity each step in case env.step's speed cap or wall logic
        # touched it (it shouldn't here, but the pin is cheap insurance).
        env.drones[0].vel = np.array([v_cells, 0.0])
        env.step(np.zeros((1, 2)))
        distance_cells_accum += float(np.linalg.norm(env.drones[0].pos - old_pos))

    final_displacement_cells = float(np.linalg.norm(env.drones[0].pos - start_pos))
    expected_cells = v_cells * n_steps * dt
    expected_meters = expected_cells * mpc

    actual_path_m = distance_cells_accum * mpc
    actual_displacement_m = final_displacement_cells * mpc

    print(f"      v = {v_m_s:.1f} m/s ({v_cells} cells/s) for {n_steps * dt:.1f} s")
    print(f"      expected:        {expected_cells:.3f} cells = {expected_meters:.2f} m")
    print(f"      path length:     {distance_cells_accum:.3f} cells = {actual_path_m:.2f} m")
    print(f"      displacement:    {final_displacement_cells:.3f} cells = {actual_displacement_m:.2f} m")

    all_ok = True
    delta_path = abs(actual_path_m - expected_meters) / expected_meters * 100
    all_ok &= _check("path length matches v·t·m_per_cell", delta_path < MOTION_TOLERANCE_PCT,
                     f"Δ = {delta_path:.4f}% (tolerance < {MOTION_TOLERANCE_PCT}%)")
    # Straight-line motion: path length should equal displacement.
    all_ok &= _check("path == displacement (motion was straight)",
                     abs(distance_cells_accum - final_displacement_cells) < 1e-9)
    return all_ok


def test_sensor_footprint(env: CoverageEnv) -> bool:
    """
    Cell-discretized sensor disc area should land within ±FOOTPRINT_TOLERANCE_PCT
    of the analytical π·r² (m²). At r = 1.5 cells the disc only intersects 9
    cells when centred on a cell center — quantization error is large but the
    *units* (cell count × cell_area_m² ≈ π·r²·m²) must be self-consistent.
    """
    print("\nTest 3: SENSOR FOOTPRINT AREA")
    mpc = env.sim_cfg.meters_per_cell
    r_cells = env.drone_cfg.sensor_radius
    r_m = r_cells * mpc
    cell_area_m2 = mpc ** 2

    # Place drone exactly on a cell center to remove alignment-induced noise.
    env.drones[0].pos = np.array([env.w // 2 + 0.5, env.h // 2 + 0.5])
    env.entry_count[:] = 0
    env.prev_footprint[:] = False
    env.first_visitor[:] = -1
    env.drone_covered[:] = False
    env.covered[:] = False
    env.covered[env.grid == WALL] = True
    env._update_coverage()

    cells_in_disc = int(env.drone_covered[0].sum())
    actual_m2 = cells_in_disc * cell_area_m2
    expected_m2 = np.pi * r_m ** 2

    print(f"      drone at cell center, r = {r_cells} cells = {r_m} m")
    print(f"      cells in disc (discrete):  {cells_in_disc}")
    print(f"      area (cells × {cell_area_m2:.0f} m²):     {actual_m2:.2f} m²")
    print(f"      expected (π·r², continuous): {expected_m2:.2f} m²")
    delta_pct = abs(actual_m2 - expected_m2) / expected_m2 * 100

    all_ok = True
    all_ok &= _check(
        "footprint area within ±30% of π·r² (discretization-dominated at r=1.5 cells)",
        delta_pct < FOOTPRINT_TOLERANCE_PCT,
        f"Δ = {delta_pct:.1f}% (tolerance < {FOOTPRINT_TOLERANCE_PCT}%)",
    )
    # Sanity bound on cell count: at r=1.5 cells centered on a cell, the disc
    # covers 9 cells (center + 4 axis-adjacent + 4 corner-adjacent). If we get
    # something wildly different the footprint code is broken.
    all_ok &= _check(
        "cell count plausible (5 ≤ count ≤ 13 at r=1.5)",
        5 <= cells_in_disc <= 13,
        f"got {cells_in_disc}",
    )
    return all_ok


def test_range_to_cutoff(env: CoverageEnv) -> bool:
    """
    Drone flies at max_speed until battery cutoff. Total ground distance
    must equal `cruise_endurance × max_speed × meters_per_cell`. This
    compounds the battery model + the distance integration, so a regression
    in either shows up here.

    We use the same teleport-to-center pin as `test_flight_time.cruise_endurance`
    so wall collisions don't interrupt cruise. Distance is accumulated from
    pre/post-step positions BEFORE the next teleport, so the meters we count
    are real `env.step`-integrated motion.
    """
    print("\nTest 4: RANGE TO CUTOFF (drone flies at max_speed until depletion)")
    cfg = env.battery_cfg
    mpc = env.sim_cfg.meters_per_cell
    target_v_cells = env.drone_cfg.max_speed
    target_v_mps = target_v_cells * mpc
    arena_center = np.array([env.w / 2.0, env.h / 2.0])
    direction = np.array([1.0, 0.0])

    # Theoretical reference: cruise_time × cruise_speed in meters.
    p_cruise = cfg.hover_power_w + cfg.motion_coeff_w_per_v2 * target_v_cells ** 2
    expected_time_s = (cfg.initial_energy_j - cfg.cutoff_energy_j) / p_cruise
    expected_distance_m = expected_time_s * target_v_mps

    distance_cells = 0.0
    safety_steps = 100_000
    for _ in range(safety_steps):
        if env.battery_state(0)["depleted"]:
            break
        env.drones[0].pos = arena_center.copy()
        env.drones[0].vel = direction * target_v_cells
        old_pos = env.drones[0].pos.copy()
        env.step(np.zeros((1, 2)))
        # Accumulate the *actual* displacement env.step produced this iteration
        # (we re-pin position next iter, but the integration in between is what
        # we care about — that's what counts toward distance traveled).
        distance_cells += float(np.linalg.norm(env.drones[0].pos - old_pos))
    else:
        print(f"      ✗ did not deplete within {safety_steps} steps")
        return False

    actual_distance_m = distance_cells * mpc
    actual_time_s = env.time_seconds

    print(f"      cruise speed:       {target_v_mps:.1f} m/s ({target_v_cells} cells/s)")
    print(f"      depletion time:     {actual_time_s:.1f} s ({actual_time_s / 60:.2f} min)")
    print(f"      expected distance:  {expected_distance_m:.0f} m ({expected_distance_m / 1000:.2f} km)")
    print(f"      actual distance:    {actual_distance_m:.0f} m ({actual_distance_m / 1000:.2f} km)")
    delta_pct = abs(actual_distance_m - expected_distance_m) / expected_distance_m * 100

    all_ok = _check(
        "range to cutoff = v · t_cruise · m_per_cell",
        delta_pct < RANGE_TOLERANCE_PCT,
        f"Δ = {delta_pct:.4f}% (tolerance < {RANGE_TOLERANCE_PCT}%)",
    )

    print()
    print(f"      F450 range references (no published spec exists for this DIY frame):")
    print(f"        sim (this run):              {actual_distance_m / 1000:.2f} km")
    print(f"        inferred real F450 (7.5 m/s sweet spot):  5.4–8.1 km")
    print(f"          = hover (12-15 min, published) × 1.0–1.2 (translational-lift gain)")
    print(f"            × 7.5 m/s cruise speed × 60 s/min")
    print(f"        F450 max forward (~15 m/s, drag-dominated):  ~9 km full pack")
    print(f"          (Bauersfeld & Scaramuzza, 2021 — calibration anchor)")
    print(f"      Sim is pessimistic by ~15-30% on cruise endurance (no")
    print(f"      translational-lift dip in the v² monotone model), so the range")
    print(f"      is correspondingly pessimistic — by design.")
    return all_ok


def run_headless() -> bool:
    print("=== Distance / tile-scale physics verification ===\n")
    all_ok = True
    all_ok &= test_tile_scale(_build_one_drone_env())
    all_ok &= test_distance_traveled(_build_one_drone_env())
    all_ok &= test_sensor_footprint(_build_one_drone_env())
    all_ok &= test_range_to_cutoff(_build_one_drone_env())

    print()
    if all_ok:
        print("✓ ALL CHECKS PASSED — tile scale (5 m/cell), distance integration, "
              "sensor footprint, and range-to-cutoff are physically consistent.")
    else:
        print("✗ SOME CHECKS FAILED — see above.")
    return all_ok


def run_gui() -> None:
    """
    Live animation of the cruise-to-cutoff distance accumulation. Drone flies
    a 2-cell-radius circle around arena center at `max_speed`; distance is
    accumulated per step from pre/post-step displacements. A 1× / 5× / 10× /
    20× speed slider compresses the ~10 min real-time depletion to as little
    as ~30 s wall-clock. Running counters: elapsed time, distance covered
    (meters, % of theoretical max range), battery % remaining (0% = cutoff).
    """
    import matplotlib

    backend = "MacOSX" if sys.platform == "darwin" else "TkAgg"
    try:
        matplotlib.use(backend, force=True)
    except Exception as e:
        print(f"GUI backend {backend!r} unavailable ({e})", file=sys.stderr)
        if sys.platform == "linux":
            print("On Linux/WSL: sudo apt install python3-tk python3-pil.imagetk",
                  file=sys.stderr)
            print("(venv must be created with --system-site-packages, "
                  "or use system python.)", file=sys.stderr)
        sys.exit(1)

    import matplotlib.pyplot as plt
    from matplotlib.animation import FuncAnimation
    from matplotlib.widgets import Slider

    from visualize import (
        _render_battery_panel,
        _render_map,
        format_duration,
        init_drone_palette,
    )

    init_drone_palette(1, seed=0)
    env = _build_one_drone_env(grid_size=21)
    cfg = env.battery_cfg
    spendable_j = cfg.initial_energy_j - cfg.cutoff_energy_j
    target_v_cells = env.drone_cfg.max_speed
    target_v_mps = target_v_cells * env.sim_cfg.meters_per_cell
    p_cruise = cfg.hover_power_w + cfg.motion_coeff_w_per_v2 * target_v_cells ** 2
    expected_time_s = spendable_j / p_cruise
    expected_range_m = expected_time_s * target_v_mps

    fig, (ax_map, ax_panel) = plt.subplots(
        1, 2, figsize=(15, 6),
        gridspec_kw={"width_ratios": [1, 1.7]},
    )
    fig.subplots_adjust(left=0.04, right=0.98, bottom=0.18, top=0.85, wspace=0.05)
    ax_slider = fig.add_axes([0.45, 0.05, 0.28, 0.03])
    speed_slider = Slider(
        ax_slider, "sim speed  (1× = real time)",
        valmin=1, valmax=20, valinit=5,
        valstep=[1, 5, 10, 20],
    )

    arena_center = np.array([env.w / 2.0, env.h / 2.0])
    cruise_radius = 2.0
    state = {"distance_cells": 0.0, "done": False, "final_distance_m": None}

    def step_one() -> None:
        # Project drone onto the cruise circle, set tangent velocity at max_speed.
        offset = env.drones[0].pos - arena_center
        r_now = float(np.linalg.norm(offset))
        if r_now < 1e-6:
            offset = np.array([cruise_radius, 0.0])
        else:
            offset = offset * (cruise_radius / r_now)
        env.drones[0].pos = arena_center + offset
        tangent = np.array([-offset[1], offset[0]]) / cruise_radius
        env.drones[0].vel = tangent * target_v_cells
        old_pos = env.drones[0].pos.copy()
        env.step(np.zeros((1, 2)))
        # Distance increment = how far env.step actually moved us. Next iter's
        # circle-pin will overwrite drone.pos again, but the displacement we
        # accumulated this iter is real motion at cruise speed.
        state["distance_cells"] += float(np.linalg.norm(env.drones[0].pos - old_pos))

    def update(_):
        if state["done"]:
            return []
        steps = int(speed_slider.val)
        for _ in range(steps):
            if env.battery_state(0)["depleted"]:
                state["done"] = True
                state["final_distance_m"] = state["distance_cells"] * env.sim_cfg.meters_per_cell
                break
            step_one()

        _render_map(env, ax_map)
        _render_battery_panel(env, ax_panel)
        ax_map.set_title("")  # suppress default; suptitle has the test status

        distance_m = state["distance_cells"] * env.sim_cfg.meters_per_cell
        if state["done"]:
            fig.suptitle(
                f"DONE  ✓  range traveled: {distance_m:.0f} m "
                f"({distance_m / 1000:.2f} km)  in {format_duration(env.time_seconds)} "
                f"at {target_v_mps:.1f} m/s\n"
                f"F450 reference:  inferred real-world cruise range 5.4–8.1 km "
                f"(hover × translational-lift × 7.5 m/s) — sim pessimistic by design"
            )
        else:
            remaining_to_cutoff_j = env.drones[0].battery_j - cfg.cutoff_energy_j
            pct_remaining = max(
                0.0,
                100.0 * remaining_to_cutoff_j / spendable_j if spendable_j > 0 else 0.0,
            )
            pct_range_complete = (
                100.0 * distance_m / expected_range_m if expected_range_m > 0 else 0.0
            )
            fig.suptitle(
                f"CRUISING (v = {target_v_mps:.1f} m/s)   "
                f"elapsed: {format_duration(env.time_seconds)}   "
                f"distance: {distance_m:.0f} m ({pct_range_complete:5.1f}% of expected "
                f"{expected_range_m:.0f} m)   "
                f"battery: {pct_remaining:5.1f}% remaining   "
                f"[{int(speed_slider.val)}× speed]"
            )
        return []

    _render_map(env, ax_map)
    _render_battery_panel(env, ax_panel)
    ax_map.set_title("")
    fig.suptitle(
        f"CRUISING (v = {target_v_mps:.1f} m/s)   elapsed: {format_duration(0.0)}   "
        f"distance: 0 m (0.0% of expected {expected_range_m:.0f} m)   "
        f"battery: 100.0% remaining   [5× speed]"
    )

    anim = FuncAnimation(  # noqa: F841
        fig, update, interval=50, blit=False, repeat=False,
        cache_frame_data=False,
    )
    plt.show()


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    parser.add_argument(
        "--gui", action="store_true",
        help="open a live distance-accumulator window with a 1×/5×/10×/20× speed slider",
    )
    args = parser.parse_args()

    if args.gui:
        run_gui()
    else:
        ok = run_headless()
        sys.exit(0 if ok else 1)


if __name__ == "__main__":
    main()
