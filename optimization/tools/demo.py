"""
Smoke test for the swarm coverage testbed.

Run modes (mutually exclusive):
    python tools/demo.py                # headless (default)
    python tools/demo.py --headless     # explicit headless
    python tools/demo.py --gui          # live animation (MacOSX backend on darwin, TkAgg elsewhere)

Persisting outputs:
    Without --save: writes demo_initial.png / demo_final.png / coverage_curve.png
                    (these always overwrite — they are scratch test artifacts).
    With --save:    writes SAVED_<tag>_<name>.png so files accumulate instead of
                    overwriting. Tag defaults to a timestamp; pass --save NAME
                    to use a custom tag.

        python tools/demo.py --save                   # SAVED_20260426_180012_demo_initial.png
        python tools/demo.py --save baseline_random   # SAVED_baseline_random_demo_initial.png
        python tools/demo.py --gui --save run3        # also saves SAVED_run3_animation.gif
"""

from __future__ import annotations

import argparse
import sys
from datetime import datetime
from pathlib import Path

# Make the project root importable when running `python tools/demo.py` directly.
sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

import matplotlib.pyplot as plt
import numpy as np

from constants import (
    DEFAULT_GRID_SIZE,
    DEFAULT_MAP_KIND,
    DEFAULT_N_DRONES,
    DEFAULT_OBSTACLE_DENSITY,
    DEFAULT_SEED,
    IMAGES_DIR,
)
from environment import CoverageEnv, DroneConfig, SimConfig
from maze import load_map, random_obstacles, recursive_backtracker
from visualize import animate, init_drone_palette, render_frame


def _output_prefix(save_arg: str | None) -> str:
    """
    save_arg semantics:
        None  -> "" (overwrite scratch files)
        ""    -> timestamp tag (--save with no value)
        other -> use as the tag
    """
    if save_arg is None:
        return ""
    # "%Y%m%d_%H%M%S" → "20260426_180012" (YYYYMMDD_HHMMSS, alphabetically sortable
    # so saved-run filenames sort in chronological order in the directory listing).
    tag = save_arg if save_arg else datetime.now().strftime("%Y%m%d_%H%M%S")
    return f"SAVED_{tag}_"


def random_policy_with_hover(env: CoverageEnv) -> np.ndarray:
    """
    Drone 0 ALWAYS hovers (zero linear AND yaw acceleration → stays put with
    constant heading from reset onward). All other drones use a "yaw +
    brake → realign → accelerate forward" pattern that keeps the camera
    looking where the drone is actually moving.

    The hover drone is a permanent battery sanity check: its drain rate must
    equal P_hover exactly (= hover_power_w / (V·3.6) mAh/s), so any regression
    in the energy model shows up as a wrong depletion time on every demo run.

    **Why this is more involved than just "push along heading"**: the env has
    no atmospheric drag, so velocity persists until actively decelerated.
    If the drone yaws while moving, its momentum keeps carrying it in the
    OLD direction even though the camera now points elsewhere. To prevent
    that, we explicitly brake when velocity is misaligned with heading:

        if |v_dir − heading| < align_threshold  (or stationary):
            push forward along heading at random magnitude
        elif moving:
            brake — push opposite to velocity until either it slows below
                    move_threshold or yaw has caught up enough to be aligned
        else:
            no force (yaw will eventually realign; translation will resume)

    Effect: drone drifts forward when aligned, brakes when yaw outpaces
    motion, and resumes forward motion once realigned. The wedge always
    points within ~17° of actual motion.

    A different controller (Potential Fields, Consensus, MARL) is free to
    emit world-frame (ax, ay) and treat yaw independently — that's what the
    env actually accepts. This braking policy is a demo choice, not a
    physics constraint.

    Action shape is (n_drones, 3) = [ax_world, ay_world, alpha_yaw]. Seeding
    by step_count makes runs reproducible across re-renders.
    """
    rng = np.random.default_rng(env.step_count)
    actions = np.zeros((env.n_drones, 3), dtype=np.float64)

    velocities = env.velocities()
    speeds = np.linalg.norm(velocities, axis=1)
    headings = env.headings()

    # Velocity direction. atan2(0, 0) = 0 in numpy, which is fine — when the
    # drone is stationary the alignment check is meaningless and we fall
    # through to the "push forward along heading" branch via the moving
    # gate below.
    v_dir = np.arctan2(velocities[:, 1], velocities[:, 0])
    v_err = (v_dir - headings + np.pi) % (2 * np.pi) - np.pi  # in [-π, π]

    align_threshold = 0.3   # rad ≈ 17°. Within this, drone can push forward.
    move_threshold = 0.1    # cells/s. Below this, momentum is small enough to ignore.
    aligned = np.abs(v_err) < align_threshold
    moving = speeds > move_threshold

    # Branch A: "push forward along heading" with random magnitude. Used
    # whenever the drone is aligned with motion OR stationary (no momentum
    # to worry about).
    fwd_mag = rng.uniform(0.0, env.drone_cfg.max_accel, size=env.n_drones)
    forward_ax = fwd_mag * np.cos(headings)
    forward_ay = fwd_mag * np.sin(headings)

    # Branch B: "brake" — push opposite to current velocity at full magnitude
    # until momentum drops below move_threshold or yaw catches up. Safe-guard
    # for the speed=0 corner so the unit-vector division doesn't NaN.
    safe_speeds = np.maximum(speeds, 1e-9)
    brake_ax = -env.drone_cfg.max_accel * velocities[:, 0] / safe_speeds
    brake_ay = -env.drone_cfg.max_accel * velocities[:, 1] / safe_speeds

    # Apply the right branch per drone. If misaligned AND moving → brake;
    # otherwise (aligned OR stationary) → push forward.
    use_brake = (~aligned) & moving
    actions[:, 0] = np.where(use_brake, brake_ax, forward_ax)
    actions[:, 1] = np.where(use_brake, brake_ay, forward_ay)

    # Yaw drift — kept gentle (std = 0.2 × max_yaw_accel) so heading doesn't
    # outpace what the velocity loop can follow. Larger std would fight the
    # brake-and-realign pattern by yawing the drone away from velocity faster
    # than the brake can decelerate.
    actions[:, 2] = rng.normal(0.0, 0.2 * env.drone_cfg.max_yaw_accel, size=env.n_drones)

    actions[0] = 0.0  # hover-verify drone — no motion, no yaw
    return actions


def _positive_drone_count(s: str) -> int:
    n = int(s)
    if n < 1:
        raise argparse.ArgumentTypeError(
            "--drones must be at least 1 (the hover-verify drone is always present)"
        )
    return n


def _completion_reason(env: CoverageEnv) -> str:
    """Diagnose which natural terminal condition ended the run."""
    if env.is_done():
        return (
            f"COMPLETED — 100% coverage reached at step {env.step_count} "
            f"(t={env.time_seconds:.1f} s)"
        )
    if env.all_depleted():
        return (
            f"ALL DRONES DEPLETED — every battery hit cutoff before mission complete. "
            f"Final coverage: {env.coverage_fraction():.2%} at t={env.time_seconds:.1f} s"
        )
    # Should be unreachable: env.is_terminal() == is_done() or all_depleted(),
    # so the run-loop only exits when one of the above is True.
    return f"UNKNOWN TERMINAL STATE — step {env.step_count}, coverage {env.coverage_fraction():.2%}"


def _print_hover_sanity_check(env: CoverageEnv) -> None:
    """
    Drone 1 (index 0) has been hovering all run, so its drain rate must equal
    P_hover exactly. Print a pass/fail compared to the theoretical prediction.
    """
    cfg = env.battery_cfg
    b = env.battery_state(0)
    p_hover = cfg.hover_power_w
    print()
    print("=== Battery sanity check (Drone 1, always hovering) ===")
    if b["depleted"]:
        # Drone 1 hit the cutoff. Predicted depletion time = spendable / P_hover.
        spendable_j = cfg.initial_energy_j - cfg.cutoff_energy_j
        expected_t = spendable_j / p_hover
        actual_t = env.time_seconds  # is_terminal fires at all-depleted, drone 1 is last
        delta_pct = abs(actual_t - expected_t) / expected_t * 100
        ok = delta_pct < 1.0
        print(f"  Theoretical hover endurance to cutoff:  {expected_t:7.1f} s   "
              f"(spendable {spendable_j:.0f} J ÷ P_hover {p_hover} W)")
        print(f"  Drone 1 actual depletion time:          {actual_t:7.1f} s   "
              f"({'✓ MATCH' if ok else '✗ MISMATCH'} — Δ = {delta_pct:.2f}%)")
    else:
        # Sim ended before drone 1 depleted (e.g., 100% coverage reached early).
        # Compare its energy spent to what hover should have spent in elapsed time.
        expected_j = p_hover * env.time_seconds
        delta_pct = abs(b["used_energy_j"] - expected_j) / expected_j * 100 if expected_j > 0 else 0
        ok = delta_pct < 1.0
        print(f"  Theoretical hover energy in {env.time_seconds:.1f} s:  {expected_j:8.1f} J")
        print(f"  Drone 1 actual energy used:           {b['used_energy_j']:8.1f} J   "
              f"({'✓ MATCH' if ok else '✗ MISMATCH'} — Δ = {delta_pct:.2f}%)")


def _print_final_batteries(env: CoverageEnv) -> None:
    print()
    print("=== Per-drone final stats (battery + coverage) ===")
    for i in range(env.n_drones):
        b = env.battery_state(i)
        status = " [DEPLETED]" if b["depleted"] else ""
        cov_m2 = env.drone_coverage_m2(i)
        cov_pct = 100.0 * env.drone_coverage_fraction(i)
        print(
            f"  Drone {i + 1}:  spent {b['used_mah']:7.2f} mAh "
            f"({100 - b['percent_remaining']:5.2f}%)  "
            f"V={b['voltage_v_current']:5.2f}  "
            f"covered {cov_m2:6.0f} m² ({cov_pct:5.2f}% of free area){status}"
        )
    sum_pct = sum(100.0 * env.drone_coverage_fraction(i) for i in range(env.n_drones))
    global_pct = 100.0 * env.coverage_fraction()
    overlap_area_m2 = env.overlap_cells_m2()
    print(
        f"  swarm:   sum-of-individuals = {sum_pct:6.2f}%   "
        f"global = {global_pct:6.2f}%"
    )
    print(
        f"           overlap area    = {overlap_area_m2:9.0f} m²   "
        f"(territory touched by ≥2 drones)"
    )


def main() -> None:
    parser = argparse.ArgumentParser()
    mode = parser.add_mutually_exclusive_group()
    mode.add_argument("--headless", action="store_true", help="run without GUI (default)")
    mode.add_argument("--gui", action="store_true", help="open live animation window")
    parser.add_argument(
        "--save",
        nargs="?",
        const="",
        default=None,
        metavar="TAG",
        help="persist outputs as SAVED_<tag>_*.png (tag defaults to a timestamp)",
    )
    parser.add_argument("--map", choices=["random", "maze"], default=DEFAULT_MAP_KIND)
    parser.add_argument("--map-file", type=str, default=None, help="load custom map from .npy/.txt")
    parser.add_argument("--size", type=int, default=DEFAULT_GRID_SIZE)
    parser.add_argument("--drones", type=_positive_drone_count, default=DEFAULT_N_DRONES,
                        help="total drones in the swarm (≥ 1). Drone 1 always hovers as a "
                             "battery sanity check; the rest run the random policy.")
    parser.add_argument("--seed", type=int, default=DEFAULT_SEED)
    # NOTE: there is intentionally no --steps flag and no step-count safety cap.
    # The sim runs until env.is_terminal(), which fires on either:
    #   - 100% coverage (mission success)
    #   - all drones depleted (mission failure, batteries dead)
    # The all-depleted condition is a *physics-bounded* stop: max sim time is
    # (initial_energy − cutoff_energy) / P_hover ≈ 121 kJ / 165 W ≈ 734 s ≈
    # 7340 steps. Past that point every drone has hit the voltage cutoff, so
    # is_terminal() always fires. No artificial step cap needed.
    args = parser.parse_args()

    prefix = _output_prefix(args.save)

    if args.map_file:
        grid = load_map(args.map_file)
        args.map = f"file:{args.map_file}"
    elif args.map == "random":
        grid = random_obstacles(args.size, density=DEFAULT_OBSTACLE_DENSITY, seed=args.seed)
    else:
        grid = recursive_backtracker(args.size, seed=args.seed)

    env = CoverageEnv(
        grid=grid,
        n_drones=args.drones,
        sim=SimConfig(step_seconds=0.1),
        drone=DroneConfig(sensor_range=1.6, max_speed=1.5, max_accel=2.5),
    )
    env.reset(seed=args.seed)
    # Random palette rotation each run (seed=None = OS entropy). Color choice
    # only affects rendering — env.step() doesn't touch it.
    init_drone_palette(args.drones, seed=None)

    mpc = env.sim_cfg.meters_per_cell
    h, w = grid.shape
    n_free = int((grid == 0).sum())
    print(f"map: {args.map}  size: {grid.shape} cells = {w * mpc:.1f} × {h * mpc:.1f} m  "
          f"({mpc:.1f} m/cell)  free cells: {n_free} (~{n_free * mpc * mpc:.0f} m²)")
    print(f"drones: {args.drones} × {env.drone_cfg.mass_kg:.1f} kg  "
          f"max speed {env.drone_cfg.max_speed * mpc:.1f} m/s  "
          f"max accel {env.drone_cfg.max_accel * mpc:.1f} m/s²")
    print(f"  Drone 1 ALWAYS hovers (battery sanity check). "
          f"Drones 2..{args.drones} run random policy.")
    print(f"initial coverage: {env.coverage_fraction():.2%}")

    b0 = env.battery_state(0)
    print()
    print("=== Battery (Hawk's Work F450 reference: 11.1V 4200mAh 3S) ===")
    # Two voltages live on the same pack: nominal (sales-label, used for the
    # E = V·mAh·3.6 energy calc) and full-charge (what a multimeter reads off
    # the charger). 3S LiPo: 11.1 V nominal, 12.6 V at 100%, 9.0 V at 0%.
    print(f"  pack: {b0['voltage_v_nominal']:.1f}V nominal  "
          f"(full {env.battery_cfg.cell_full_voltage_v * env.battery_cfg.n_cells:.1f}V → "
          f"empty {env.battery_cfg.cell_dead_voltage_v * env.battery_cfg.n_cells:.1f}V)  "
          f"{b0['capacity_mah']:.0f} mAh  ({b0['initial_energy_j']:.0f} J)")
    print(f"  starting voltage (all drones): {b0['voltage_v_current']:.2f}V (full charge)  "
          f"cutoff: {b0['min_voltage_v']:.1f}V")

    if args.gui:
        gif_path = None
        if args.save is not None:
            IMAGES_DIR.mkdir(parents=True, exist_ok=True)
            gif_path = str(IMAGES_DIR / f"{prefix}animation.gif")
        animate(env, random_policy_with_hover, interactive=True, save_path=gif_path)
        print(f"\n*** {_completion_reason(env)} ***")
        _print_final_batteries(env)
        _print_hover_sanity_check(env)
        if gif_path:
            print(f"saved animation: {gif_path}")
        return

    IMAGES_DIR.mkdir(parents=True, exist_ok=True)
    initial = IMAGES_DIR / f"{prefix}demo_initial.png"
    final = IMAGES_DIR / f"{prefix}demo_final.png"
    curve = IMAGES_DIR / f"{prefix}coverage_curve.png"

    render_frame(env, save_path=str(initial))

    history = [env.coverage_fraction()]
    while not env.is_terminal():
        env.step(random_policy_with_hover(env))
        history.append(env.coverage_fraction())
    print(f"\n*** {_completion_reason(env)} ***")

    render_frame(env, save_path=str(final))

    fig, ax = plt.subplots(figsize=(6, 4))
    ax.plot(history)
    ax.set_xlabel("step")
    ax.set_ylabel("coverage fraction")
    ax.set_title(f"random-policy coverage on {args.map} map ({args.drones} drones)")
    ax.grid(True, alpha=0.3)
    fig.savefig(str(curve), dpi=120, bbox_inches="tight")

    _print_final_batteries(env)
    _print_hover_sanity_check(env)

    mode_label = "saved" if args.save is not None else "scratch (overwritten next run)"
    print()
    print(f"wrote [{mode_label}]: {initial.name}, {final.name}, {curve.name}")
    print(f"  in: {IMAGES_DIR}")


if __name__ == "__main__":
    main()
