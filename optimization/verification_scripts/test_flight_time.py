"""
Flight-time physics verification.

Asserts that ``env.step()``'s battery drain reproduces realistic F450 flight
times — both against the analytical battery model (P = P_hover + k·v²) and
against the published real-world hover-time range for an F450 with 3S 4200 mAh.

Two regimes, one drone in each:

  1. **Hover** (v = 0):     drain = P_hover only.
  2. **Cruise** (v = max_speed = 1.5 cells/s = 7.5 m/s): drain = P_hover + k·v².

For the cruise regime we keep the drone in a known velocity state by
teleporting it back to the arena center each step before passing zero
acceleration to ``env.step()`` — that makes ``new_vel = old_vel + 0 = target``
and avoids wall-collision velocity zeroing. Coverage tracking still runs but
isn't asserted (different verification).

Reference flight times (from real-world sources, see optimization/README.md
"Reference F450 flight times"):

  Battery       | Payload | Hover time
  3S 5000 mAh   | 1.8 kg  | ~15 min   (with gimbal + video tx, not pushed)
  3S 5000 mAh   | light   | ~18 min   (E300 motors, calm flying)
  3S 4200 mAh   | 1.9 kg  | 12-15 min (interpolated for our config)
  3S 2700 mAh   | 1.0 kg  | 12+ min   (no gimbal)
  3S 2200 mAh   | light   | ~10 min

Our model (P_hover = 165 W, 10V cutoff at ~28% remaining): 17.0 min to empty
pack, 12.2 min to cutoff. The 12.2 min value sits inside the published 12-15 min
hover range for the 3S 4200 mAh + 1.9 kg payload config — physics is realistic.

Usage:  python verification_scripts/test_flight_time.py
"""

from __future__ import annotations

import argparse
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

import numpy as np

from environment import BatteryConfig, CoverageEnv, DroneConfig, SimConfig
from maze import FREE, WALL


# Tolerance vs analytical formula. env.step's drain uses the same formula, so
# agreement should be near machine epsilon — but the cutoff-clamp on the final
# step can leave us up to one dt off, so 0.5% gives plenty of headroom.
TOLERANCE_PCT = 0.5


def _build_one_drone_env() -> CoverageEnv:
    """Open arena, big enough that wall collisions are impossible at our scales."""
    H, W = 21, 21
    grid = np.full((H, W), FREE, dtype=np.int8)
    grid[0, :] = WALL
    grid[-1, :] = WALL
    grid[:, 0] = WALL
    grid[:, -1] = WALL

    env = CoverageEnv(
        grid=grid,
        n_drones=1,
        sim=SimConfig(step_seconds=0.1),
        drone=DroneConfig(sensor_range=1.6, max_speed=1.5, max_accel=2.5),
        battery=BatteryConfig(),  # F450 reference defaults
    )
    env.reset(seed=0)
    # Park the drone in the geometric center so cruise teleports stay legal.
    env.drones[0].pos = np.array([W / 2.0, H / 2.0])
    return env


def hover_endurance_seconds(env: CoverageEnv) -> float:
    """Drone stays at v=0; run until voltage cutoff. Returns sim time elapsed."""
    # Pin heading + yaw_rate so the visual is consistent (camera doesn't drift
    # while hovering). Energy model is heading-independent so this is purely
    # cosmetic for any saved frames / GUI.
    env.drones[0].heading = 0.0
    env.drones[0].yaw_rate = 0.0
    safety_steps = 100_000  # ~10,000 sim seconds — far above any realistic depletion
    for _ in range(safety_steps):
        if env.battery_state(0)["depleted"]:
            return env.time_seconds
        env.step(np.zeros((1, 2)))
    raise RuntimeError(f"Hover sim did not deplete within {safety_steps} steps")


def cruise_endurance_seconds(env: CoverageEnv, target_speed_cells: float) -> float:
    """
    Drone flies at constant ``target_speed_cells`` (cells/s). Each iteration
    we teleport back to arena center and pin velocity, so wall collisions
    don't zero v and the drain rate stays at the cruise-power level for the
    entire run. Heading is pinned to the velocity direction so the camera
    looks where the drone is moving (matches the demo's policy).
    """
    arena_center = np.array([env.w / 2.0, env.h / 2.0])
    direction = np.array([1.0, 0.0])
    heading = float(np.arctan2(direction[1], direction[0]))
    safety_steps = 100_000

    for _ in range(safety_steps):
        if env.battery_state(0)["depleted"]:
            return env.time_seconds
        # Pin state before step so env.step computes new_vel = vel + 0·dt = target,
        # and heading stays fixed (yaw_rate=0 means env.step doesn't drift it).
        env.drones[0].pos = arena_center.copy()
        env.drones[0].vel = direction * target_speed_cells
        env.drones[0].heading = heading
        env.drones[0].yaw_rate = 0.0
        env.step(np.zeros((1, 2)))
    raise RuntimeError(f"Cruise sim did not deplete within {safety_steps} steps")


def _expected_seconds(env: CoverageEnv, speed_cells: float) -> float:
    """Theoretical depletion time at constant `speed_cells`: spendable energy / P(v)."""
    cfg = env.battery_cfg
    spendable = cfg.initial_energy_j - cfg.cutoff_energy_j
    p_inst = cfg.hover_power_w + cfg.motion_coeff_w_per_v2 * speed_cells * speed_cells
    return spendable / p_inst


def _check(label: str, actual_s: float, expected_s: float) -> bool:
    delta_pct = abs(actual_s - expected_s) / expected_s * 100
    ok = delta_pct < TOLERANCE_PCT
    mark = "✓" if ok else "✗"
    print(f"  {mark} {label}")
    print(f"      actual:    {actual_s:7.2f} s  ({actual_s / 60:5.2f} min)")
    print(f"      expected:  {expected_s:7.2f} s  ({expected_s / 60:5.2f} min)")
    print(f"      Δ:         {delta_pct:.3f}%  (tolerance < {TOLERANCE_PCT}%)")
    return ok


def run_headless() -> bool:
    """Original CLI flow — runs both regimes to depletion, asserts, returns True iff ok."""
    print("=== F450 flight-time physics verification ===\n")

    # Print the reference numbers we're going to compare to so the assertion
    # output reads cleanly without hunting for them in the docstring.
    print("F450 supported batteries (per DJI ARF spec):")
    print("  3S LiPo (11.1V nominal): 2200/2700/4000/4200/5000 mAh, with 10\" props (stock)")
    print("  4S LiPo (14.8V nominal): 3300/4000/6000 mAh, REQUIRES 8\" props (motors overheat at 4S+10\")")
    print()
    print("Published F450 hover-time references (real-world):")
    print("  4S 6000 mAh,  + GoPro:           20+ min  (DroneVibes)")
    print("  3S 5000 mAh,  light:             ~18 min  (DJI forum / E300 motors)")
    print("  3S 5000 mAh,  1.8 kg payload:    ~15 min  (DJI forum, gimbal + video tx)")
    print("  4S 4000 mAh,  1.8 kg payload:    ~12 min  (DJI forum)")
    print("→ 3S 4200 mAh, 1.9 kg payload:     12-15 min (interpolated — OUR CONFIG)")
    print("  3S 4000 mAh,  2212/920 + 10×5:   ~13 min  (HeliFreak)")
    print("  3S 2700 mAh,  1.0 kg, no gimbal: ~12 min  (DroneVibes)")
    print("  3S 2200 mAh,  light:             ~10 min  (amainhobbies)")
    print()
    print("Cruise-time references (sparse — F450-specific cruise endurance is rarely")
    print("published; community + multirotor aero theory is the best signal):")
    print("  • Translational lift: at 5-9 m/s forward speed, induced power DROPS")
    print("    below hover → real cruise endurance can be 10-20% LONGER than hover")
    print("    (Quadcopter Flight School; DJI forum thread 91091).")
    print("  • ArduPilot Discourse thread 32200: a 680 mm quad logged identical")
    print("    10 A current at hover and slow level cruise — translational-lift")
    print("    dip cancels the v² rise.")
    print("  • Above ~10 m/s, parasitic drag dominates: F450 at ~15 m/s max ≈ 280 W,")
    print("    ≈10 min full pack (Bauersfeld & Scaramuzza, 2021).")
    print("  • Our model is intentionally MONOTONE in speed (P = P_hover + k·v²)")
    print("    and ignores the translational-lift dip — see README \"Reference F450")
    print("    flight times\". This makes the sim pessimistic at our 7.5 m/s cap.")
    print()

    # Tests below construct a fresh env each time so battery state is clean.
    all_ok = True

    # --- Test 1: hover ---
    env = _build_one_drone_env()
    cfg = env.battery_cfg
    print(f"BatteryConfig: {cfg.voltage_v}V nominal, {cfg.capacity_mah}mAh, "
          f"P_hover={cfg.hover_power_w}W, k={cfg.motion_coeff_w_per_v2}W·(cells/s)⁻²")
    print(f"E_initial = {cfg.initial_energy_j:.0f} J,  "
          f"E_cutoff = {cfg.cutoff_energy_j:.0f} J,  "
          f"spendable = {cfg.initial_energy_j - cfg.cutoff_energy_j:.0f} J")
    print()

    print("Test 1: HOVER endurance (v = 0)")
    print(f"        P = P_hover = {cfg.hover_power_w} W")
    actual = hover_endurance_seconds(env)
    expected = _expected_seconds(env, speed_cells=0.0)
    all_ok &= _check("hover depletion time matches E_spendable / P_hover",
                     actual, expected)
    print()

    # --- Test 2: cruise at max_speed ---
    env = _build_one_drone_env()
    target_v = env.drone_cfg.max_speed  # 1.5 cells/s = 7.5 m/s
    target_v_mps = target_v * env.sim_cfg.meters_per_cell
    p_cruise = cfg.hover_power_w + cfg.motion_coeff_w_per_v2 * target_v ** 2
    print(f"Test 2: CRUISE endurance (v = max_speed = {target_v_mps:.1f} m/s = {target_v} cells/s)")
    print(f"        P = P_hover + k·v² = {cfg.hover_power_w} + "
          f"{cfg.motion_coeff_w_per_v2}·{target_v}² = {p_cruise:.2f} W")
    actual = cruise_endurance_seconds(env, target_speed_cells=target_v)
    expected = _expected_seconds(env, speed_cells=target_v)
    all_ok &= _check("cruise depletion time matches E_spendable / (P_hover + k·v²)",
                     actual, expected)
    print()

    # --- Reality check: how do our numbers stack up against published /
    # inferred F450 ranges? Hover has a clean published range; cruise has
    # no F450-specific measurement and must be inferred from hover × the
    # translational-lift gain documented across the multirotor literature.
    hover_min = _expected_seconds(env, 0.0) / 60.0
    cruise_min = _expected_seconds(env, target_v) / 60.0

    # Hover: published F450 hover-time range for 3S 4200 mAh + ~1.9 kg payload.
    HOVER_MIN_PUBLISHED = 12.0
    HOVER_MAX_PUBLISHED = 15.0
    # Cruise at 7.5 m/s: NO published F450-specific measurement exists. Range
    # is inferred from hover × translational-lift gain (1.00x at no benefit,
    # 1.20x at the typical sweet-spot peak — see README "Cruise (forward flight)").
    CRUISE_MIN_INFERRED = HOVER_MIN_PUBLISHED * 1.00  # 12.0
    CRUISE_MAX_INFERRED = HOVER_MAX_PUBLISHED * 1.20  # 18.0

    print(f"Sim hover endurance:  {hover_min:.2f} min   "
          f"(published F450 3S 4200 mAh, ~1.9 kg: "
          f"{HOVER_MIN_PUBLISHED:.0f}-{HOVER_MAX_PUBLISHED:.0f} min)  "
          f"{'✓ in range' if HOVER_MIN_PUBLISHED <= hover_min <= HOVER_MAX_PUBLISHED else '✗ outside published range'}")
    in_inferred = CRUISE_MIN_INFERRED <= cruise_min <= CRUISE_MAX_INFERRED
    print(f"Sim cruise endurance: {cruise_min:.2f} min   "
          f"(at {target_v_mps:.1f} m/s; inferred real F450 range: "
          f"{CRUISE_MIN_INFERRED:.0f}-{CRUISE_MAX_INFERRED:.0f} min)  "
          f"{'✓ in range' if in_inferred else '✗ below inferred range — pessimistic by design'}")
    print(f"  Cruise range is INFERRED, not directly measured. Source flow:")
    print(f"    hover (published)     = {HOVER_MIN_PUBLISHED:.0f}-{HOVER_MAX_PUBLISHED:.0f} min")
    print(f"    × translational-lift  = ×1.00 (no benefit) to ×1.20 (sweet-spot peak)")
    print(f"    → cruise (inferred)   = {CRUISE_MIN_INFERRED:.0f}-{CRUISE_MAX_INFERRED:.0f} min")
    print(f"  No F450-specific cruise-endurance measurement at a defined forward")
    print(f"  speed has been published — community + academic data covers either")
    print(f"  hover or different airframes (e.g., Bauersfeld 2022 = 6-inch frame).")
    print(f"  Our v² monotone model deliberately omits the translational-lift dip,")
    print(f"  so the sim sits BELOW the inferred range — pessimistic by design:")
    print(f"  a monotone cost makes optimization well-behaved, and a controller")
    print(f"  that tries to game the dip can't gain anything in this sim.")
    if not (HOVER_MIN_PUBLISHED <= hover_min <= HOVER_MAX_PUBLISHED):
        all_ok = False
    # Cruise being outside the inferred range is EXPECTED (model is monotone-pessimistic
    # by design) — don't flunk the verification on that.

    print()
    if all_ok:
        print("✓ ALL CHECKS PASSED — env.step's energy model agrees with "
              "the analytical formula and lands inside the published F450 range.")
    else:
        print("✗ SOME CHECKS FAILED — see above.")
    return all_ok


def run_gui() -> None:
    """
    Live animation of both regimes with a 1×–10× speed slider. The slider
    controls how many ``env.step()`` calls happen between renders — at 10×,
    a 12-min hover compresses to ~75 s real time. Energy drain per step is
    unchanged (same formula as headless), so the assertion semantics still
    hold; the slider only changes how many sim seconds pass per displayed
    frame.

    For visual interest the cruise phase puts the drone on a 2-cell-radius
    circle around the arena center (centripetal motion). Speed magnitude
    stays at ``max_speed``, so ``P = P_hover + k·v²`` is identical to the
    headless teleport-pin path — the verification is the same; only the
    eye candy differs.
    """
    import matplotlib

    backend = "MacOSX" if sys.platform == "darwin" else "TkAgg"
    try:
        # `force=True` raises ImportError if the backend can't actually load —
        # so a missing tkinter or PIL.ImageTk shows up here as a real exception
        # instead of silently falling back to Agg later.
        matplotlib.use(backend, force=True)
    except Exception as e:
        print(f"GUI backend {backend!r} unavailable ({e})", file=sys.stderr)
        if sys.platform == "linux":
            print("On Linux/WSL the TkAgg backend needs both apt packages:", file=sys.stderr)
            print("  sudo apt install python3-tk python3-pil.imagetk", file=sys.stderr)
            print("If you're inside a venv that wasn't created with "
                  "--system-site-packages, the apt packages are invisible — "
                  "run from system python or recreate the venv with that flag.",
                  file=sys.stderr)
        sys.exit(1)

    import matplotlib.pyplot as plt
    from matplotlib.animation import FuncAnimation
    from matplotlib.widgets import Slider

    # Sanity: verify the backend actually became the requested one. If something
    # imported pyplot earlier and locked Agg in, plt.get_backend() will reveal it.
    if plt.get_backend().lower() != backend.lower():
        print(f"Backend switch silently failed: requested {backend}, "
              f"got {plt.get_backend()}", file=sys.stderr)
        print("This usually means a module imported pyplot before run_gui ran. "
              "Check that visualize.py's matplotlib.use('Agg') guard is intact.",
              file=sys.stderr)
        sys.exit(1)

    # Internal renderers — our own private helpers, fine to call from a verification rig.
    from visualize import (
        _render_battery_panel,
        _render_map,
        format_duration,
        init_drone_palette,
    )

    init_drone_palette(1, seed=0)

    env_initial = _build_one_drone_env()
    cfg = env_initial.battery_cfg
    spendable_j = cfg.initial_energy_j - cfg.cutoff_energy_j

    # Two-pane layout (map left, panel right) plus a slider strip at the bottom.
    fig, (ax_map, ax_panel) = plt.subplots(
        1, 2, figsize=(15, 6),
        gridspec_kw={"width_ratios": [1, 1.7]},
    )
    # `top=0.85` reserves vertical room for fig.suptitle. Using suptitle (not
    # ax_map.set_title) is important here: the test status string is wider than
    # ax_map's column and would clip on the left edge of the window if we
    # centered it over the map only. suptitle centers over the *figure*, so the
    # 100-character "DONE  ✓  hover: ... | cruise: ... | published F450 ..."
    # line fits comfortably inside a 15-inch figure.
    fig.subplots_adjust(left=0.04, right=0.98, bottom=0.18, top=0.85, wspace=0.05)
    # Slider sized to ~28 % of figure width and right-centered so the long
    # explanatory label has room without crowding the panel above it.
    # `valstep=[1, 5, 10, 20]` snaps to those four values only — no in-between
    # positions — which avoids "is it 7× or 8×?" guesswork during reading.
    ax_slider = fig.add_axes([0.45, 0.05, 0.28, 0.03])
    speed_slider = Slider(
        ax_slider, "sim speed  (1× = real time)",
        valmin=1, valmax=20, valinit=5,
        valstep=[1, 5, 10, 20],
    )

    arena_center = np.array([env_initial.w / 2.0, env_initial.h / 2.0])
    cruise_radius = 2.0  # cells; small enough to fit, big enough to read

    # Mutable state across animation frames — closure over a dict so the inner
    # update() can rebuild the env when phases transition.
    state = {
        "env": env_initial,
        "phase": "hover",          # "hover" → "cruise" → "done"
        "hover_time_s": None,
        "cruise_time_s": None,
    }

    def _step_one(env: CoverageEnv) -> None:
        """One physics step in whichever regime is active."""
        if state["phase"] == "hover":
            # Hover: zero velocity, pin heading so the wedge doesn't drift
            # visually (energy model is heading-independent).
            env.drones[0].heading = 0.0
            env.drones[0].yaw_rate = 0.0
            env.step(np.zeros((1, 2)))
            return
        # Cruise: pin drone onto the circle, set velocity tangent + heading
        # along that tangent (so the camera looks where the drone is going,
        # matching the demo's policy), then step. env.step computes new_vel
        # = old_vel + 0·dt = vel, drains P_hover + k·v².
        offset = env.drones[0].pos - arena_center
        r_now = float(np.linalg.norm(offset))
        if r_now < 1e-6:
            offset = np.array([cruise_radius, 0.0])
        else:
            offset = offset * (cruise_radius / r_now)
        env.drones[0].pos = arena_center + offset
        tangent = np.array([-offset[1], offset[0]]) / cruise_radius
        env.drones[0].vel = tangent * env.drone_cfg.max_speed
        env.drones[0].heading = float(np.arctan2(tangent[1], tangent[0]))
        env.drones[0].yaw_rate = 0.0
        env.step(np.zeros((1, 2)))

    def update(_frame_idx: int):
        env = state["env"]
        if state["phase"] == "done":
            # Hold the final frame; nothing further to advance.
            return []

        steps_this_frame = int(speed_slider.val)
        for _ in range(steps_this_frame):
            if env.battery_state(0)["depleted"]:
                if state["phase"] == "hover":
                    state["hover_time_s"] = env.time_seconds
                    state["phase"] = "cruise"
                    state["env"] = _build_one_drone_env()
                    env = state["env"]
                else:
                    state["cruise_time_s"] = env.time_seconds
                    state["phase"] = "done"
                    break
            _step_one(env)

        _render_map(env, ax_map)
        _render_battery_panel(env, ax_panel)
        # _render_map sets a default ax_map.set_title with step / time /
        # coverage. None of that is what this verification cares about (we're
        # testing energy, not coverage), so suppress it and put test-specific
        # status on fig.suptitle instead.
        ax_map.set_title("")

        if state["phase"] == "done":
            # Two-line title so both reference ranges fit without horizontal
            # crowding. Hover has a published F450 range; cruise is inferred
            # from hover × translational-lift gain — see README "Cruise
            # (forward flight)" for the derivation.
            fig.suptitle(
                f"DONE  ✓  hover: {format_duration(state['hover_time_s'])}  |  "
                f"cruise: {format_duration(state['cruise_time_s'])}\n"
                f"F450 reference:  hover 12–15 min (published)  |  "
                f"cruise 12–18 min (inferred — sim pessimistic by design)"
            )
        else:
            # Battery framed as "100% full → 0% cutoff" — 0% is the 10V LiPo
            # cutoff (the unrecoverable-damage threshold the firmware enforces),
            # not the pack's true zero-energy point. Watchers see a familiar
            # "100 → 0" descent matching real drone telemetry conventions.
            remaining_to_cutoff_j = env.drones[0].battery_j - cfg.cutoff_energy_j
            pct_remaining = max(
                0.0,
                100.0 * remaining_to_cutoff_j / spendable_j if spendable_j > 0 else 0.0,
            )
            v_label = "0 m/s" if state["phase"] == "hover" else "7.5 m/s"
            fig.suptitle(
                f"{state['phase'].upper()} (v = {v_label})   "
                f"elapsed: {format_duration(env.time_seconds)}   "
                f"battery: {pct_remaining:5.1f}% remaining (full → cutoff)   "
                f"[{int(speed_slider.val)}× speed]"
            )
        return []

    # Render the opening frame so the window doesn't appear blank for a beat.
    _render_map(env_initial, ax_map)
    _render_battery_panel(env_initial, ax_panel)
    ax_map.set_title("")
    fig.suptitle(
        f"HOVER (v = 0 m/s)   elapsed: {format_duration(0.0)}   "
        f"battery: 100.0% remaining (full → cutoff)   [5× speed]"
    )

    # 50 ms interval = 20 fps. The slider snaps to {1, 5, 10, 20} sim steps
    # per render frame:
    #   1× → 20 sim steps/sec  =  real time, 12-min hover plays in 12 min real
    #   5× → 100 sim steps/sec ≈  10 sim sec / real sec → hover in ~74 s real
    #  10× → 200 sim steps/sec ≈  20 sim sec / real sec → hover in ~37 s real
    #  20× → 400 sim steps/sec ≈  40 sim sec / real sec → hover in ~18 s real
    # cache_frame_data=False suppresses matplotlib's "unbounded cache" warning
    # — we don't replay frames, so caching them would just leak memory.
    anim = FuncAnimation(  # noqa: F841 — keep ref alive for matplotlib
        fig, update, interval=50, blit=False, repeat=False,
        cache_frame_data=False,
    )
    plt.show()


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    parser.add_argument(
        "--gui", action="store_true",
        help="open a live animation with a 1×-10× speed slider instead of running to depletion in the console",
    )
    args = parser.parse_args()

    if args.gui:
        run_gui()
    else:
        ok = run_headless()
        sys.exit(0 if ok else 1)


if __name__ == "__main__":
    main()
