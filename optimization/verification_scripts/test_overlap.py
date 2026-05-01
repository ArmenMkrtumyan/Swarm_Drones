"""
Smoke test for the entry-count overlap model.

Runs a deterministic 2-drone scenario by directly setting drone positions and
calling _update_coverage() each step (bypassing the action/integration loop).
This isolates the coverage bookkeeping from physics so the predicted counts
fall out of pure geometry.

Scenario (corridor of 25×5 cells, walls on borders, sensor_range = 0.01 so
the wedge is degenerate and only the drone's own-cell special case contributes
— each drone covers exactly its current cell, regardless of heading. This
makes "tile entered" = "cell entered", giving us hand-computable expectations):

  Phase 0: Drone 1 spawns at (2.5, 2.5). Drone 2 parks at (2.5, 1.5).
  Phase A: Drone 1 walks east (2.5→22.5, 2.5) painting cells (2,2)..(22,2).
           Drone 2 stays. Drone 1 racks up 21 unique entries.
  Phase B: Drone 1 freezes at (22.5, 2.5). Drone 2 hops onto row y=2 at
           (2.5, 2.5), then walks east to (22.5, 2.5). Every cell entered is
           already drone 1's territory — pure cross-overlap.
  Phase C: Drone 2 walks back west to (2.5, 2.5). Every cell entered is
           BOTH drone 1's territory (cross-overlap) AND drone 2's own
           previous trail (self-revisit).

Run modes:
  python tools/test_overlap.py            # headless: writes 4 PNGs, asserts counts
  python tools/test_overlap.py --gui      # live animation: watch metrics tick up
"""

from __future__ import annotations

import argparse
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

import numpy as np

from constants import IMAGES_DIR
from environment import CoverageEnv, DroneConfig, SimConfig
from maze import FREE, WALL
from visualize import init_drone_palette, render_frame


def _clear_coverage_state(env: CoverageEnv) -> None:
    """Wipe everything _update_coverage maintains, leaving drone.pos alone."""
    env.entry_count[:] = 0
    env.prev_footprint[:] = False
    env.first_visitor[:] = -1
    env.drone_covered[:] = False
    env.covered[:] = False
    env.covered[env.grid == WALL] = True


def _teleport_drone(env: CoverageEnv, drone_idx: int, new_pos: tuple) -> None:
    """
    Move drone to ``new_pos`` and snap its heading to point in the direction
    of the teleport (so the camera looks where the drone just went). This is
    purely for visual consistency in the GUI / saved PNGs — the test
    assertions are coverage counts, which are heading-independent at our
    sensor_range=0.01.
    """
    target = np.array(new_pos, dtype=np.float64)
    delta = target - env.drones[drone_idx].pos
    if float(np.linalg.norm(delta)) > 1e-9:
        env.drones[drone_idx].heading = float(np.arctan2(delta[1], delta[0]))
        env.drones[drone_idx].yaw_rate = 0.0
    env.drones[drone_idx].pos = target


def _print_metrics(env: CoverageEnv, header: str) -> None:
    print(f"\n--- {header} ---")
    print(f"{'metric':25s}  " + "  ".join(f"D{i+1:>2}" for i in range(env.n_drones)))
    for label, fn in [
        ("total_visits", env.total_visits),
        ("unique_cells_visited", env.unique_cells_visited),
        ("self_revisits", env.self_revisits),
        ("cross_overlap_visits", env.cross_overlap_visits),
    ]:
        vals = "  ".join(f"{fn(i):>3d}" for i in range(env.n_drones))
        print(f"{label:25s}  {vals}")
    print(f"  wasted_visits_total = {env.wasted_visits_total()}")
    print(f"  global coverage     = {env.coverage_fraction():.2%}  "
          f"({int((env.covered & (env.grid == FREE)).sum())} unique free cells)")
    print(f"  overlap area (m²)   = {env.overlap_cells_m2():.0f}  "
          f"(cells touched by ≥2 distinct drones)")


def _check(label: str, actual: int, expected: int) -> bool:
    ok = actual == expected
    mark = "✓" if ok else "✗"
    print(f"  {mark} {label}: actual={actual}  expected={expected}")
    return ok


def _build_env() -> CoverageEnv:
    """Construct the corridor env and seat both drones at their phase-0 cells."""
    H, W = 5, 25
    grid = np.full((H, W), FREE, dtype=np.int8)
    grid[0, :] = WALL
    grid[-1, :] = WALL
    grid[:, 0] = WALL
    grid[:, -1] = WALL

    env = CoverageEnv(
        grid=grid,
        n_drones=2,
        sim=SimConfig(step_seconds=0.1),
        drone=DroneConfig(sensor_range=0.01, max_speed=1.5, max_accel=2.5),
    )
    env.reset(seed=0)
    # Fixed palette seed so the visuals look the same on every run (visual diff
    # against expectations stays trivial).
    init_drone_palette(env.n_drones, seed=0)

    env.drones[0].pos = np.array([2.5, 2.5])  # Drone 1 — the painter
    env.drones[1].pos = np.array([2.5, 1.5])  # Drone 2 — parked off the y=2 row
    # Pin heading + yaw_rate so the camera always points along the direction
    # of teleport-motion (set per scripted step in the loop below). Without
    # this, the wedge would point in whatever random direction reset() drew
    # — fine for the assertions (sensor_range=0.01 makes the wedge degenerate)
    # but visually misleading.
    env.drones[0].heading = 0.0  # facing east; updated per scripted step
    env.drones[0].yaw_rate = 0.0
    env.drones[1].heading = 0.0
    env.drones[1].yaw_rate = 0.0
    _clear_coverage_state(env)
    env._update_coverage()
    return env


def _scripted_steps():
    """
    Yield ``(drone_idx, (x, y), phase_label)`` for every position update the
    scenario performs. Consumed by both modes — headless snapshots after each
    phase, GUI animates every step so the user can watch counters tick.
    """
    # Phase A: D1 walks east through row y=2. D2 stays parked at (2.5, 1.5).
    for x in range(3, 23):
        yield (0, (x + 0.5, 2.5),
               f"Phase A: D1 paints (x={x},y=2) — uncharted, no overlap")
    # Phase B: D2 hops onto D1's row at (2,2), then walks east to (22,2).
    yield (1, (2.5, 2.5),
           "Phase B: D2 enters D1's territory at (2,2) — cross-overlap +1")
    for x in range(3, 23):
        yield (1, (x + 0.5, 2.5),
               f"Phase B: D2 walks east through D1's territory at (x={x},y=2)")
    # Phase C: D2 retraces west — every entry is BOTH cross + self-revisit.
    for x in range(21, 1, -1):
        yield (1, (x + 0.5, 2.5),
               f"Phase C: D2 retraces (x={x},y=2) — self↺ +1, cross-overlap +1")


def run_headless(env: CoverageEnv) -> bool:
    """Original PNG-and-assertions mode. Returns True iff every check passed."""
    out_dir = IMAGES_DIR
    out_dir.mkdir(parents=True, exist_ok=True)
    render_frame(env, save_path=str(out_dir / "test_overlap_00_initial.png"))
    _print_metrics(env, "Phase 0 — initial placement")

    # Replay the scripted positions, taking a snapshot each time the phase
    # label transitions ("Phase A" → "Phase B" etc.) so the saved PNGs match
    # the original three-snapshot story.
    snapshot_path = {
        "A": out_dir / "test_overlap_01_after_phaseA.png",
        "B": out_dir / "test_overlap_02_after_phaseB.png",
        "C": out_dir / "test_overlap_03_final.png",
    }
    last_phase = "A"
    for drone_idx, pos, label in _scripted_steps():
        phase = label.split(":")[0].split()[-1]  # "A", "B", or "C"
        if phase != last_phase:
            render_frame(env, save_path=str(snapshot_path[last_phase]))
            _print_metrics(env, f"After Phase {last_phase}")
            last_phase = phase
        _teleport_drone(env, drone_idx, pos)
        env._update_coverage()
    render_frame(env, save_path=str(snapshot_path[last_phase]))
    _print_metrics(env, f"After Phase {last_phase}")

    print("\n=== Expected vs actual (hand-computed) ===")
    expected = {
        ("D1", "total_visits"):        21,
        ("D1", "unique_cells"):        21,
        ("D1", "self_revisits"):        0,
        ("D1", "cross_overlap"):       21,
        ("D2", "total_visits"):        42,
        ("D2", "unique_cells"):        22,
        ("D2", "self_revisits"):       20,
        ("D2", "cross_overlap"):       41,
        ("swarm", "wasted_visits"):    41,
    }
    all_ok = True
    all_ok &= _check("D1 total_visits",         env.total_visits(0),         expected[("D1", "total_visits")])
    all_ok &= _check("D1 unique_cells",         env.unique_cells_visited(0), expected[("D1", "unique_cells")])
    all_ok &= _check("D1 self_revisits",        env.self_revisits(0),        expected[("D1", "self_revisits")])
    all_ok &= _check("D1 cross_overlap_visits", env.cross_overlap_visits(0), expected[("D1", "cross_overlap")])
    all_ok &= _check("D2 total_visits",         env.total_visits(1),         expected[("D2", "total_visits")])
    all_ok &= _check("D2 unique_cells",         env.unique_cells_visited(1), expected[("D2", "unique_cells")])
    all_ok &= _check("D2 self_revisits",        env.self_revisits(1),        expected[("D2", "self_revisits")])
    all_ok &= _check("D2 cross_overlap_visits", env.cross_overlap_visits(1), expected[("D2", "cross_overlap")])
    all_ok &= _check("swarm wasted_visits",     env.wasted_visits_total(),   expected[("swarm", "wasted_visits")])

    print()
    if all_ok:
        print("✓ ALL CHECKS PASSED — entry-count overlap model agrees with hand-computed expectations.")
    else:
        print("✗ SOME CHECKS FAILED — see above.")
    print(f"\nFrames saved to: {out_dir}")
    print("  test_overlap_00_initial.png       (drones at start, no coverage yet)")
    print("  test_overlap_01_after_phaseA.png  (D1's territory painted along row y=2)")
    print("  test_overlap_02_after_phaseB.png  (D2 walked east through D1's row → overlap darkens)")
    print("  test_overlap_03_final.png         (D2 retraced west → all cross + self-revisit)")
    return all_ok


def run_gui(env: CoverageEnv) -> None:
    """
    Animate the scripted scenario at ~4 frames per second. Each frame: advance
    one position, update coverage, redraw map + battery panel. The phase label
    rides on the map title so it's obvious which counters should be moving on
    the panel — watch ``self↺`` and ``wasted visits`` during Phase C.
    """
    import matplotlib

    backend = "MacOSX" if sys.platform == "darwin" else "TkAgg"
    try:
        matplotlib.use(backend, force=True)
    except Exception as e:
        print(f"GUI backend {backend!r} unavailable ({e}). Falling back to headless.")
        run_headless(env)
        return

    import matplotlib.pyplot as plt
    from matplotlib.animation import FuncAnimation

    # Internal renderers — they're prefixed with "_" but they're our own code,
    # not third-party. The test rig is a legitimate caller.
    from visualize import _make_figure, _render_battery_panel, _render_map

    fig, ax_map, ax_panel = _make_figure(env)
    # Reserve room at the top for fig.suptitle (the phase label can be ~80
    # chars wide and would clip on the left if centered over ax_map only).
    fig.subplots_adjust(top=0.85)
    script = list(_scripted_steps())
    n_steps = len(script)

    # Render the initial frame so the window opens with the starting state.
    _render_map(env, ax_map)
    _render_battery_panel(env, ax_panel)
    fig.suptitle("Phase 0: starting placement (D1 at (2,2), D2 parked at (2,1))")

    def update(frame_idx: int):
        # Frames 0..n_steps-1 advance the scenario; trailing frames hold the
        # final state for ~2 seconds so the user has time to read the counters.
        if frame_idx < n_steps:
            drone_idx, pos, label = script[frame_idx]
            _teleport_drone(env, drone_idx, pos)
            env._update_coverage()
        else:
            label = "Done — every visit counted. Read self↺ + wasted visits on the panel."
        _render_map(env, ax_map)
        _render_battery_panel(env, ax_panel)
        fig.suptitle(f"step {min(frame_idx + 1, n_steps)}/{n_steps}  |  {label}")
        return []

    # Capture the FuncAnimation in a name so it isn't garbage-collected mid-run
    # — matplotlib's "Animation was deleted without rendering anything" warning
    # bites otherwise.
    anim = FuncAnimation(  # noqa: F841 (deliberately retained reference)
        fig, update,
        frames=n_steps + 8,
        interval=250,
        blit=False, repeat=False,
    )
    plt.show()


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    parser.add_argument("--gui", action="store_true",
                        help="open a live animation window instead of writing PNGs")
    args = parser.parse_args()

    env = _build_env()

    if args.gui:
        run_gui(env)
    else:
        ok = run_headless(env)
        sys.exit(0 if ok else 1)


if __name__ == "__main__":
    main()
