# Simulation model

The 2D platform has just enough physics to expose the algorithmic question: 

**does this controller cover ground efficiently?**



## World scale

`SimConfig.meters_per_cell` (default **5.0**) is the single knob that maps the cell-unit physics to the real world. At 5 m/cell:

| Param | In cells | Real units (at 5 m/cell) | Real-world reference (Hawk's Work F450 build) |
|---|---|---|---|
| `sensor_range` | 1.6 | 8.0 m | STEEReoCAM Nano stereo depth ceiling (datasheet) |
| `sensor_hfov_rad` | — | 54° | STEEReoCAM Nano lens datasheet §5 |
| `max_speed` | 1.8 | 9.0 m/s | translational-lift sweet-spot peak for our 1.3 kg build (`≈ 1.7 × v_induced`); see [`f450-reference.md` → Default `max_speed`](f450-reference.md) |
| `max_accel` | 2.5 | 12.5 m/s² ≈ 1.3 g | below F450 hardware max — at 1.3 kg with TWR ≈ 2:1, physical max horizontal accel = √(T²−W²)/m ≈ 1.6–2.0 g; we use ~65–80 % of that for stability margin |
| `max_yaw_rate` | — | 1.5 rad/s ≈ 86°/s | conservative scan rate (full 360° in ~4.2 s); PX4 / ArduPilot firmware defaults are ~200°/s — ours is well below for smooth coverage flight |
| `max_yaw_accel` | — | 4.0 rad/s² | chosen for ~0.4 s yaw-settle time (`max_yaw_rate / max_yaw_accel = 1.5 / 4.0 ≈ 0.38 s`); engineering judgment, not a physics anchor |
| `drone_radius` | 0.05 | 0.25 m | midpoint approximation of Hawk's Work F450 footprint — bare-frame half-width (X-config) ≈ 0.16 m, prop-tip extent ≈ 0.28 m (159 mm motor offset + 120 mm 9450 prop radius); collision use only |
| 21×21 map | 21 | 105 × 105 m | small park / parking lot |
| `flight_altitude_max_m` (informational) | — | 10 m AGL | operational ceiling; informational only — sim is 2D, altitude isn't part of the dynamics |

**Why this scale (and not "1 cell = 1 m")?** Coverage problems are defined by how much area the sensor can resolve per cell. Picking `meters_per_cell` from the sensor footprint keeps the per-cell discretization meaningful. Picking it from drone body size instead would make the map only 21×21 m and the drone slow at 1.35 m/s. Picking from max horizontal speed (15 m/s) gives a 210×210 m map with sensor implying ~20 m altitude. The middle option — sensor footprint — is what the default targets.

## Per-step flow

Each call to `env.step(actions)` advances the world by `step_seconds`. The ordering is fixed — controllers can rely on this contract:

1. **Receive actions.** Caller passes shape `(n_drones, 3)` = `[ax, ay, α_yaw]` per drone (cell-units/s² for translation, rad/s² for yaw). 2D linear vector is magnitude-rescaled to `|a| ≤ max_accel` (direction preserved); `α_yaw` is scalar-clipped to `±max_yaw_accel`. Linear and yaw are independent. Shape `(n_drones, 2)` is also accepted (treated as `α_yaw = 0`).

2. **For each drone, in index order:**

   a. **Integrate linear velocity.** `v_new = v_old + a · dt`; if `|v_new| > max_speed` rescale to the cap (direction preserved). Explicit Euler.

   b. **Propose a position.** `p_new = p_old + v_new · dt` — uses the new velocity, not a midpoint.

   c. **Resolve walls.** Axis-separated: try the X move first (`drone_radius` bounding-box vs walls/grid bounds); if it would clip, zero `vel.x` and reject the X step. Then try Y with the accepted X. This lets a drone slide along one wall instead of sticking.

   d. **Integrate yaw.** `yaw_rate_new = yaw_rate_old + α_yaw · dt`, clipped to `±max_yaw_rate`; then `heading_new = heading_old + yaw_rate_new · dt`, wrapped into `[-π, π]`. Independent of translation (real quads yaw via differential motor torque, no body tilt).

   e. **Drain battery.** `P = P_hover + k · |v_new|²`; subtract `P · dt` joules from `battery_j`, clamped at the cutoff. Uses the *post-collision* velocity, so a wall-blocked drone pays only `P_hover · dt`. Yaw doesn't enter the energy model — see [`battery-model.md`](battery-model.md).

3. **Update coverage.** After all drones moved, every free cell inside any drone's forward wedge is marked covered. Wedge: apex at `pos`, half-angle `sensor_hfov_rad/2`, radial range `sensor_range`. The drone's own cell is always included. Cumulative — once covered, always covered (no decay).

4. **Advance clock.** `time_seconds += dt`; `step_count += 1`.

What the caller can rely on:

- All drones are stepped against the *same* `actions` snapshot — no within-step communication. A controller that needs to react to drone *i*'s post-step state must wait for the next step.
- Coverage reflects the *post-step* world, so `env.coverage_fraction()` after `step()` includes the new positions.
- Battery is drained per drone independently and cannot go negative.

## Drone dynamics

Each drone has state `(pos, vel, heading, yaw_rate)` — a 2D point mass plus an orientation around the vertical axis. The integration sequence lives in *Per-step flow* above. Two design choices worth justifying:

- **Explicit Euler.** Crude but adequate at `dt = 0.1 s` and the speeds we exercise (worst-case displacement per step ≈ 0.18 cells, well under `drone_radius` so wall logic stays sound). Higher-order integrators (RK4) would buy nothing here.
- **Magnitude clip on `(ax, ay)`, not per-axis.** Per-axis clipping would change the *direction* of a commanded action whenever it lay outside the accel box — fatal for any controller that emits direction vectors (Potential Fields, Consensus). Magnitude clipping preserves direction and only attenuates magnitude.

`DroneConfig` is currently shared across every drone (homogeneous fleet). To support heterogeneous swarms later, pass a list of `DroneConfig` to `CoverageEnv` instead of one shared instance.

## Coverage and overlap metrics

Two families of metrics live on `CoverageEnv`, addressing different questions:

- **Set-based** (monotone, "is this cell covered?"):
  - `coverage_fraction()` — global fraction of free cells covered by any drone.
  - `drone_coverage_fraction(i)` / `drone_coverage_m2(i)` — per-drone coverage.
  - `overlap_cells_m2()` — area in m² touched by ≥ 2 drones at some point. Headline coordination-quality metric — a working consensus controller should drive this *down* relative to a non-coordinating baseline.

- **Visit-count** (entry-event, "how many times?"):
  - `total_visits(i)` — every cell-entry event by drone *i*. Hovering doesn't inflate (cells stay inside the wedge until the drone moves or yaws away). Yawing in place *does* generate new entries as the wedge sweeps over cells that weren't in it last step.
  - `unique_cells_visited(i)`, `self_revisits(i)`, `cross_overlap_visits(i)` — derived from `entry_count`.
  - `wasted_visits_total()` — swarm-wide redundant entries (self + cross). Single-number proxy for "energy spent re-covering ground."

Visit-count semantics are verified by [`verification_scripts/test_overlap.py`](../verification_scripts/test_overlap.py).

## Intentionally omitted in 2D (gap to Isaac / 3D bridge)

The 2D model leaves out everything that doesn't change algorithmic feasibility. Each effect below is tagged with its status in the 3D Isaac bridge (`my_drone_simulation/Nvidia_SITL_connecter.py`) so the 2D-vs-3D contract is explicit:

| Effect | 2D | 3D bridge |
|---|---|---|
| Attitude / tilt dynamics (a real quad must tilt to translate) | omitted — velocity is direct | ✓ PhysX articulation + per-motor thrust torques |
| Aerodynamic drag | omitted | ✓ quadratic body drag, `K_DRAG = 0.028 N·s²/m²` |
| Translational lift (5–9 m/s sweet spot) | **deliberately omitted** — keeps energy cost monotone-in-speed for the optimizer (see [battery-model.md → Why `v²` and not `v³`](battery-model.md#why-v²-and-not-v³)) | ✓ per-motor Gaussian thrust bonus, peak +15 % near 7 m/s (Bauersfeld & Scaramuzza 2021) |
| Ground effect | omitted — no altitude | ✓ per-rotor boost when AGL < rotor diameter (≈ 0.24 m) |
| Actuator / motor lag | omitted | ✓ first-order lag, `MOTOR_TIME_CONSTANT_S = 0.05 s` |
| IMU / GPS noise, EKF lag | omitted | ✓ via ArduPilot SITL's EKF + simulated sensors |
| Wind disturbance | omitted | omitted |
| Battery voltage sag (transient drop under load + non-linear LiPo discharge) | linear `V(E)` only, for cutoff threshold (see [battery-model.md](battery-model.md)) | **omitted** — `K_THRUST` constant for whole flight; ArduPilot's `FS_BATT_VOLTAGE` failsafe is the safety net |
