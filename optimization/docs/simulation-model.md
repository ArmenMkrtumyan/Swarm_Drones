# Simulation model

The 2D platform has just enough physics to expose the algorithmic question (does this controller cover ground efficiently?), no more.

## Coordinate frame and units

- The grid is an `int8` numpy array indexed `[y, x]`, shape `(H, W)`. Origin is top-left; `y` increases downward (matches numpy row-major and matplotlib's `origin="upper"`).
- Positions are floats in **cell units**. The cell at integer index `(cx, cy)` has its center at `(cx + 0.5, cy + 0.5)` and spans `[cx, cx+1) × [cy, cy+1)`.
- Time advances in fixed steps of `step_seconds = 0.1 s` by default (the simulation timestep, often written `dt` in physics formulas).

## World scale

`SimConfig.meters_per_cell` (default **5.0**) is the single knob that maps the cell-unit physics to the real world. At 5 m/cell:

| Param | In cells | Real units (at 5 m/cell) | Real-world reference (Hawk's Work F450 build) |
|---|---|---|---|
| `sensor_range` | 1.6 | 8.0 m | STEEReoCAM Nano stereo depth ceiling (datasheet) |
| `sensor_hfov_rad` | — | 54° | STEEReoCAM Nano lens datasheet §5 |
| `max_speed` | 1.8 | 9.0 m/s | translational-lift sweet-spot peak for our 1.3 kg build (`≈ 1.7 × v_induced`); see [`f450-reference.md` → Default `max_speed`](f450-reference.md) |
| `max_accel` (demo) | 2.5 | 12.5 m/s² ≈ 1.3 g | typical multirotor punch |
| `max_yaw_rate` | — | 1.5 rad/s ≈ 86°/s | typical quadcopter yaw rate |
| `max_yaw_accel` | — | 4.0 rad/s² | reaches max_yaw_rate in ~0.4 s |
| `drone_radius` | 0.05 | 0.25 m | Hawk's Work F450 half-width |
| 21×21 map | 21 | 105 × 105 m | small park / parking lot |
| `flight_altitude_max_m` (informational) | — | 10 m AGL | operational ceiling; well within camera's 0.95–8 m depth range |

The camera is **forward-facing and fixed** to the drone body — there is no gimbal. To redirect what the camera sees, the drone yaws (see *Drone state* below). For the simulator's coverage geometry that means the drone's footprint isn't a disc around its position; it's a **forward wedge** — apex at the drone, opening along the drone's heading, half-angle = HFOV/2 = 27°, range = `sensor_range` cells. See [`f450-reference.md` → Camera](f450-reference.md) for the lens datasheet and the geometric derivation.

The energy model's `motion_coeff_w_per_v2 = 13` is calibrated against `meters_per_cell = 5`, so changing the cell scale requires recalibrating via `k_cells = 0.51 × meters_per_cell²` (see [`battery-model.md`](battery-model.md)).

**Why this scale (and not "1 cell = 1 m")?** Coverage problems are defined by how much area the sensor can resolve per cell. Picking `meters_per_cell` from the sensor footprint keeps the per-cell discretization meaningful. Picking it from drone body size instead would make the map only 21×21 m and the drone slow at 1.35 m/s. Picking from max horizontal speed (15 m/s) gives a 210×210 m map with sensor implying ~20 m altitude. The middle option — sensor footprint — is what the default targets.

**Drone-radius caveat.** At the default scale, `drone_radius = 0.05` cells = 0.25 m matches the Hawk's Work F450 half-width and is small relative to the 5 m corridor, so collision logic essentially treats drones as point masses (correct for coverage missions; would need bumping for tight-tunnel scenarios).

**Mass.** `DroneConfig.mass_kg = 1.3` (Hawk's Work F450 frame + 4× A2212 920 KV motors + 4× 20 A ESCs + Pixhawk 2.4.8 + Jetson Nano + STEEReoCAM + 3S 4200 mAh LiPo + cabling) feeds directly into the power model: `BatteryConfig.hover_power_w` is **derived** from `mass_kg` via momentum theory (`P = T^1.5 / (FoM·√(2·ρ·A_disk))`, with `T = m·g`) at env-construction time, and `FoM = 0.4168` is calibrated so a 1.3 kg build yields ≈165 W. Bumping `mass_kg` automatically rescales `hover_power_w` (e.g., 1.5 kg → ~204 W). Override via `BatteryConfig(hover_power_w=...)` if you have a measured value. See [`f450-reference.md`](f450-reference.md) for the per-component mass breakdown and [`battery-model.md`](battery-model.md) (*Mass-aware hover power*) for the full derivation.

## Per-step flow

Each call to `env.step(actions)` advances the world by `step_seconds`. The ordering is fixed — controllers can rely on this contract:

1. **Receive actions.** The caller passes a NumPy array of shape `(n_drones, 3)` containing per-drone commands in cell-units / s² and rad/s²:
   - `actions[i, 0:2]` = linear acceleration `(ax, ay)` in **world frame**. The 2D vector is magnitude-rescaled so `|a| ≤ max_accel` (direction preserved).
   - `actions[i, 2]` = yaw acceleration `α_yaw` in rad/s², scalar-clipped to `±max_yaw_accel`.
   Linear and yaw commands are **independent** — real quadcopters yaw via differential motor torque, no body tilt, so a drone can translate while yawing.
   For backwards compatibility, controllers that emit shape `(n_drones, 2)` are also accepted (treated as `α_yaw = 0`).

2. **For each drone, in index order:**

   a. **Integrate linear velocity.** `v_new = v_old + a · step_seconds`. If `|v_new| > max_speed`, rescale to the speed cap (direction preserved). [Explicit Euler integration][src-euler-wiki] — see also [Gaffer On Games' "Integration Basics"][src-euler-game] for the same `position += velocity · dt` / `velocity += acceleration · dt` form used in game physics.

   b. **Propose a position.** `p_new = p_old + v_new · step_seconds`. Uses the *new* velocity, not a midpoint.

   c. **Resolve walls** axis-by-axis (X first, then Y with the accepted X). Either axis may have its velocity component zeroed if the move would put the drone-radius footprint into a wall or off-grid (see *Wall collision*).

   d. **Integrate yaw.** `yaw_rate_new = yaw_rate_old + α_yaw · dt`, clipped to `±max_yaw_rate`. Then `heading_new = heading_old + yaw_rate_new · dt`, wrapped to `[-π, π]`. Yaw is independent of translation in this model — real quadcopters can yaw while hovering or while translating.

   e. **Drain battery.** `P = P_hover + k · |v_new|²` watts; subtract `P · step_seconds` joules from `battery_j`, clamped at zero. The drain uses the *post-collision* velocity so a fully blocked drone pays only `P_hover · step_seconds`. **Yaw doesn't enter the energy model** in this version — see [battery-model.md](battery-model.md) for the rationale.

3. **Update coverage.** After all drones have moved (and yawed), every free cell inside *any* drone's forward wedge is marked covered. The wedge is anchored at the drone's position, opens along its heading, has half-angle = `sensor_hfov_rad/2`, and reaches `sensor_range` cells radially. The drone's own cell is always included (the wedge apex is *at* the drone). The mask is cumulative — once covered, always covered.

4. **Advance clock.** `time_seconds += step_seconds`; `step_count += 1`.

What the caller can rely on:

- All drones are stepped against the *same* `actions` snapshot — no within-step communication. A controller that needs to react to drone *i*'s post-step state must wait for the next step.
- Coverage reflects the *post-step* world, so `env.coverage_fraction()` after `step()` includes the new positions.
- Battery is drained per drone independently and cannot go negative.

## Drone dynamics

Each drone has state `(pos, vel, heading, yaw_rate)` — a 2D point mass plus an orientation around the vertical axis. The full integration sequence is described in *Per-step flow* above; this subsection just notes the design choices and defaults.

Configuration is split across two dataclasses so the names match what the values mean:

| Param | Lives in | Default | Demo override | Notes |
|---|---|---|---|---|
| `step_seconds` (`dt`) | `SimConfig` | 0.1 s | (kept) | Sim timestep — environment-level. |
| `meters_per_cell` | `SimConfig` | 5.0 m | (kept) | World-scale factor; see *World scale* above. |
| `flight_altitude_max_m` | `SimConfig` | 10.0 m | (kept) | Operational ceiling; informational only. |
| `sensor_range` | `DroneConfig` | 1.6 cells (= 8 m) | (kept) | Wedge radial range; STEEReoCAM stereo depth ceiling. |
| `sensor_hfov_rad` | `DroneConfig` | 54° | (kept) | Wedge horizontal FOV; STEEReoCAM lens datasheet. |
| `max_speed` | `DroneConfig` | 1.8 cells/s (= 9.0 m/s) | (kept) | Magnitude cap on `|vel|`, not per-axis. Translational-lift sweet-spot peak for 1.3 kg. |
| `max_accel` | `DroneConfig` | 2.0 cells/s² (= 10 m/s²) | 2.5 (= 12.5 m/s²) | Magnitude cap on `|a|`, not per-axis. |
| `max_yaw_rate` | `DroneConfig` | 1.5 rad/s ≈ 86°/s | (kept) | Scalar cap on `|yaw_rate|`. |
| `max_yaw_accel` | `DroneConfig` | 4.0 rad/s² | (kept) | Scalar cap on `|α_yaw|`. |
| `drone_radius` | `DroneConfig` | 0.05 cells (= 0.25 m) | (kept) | Used by wall collision only. |
| `mass_kg` | `DroneConfig` | 1.3 kg | (kept) | F450 + payload; **used by the energy model** — `BatteryConfig.hover_power_w` is derived from this via momentum theory. See *World scale*. |

`DroneConfig` is currently shared across every drone (homogeneous fleet). To support heterogeneous swarms later, pass a list of `DroneConfig` to `CoverageEnv` instead of one shared instance.

Two design choices worth justifying:

- **Explicit Euler.** Crude but adequate at `step_seconds = 0.1` and the speeds we exercise (worst-case displacement per step ≈ 0.15 cells, well under `drone_radius`). Higher-order integrators (RK4) would buy nothing here.
- **Magnitude clip, not per-axis.** Clipping each axis independently would change the *direction* of a commanded action whenever it lay outside the speed/accel box — fatal for any controller that produces direction vectors (Potential Fields, Consensus). Clipping by magnitude preserves direction and only attenuates magnitude.

## Wall collision

Drones have a `drone_radius` of 0.05 cells (≈ 0.25 m at the default scale, matching real F450 half-width). Collision is checked **axis-separately** so a blocked X move doesn't cancel a free Y move:

1. Try the candidate `(p_new.x, p.y)`. If the bounding box `[x − r, x + r] × [y − r, y + r]` lies inside the grid and overlaps no wall cell, accept the X move. Otherwise, zero `vel.x`.
2. Try `(accepted_x, p_new.y)` with the same test. Accept Y or zero `vel.y`.

This avoids the common point-mass gotcha where a drone moving diagonally into a corner sticks instead of sliding along one wall.

## Coverage and sensing

Each drone's camera is a fixed forward-facing stereo unit (STEEReoCAM Nano — see [`f450-reference.md`](f450-reference.md) for specs and the geometric derivation). The coverage footprint is a **forward wedge in the XY plane**: apex at the drone's position, opening along the drone's heading, half-angle = `sensor_hfov_rad / 2`, radial range = `sensor_range` cells. To redirect what the camera sees, the drone yaws.

After every step the coverage mask updates as:

```
for each drone d:
    h = (cos(d.heading), sin(d.heading))   # drone's forward unit vector
    for each cell (cx, cy) with center (mx, my) = (cx + 0.5, cy + 0.5):
        dx, dy = mx − d.pos.x, my − d.pos.y
        # In radial range AND inside the angular wedge?
        if (dx² + dy² ≤ sensor_range²)  AND
           (dx·h.x + dy·h.y > 0)  AND
           ((dx·h.x + dy·h.y)² ≥ cos²(hfov/2) · (dx² + dy²)):
            covered[cy, cx] = True
    # Always include the drone's own cell — wedge apex is *at* the drone,
    # angular test is degenerate when (dx, dy) = (0, 0).
    covered[int(d.pos.y), int(d.pos.x)] = True
```

The angular test is a `cos²` form so we avoid a `sqrt`; it's equivalent to `cos(angle_between(d.heading, cell_direction)) ≥ cos(hfov/2)`. Drones that yaw in place see the wedge sweep across new cells without translation; drones that translate without yawing see the wedge slide forward along the heading direction.

Wall cells are pre-marked covered at `reset()`, so the headline metric divides only by coverable cells:

```
coverage_fraction = (covered ∧ free).sum() / free.sum()
```

Coverage is monotone in this model — once a cell is seen, it stays covered. Methods that try to re-cover decayed regions can be added later by adding a decay step in `_update_coverage`.

### Per-drone coverage and overlap metrics (set-based)

Every drone keeps its own boolean mask `env.drone_covered[i]` of free cells it has personally swept through. From these we derive three monotone metrics. Coverage stays as a fraction (it's a ratio of "this map" to "this map"); overlap is in **square meters** since the world is physically scaled.

- `env.coverage_fraction()` — **global coverage** (fraction of free cells covered by *any* drone). Bounded `[0, 1]`. Monotone.
- `env.drone_coverage_fraction(i)` — **per-drone coverage** fraction. Bounded `[0, 1]`. Monotone.
- `env.overlap_cells_m2()` — **overlap area** (m²): free territory touched by ≥ 2 drones at some point. Bounded by the total free area. Monotone. Reads as "X m² of the map is being double-covered." This is the headline coordination-quality metric — a working consensus controller should drive it *down* relative to a non-coordinating baseline.

### Visit-count metrics (entry-event)

The above are *set-based* — they answer "is this cell covered?" not "how many times?". Visit-count metrics use `env.entry_count[i]`, incremented on every `outside-wedge → inside-wedge` transition, so revisits *are* tallied. Hovering with constant heading doesn't inflate counts (cells stay inside the wedge until the drone moves or yaws away). **Yawing in place does generate new entries** as the wedge sweeps across cells that weren't in it last step — that's correct: the drone scanned new ground.

- `env.total_visits(i)` — every cell-entry event by drone *i*.
- `env.unique_cells_visited(i)` — distinct free cells drone *i* ever entered.
- `env.self_revisits(i)` = `total_visits(i) − unique_cells_visited(i)` — drone *i* re-entering its own past trail.
- `env.cross_overlap_visits(i)` — drone *i*'s entries into cells some *other* drone has visited (counted statically across the full episode).
- `env.wasted_visits_total()` — swarm-wide redundant entries (self + cross). Single-number proxy for "energy spent re-covering ground." Pair with `coverage_fraction()` for an efficiency ratio.

These are entry-count metrics, not set membership, so a drone walking 20 cells over its own trail registers 20 self-revisits even though `drone_coverage_fraction` doesn't change. Renderer paints the `first_visitor` map with a per-drone color and darkens cells touched by ≥ 2 drones; `init_drone_palette(n_drones, seed)` in `visualize.py` builds the palette (HSV evenly-spaced, random rotation per run).

The visit-count model is verified by [`verification_scripts/test_overlap.py`](verification.md#test_overlappy).

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
| Altitude / 3D sensor footprint | flattened to horizontal wedge (HFOV/2 = 27°, range 1.6 cells = 8 m); see [f450-reference.md](f450-reference.md) | ✓ full stereo camera, 8 m depth range, 54° × 49.5° FOV |
| Rotor wake interactions | omitted | omitted |
| Rotor blade flapping (high-speed regime) | n/a | omitted |
| Motor saturation (current/ESC limit before theoretical max ω) | n/a | omitted |

These are the gaps that make 2D **falsification but not validation**: a controller that fails here will fail in Isaac; a controller that works here may still need re-tuning once these effects appear.

**Don't propagate translational lift from 3D back into 2D.** The 2D energy cost must remain monotone in speed for the optimizer to be well-behaved — that's the whole reason it's omitted from 2D, not an oversight. If you ever want lift in 2D for some reason, take a hard look at what optimization invariant breaks and decide explicitly.

[src-euler-wiki]: https://en.wikipedia.org/wiki/Euler_method
[src-euler-game]: https://gafferongames.com/post/integration_basics/
