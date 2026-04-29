# Simulation model

The 2D platform has just enough physics to expose the algorithmic question (does this controller cover ground efficiently?), no more.

## Coordinate frame and units

- The grid is an `int8` numpy array indexed `[y, x]`, shape `(H, W)`. Origin is top-left; `y` increases downward (matches numpy row-major and matplotlib's `origin="upper"`).
- Positions are floats in **cell units**. The cell at integer index `(cx, cy)` has its center at `(cx + 0.5, cy + 0.5)` and spans `[cx, cx+1) × [cy, cy+1)`.
- Time advances in fixed steps of `step_seconds = 0.1 s` by default (the simulation timestep, often written `dt` in physics formulas).

## World scale

`SimConfig.meters_per_cell` (default **5.0**) is the single knob that maps the cell-unit physics to the real world. The default 5 m/cell is derived from the **STEEReoCAM Nano** camera footprint (HFOV 54°, VFOV 49.5° — see [`f450-reference.md` → Camera](f450-reference.md#camera-steereocam-nano--used-to-derive-sensor_radius) for the datasheet). At altitude *h* the inscribed-disc footprint radius is `h × tan(VFOV/2) = h × 0.4607`. The default `sensor_radius = 1.5 cells × 5 m/cell = 7.5 m` corresponds to **h ≈ 16.3 m AGL**:

| Param | In cells | Real units (at 5 m/cell) | Real-world reference (Hawk's Work F450 build) |
|---|---|---|---|
| `sensor_radius` | 1.5 | 7.5 m | STEEReoCAM Nano inscribed disc (54° × 49.5° FOV) at ~16.3 m AGL |
| `max_speed` (demo) | 1.5 | 7.5 m/s | realistic F450 cruise (sweet spot for translational lift) |
| `max_accel` (demo) | 2.5 | 12.5 m/s² ≈ 1.3 g | typical multirotor punch |
| `drone_radius` | 0.05 | 0.25 m | actual Hawk's Work F450 half-width (~0.225 m to motor mount) |
| 21×21 map | 21 | 105 × 105 m | small park / parking lot |
| `flight_altitude_m` (informational) | — | 16.3 m AGL | camera footprint anchor; **above the 0.95–8 m stereo-depth ceiling** so coverage is visual-only at this altitude |

To rescale for a different altitude (e.g., to fly within the 8 m depth-reliable range), use `environment.sensor_radius_from_altitude(altitude_m, shape="inscribed")` and adjust `meters_per_cell` + `sensor_radius` together. The energy model's `motion_coeff_w_per_v2 = 13` is calibrated against `meters_per_cell = 5`, so changing `meters_per_cell` requires recalibrating that constant via `k_cells = 0.51 × meters_per_cell²` (see [`battery-model.md`](battery-model.md)).

**Why this scale (and not "1 cell = 1 m")?** Coverage problems are defined by how much area the sensor can resolve per cell. Picking `meters_per_cell` from the sensor footprint keeps the per-cell discretization meaningful. Picking it from drone body size instead would make the map only 21×21 m and the drone slow at 1.35 m/s. Picking from max horizontal speed (15 m/s) gives a 210×210 m map with sensor implying ~20 m altitude. The middle option — sensor footprint — is what the default targets.

**Drone-radius caveat.** At the default scale, `drone_radius = 0.05` cells = 0.25 m matches the Hawk's Work F450 half-width and is small relative to the 5 m corridor, so collision logic essentially treats drones as point masses (correct for coverage missions; would need bumping for tight-tunnel scenarios).

**Mass.** `DroneConfig.mass_kg = 1.9` (Hawk's Work F450 frame + 4× A2212 920 KV motors + 4× 20 A ESCs + Pixhawk 2.4.8 + Jetson Nano + STEEReoCAM + 3S 4200 mAh LiPo + cabling) is **informational only** — the power model uses back-calculated `hover_power_w = 165 W` rather than deriving it from `m·g` and rotor area. Adding aerodynamic derivation would require rotor area and air density. See [`f450-reference.md`](f450-reference.md) for the full component list and links to the Hawk's Work product page.

## Per-step flow

Each call to `env.step(actions)` advances the world by `step_seconds`. The ordering is fixed — controllers can rely on this contract:

1. **Receive actions.** The caller passes a NumPy array of shape `(n_drones, 2)` containing per-drone acceleration commands in cell-units / s² (any direction, any magnitude). Each drone's vector is rescaled so `|a| ≤ max_accel` while preserving direction. No clipping per-axis.

2. **For each drone, in index order:**

   a. **Integrate velocity.** `v_new = v_old + a · step_seconds`. If `|v_new| > max_speed`, rescale to the speed cap (direction preserved). [Explicit Euler integration][src-euler-wiki] — see also [Gaffer On Games' "Integration Basics"][src-euler-game] for the same `position += velocity · dt` / `velocity += acceleration · dt` form used in game physics.

   b. **Propose a position.** `p_new = p_old + v_new · step_seconds`. Uses the *new* velocity, not a midpoint.

   c. **Resolve walls** axis-by-axis (X first, then Y with the accepted X). Either axis may have its velocity component zeroed if the move would put the drone-radius footprint into a wall or off-grid (see *Wall collision*).

   d. **Drain battery.** `P = P_hover + k · |v_new|²` watts; subtract `P · step_seconds` joules from `battery_j`, clamped at zero. The drain uses the *post-collision* velocity so a fully blocked drone pays only `P_hover · step_seconds`. See [battery-model.md](battery-model.md) for the equations and their derivation.

3. **Update coverage.** After all drones have moved, every free cell within `sensor_radius` of any drone center is marked covered. The mask is cumulative — once covered, always covered.

4. **Advance clock.** `time_seconds += step_seconds`; `step_count += 1`.

What the caller can rely on:

- All drones are stepped against the *same* `actions` snapshot — no within-step communication. A controller that needs to react to drone *i*'s post-step state must wait for the next step.
- Coverage reflects the *post-step* world, so `env.coverage_fraction()` after `step()` includes the new positions.
- Battery is drained per drone independently and cannot go negative.

## Drone dynamics

Each drone is a 2D point mass with state `(pos, vel)`. The full integration sequence is described in *Per-step flow* above; this subsection just notes the design choices and defaults.

Configuration is split across two dataclasses so the names match what the values mean:

| Param | Lives in | Default | Demo override | Notes |
|---|---|---|---|---|
| `step_seconds` (`dt`) | `SimConfig` | 0.1 s | (kept) | Sim timestep — environment-level. |
| `meters_per_cell` | `SimConfig` | 5.0 m | (kept) | World-scale factor; see *World scale* above. |
| `sensor_radius` | `DroneConfig` | 1.5 cells (= 7.5 m) | (kept) | Disc sensor radius. |
| `max_speed` | `DroneConfig` | 1.0 cells/s (= 5 m/s) | 1.5 (= 7.5 m/s) | Magnitude cap, not per-axis. |
| `max_accel` | `DroneConfig` | 2.0 cells/s² (= 10 m/s²) | 2.5 (= 12.5 m/s²) | Magnitude cap, not per-axis. |
| `drone_radius` | `DroneConfig` | 0.05 cells (= 0.25 m) | (kept) | Used by wall collision only. |
| `mass_kg` | `DroneConfig` | 1.9 kg | (kept) | F450 + payload; informational, see *World scale*. |

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

Each drone has an omnidirectional disc sensor of `sensor_radius` cells (default 1.5). After every step the coverage mask updates as:

```
for each drone d:
    for each cell (cx, cy) with center (cx + 0.5, cy + 0.5):
        if (cx + 0.5 − d.pos.x)² + (cy + 0.5 − d.pos.y)² ≤ sensor_radius²:
            covered[cy, cx] = True
```

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

The above are *set-based* — they answer "is this cell covered?" not "how many times?". Visit-count metrics use `env.entry_count[i]`, incremented on every `outside-disc → inside-disc` transition, so revisits *are* tallied. Hovering doesn't inflate counts (cells stay inside the disc until the drone leaves and returns).

- `env.total_visits(i)` — every cell-entry event by drone *i*.
- `env.unique_cells_visited(i)` — distinct free cells drone *i* ever entered.
- `env.self_revisits(i)` = `total_visits(i) − unique_cells_visited(i)` — drone *i* re-entering its own past trail.
- `env.cross_overlap_visits(i)` — drone *i*'s entries into cells some *other* drone has visited (counted statically across the full episode).
- `env.wasted_visits_total()` — swarm-wide redundant entries (self + cross). Single-number proxy for "energy spent re-covering ground." Pair with `coverage_fraction()` for an efficiency ratio.

These are entry-count metrics, not set membership, so a drone walking 20 cells over its own trail registers 20 self-revisits even though `drone_coverage_fraction` doesn't change. Renderer paints the `first_visitor` map with a per-drone color and darkens cells touched by ≥ 2 drones; `init_drone_palette(n_drones, seed)` in `visualize.py` builds the palette (HSV evenly-spaced, random rotation per run).

The visit-count model is verified by [`verification_scripts/test_overlap.py`](verification.md#test_overlappy).

## Intentionally omitted (gap to Isaac)

The 2D model leaves out everything that doesn't change algorithmic feasibility:

- attitude / tilt dynamics (a real quad must tilt to translate; here velocity is direct)
- aerodynamic drag and rotor wake interactions
- actuator delay and motor lag
- IMU / GPS noise, EKF lag
- wind disturbance
- battery sag (transient voltage drop under load) and the actual non-linear LiPo discharge curve — we use a linear `V(E)` approximation only for the voltage cutoff threshold (see [battery-model.md](battery-model.md))
- altitude — everything is at one height; no 3D sensor footprint

These are the gaps that make 2D **falsification but not validation**: a controller that fails here will fail in Isaac; a controller that works here may still need re-tuning once these effects appear.

[src-euler-wiki]: https://en.wikipedia.org/wiki/Euler_method
[src-euler-game]: https://gafferongames.com/post/integration_basics/
