# F450 reference numbers

Real-world F450 specs and community-measured flight times, used to sanity-check the simulator's energy and motion models. The F450 is a DIY frame kit, so most "specs" depend on which battery, motors, and props the builder picks. **Hawk's Work** (the build this project uses) and **DJI Flame Wheel** are functionally the same airframe — DJI released the original ~2012 and Hawk's Work / Readytosky / others ship clones at lower prices with identical wheelbase, frame weight, and motor / battery / prop classes. Most published flight-time data is from DJI users because the platform is older and more popular; that data applies equally to Hawk's Work builds.

## This project's drone (Hawk's Work F450 build)

The components our simulator's calibration is anchored to, sourced from the [Hawk's Work F450 Drone product page](https://www.hawks-work.com/pages/f450-drone):

| Component | Spec | Notes |
|---|---|---|
| Frame | Hawk's Work F450, 450 mm wheelbase, 280 g | Clone of DJI Flame Wheel F450 — same airframe class |
| Motors | **A2212 920 KV** brushless (or MT2213 935 KV variant) | Equivalent to DJI E300 920 KV — our `P_hover` calibration is for this motor class |
| ESC | 20 A brushless with 5 V / 1 A BEC | Within Hawk's Work's recommended 15–30 A range |
| Battery | **3S 4200 mAh 25 C, 11.1 V nominal, XT60** ([Hawks-Work product link][src-3s-lipo]) | Default `BatteryConfig` values |
| Props | **9450** self-tightening (9.4" × 5" pitch, CW + CCW) | Slightly smaller than DJI's stock 1045 — effect on hover power is ~5–8 %, within our model tolerance |
| Flight controller | Pixhawk 2.4.8 (open-source PX4 autopilot) | Doesn't affect physics calibration |
| Camera | e-con Systems STEEReoCAM Nano (2× OV2311 stereo, 100 mm baseline, 0.95 – 8 m depth range, **HFOV 54° / VFOV 49.5° / DFOV 77.9°**, 4.3 mm f/2.8 M12 lens, 6-axis on-board IMU) | See `docs/camera_specs/` for full datasheet + lens datasheet |
| Companion computer | NVIDIA Jetson Nano | Adds ~140 g payload |
| **Total mass** | **~1.9 kg** (frame + motors + ESCs + battery + Pixhawk + Jetson + STEEReoCAM + cabling) | Within Hawk's Work's 1.8 kg max-takeoff spec; our `DroneConfig.mass_kg = 1.9` |

## Camera (STEEReoCAM Nano) — used to derive `sensor_radius`

Full datasheet + lens datasheet are in [`docs/camera_specs/`](camera_specs/). Key numbers, all cited from those documents:

| Parameter | Value | Source |
|---|---|---|
| Lens focal length | 4.3 mm | Lens Datasheet §5 |
| Aperture | f/2.8 | Lens Datasheet §5 |
| **HFOV** | **54°** | Lens Datasheet §5 |
| **VFOV** | **49.5°** | Lens Datasheet §5 |
| DFOV | 77.9° | Lens Datasheet §5 |
| Image sensor | 2× OmniVision OV2311, 1/2.9", monochrome global shutter | Datasheet §3.2 |
| Sensor active area | 4.857 × 3.956 mm | Datasheet §3.2 |
| Pixel size | 3 µm × 3 µm | Datasheet §3.2 |
| Per-eye resolution | 1600 × 1300 (2 MP) @ 30 fps | Datasheet §1 |
| Stereo baseline | 100 mm | Datasheet §1 (mechanical drawing) |
| **Stereo depth range** | **0.95 – 8 m** | e-con product page / FAQ |
| On-board IMU | 6-axis (3D accel + 3D gyro) | Datasheet §3 |
| Lens mount | M12 (S-mount), pre-calibrated lens pair | Datasheet §3.1 |

### What the camera actually sees at altitude *h*

The camera's ground footprint is a **rectangle**, not a disc. From the FOV values above, at altitude *h* (AGL) the rectangle dimensions are:

```
half-width  = h × tan(54°/2)   = h × tan(27°)    = h × 0.5095
half-height = h × tan(49.5°/2) = h × tan(24.75°) = h × 0.4607
diagonal    = h × tan(77.9°/2) = h × tan(38.95°) = h × 0.6868     ← farthest visible corner
```

So at any altitude there are three meaningful "radial distances":

| From drone | What's there | Formula | At h = 16.3 m | At h = 8 m | At h = 5 m |
|---|---|---|---|---|---|
| 0 → `h × 0.461` | **Guaranteed in view in all directions** (= inscribed disc, our `sensor_radius`) | `h × tan(VFOV/2)` | **7.51 m** | 3.69 m | 2.30 m |
| `h × 0.461` → `h × 0.510` | In view along the H-axis (rectangle's longer side) | `h × tan(HFOV/2)` | 8.30 m | 4.08 m | 2.55 m |
| `h × 0.510` → `h × 0.687` | In view only at the rectangle's corners | `h × tan(DFOV/2)` | **11.19 m** | 5.49 m | 3.43 m |
| **beyond `h × 0.687`** | **Not in the camera's view at all** — | | | | |

The **diagonal radius (`h × 0.687`)** is the maximum distance the camera can see at altitude *h*. The **inscribed radius (`h × 0.461`)** is the largest disc that's entirely inside the rectangular FOV. The simulator uses the inscribed disc because:

1. **Honest coverage semantics.** "Cell is covered" must mean "the camera definitely photographed this cell". A disc bigger than inscribed would mark cells at the corner-extremes covered when those cells aren't in view on the H- or V-axis directions.
2. **Worst-case bound.** Whatever direction the drone happens to be facing, the inscribed-disc cells are guaranteed in frame.

If you'd rather model the *farthest* visible point, use `sensor_radius_from_altitude(altitude, shape="diagonal")`. The `equivalent_area` option splits the difference (disc area = rectangle area).

### How far can the camera *visually* resolve detail? (Ground sampling distance)

The camera's "visual range" isn't a fixed number — it depends on how much detail you need. **Ground sampling distance (GSD)** is the meters of ground each pixel represents:

```
GSD = h × pixel_pitch / focal_length = h × 3 µm / 4.3 mm = h × 0.698 mm per meter of altitude
```

| Altitude | GSD | Smallest detectable feature (~3 px) |
|---|---|---|
| 5 m | 3.5 mm/pixel | ~1 cm |
| 8 m | 5.6 mm/pixel | ~1.7 cm |
| 16.3 m (default) | 11.4 mm/pixel | ~3.4 cm |
| 30 m | 20.9 mm/pixel | ~6.3 cm |
| 50 m | 34.9 mm/pixel | ~10.5 cm |
| 100 m | 69.8 mm/pixel | ~21 cm |

So the camera could in principle visually cover the ground from 50 m AGL or higher, with the practical limit being whatever feature size your mission needs to resolve. There is **no hard "max visual range" in the camera spec** — only the depth range has one.

### Stereo depth range (separate from visual)

The **0.95 – 8 m** range applies only to **stereo-derived depth** (i.e., 3D position of points the camera sees). It comes from the 100 mm stereo baseline geometry:

- **Below 0.95 m**: the two stereo views diverge too much (large parallax), correspondence matching fails.
- **Above 8 m**: parallax drops below the disparity-resolution threshold (sub-pixel for our 3 µm pixels), so depth becomes too noisy to use.

Visual coverage (does the cell appear in any image?) **works at any altitude**. Depth-localized observations (where in 3D is this thing?) require `0.95 m ≤ altitude ≤ 8 m`.

For mapping use-cases that need depth (3D reconstruction, obstacle avoidance, terrain modeling), constrain flight altitude to ≤ 8 m. For pure visual coverage (the question this simulator answers), altitude is unconstrained by the camera — only by mission requirements (target detail, area to cover, battery).

### Why specifically `sensor_radius = 7.5 m` (= 1.5 cells × 5 m/cell)?

Two compounding choices:

1. **`meters_per_cell = 5.0`** was picked so the discrete grid resolves features at sensor scale — see [`simulation-model.md` → World scale](simulation-model.md#world-scale) for the trade-off vs. picking the cell size from drone-body or max-speed.
2. **`sensor_radius = 1.5 cells`** is the smallest integer-and-a-half value that gives the discrete sensor disc more than a single cell (at 1.5 cells centered on a cell, the disc covers 9 cells: the center + 4 axis-adjacent + 4 diagonal-adjacent — verified in [`test_distance.py`](verification.md#test_distancepy)).

Multiplying: `1.5 cells × 5 m/cell = 7.5 m`. Working backwards through the inscribed-disc formula: `h = 7.5 / 0.461 = 16.3 m AGL`. That's the altitude this default implicitly assumes. **The 7.5 m number didn't come from "we want the camera to see 7.5 m"; it came from clean cell-grid math, with the implicit altitude derived afterward.**

If you want the model to reflect a different operating altitude, use the helper:

```python
from environment import sensor_radius_from_altitude, SimConfig, DroneConfig

# Example: depth-reliable coverage at the camera's max useful altitude (8 m)
r_m = sensor_radius_from_altitude(altitude_m=8.0, shape="inscribed")  # → 3.69 m
mpc = 2.5  # tighter cell size for the smaller footprint
sim = SimConfig(meters_per_cell=mpc, flight_altitude_m=8.0)
drone = DroneConfig(sensor_radius=r_m / mpc)  # → 1.48 cells

# Example: max visible (diagonal corner) instead of inscribed-disc conservative
r_max = sensor_radius_from_altitude(altitude_m=16.3, shape="diagonal")  # → 11.19 m
```

`sensor_radius_from_altitude()` takes `shape` ∈ `{"inscribed", "equivalent_area", "diagonal"}`. Note that changing `meters_per_cell` requires recalibrating the energy model's `motion_coeff_w_per_v2 = 0.51 × meters_per_cell²` — the verification scripts will catch any drift.

## Battery options the F450 supports

The F450 frame officially accepts **3S or 4S LiPo** packs (per both [Hawk's Work product spec](https://www.hawks-work.com/pages/f450-drone) and [DJI ARF spec](https://www.dji.com/flame-wheel-arf/spec) — same airframe class). Anything outside 3S–4S is out of spec for typical 920 KV motors and 20 A ESCs.

| Pack | Voltage | Common capacities | Prop size | Notes |
|---|---|---|---|---|
| **3S LiPo** (this project) | 11.1 V nominal (12.6 V full → 9.0 V empty) | 2200 / 2700 / 4000 / 4200 / 5000 mAh | 9450 (Hawk's Work stock) or 1045 | Hawk's Work F450 ships with 9450 props on 3S. Hover power ≈ 145–185 W depending on mass and prop choice. |
| **4S LiPo** (alternative) | 14.8 V nominal (16.8 V full → 12.0 V empty) | 3300 / 4000 / 6000 mAh | **8" required** (NOT 9450 or 1045) | Faster forward flight and more thrust headroom; mandatory smaller props because 9450/10" at 4S overspeed and overheat 920 KV motors. |

**Currently modelled in the sim**: the **Hawk's Work F450 reference pack — 3S 4200 mAh, 11.1 V nominal**, set in `BatteryConfig` defaults in `environment.py`. That's what every demo and verification script uses unless you override:

```python
from environment import CoverageEnv, BatteryConfig
env = CoverageEnv(
    grid=grid, n_drones=4,
    battery=BatteryConfig(voltage_v=14.8, capacity_mah=4000),  # → 4S 4000 mAh
)
```

`BatteryConfig.n_cells` auto-derives from `voltage_v / 3.7`, so per-cell full/empty/cutoff voltages all scale correctly — change `voltage_v` to 14.8 V and `cell_full_voltage_v × n_cells` becomes 16.8 V automatically. `hover_power_w` and `motion_coeff_w_per_v2` are calibrated to the 3S 920 KV + 9450 prop combo, though, so a meaningful 4S run would also need them re-tuned against measured 4S+8" data.

## Reference F450 hover times (real-world)

Published hover-time figures from owners, listings, and community threads. Most data is from DJI Flame Wheel F450 users (older, more popular platform), but the airframe class is identical to Hawk's Work F450, so these numbers apply directly to our build. Our 3S 4200 mAh + 1.9 kg config sits in the middle of this table:

| Pack | Payload | Hover time | Source |
|---|---|---|---|
| 4S 6000 mAh | + GoPro | 20+ min | [DroneVibes][src-dronevibes-battery-size] |
| 3S 5000 mAh | light (no gimbal) | ~18 min | [DJI forum / E300 motors][src-dji-forum-flight-time] |
| 3S 5000 mAh | 1.8 kg (gimbal + video tx) | ~15 min | [DJI forum][src-dji-forum-flight-time] |
| 4S 4000 mAh | 1.8 kg (gimbal + video tx) | ~12 min | [DJI forum][src-dji-forum-flight-time] |
| **→ 3S 4200 mAh** | **~1.9 kg (Jetson + camera, OUR CONFIG)** | **12–15 min** | interpolated |
| 3S 4000 mAh | 2212/920 motors + Graupner 10×5 props | ~13 min | [HeliFreak][src-helifreak-battery-choices] |
| 3S 2700 mAh | 1.0 kg (no gimbal) | ~12 min | [DroneVibes][src-dronevibes-battery-size] |
| 3S 2650 mAh | 10" props, light | 8–8.5 min | [HeliFreak][src-helifreak-battery-choices] |
| 3S 2200 mAh | light | ~10 min | [amainhobbies F450 listing][src-f450-times] |

Our model (`P_hover = 165 W`, 10 V cutoff at ~28% remaining) predicts **17.0 min** to a fully empty pack and **12.2 min** to the conservative cutoff — the latter sits inside the published 12–15 min range for our 3S 4200 mAh + 1.9 kg config.

## Cruise (forward flight)

**No F450-specific published cruise-endurance measurement exists** at a defined forward speed — community discussions and academic papers measure either hover or a different airframe. The cruise comparison range is therefore *inferred* from hover endurance + the well-documented multirotor power-vs-speed curve:

| Speed regime | Power vs hover | Endurance vs hover |
|---|---|---|
| 5–9 m/s (translational-lift sweet spot) — **our 7.5 m/s sits here** | **−10 to −20 %** | **+10–20 % (longer)** |
| 9–12 m/s (lift dip ends, drag rising) | ~hover | ~hover |
| 12–15 m/s (drag-dominated, F450 max forward) | +50 to +70 % | −30 to −40 % (~10 min for our pack) |

**Best-estimate cruise range for our 7.5 m/s + 3S 4200 mAh + 1.9 kg config: 12–18 minutes** (= hover × [1.00 … 1.20]). The lower bound is "no translational-lift benefit"; the upper bound is the +20 % gain reported in community measurements.

Sources for the speed-vs-power curve: [Quadcopter Flight School: hover vs. forward flight power efficiency][src-translational-lift], [DJI forum — translational lift for endurance][src-dji-translational-lift], [ArduPilot Discourse — community hover vs. cruise current measurements][src-ardupilot-power-feedback] (a 680 mm quad reported identical 10 A current at hover and level cruise — consistent with the translational-lift dip cancelling the `v²` rise), [Bauersfeld & Scaramuzza, 2021 — Range, Endurance, and Optimal Speed Estimates for Multicopters][src-bauersfeld] (first-principles model validated up to 65 km/h on a 6-inch frame; their power curve is the basis for our `k = 13` calibration at the 15 m/s extreme).

**Our sim is intentionally pessimistic at cruise.** The energy model uses `P = P_hover + k·v²` (monotone in speed) and omits the translational-lift dip — see [`battery-model.md` → Why `v²` and not `v³`](battery-model.md#why-v²-and-not-v³). At our `max_speed = 7.5 m/s` (right in the sweet spot), the sim predicts **10.4 min cruise** vs. the inferred real-world 12–18 min. The trade-off is deliberate: a monotone-in-speed cost makes optimization well-behaved, at the cost of slightly under-predicting cruise endurance in the 5–9 m/s band. Controllers that try to game a translational-lift dip won't win anything in this sim.

Verified by [`verification_scripts/test_flight_time.py`](verification.md#test_flight_timepy), which runs `env.step()` to depletion in two regimes (hover, cruise at 7.5 m/s), asserts agreement with the analytical formula within 0.5 %, and prints the cruise number alongside the inferred 12–18 min real-world range so the gap is visible at run time.

## F450 max forward speed and range (no published spec)

Neither **max forward speed** nor **range** are published as official manufacturer specs — they depend on motors / props / battery / payload. The defensible numbers we use:

- **Max forward speed (920 KV motors + 9450 / 10" props + 3S)**: ~15 m/s (~54 km/h, ~33 mph). This is the calibration anchor for our `motion_coeff_w_per_v2 = 13` constant ([Bauersfeld & Scaramuzza, 2021][src-bauersfeld]). One review site quotes 130 km/h (36 m/s) for an F450, but the stock-config evidence points to ~15 m/s; the higher figure likely assumes mods (4S, faster motors, smaller props).
- **Range = endurance × cruise speed**, derived not measured. For our config (3S 4200 mAh, 1.9 kg payload, 7.5 m/s sweet-spot cruise):
  - **Inferred real F450**: 12–18 min cruise endurance × 7.5 m/s = **5.4–8.1 km** one-way.
  - **At F450 max forward (~15 m/s)**: ~10 min × 15 m/s ≈ 9 km full pack (drag-dominated).
  - **Our sim**: 624 s × 7.5 m/s = **4.68 km** — pessimistic by the same translational-lift omission that affects cruise endurance.

[`verification_scripts/test_distance.py`](verification.md#test_distancepy) asserts `range = cruise_endurance × cruise_speed × meters_per_cell` exactly through `env.step` (within 0.5 %, accounting for the final-step cutoff clamp), and prints the inferred-vs-actual range comparison so the pessimism is visible at run time.

[src-f450-times]: https://www.amainhobbies.com/dji-flame-wheel-f450-quadcopter-drone-combo-kit-dji-nzm450c1/p297771
[src-dji-forum-flight-time]: https://forum.dji.com/thread-4327-1-1.html
[src-dronevibes-battery-size]: https://www.dronevibes.com/forums/threads/dji-f450-battery-size.12486/
[src-helifreak-battery-choices]: https://www.helifreak.com/showthread.php?t=406895
[src-dji-translational-lift]: https://forum.dji.com/thread-91091-1-1.html
[src-ardupilot-power-feedback]: https://discuss.ardupilot.org/t/feedback-needed-your-power-consumption-in-hover-climb-cruise-descent-etc/32200
[src-translational-lift]: http://quadcopter101.blogspot.com/2014/02/flight-school-5-power-efficiency-hover.html
[src-bauersfeld]: https://rpg.ifi.uzh.ch/docs/Arxiv21_Bauersfeld.pdf
