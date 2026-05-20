# Swarm_Drones

Bridge between Isaac Sim and ArduPilot SITL for the F450 capstone, plus the `capstone/` package for stage tooling and disturbance harness.

## Layout

- `my_drone_simulation/` — Isaac↔SITL bridge (`Nvidia_SITL_connecter.py`), takeoff and mission scripts.
- `capstone/` — stage gates, disturbance profiles, metrics, mission runner.
- `sitl_params/` — ArduCopter SITL param files; `sitl_params_test.parm` is the active one loaded by the WSL launch command.
- `scene/` — drone USDs and AUA world (`AUA_world_500m.usd` + `aua_bake/`). See **Heavy assets** below for the textures download.
- `swarm_optimization/` — 2D swarm coverage testbed and benchmarking experiments (see [`swarm_optimization/README.md`](swarm_optimization/README.md)).
- `docs/` — hardware reference docs shared across all sub-projects: [`f450-reference.md`](docs/f450-reference.md) (F450 specs, motor convention, flight-time data), [`battery-model.md`](docs/battery-model.md) (energy model, 3S/4S battery options).

## Heavy assets (not in git)

Some assets are too large to track in git. They live in the shared Google Drive folder:

**https://drive.google.com/drive/folders/1YoCJpJJ7Y0DpSdNDwIO5RWdEnv13tBaf?usp=drive_link**

Required for a textured AUA world (otherwise terrain renders untextured but everything still loads and physics/RL/SITL work fine):

- `textures/` — 1.3 GB, 13,341 PNGs. Place at `scene/aua_bake/textures/` so `aua_terrain_500m_active.usd` can resolve `tex_NNNNN.png` references.

The same Drive folder also archives optional/historical backup files (`scene/_backup/` is gitignored) — alternative drone USD packages, the source URDF, the heavy/2 km terrain variants if they get re-uploaded, and Isaac Sim debug snapshots. Pull only what you need.

## How the USDs were built

The world and drone USDs in `scene/` were not authored by hand. They came out of three different pipelines, each of which is archived under `scene/_backup/aua_bake_source/` for the bake scripts or `scene/_backup/` for the drone source. None of these need to run at simulation time — the baked USDs are self-contained.

- **AUA terrain (`aua_terrain_500m_active.usd`).** Cesium 3D Tiles were crawled from a Google Photorealistic Tileset around the AUA campus (`crawl.py` + `root.json` walk the tileset and pull the `.glb` leaves). The tiles were imported into **Blender**, materials baked to per-mesh PNG textures, and the result exported to USD via `bake_to_usd.py` (which deduplicates textures into the shared `textures/` folder). `subset_from_2km.py` then carves out the 500 m slim variant from the larger 2 km bake — the active terrain is a subset of the 13,341-mesh original.
- **AUA buildings (`aua_buildings_500m.usd`).** Building footprints come from **OpenStreetMap** polygons (`fetch_osm_buildings.py`, radii 500 m / 2 km, with `manual_buildings.json` patching missing structures). Heights are sampled from **SRTM** elevation rasters and reconciled against the Cesium terrain (the empirical +18.76 m geoid-vs-ellipsoid lift is documented in `aua_scene_500m.usda`'s header). Footprints were extruded in Blender and exported as USD by `bake_buildings.py`.
- **Hawks Work F450 drone (`scene/hawks_work_f450_basefile/`).** Source artifact is `scene/_backup/hawks_work_f450_basefile.urdf`. It was imported through Isaac Sim's URDF importer extension, which produces the `_base.usd` / `_physics.usd` / `_robot.usd` / `_sensor.usd` package structure visible under `scene/hawks_work_f450_basefile/configuration/`. Post-import edits (rescale, motor reorientation, arched legs) are recorded in the `.bak_*` snapshots next to `_base_F450scaled.usd` and reproduced by `tools/replace_legs_with_arches.py`.

## Motor convention (ArduPilot QuadX)

The bridge sends/receives 4 PWM channels in this order, and assumes these spin directions. Verified against `ardupilot/libraries/AP_Motors/AP_MotorsMatrix.cpp:592-601` (`MOTOR_FRAME_TYPE_X`).

| Channel | SERVO | Position | Spin (viewed from above) | Angle from forward |
|--------:|------:|----------|--------------------------|-------------------:|
| pwm[0] | SERVO1 | front-right (FR) | CCW | +45° |
| pwm[1] | SERVO2 | rear-left  (RL) | CCW | -135° |
| pwm[2] | SERVO3 | front-left (FL) | CW  | -45° |
| pwm[3] | SERVO4 | rear-right (RR) | CW  | +135° |

In the bridge:
- `MOTOR_LINK_PATHS` is in `[FR, RL, FL, RR]` order to match.
- `MOTOR_SPIN_DIR = [-1, -1, +1, +1]` is the **body reaction-torque sign** (CCW prop pushes the body CW = -Z body, so CCW prop → -1).
- `ROTOR_SPIN_SIGN = -MOTOR_SPIN_DIR = [+1, +1, -1, -1]` is the **rotor's own angular velocity sign in body +Z**, used for visual propeller spin.

If you ever rewire motors or change frame type, both the bridge constants and this table must be updated together.

## Network ports

- UDP 9002 — Isaac↔SITL state and PWM (JSON FDM).
- UDP 14551 — MAVLink to ArduCopter SITL (used by `arm_takeoff.py`, mission scripts, `capstone/missions/runner.py`).

## SITL launch

Run from WSL inside `ardupilot/`:

```bash
Tools/autotest/sim_vehicle.py -v ArduCopter -f X -N -w \
  --add-param-file=/mnt/c/Users/user1811/Desktop/armen-capstone/Swarm_Drones/sitl_params/sitl_params_test.parm \
  -A '--home 40.192,44.50446,1200,0' \
  --model JSON:192.168.208.1 \
  --map --console \
  --out=udp:127.0.0.1:14551
```

---

## Swarm coverage optimization (`swarm_optimization/`)

A NumPy + matplotlib 2D testbed for evaluating swarm coverage algorithms against the same F450-calibrated physics model used in Isaac. Algorithms are benchmarked on three maps (open / partial / dense obstacles, 33×33 cells at 5 m/cell) across 4 swarm sizes × 3 seeds = 36 runs each.

Three research tracks were implemented and fully benchmarked:

| Track | Family | Winner |
|---|---|---|
| 1 | Classical geometry-based (Boustrophedon, Spiral, VoronoiPartition, GridDecomposition, STC) | **VoronoiPartition** |
| 2 | Metaheuristic (PSO, GA, ACO, SA, GWO) | **SA** (98.0 % mean coverage) |
| 3 | Learning / control-based (Potential Fields, Consensus, MARL PPO) | **MARL** on hardest map×swarm cell |

All configs tuned via **Bayesian Optimization** (Optuna TPE, 30 trials each). Cross-track leaderboards and per-algorithm CSVs live in `swarm_optimization/outputs/leaderboards/`.

**Quick start:**

```bash
cd swarm_optimization
python3 -m venv .optim_env && source .optim_env/bin/activate
pip install -r requirements.txt
python3 tools/demo.py --gui --policy voronoi --drones 5 --map-file maps/open_33.npy --all-active --seed 1
```

Full documentation in [`swarm_optimization/README.md`](swarm_optimization/README.md).
