# SITL parameter files

Two files live here:

- **`sitl_params_best.parm`** — known-good baseline. Don't edit casually.
  Contains the most recent config that's been verified to fly stably (or as
  stable as we've gotten). When a new config beats this one in a flight,
  promote it: `cp sitl_params_test.parm sitl_params_best.parm`. This file is
  the rollback target whenever an experiment goes wrong.

- **`sitl_params_test.parm`** — the working file, what SITL actually loads.
  Edit freely while iterating. To start from a known-good baseline:
  `cp sitl_params_best.parm sitl_params_test.parm`.

## SITL launch

The WSL launch command points at `sitl_params_test.parm`:

```
Tools/autotest/sim_vehicle.py -v ArduCopter -f X -N -w \
  --add-param-file=/mnt/c/Users/user1811/Desktop/armen-capstone/Swarm_Drones/sitl_params/sitl_params_test.parm \
  -A '--home 40.192,44.50446,1200,0' \
  --model JSON:192.168.208.1 \
  --map --console --out=udp:127.0.0.1:14551
```

`-w` wipes EEPROM each run, so the param file is the source of truth on
every restart.

## Promote / rollback workflow

```
# Roll back to last known-good after a bad experiment:
cp sitl_params_best.parm sitl_params_test.parm

# Promote current test to best, after a flight has been verified:
cp sitl_params_test.parm sitl_params_best.parm

# Diff (see what's about to change):
diff sitl_params_best.parm sitl_params_test.parm
```
