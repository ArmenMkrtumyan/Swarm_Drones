# SITL parameter file

`params.parm` is the single ArduCopter SITL parameter set loaded by the WSL launch command. It captures the verified-stable F450 PID configuration; edit freely while iterating, and the SITL `-w` flag (wipe EEPROM) guarantees the file is the source of truth on every restart.

## SITL launch

```bash
Tools/autotest/sim_vehicle.py -v ArduCopter -f X -N -w \
  --add-param-file=/mnt/c/Users/user1811/Desktop/armen-capstone/Swarm_Drones/sitl/params.parm \
  -A '--home 40.192,44.50446,1200,0' \
  --model JSON:192.168.208.1 \
  --map --console --out=udp:127.0.0.1:14551
```

## Editing safely

`params.parm` is plain text — open in any editor and tweak. The header comment block inside the file documents the reference flight that locked this config.

When iterating on gains, the safest workflow is:

```bash
# 1. Snapshot before changing
cp params.parm params.parm.bak

# 2. Edit, restart SITL, fly. If it works, delete the backup.
# 3. If it doesn't, roll back:
cp params.parm.bak params.parm
```

Or commit your change to git first and roll back with `git checkout -- params.parm` if needed.
