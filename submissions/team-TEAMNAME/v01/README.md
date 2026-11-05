# Team TEAMNAME, submission v01

- Team: TEAMNAME (TODO: fill in, and rename the folder `team-TEAMNAME`)
- Drone: LABEL 4, `radio://0/30/2M/E7E7E7E704`
- Course: `race2-hoops-321`, built from `race2/plan_cons` (conservative profile, `trajectory_version: 1`):
  7 crossings (hoops 3-2-1-3-2-1-3), planned 150.4 s plus takeoff/landing, peak 0.24 m/s, 0.50 m/s^2.
- `baseline/` and `optimized/` carry the **same trajectory** (course.json, summary.json, trajectory.csv are
  byte-identical to `race2/plan_cons`). We have no faster validated plan.
- `optimized/`: TODO, one of
  - identical to baseline (no gain change), or
  - `posCtlPid.xKp/yKp` x 1.1 (range [1, 1.1]): following lag 1/xKp drops from 0.50 s to 0.45 s.
- Flight is disabled (`enabled_for_flight: false`). The flight laptop enables it in a copy only.

## TODO before handover: tuning.json (needs the drone, motors stopped, no flight)

The `tuning.json` files need this drone's captured baseline gains. Drone on the ground, Crazyradio
plugged in (the tuning file must hold the `radio://` URI; a `usb://` capture is rejected), from `team-crazyflie/`:

```
mkdir -p submissions/baseline-drone4
(cd code && ../../crazyflie-topical-lectures/.venv/bin/python -u -m day3.tuning \
    --uri radio://0/30/2M/E7E7E7E704 --output ../submissions/baseline-drone4/baseline.json)
../crazyflie-topical-lectures/.venv/bin/python submissions/make_v01.py submissions/baseline-drone4/baseline.json        # optimized = baseline
# or:  ... make_v01.py submissions/baseline-drone4/baseline.json 1.1                                                   # optimized = xKp/yKp x 1.1
```

The capture leaves firmware stop on: power-cycle the drone before preflight.
