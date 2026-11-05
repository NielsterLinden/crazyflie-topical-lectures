# Team Hoop Storm, submission v01

- Team: Hoop Storm
- Drone: LABEL 4, `radio://0/30/2M/E7E7E7E704`
- Course: `race2-hoops-321`, built from `race2/plan_cons` (conservative profile, `trajectory_version: 1`):
  7 crossings (hoops 3-2-1-3-2-1-3), planned 150.4 s plus takeoff/landing, peak 0.24 m/s, 0.50 m/s^2.
- `baseline/` and `optimized/` carry the **same trajectory** (course.json, summary.json, trajectory.csv are
  byte-identical to `race2/plan_cons`). We have no faster validated plan.
- `optimized/`: `posCtlPid.xKp/yKp` x 0.95 (parameter group `position_p_xy`, range [0.95, 1.0]): slightly softer
  position loop. Trajectory identical to baseline.
- Flight is disabled (`enabled_for_flight: false`). The flight laptop enables it in a copy only.

Baseline gains captured from drone 4 by radio (motors stopped) into `submissions/baseline-drone4/`.
