# Handover: drone 4 calibration and tuning (state 4 Nov 2026, ~16:00)

For the next agent who takes over from here. Read this file first, then the docstring at the top of
`drone_pipeline.py`. The visual version of this state is `drone_roadmap.html`, also published (public) at
https://claude.ai/artifact/S186mYQyL8f8hYWN44vBrY.

## Rules (non-negotiable)

- **Never fly without Niels's explicit go for that specific flight.** One go per flight. Before asking, say
  what the flight does: arm, take-off height, path or step, fence, how to abort. Ground commands need no go:
  `preflight`, `plan`, `table`, `key_test`, `abort_test`, `tune ... model=True`, `crazyflie.py status / flow / params`.
- **Only drone label 4, `radio://0/30/2M/E7E7E7E704`.** Other groups' drones are in radio range: never connect
  to them. Another group once connected to our address and flew/crashed our drone. If the drone does something
  nobody commanded, abort and check.
- Every flight: pilot clicked into the abort window (Enter or Esc lands), a second person watching, nobody
  within arm's length, 1.5 m clear around it.
- Commit or push only when Niels asks. In this repo (team fork `clvaneck/crazyflie-topical-lectures`) Niels
  has no push rights: push to the `fork` remote (`NielsterLinden/crazyflie-topical-lectures`) and open a PR
  into `caspar-tuning`, as PR #1 and this branch did.

## Setup

```bash
cd code/day1
PY=../../.venv/bin/python        # the course venv: Python 3.12, built by `bash scripts/setup-unix.sh`
# on Niels's laptop this clone has no venv; use the GitLab course clone next to it:
PY="../../../crazyflie-topical-lectures/.venv/bin/python"
$PY testing/test_drone_pipeline.py   # dry run on the course's fake drone, ~40 s: "all checks passed"
```

Until now the pipeline ran in Niels's GitLab course clone (`crazyflie-topical-lectures/code/day1`). This
branch copies the code and all of today's data into the team repo. Use this copy from now on, so the data stays
in one place.

## Where we are

Calibration of drone 4 (flow deck, over carpet), in `data/drone_4_params.json`:
tau 0.13 s, dead time 0.18 s, noise bounds |Kp| < 2.1, |Kd| < 2.9; hover noise x 10 mm, y 24 mm.

| Run (`data/flight_4_<run>_*.csv`) | What | Result |
|---|---|---|
| 1533, 1535 `firmware_fwd` | firmware path flights | 1533 flew at 95 deg yaw and went sideways: the yaw lesson below |
| 1537, 1546 `helix` | `extra_drone_helix` notebook runs | not part of this pipeline |
| 1520 `firmware_hold` | hover over the plain floor | **invalid**: drifted ~4 m, the flow estimate saw 15 cm |
| 1540 `firmware_hold` | hover over the carpet | near perfect: std x 9.6 mm, y 2.7 mm |
| 1543 `identify` | two 0.2 m/s velocity steps | the calibration above |
| 1545 `kp0.7` | `tune -0.7` | rise 3.40 s (model 2.88), no overshoot, mean error last 2 s 6.7 cm |
| 1546 `kp1.4` | `tune -1.4` | rise 2.34 s (model 1.86), overshoot 2.6 cm, mean error 2.5 cm, 37 % at the 0.3 m/s clamp |

The model (`plan`) ranks Kp −2.0 best (rise 1.78 s, overshoot 1.7 cm). −4 and −8 exceed the noise bound.
Measured runs are about 0.5 s slower than the model, partly because of the V_MAX = 0.3 m/s clamp.

**Blocked:** Niels approved `tune -2.0`, but preflight refused twice with "position estimate still on the
ground" (drift 0.054 m, then 0.153 m in 2 s). Nothing flew. Likely cause: the drone sits in the carpet pile,
the height sensor reads 0.000 m, and the flow estimate runs away. Fix to try: a thin flat card or book under the
drone at the take-off spot. In the air the carpet texture works well (the last three flights).

Battery: 16 % (3.81 V at rest) after those flights. **Recharge before the next series.**

No `best` gains are saved yet. Niels was asked whether to save −1.4 as the provisional best and has not
answered.

## Next steps, in order

1. Recharge with USB in. To watch it: `$PY -u crazyflie.py charge --label 4` (`-u`, or the output stays
   buffered). Unplug before flying: preflight refuses while the cable is in.
2. On the carpet, put a card under the drone, facing +x. Run `$PY drone_pipeline.py preflight` (ground). It
   must print `PREFLIGHT OK`.
3. **Go needed:** `$PY drone_pipeline.py tune -2.0`. Compare with the model (rise 1.78 s, overshoot 1.7 cm).
4. **Go needed per flight**, around the better of −1.4 and −2.0, all on one charge so they compare:
   - Kd −0.3 (damping) and +0.3: `tune -2.0 -0.3` and `tune -2.0 0.3`.
   - A small Ki: `tune -2.0 0 -0.1`.
   - One repeat of the best.

   Kd and Ki carry the same sign as Kp (`module.PIDController` uses error = value − setpoint). `tune` refuses
   Kp > 0. Use `tune ... model=True` (no radio) to preview a gain set. Compare the flights with `table`.
5. `$PY drone_pipeline.py set_best <Kp> <Kd> <Ki> why=...` (ground).
6. `$PY crazyflie.py params --label 4` (ground). It writes `data/params_4.csv`, which `handover` reads instead
   of the assumed firmware posCtlPid.xKp = 2.0.
7. `$PY drone_pipeline.py handover` (ground) gives:
   - Day 2 E7: the following lag is 1/xKp, 0.5 s at xKp 2.0, against the 0.25 s E7 assumed. Check those margins
     (`code/day2/RESULTS_day2.md`).
   - Day 3 R3: the posCtlPid.xKp/yKp factor, allowed range 0.9–1.1.
8. **Go needed:** the first keyboard flight, `$PY drone_pipeline.py keyboard speed=0.1`.
   - Controls: arrows move it, W/S climb and descend, A/D yaw, Enter lands.
   - Run `key_test` first; it needs no drone.
9. Day 3 race:
   - The runner is not in the repo: its path, Python and directory come from the run card on the shared flight
     laptop. The repo has `code/day3/flight.py` (the launcher) and the bounded tuning file.
   - Gate the race on `drone_pipeline.py preflight vbat_min=4.10`.
   - Check the floor texture in the race area first (see below).

## Pitfalls that cost a flight today

- **Floor texture.** The flow deck needs texture. Over the plain floor the drone drifted 4 m while the estimate,
  and so the 1.5 m fence, saw almost nothing. Before flying on any new floor, run
  `$PY crazyflie.py flow --label 4`, hold the drone ~0.5 m up and move it around. Surface quality should be
  about 100; the carpet gives 110–130.
- **Carpet pile on the ground** blinds the height sensor at rest: preflight then fails "on the ground". Put a
  card under the drone.
- **Supervisor lock after a crash.** `preflight recover=True` and a power cycle did not clear it.
  `$PY crazyflie.py reboot --label 4` (a soft reboot over the radio) did. Right after the reboot, "is armed:
  yes" is normal, and preflight accepts it.
- **USB attached.** While charging, the voltage reads high. Preflight requires power state "in use".
- **Yaw.** `crazyflie.fly()` turns world-frame errors into body-frame velocity setpoints, which is only right
  while the drone faces +x. Preflight refuses |yaw| > 10 deg after the estimator reset.
  - Open question: the yaw estimate swung −20..+15 deg in the carpet hover, although the drone looked steady.
    Watch `stabilizer.yaw` in the next runs. `_check_heading` warns after a flight if the median exceeds 10 deg.
- **Preflight parses the text of `crazyflie.status()`** as printed in release 0.1.11. If a course update
  changes the wording, preflight fails closed; then update the strings in `preflight()`.
- **Keyboard mode monkeypatches** `crazyflie.start_abort` and `crazyflie.MotionCommander` during the flight,
  and restores them afterwards. Re-run the dry run after any update to `crazyflie.py`.
- The abort window is Qt (PyQt6 via matplotlib) and works under Wayland. Caspar's `crazyflie.py` adds a native
  macOS window. The dry run passes against Caspar's version.

## Files on this branch

- `drone_pipeline.py`: drone_02/04/05 as CLI functions, with preflight, keyboard mode and handover.
- `testing/test_drone_pipeline.py`: the dry run on the fake drone, covering every function including the keyboard.
- `data/drone_4_params.json`: the calibration. `set_best` and `handover` add to it.
- `data/flight_4_15*_*.csv`: today's runs.
- `data/plots/<run>.png`: one plot per run.
- `drone_roadmap.html`: the roadmap page.
  - To update the public artifact, use `edit_version` with `shared_live="public"` and
    `artifact_title="Drone 4 Flight Roadmap"`. Otherwise, edit the HTML only.
