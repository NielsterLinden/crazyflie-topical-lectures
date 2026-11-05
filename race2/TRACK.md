# Race 2 at the track: runbook (drone 4, radio://0/30/2M/E7E7E7E704)

`PY` = `../crazyflie-topical-lectures/.venv/bin/python` (run from `team-crazyflie/`). Every flight needs a go.

## 0. Bring
Charged batteries (race plan 150 s + takeoff/landing), Crazyradio, laptop charger, this repo pushed.

## 1. Ask the organisers (2 min)
- Lighthouse setup file (cfclient export `.yaml`), or is it already stored on our drone?
- Arena walls in the Lighthouse frame (x is the 5 m side; we assume x -1.6..3.0, y -2.2..1.6, z <= 1.6).
- Start (0, 0, 1) confirmed, drone faces +x on the floor below it.

## 2. Lighthouse on the ground (no flight)
```
cd code/day1
$PY crazyflie.py lighthouse --label 4 --yaml <organisers.yaml>     # only if they give a file
$PY drone_pipeline.py lh_check                                     # want LIGHTHOUSE OK: status 2, >= 2 stations, still
```
Place the drone at the start mark facing +x. Check printed x, y close to the mark (0, 0) and yaw ~0.
Hold it by hand at hoop 2 (centre -0.50, -0.78, 0.43) and re-run lh_check: estimate within ~5 cm = frame matches.

## 3. Preflight
`$PY drone_pipeline.py preflight vbat_min=4.10` (USB unplugged, on the floor, facing +x).

## 4. If bounds/start change
Edit `race2/make_course.py` (start / bounds_min/max), then
```
f=$($PY race2/make_course.py 1); (cd code && $PY -c "import sys; from day3.notebook_tools import plan_race, save_plan; from trajectory_tools.course import read_json
p = plan_race(read_json(sys.argv[1]), provenance={'source_course': sys.argv[1]}); save_plan(p, sys.argv[2]); print(round(p.summary()['planned_duration_s'], 1), 's')" ../$f ../race2/plan_cons)
$PY code/day3/hoop_view.py race2/plan_cons/course.json --plan race2/plan_cons --png race2/plan_cons/hoops.png
```
Look at the picture: arrows = flying direction (3: +y, 2: +x, 1: up), path inside the red box.

## 5. Fly
Organisers' runner (03_flight.ipynb, RACE_READY.md step 8) with `race2/plan_cons/`. Copy the folder, set
`enabled_for_flight: true` in the copy only, dry check must print TRAJECTORY PROTOCOL 1.

## Known
- Rehearsal on carpet (flow deck, 71-84 % scale): tracking mean 0.10 m, clean climbs to 1.38 m.
- After a crash: reboot (`crazyflie.py reboot --label 4`) resets the yaw estimate; then preflight.
- Our own setpoint landing used to trip the tumble lock; the runner lands itself.
