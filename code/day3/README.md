# Day 3: race preparation

These commands construct and preview courses using the shared
[trajectory tools](../../docs/trajectory_construction.md), and show how
[race geometry/policies](../race_common/README.md) use estimated uncertainty.

## Guided notebook

Run `jupyter lab` from the project root, then open
[day3_race.ipynb](day3_race.ipynb) under `code/day3/` in the file browser. Use the
**Crazyflie lectures (.venv)** kernel. Its cells follow the
existing R1-R7 work: inspect limits, load survey files, choose crossing order and
laps, compare two reference paths, save a disabled course and analyse a recorded
attempt. Compare the reference durations and constraints before choosing a plan.
The notebook keeps each course and trajectory in one validated plan. Failed
replanning invalidates the candidate; export requires a successful plan whose
course and trajectory have not been edited since validation. A previously
validated baseline can be selected explicitly instead.

Use the team's checked template and survey files for race preparation. The
default is a labelled synthetic offline example. No cell arms or commands a
drone; hardware checks and flight stay with the separate tested tools. Choose a
new `run_id` on a repeat. The commands below are the alternative CLI route, not
additional exercises.

From `code/` with the environment activated:

```text
python -m day3 --course ../examples/course.shuttle.synthetic.json --laps 2 --output ../plans/example
python -m day3.uncertainty
```

E7 can also select `--scenario nominal`, `dropout` or `outlier`. Use the same
`--seed` for matched noise, and `--output ../E7/nominal.png --no-show` to save
figures. The [E7 worksheet](../../docs/kalman_filter_practical.md#e7-turn-covariance-into-a-racing-decision)
describes the baseline/candidate comparison.

The first command is a disabled synthetic planning example, not a surveyed
course or a flight runner. The second supports Day 2 E7: gate-plane uncertainty,
clearance and speed choices on the shared estimation baseline. Its speed policy
does not feed back into truth motion.

The repository-root command `python scripts/plan_trajectory.py` uses this same
planner. The existing [survey](../../docs/hoop_survey_challenge.md) and
[post-flight plotter](../../docs/post_flight_analysis.md) also have root-level
script commands. Follow their guides for acquisition, checking and analysis.

Use the [race worksheet](../../docs/race_practical.md) for the timed programme,
roles, team flight checks, conservative fallback and limits. Hardware flight uses
the separately supplied tested runner; these commands neither arm nor fly.
