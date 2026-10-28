# Day 3: race preparation

The front end constructs and previews courses using the shared
[trajectory tools](../../docs/trajectory_construction.md), and shows how
[race geometry/policies](../race_common/README.md) use estimated uncertainty.

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
planner front end. The existing [survey](../../docs/hoop_survey_challenge.md) and
[post-flight plotter](../../docs/post_flight_analysis.md) also have root-level
script commands. Follow their guides for acquisition, checking and analysis.

Use the [race worksheet](../../docs/race_practical.md) for the timed programme,
roles, team flight checks, conservative fallback and limits. Hardware flight uses
the separately supplied tested runner; these commands neither arm nor fly.
