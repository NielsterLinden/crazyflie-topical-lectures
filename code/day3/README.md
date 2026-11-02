# Day 3: race

Start `jupyter lab` from the project root and open
[day3_race.ipynb](day3_race.ipynb) in the file browser, using
**Crazyflie lectures (.venv)**.

The notebook is the main route: check shared gates, choose margins from Day 1
tracking and Day 2 covariance, compare one speed change, then analyse a recording.
Prefer actual telemetry; the labelled delayed-reference example also works offline.
Choose a fresh `run_id` for another full session.
For surveying, set `LABEL` to your drone's sticker label and confirm the blinking
drone. The notebook verifies the firmware stop and prints the survey command.
Power-cycle after handling and repeat preflight before flight.

## Optional command-line route

From the project root, the same planner can save a disabled course, reference
CSV, summary and preview:

```text
python scripts/plan_trajectory.py --course examples/course.shuttle.synthetic.json --laps 2 --output plans/example
```

Use `--help` for measured surveys and crossing order.
The [planner reference](../../docs/trajectory_construction.md) gives mathematics,
limitations and optional extensions.

Day 2 E7's uncertainty example is also available from `code/`:
`python -m day3.uncertainty --scenario dropout`. Its speed recommendation does
not change the prescribed motion.
