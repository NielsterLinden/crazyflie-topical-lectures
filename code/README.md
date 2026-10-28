# Course code

The [day guides](../docs/README.md) and worksheets describe the exercises;
the [glossary](../docs/glossary.md) explains their terminology.
For a guided route, open the [Day 1](andreas/day1_control.ipynb),
[Day 2](day2/day2_kalman.ipynb) or [Day 3](day3/day3_race.ipynb) notebook in
`jupyter lab` from the project root, using **Crazyflie lectures (.venv)**.
Navigate into `code/` to open a notebook; `docs/` remains available for its links.
The notebooks cover the same exercises as the commands below.
After activating the native environment, run from the course project root
(the cloned folder containing `code/`, `scripts/` and the requirements files):

```bash
cd code
python -m day1 --output ../day1/baseline --no-show
python -m unittest andreas.test_practical
python -m day2.simulate
python -m day1.feedforward
python -m unittest kalman.test_filter
```

| Exercise commands | Reusable code |
| --- | --- |
| [day1](day1/README.md): controller experiments and feed-forward comparison | `andreas` controllers/plants and `flight_control` outer-loop models. |
| [day2](day2/README.md): live/replay studies, plots and gate/ZUPT decisions | [kalman](kalman/README.md): filter, callbacks, recording validation and baseline simulation. |
| [day3](day3/README.md): course construction and gate-uncertainty example | [race_common](race_common/README.md): gate geometry/policies; `trajectory_tools`: course construction. |

Run the exercises through the day packages; they use the shared filter, control
and geometry modules. `andreas` contains the Day 1 controllers, toy plants and guided notebook.
`trajectory_tools` supplies [offline course construction and preview](../docs/trajectory_construction.md),
without radio access or flight execution.
The starter's TODOs connect to
the worksheets, and optional profiles describe which estimator and controller
would run on the laptop or drone. Hardware execution uses a separately supplied,
tested flight program and the team's documented preflight checks.

The [Day 2 guide](day2/README.md) documents APIs, assumptions, and
simulation limitations. The [post-flight guide](../docs/post_flight_analysis.md)
describes the standalone offline plotter. Tested live integration
and fallback tools are provided separately when needed.
