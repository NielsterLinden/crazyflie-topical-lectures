# Course code

The [day guides](../docs/README.md) and worksheets describe the exercises;
the [glossary](../docs/glossary.md) explains their terminology.
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

| Exercise front end | Shared implementation |
| --- | --- |
| [day1](day1/README.md): controller experiments and feed-forward comparison | `andreas` controllers/plants and `flight_control` outer-loop models. |
| [day2](day2/README.md): live/replay studies, plots and gate/ZUPT decisions | [kalman](kalman/README.md): filter, callbacks, recording validation and baseline simulation. |
| [day3](day3/README.md): course construction and gate-uncertainty example | [race_common](race_common/README.md): gate geometry/policies; `trajectory_tools`: course construction. |

The day packages are the entry points; the shared back ends contain reusable
computations. `andreas` contains the Day 1 controllers, toy plants and guided notebook.
`trajectory_tools` supplies [offline course construction and preview](../docs/trajectory_construction.md),
without radio access or flight execution.
The starter's TODOs connect to
the worksheets, and optional profiles describe possible fallback/integration
choices. Hardware execution uses a separately supplied, qualified flight runner
and the team's documented preflight checks.

The [Day 2 guide](day2/README.md) documents APIs, assumptions, and
simulation limitations. The [post-flight guide](../docs/post_flight_analysis.md)
describes the standalone offline plotter. Tested live integration
and fallback tools are provided separately when needed.
