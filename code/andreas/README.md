# Andreas's controller experiments

The Day 1 practical uses a position [PID](../../docs/glossary.md#pd-and-pid), velocity-to-tilt controller,
[cascade](../../docs/glossary.md#cascade), toy plants and recorded-data helpers. Start with the
[Day 1 software guide](../../docs/day_1_software.md),
[worksheet](../../docs/controller_practical.md) and
[guided notebook](day1_control.ipynb).

With the native environment activated, run from the course project root:

```text
cd code
python -m unittest andreas.test_practical
python -m day1 --output ../day1/baseline --no-show
cd ..
jupyter lab code
```

The notebook contains simulation and offline replay only. The runner also offers
a separately confirmed motor-disabled hand capture; none of its modes sends
flight commands. See the guide for assigned radio, restraint, frame and logging
requirements, real-data replay, and the supervised operator handoff.

| File | Purpose |
| --- | --- |
| `hover_pid.py` | Position-to-velocity PID, conditional anti-windup, toy plant and replay. |
| `velocity_pid.py` | Velocity-to-tilt controller, cascade, tilt/drag plant and replay. |
| `practical.py` | Reproducible experiments and saved CSV/PNG/settings. |
| `day1_control.ipynb` | Guided experiments using the same classes/runner. |
| `test_practical.py` | Motor-free regression checks. |

Keep gains/units and onboard versus offboard loop ownership explicit. Day 2
studies the state estimator supplying feedback; Day 3 uses those lessons with
qualified trajectories and a declared command interface. Each velocity, tilt or
acceleration command layer has its own gain units and plant model.
