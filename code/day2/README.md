# Day 2: timestamped sensor fusion

Run `jupyter lab` from the project root and open these in the file browser,
using the Day 1 environment and kernel:

1. [01_tracking.ipynb](01_tracking.ipynb): lecture demonstration and E1.
2. [02_calibration.ipynb](02_calibration.ipynb): E2 stationary IMU calibration.
3. [03_height_velocity.ipynb](03_height_velocity.ipynb): E3-E7, sharing one filter and training recording.

Each notebook starts independently. Set `run_id` for the session; results go to
local `data/<run_id>/` with fresh attempt folders. For live capture, enter the
sticker `LABEL` as on Day 1, or use `CF_URI` with `LABEL=None`. The numbered
notebooks explain the motor-stop checks, data choices and completion questions.
The notebooks are the working instructions; the [filter reference](../../docs/kalman_filter_practical.md) adds conventions and optional studies.

Participants fill four mathematical expressions and two short policy functions.
`vertical.py` supplies timestamps, stationary calibration, replay, diagnostics and
recording validation; `vertical_live.py` captures accelerometer/ToF data with
motor-stop verification and a wall-clock bound. `notebook_tools.py` supplies
capture, plots and file handling. `tracking.py` is the E1 reference implementation.

From `code/`, the corresponding tests and optional CLI capture are:

```text
python -m unittest day2.test_vertical day2.test_tracking day2.test_layout
python -m day2.vertical_live --output day2/data/vertical-training --prepare-for-handling
```

Follow the [recording protocol](../../docs/sensor_recording_contract.md#vertical-sensor-recordings-for-e3-e7)
before handling. Recorded accelerometer/ToF samples are distinct from onboard
position estimates. Failed or incomplete selections remain errors; simulation
is an explicit choice. E6 freezes calibration and settings; E7 uses covariance
from the resulting runs.

## Additional six-state experiments

Open [extra_six_state.ipynb](extra_six_state.ipynb) for the optional model and
recording inspection. Its `extra_notebook_tools.py` helpers load the separate
six-state backend. The [six-state reference](six_state_reference.md) documents
CLI commands, model assumptions, output columns and diagnostics.
