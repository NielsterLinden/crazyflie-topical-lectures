# Day 1 software and bring-up

We use Andreas's controllers and toy [plants](glossary.md#plant) to compare position and velocity
feedback, then replay recordings and check signs with motors disabled. The
runner saves results from the same controller classes throughout. Follow the
[C1-C7 worksheet](controller_practical.md); the [short notes template](templates/controller_report.md)
is optional.

## Start in the native environment

Use the [installation and activation guide](software_setup.md#activate-and-check).
The simulations use NumPy and Matplotlib; the guided notebook runs in Jupyter.
The radio-only hand check also uses cflib. These packages are installed by the
setup scripts. From the repository/package root:

```text
cd code
python -m unittest andreas.test_practical
python -m day1 --output ../day1/baseline --no-show
cd ..
jupyter lab code
```

Open `andreas/day1_control.ipynb` in Jupyter for the same experiments; it contains
no flight commands. The single-line CLI commands work in Bash and PowerShell.
Use a **new directory** for each run and keep failed runs too. Omit `--no-show`
for interactive plots.

## Controller layers and units

The controller experiments support the following exercises:

| Model or tool | Use in the programme |
| --- | --- |
| Position PID and velocity-lag simulation | C1-C3 model, baseline and gain studies; C5 integral action; C6 noise/limiting. |
| Velocity PID, tilt/drag plant and cascade | C4 nested-loop experiments, then C6 noise/authority checks. |
| Recorded-flight replay helpers | C7 computed-command sign/scale checks on immutable recordings. |
| Motor-disabled hand tests | C7's confirmed logging-only capture and analysis. |
| Settled-hover, step and landing procedure | Optional controller application plus B5 power/motor/attitude and step-response evidence, using a qualified operator interface. |
| Tilt-versus-acceleration and lag/drag analysis | The [system-identification](glossary.md#system-identification) extension below; fitted parameters need validation on the configured drone. |

| Experiment | Participant controller | What remains onboard in an approved flight |
| --- | --- | --- |
| Position loop | `hover_pid.AxisPID` / `HoverPID`: [PID feedback](glossary.md#pd-and-pid) from position error and measured velocity to velocity command in m/s. | Velocity, attitude and height control. |
| Velocity loop | `velocity_pid.AxisVelocityPID`: velocity error to tilt toward the positive axis, in degrees. | Attitude and height control. |
| Cascade | `CascadePID` connects those two controllers; the toy simulation uses their single-axis equivalents. | Attitude and height control. |

For the position loop, `u_v = kp*(r-x) - kd*v + ki*integral(r-x) dt`.
`kp` has units 1/s, `kd` is dimensionless, and `ki` has units 1/s2. For a fixed
target and the ideal first-order velocity plant, PD gives
`tau*e'' + (1+kd)*e' + kp*e = 0`. This velocity-command model and the later
acceleration-command double integrator have different gain units and dynamics.

The tilt plant is `tau_att*tilt' + tilt = tilt_command` and
`v' = g*tan(tilt) - drag*v`. Here `kv` has units deg/(m/s), and `kvi` deg/m.
The default `tau=0.30 s`, `tau_att=0.20 s`, and `drag=0.25 /s` are model
assumptions for these experiments. Applying them
to a particular airframe requires validation. Plant integration is explicit and
updates velocity before position. Saved states precede the next integration
step; rerun with smaller dt.

## Experiments and outputs

Each simulation run produces `position.csv`, `velocity.csv`, `cascade.csv`,
`comparison.png`, and `settings.json` with gains, limits, seed and metrics.
Targets remain zero for 4 s, then step by 0.5 m (position) or 0.3 m/s (velocity).
The figure separates m/s and degree command scales. The CSV retains commands,
intermediate velocity targets and true versus noise-perturbed states.

Run from `code/` after activation:

```text
python -m day1 --kd 0 --output ../day1/lower-damping --no-show
python -m day1 --kd 1.2 --output ../day1/higher-damping --no-show
python -m day1 --kv 6 --output ../day1/velocity-low --no-show
python -m day1 --kv 18 --output ../day1/velocity-high --no-show
python -m day1 --bias 0.08 --output ../day1/disturbance-pd --no-show
python -m day1 --bias 0.08 --ki 0.4 --output ../day1/disturbance-pid --no-show
python -m day1 --position-noise-std 0.01 --velocity-noise-std 0.03 --seed 20 --output ../day1/noise-20 --no-show
```

These settings define simulation experiments. Hardware gains require separate
qualification. `--bias` adds a constant velocity disturbance to the
position-loop plant: `x' = v + bias`, where `v` is the lagged velocity state.
The physical position derivative therefore includes both terms. Compare
disturbance rejection using this explicitly defined model.
`--ki` and `--kvi` enable the supplied integral terms with conditional
[anti-windup](glossary.md#anti-windup);
study their bounds, recovery and failure cases. Noise perturbs controller inputs
only, leaving truth separately available. Noise units are m and m/s respectively;
the CLI's seed supports the shared development/held-out protocol.

[RMS](glossary.md#rms-and-rmse)/max/tail errors are in m for position/cascade and m/s for velocity. Tail means
the final 3 s. Derive rise, overshoot,
[settling](glossary.md#response-metrics), command variability and saturation fractions from the saved time series,
stating the interval. Velocity-squared effort has units m2/s; tilt-squared effort
has units deg2*s. These integrated squared-command measures describe effort at
their respective command layers; electrical energy would require a power model.

## Replay and motor-disabled sign checks

First verify the file format without hardware, from `code/`:

```text
python -m day1 --mode replay --recording ../examples/day1-state.synthetic.csv --output ../day1/replay-format --no-show
```

The fixture is synthetic and checks software only. For experimental evidence,
use a supplied real recording or a motor-disabled capture after the team's checks:

```text
python -m day1 --mode handcheck --duration 20 --output ../day1/handcheck --no-show
```

Set the assigned `CF_URI` first (or supply `--uri`). Close cfclient, disable motors
and remove propellers or use a secure restraint that prevents propeller contact.
The command asks for
`RECORD ONLY`, logs position/velocity and motor-status blocks, and computes commands
after capture. It **never arms, resets the estimator or sends setpoints**.
Nonzero motor commands or stale motor-status telemetry reject the capture; these
checks do not replace physical restraint or an independent stop operator.

Hold still at the initial point, move slowly along +X, stop displaced, return,
and repeat in Y. Position-loop output should oppose displacement; velocity-loop
output should oppose motion. The cascade holds a corrective tilt while displaced.
This lets us compare the computed feedback responses during hand motion.

Each axis CSV (`x.csv`, `y.csv`) saves `cmd` (position-loop velocity command,
m/s), `velocity_target` (zero m/s), `velocity_tilt_cmd` (standalone velocity-loop
tilt) and `cascade_tilt_cmd` (tilt tracking `cmd`). Both tilt columns are degrees
toward the positive world axis. Mapping to firmware roll/pitch requires the
frame and sign conversion described below. `comparison.png` plots
the two tilt outputs separately labelled, on the degree axis. With integral
gains zero, stopping at a displaced position gives zero standalone tilt but a
restoring cascade tilt. Nonzero integral gains can retain a command after stopping;
use the zero-integral baseline for the sign check.

Capture writes `state.csv` and an `analysis/` directory. Normalized input requires
`t_s,x,y,vx,vy`: one synchronized horizontal state sample per row, seconds relative
to the session origin, world m and m/s. UTF-8 with/without BOM is accepted. Times
must increase strictly; invalid/missing components are rejected. For separate
state and sensor logs, prepare a synchronized CSV with these fields and document
the alignment. Nearest-neighbour matching supports offline diagnostics. Live control
requires a causal adapter using only measurements available at each update.
Hand captures unwrap the 24-bit firmware log time; it is a logging-time proxy.
Record firmware, frame, geometry and calibration provenance separately.

The replay target is the first measured position on each axis. A supplied
recording completes the analysis if hardware is unavailable; mark the hand check
unperformed. EKF velocity is a derived estimate. Replay shows the commands each
controller would compute on that recording; comparing their resulting motion
requires a closed-loop experiment or simulation.

## Supervised flight and the next days

For a system-identification extension, use an instructor-provided synchronized
recording with world velocity, compatible attitude/heading and command channels,
plus timing and calibration provenance. Check the tilt/acceleration
scale check: near level flight, `degrees(1/g)` is about 5.84 deg per m/s2.
Distinguish actual tilt, firmware log signs and world-axis direction; model yaw
and transform before fitting. A lag scan and a stated smoothing/differentiation
method can study drag and inner-loop response, but differentiating an already
filtered EKF velocity is not an independent acceleration measurement. Retain
held-out validation and uncertainty; neither a best-lag correlation nor a fit
to one flight establishes a stable controller. This extension needs additional
analysis time/recordings, rather than being hidden inside C4's 25 minutes.

In the [15:45 first-hover session](day_1_control.md#first-hover-1545-1630), cfclient
sends takeoff and landing commands, while the qualified onboard estimator and
controller handle stabilization and hover. [B5](day_1_baseline_checks.md#b5-battery-and-first-supervised-hover)
describes the procedure and logging.

### Optional extension: fly with the participant controller

This extension has its own time allocation and flight qualification.

The flight sequence is: takeoff after preflight checks, settled hover,
transfer to the participant position PID with a continuous command at the
switch (a [bumpless handover](glossary.md#bumpless-handover)), a bounded one-axis step,
logging, abort/landing, then comparison with the simulation. Instructors provide
the qualified launcher/run card described in the [course interface](course_interface.md).
Its Day 1 velocity-command experiment is distinct from the Day 3 position-command
runner. The card identifies controller class/gains, frames, height convention,
stop path, limits and telemetry. Test these on the configured drone before
applying the participant controller in flight.

Before velocity-command flight, explicitly map world-frame output into the
firmware command frame. With fixed yaw, `u_body = Rz(yaw)^T * u_world` when the
API takes body velocity. The release-pinned
[hover packet decoder](https://github.com/bitcraze/crazyflie-firmware/blob/54f31e243a0b/src/modules/src/crtp_commander_generic.c#L365-L395)
uses body-frame horizontal velocity and absolute world Z; verify the installed
firmware and chosen API against that convention. Likewise map abstract tilt direction to signed firmware
roll/pitch, with yaw and unit conventions checked against the installed firmware.
Never paste degree tilt commands into an acceleration API. Preserve the qualified
onboard estimator; changing its origin invalidates surveyed room coordinates.

Day 2 explores state estimation, sensor bias, noise, innovation consistency and
dropout. In E6, the supplied helper compares two filters on live drone data
during slow hand motion, with motors disabled. It records acceleration, onboard
orientation and position, and motor status as described in the
[sensor recording guide](sensor_recording_contract.md#live-e6-session-and-replay-fallback).
Saved data support repeatable analysis; a prepared recording provides the fallback
if live acquisition is unavailable. Day 3 carries forward
response lag, limits, frames, noise sensitivity and qualification evidence. A
continuous trajectory may need reference velocity/acceleration feed-forward and
a different command layer; the acceleration scaffold demonstrates that extension.
Neither selecting a profile nor reusing gains automatically integrates hardware.
