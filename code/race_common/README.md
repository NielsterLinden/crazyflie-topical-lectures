# Race models and policies

These shared modules contain gate geometry, survey fitting, uncertainty margins,
race policy examples and integration profiles. The Day 3
[course planner](../day3/README.md) and Day 2 E7 uncertainty exercise use these
models. It contains no flight commander.

- `hoop.py`: plane basis, radial offset, covariance projection and conservative clearance.
- `hoop_fit.py`: circle/rectangle fits used by the survey tool.
- `strategy.py`: approach points, central-crossing baseline and speed-policy extension.
- `profiles.py`: explicit onboard/external ownership choices below.
- `geometry.py`, `dynamics.py`: shared geometry, kinematics and acceleration limits.

The policy extensions retain a central-crossing, fixed-speed baseline.
For E7, choose a justified uncertainty/tracking margin and braking-aware speed
policy; flight use still needs tested code connecting these decisions to drone
commands, plus the team's flight checks.
Run `python -m day3.uncertainty` from `code/` to inspect the estimator-to-clearance
example. No speed recommendation in that example changes its prescribed motion.

[Day 1's outer-loop extension](../flight_control/README.md) explains the
controller layer, feed-forward, gain units and actuator model. Its optional
integral/anti-windup exercise is separate from Andreas's supplied Day 1 integral
terms. The simulation dependencies include SciPy and python-control for optional
LQR/LQG, loop-analysis, scheduling and MPC studies.

## Fallback profiles

Do not make the race depend on unfinished participant code. Keep changes behind
one of the profiles in `profiles.py`:

- `fallback`: onboard state and position control, fixed 0.30 m/s speed, and
  central hoop crossings;
- `estimation_only`: participant Kalman filter is evaluated, while the tested
  onboard position controller still controls flight at fixed 0.30 m/s with central crossings;
- `full`: participant estimator, outer-loop controller, covariance-dependent
  speed, and optimized crossing points.

Select a profile with `get_profile(name)`. Check the supplied `fallback`
against the actual hardware and course, and preserve it unchanged. New work goes into the
other profiles and must retain a simulation switch back to `fallback`.

`get_profile(name)` returns configuration values. The flight runner and its
adapter connect the chosen estimator, commander and policy, with separate
team checks and low-speed testing.
`full` also carries `fixed_speed_m_s=0.30`, but selects the covariance speed policy.
The [course format and flight-tool guide](../../docs/course_interface.md) specifies the
pilot-operated launch interface, command units, schema and synthetic example.
It does not implement automatic profile switching. The
[recording guide](../../docs/sensor_recording_contract.md) defines the prepared
core replay and optional adapter inputs; fused-position replay is labelled
cascaded rather than independent raw-sensor fusion.

`controller="onboard_position"` identifies onboard position control as the
feedback owner. The supplied conservative runner streams low-level position setpoints from
the laptop to onboard PID; it does not upload a high-level trajectory. Host
scheduling and link continuity therefore matter, including during best-effort
landing. The advanced onboard-trajectory option is a separate qualified runner.

For the 30-minute E6 core, use `python -m day2.live` from `code/` with
the assigned URI, `--prepare-for-handling` and a new `--output` directory. Place
the drone down, close other radio clients, and wait for **STOP VERIFIED** before
handling it: the helper sets `stabilizer.stop=1` and verifies the lock and zero
motor outputs. The stop remains engaged; power-cycle on the surface and repeat
preflight checks before flight. It compares
two configurations on the same stream and saves input for exact offline replay.
Use `python -m day2.replay` with `--metadata` and a fresh output prefix
as the immediate fallback. The
[E6 worksheet](../../docs/kalman_filter_practical.md#e6-compare-filters-on-live-drone-data)
provides complete commands and a time budget. Raw acquisition/conversion and
asynchronous timing tests are not prerequisites for that slot.

If live integration is incomplete, use the checked flight fallback
and evaluate the participant estimator offline on a suitable sensor recording.
Check the channels, units and timing against that guide; a position-only race log is not an
inertial sensor dataset. This still permits
innovation, covariance, controller, and crossing-error analysis.

### Time-boxed decision points

Follow the [race configuration deadlines and fallback steps](../../docs/race_practical.md#freeze-points-and-fallback),
including the response to failed safety checks.

A team may progress from fallback to full only one feature at a time. Record the
active profile with every run so results remain interpretable.

## Stretch goal: attitude and banking

After completing the baseline, the [equations of motion](../../docs/quadrotor_equations_of_motion.md)
and worksheet stretch exercises provide context for three possible extensions:
bank-aware planning, orientation-aware process/measurements, then an attitude
error-state EKF.

Retain the six-state implementation as the comparison baseline. Evaluate whether
the extension improves repeated-run completion time, tracking error, and
covariance consistency rather than judging it only by code complexity.

## Suggested comparison table

Use the [estimation metrics](../../docs/kalman_filter_practical.md),
[controller metrics](../../docs/controller_practical.md), and
[race analysis requirements](../../docs/race_practical.md#r7-compare-planned-estimated-and-physical-performance).
Retain median and worst crossing speed and perpendicular crossing error in the
per-configuration comparison, alongside clearance, consistency, control effort,
saturation, and repeated-run completion/contact rates.

The objective is a fast, repeatable strategy
whose uncertainty claims remain compatible with observed crossing performance.
