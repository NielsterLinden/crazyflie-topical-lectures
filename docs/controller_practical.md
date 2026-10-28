# Controller practical tasks and deliverables

We start with Andreas's position [PID](glossary.md#pd-and-pid), then try the velocity-to-tilt loop and
their [cascade](glossary.md#cascade). The same controllers run in simulation, replay and motor-disabled
checks. The [software guide](day_1_software.md) has commands and units;
the [shared conventions](practical_workflow.md) help us compare results.

Optional background: Brian Douglas's [What Is PID Control? (Understanding PID
Control, Part 1)](https://www.youtube.com/watch?v=wkfEZmsQqiA), with the
[accompanying playlist](https://www.youtube.com/playlist?list=PLn8PRpmsu08pQBgjxYFXSsODEF3Jqmm-y).

## Scope and schedule

For the scheduled practical, work through C1-C7 and the first-hover checks,
using the software guide and notebook for commands and model details. Items
labelled **Optional** and the **Stretch exercises** section offer further
exploration if time remains. The full dynamics derivation, detector analogies
and external videos are background references; preflight and handling checks
apply to every hardware session.

Allow 170 minutes for C1-C7, plus a protected 45-minute first-hover session,
10 minutes for catch-up/preflight, lectures, breaks and discussion; see the
[timetable](day_1_control.md#schedule). Recordings can replace live acquisition,
but we still note unperformed hardware checks. Simulation does not qualify a
drone to fly.

Keep one shared team notebook or notes file, at most one page for today, with
links to saved outputs. The deliverables below are things to show or discuss,
not seven separate reports. Keep the mass/preflight record for any drone we fly.

| Exercise | Minutes | Result |
| --- | --- | --- |
| C1: model and interfaces | 25 | Explain the controller layers, units, plant assumptions and mass baseline. |
| C2: reproduce Andreas's baseline | 20 | Reproduce position, velocity and cascade simulations with saved evidence. |
| C3: position-loop gains | 35 | Compare P/PD step responses and lag dependence. |
| C4: velocity loop and cascade | 25 | Compare velocity gains, tilt, damping and nested loops. |
| C5: integral action and disturbance | 25 | Compare supplied PD/PID responses and inspect [anti-windup](glossary.md#anti-windup). |
| C6: limits and sensing noise | 20 | Make one matched comparison under fixed authority. |
| C7: hand checks, replay and handover | 20 | Verify signs/frames using one recording or hand capture. |

## Preparation

Complete [native setup](software_setup.md#native-setup), activate `.venv`, and run
from the repository/package root (single-line Python commands work in Bash and PowerShell):

```text
cd code
python -m unittest andreas.test_practical
python -m day1 --output ../day1/baseline --no-show
cd ..
jupyter lab code
```

Use `andreas/day1_control.ipynb` for a guided notebook. It calls the same runner
and controller classes as the command-line experiments. Each run needs a new
output directory.
Read `andreas/hover_pid.py`, `andreas/velocity_pid.py` and `andreas/practical.py`.
The reduced plants approximate the inner-loop response at each command layer.
Gains and model constants define the experiments; hardware application requires
separate calibration and qualification.

Use the [cfclient baseline-check guide](day_1_baseline_checks.md) alongside C1/C2:
B1/B2 support model/mass checks; B3/B4/B6 support state/frame/timing/limits evidence.
B5 hover belongs to the [scheduled 15:45 first-hover session](day_1_control.md#first-hover-1545-1630),
using cfclient and the qualified onboard controller, not participant code. Baseline verification
precedes optimization; it is not a gain-tuning session.

## C1: identify the plant and controller layers

**Task.** Draw the position-to-velocity loop and its cascade with the velocity-to-tilt loop.

1. Label target, true state, measured/onboard-estimated state, position PID,
   velocity target, velocity controller, tilt command and inner-loop plant.
   Explain what remains onboard for each interface and what the laptop owns.
2. Write `u_v = kp*(r-x) - kd*v + ki*integral(r-x)dt`. Give gain/signal units.
   With integral disabled and `tau*v' + v = u_v`, derive
   `tau*e'' + (1+kd)*e' + kp*e = 0` for a fixed target. Compare its damping and
   natural frequency with the acceleration-command double integrator; the gains
   have different dimensions and cannot simply be copied.
3. Trace the toy tilt plant and `v' = g*tan(tilt) - drag*v`. Distinguish tilt
   toward a positive world axis from signed firmware roll/pitch, and world versus
   body velocity. On the same sketch, resolve thrust into vertical and horizontal
   components: ideal level-height motion needs `T*cos(tilt) = m*g`, giving
   `a_horizontal = g*tan(tilt)` before drag. This connects the toy plant to the
   thrust reserve needed for banked flight. Note yaw, lag and sample order.
4. With motors stopped, weigh the complete flight-ready drone and follow the
   [mass/preflight procedure](practical_workflow.md#hardware-preflight-mass-and-controller-assumptions).
   Record total g/kg, scale resolution, controller identity and mass applicability.
   PID does not use `ctrlMel.mass`; for intended Mellinger operation, include its
   Set/Store and reboot-readback evidence. No flight is needed.

**Deliverable.** A sketch of the loops with equations, units and predicted gain
effects. In the mass/preflight record, note the drone/configuration, measured
mass, controller applicability, old/new/stored values and reboot readback or
outstanding checks. Label supplied data.

**Completion check.** Trace each loop and command unit, explain which sensor
quantities are already estimated, and identify two omitted effects. Explain why
mass cancels in an ideal acceleration plant but still matters for thrust reserve
and force conversion. State the integration order and assess smaller-dt sensitivity.

## C2: reproduce the unchanged baseline

**Task.** Run the supplied position, velocity and cascade experiments unchanged.

1. Save CSVs, `settings.json` and `comparison.png`. Identify the four-second
   initial hold and subsequent position/velocity step, lag, drag and command limits.
2. Reproduce [RMS](glossary.md#rms-and-rmse), maximum and last-three-second errors for each run. Keep position
   errors in m and velocity errors in m/s. The last-three-second maximum is a
   final-window error metric. A settling-time measurement additionally needs a specified
   tolerance band and observation interval.
3. Identify position-loop terms, intermediate velocity setpoint, commanded versus
   achieved tilt and saturation. Distinguish true simulated state from noisy input
   channels. Preserve the unchanged baseline and code revision.

**Deliverable.** Saved baseline outputs and reproduction commands. Add B2-B6
results to the preflight record: logs/settings, frame/timing checks, agreed
acceptance criteria, team cross-checks and open actions. Add the scheduled hover outcome later.

**Completion check.** Another team reproduces a comparison. Explain why errors
or command effort from different physical layers cannot be compared without units
and a common objective. Unperformed powered checks and unknown latency are not passes.

## C3: tune the position loop

**Task.** Compare at least three position-loop settings with integral disabled.
Use Andreas's velocity-lag plant and the same step/limits in each comparison.

1. Compare P-only and lower/higher derivative feedback; state a damping/gain rule
   before inspecting plots. Relate observed behaviour to C1's characteristic equation.
2. Measure [rise time, overshoot and settling](glossary.md#response-metrics),
   peak error and velocity-command effort.
   Define 10-90% rise and a settling band that stays satisfied through the end;
   report non-settling runs explicitly rather than forcing a numeric answer.
3. Increase `--tau` and reduce `--dt` separately. Identify model mismatch,
   limiting or discretization before interpreting discrepancies as new physics.
4. Predict the sign of the command at positive displacement and the effect of
   derivative-on-measured-velocity on a target step.

**Deliverable.** Saved responses and metrics for the three gain settings, your
prediction and chosen candidate. Include the zero-error/zero-velocity test.

**Completion check.** Explain the gain units, damping/speed trade-off and one
theory/model discrepancy. Oscillation in simulation is evidence to diagnose,
not a candidate for hardware gain searching.

## C4: close the velocity loop and form the cascade

**Task.** Use Andreas's velocity-to-tilt controller and cascade on the same toy plant.

1. At fixed position gains, compare at least three velocity gains with integral
   disabled. Before running, predict which gives faster response and which risks
   oscillation. Plot velocity error, tilt request/response and intermediate velocity
   targets. Include steady drag and the actuator lag assumptions.
2. Explain the steady velocity error of P-only velocity control against drag.
   Discuss how the outer position loop and a velocity integral term affect it;
   do not claim that the two integrators estimate independent physical biases.
3. Compare the single position loop and full cascade under the same reference
   and declared authority. They use different inner-plant approximations, so this
   is a model/interface comparison, not evidence one is physically faster.
4. Explain bandwidth separation, sign/frame verification and why larger inner
   gain does not automatically improve the nested response.

**Deliverable.** Velocity/cascade plots, including intermediate commands, and the
settings used. Explain the interfaces on your C1 sketch; note missing hardware references.

**Completion check.** Trace position error through velocity setpoint to tilt and
motion; distinguish measured attitude, requested tilt and world acceleration.
Explain where feedback can amplify sensing noise and couple horizontal motion to height.

## C5: disturbance rejection, integral action and anti-windup

**Task.** Reuse the supplied integral terms and compare PD with PID in the
position-loop plant using a positive/negative modelled `--bias`.

**25-minute core.** Use the supplied PD/PID commands with one disturbance value,
compare offset/effort, and inspect the anti-windup code and available evidence.
The additional sign/magnitude, removal/reset experiments and custom disturbance
schedule below are extensions, not prerequisites for the first hover.

1. State exactly where this velocity disturbance enters: `x' = v + bias`.
   Here `bias` is a constant added velocity: the logged lag-state `v` describes
   the plant response, and physical `x'` includes the disturbance as well.
2. Predict the offset's direction before running; compare residual offset,
   transient/recovery and effort under identical limits. Keep gains and noise
   fixed when introducing the disturbance, then compare PD with PID.
   As an extension, test two magnitudes/signs and reserve one for held-out validation.
3. Trace conditional anti-windup and the integral-contribution limit. Optional
   experiments test long saturation, both signs, disturbance removal and controller reset. The supplied
   implementation is a baseline to validate, not an extra controller to write.
4. Relate the disturbance to a force/actuation nuisance in interferometer control,
   while stating the limits of this toy model. Propulsion faults require repair,
   not compensation by increasing integral gain.

**Deliverable.** PD/PID comparisons, integral/saturation history,
one explanation of the anti-windup behaviour. Link held-out/removal/reset evidence
if explored; otherwise mark it untested. A copied disturbance schedule and its
code diff are only needed for the optional removal/recovery extension.

**Completion check.** Explain offset rejection, wind-up and recovery, including
one case in which adding integral action makes the response worse.

## C6: limits, noise and robustness

**Task.** Keep velocity/tilt limits fixed and use seeded sensing-noise injection.

**20-minute core.** Compare the baseline and one candidate on the same held-out
noise seed and increased step demand; inspect true error and command saturation.
The larger seed ensemble, lag scans and new tests below are optional extensions.

1. Increase step demand in simulation to expose velocity and tilt limiting.
   Add combined-axis checks before proposing two-axis hardware use; per-axis
   clipping is not a vector-norm, total-bank or swept-volume guarantee.
2. Compare gains using identical position/velocity noise draws. Log true error,
   measured residual, command variability, effort and saturation. Explain why
   a reduced [in-loop residual](glossary.md#ground-truth-and-in-loop-measurements)
   alone does not establish better physical motion.
3. **Extension:** develop on seeds 0-4, evaluate the selected configuration on seeds 20-39,
   and retain every run. Check increased lag with explicit dt. A delay extension
   needs a causal measurement buffer, not a retrospective time shift.

**Deliverable.** Noise/limit tests, scenario settings, all-run plots and
metrics for the matched comparison, truth versus measured-residual observations
and a limited robustness recommendation. Keep all runs if extending to a seed
ensemble; a new regression test is an extension, not a core submission.

**Completion check.** Explain sensing-noise injection versus plant disturbance
and the authority cost of a faster response. Toy-model robustness is not a
certified hardware stability margin.

## C7: sign checks, replay and handover

**Task.** Follow Andreas's motor-disabled stage using the supplied capture/replay
commands in the [software guide](day_1_software.md#replay-and-motor-disabled-sign-checks).
Use one supplied recording if capture is not immediately available. Finish the
sign/frame check and handover in 20 minutes; do not let connection debugging
displace the scheduled first hover.

1. Inspect a synthetic format check, then a supplied real recording or checked
   hand capture. Log displaced/rest/moving intervals and compute position-loop,
   velocity-loop and cascade commands without transmitting them. Command signs
   should oppose the appropriate displacement or motion.
2. Compare the measured displacement and motion with the commands and model.
   Label onboard states as derived; a fixed recording does not close the loop.
   Preserve the recording's source, timestamps, frames and calibration information.
3. Freeze a baseline and candidate with explicit interfaces, limits, frame/units,
   delays, source/calibration IDs and fallback. Identify what the qualified
   operator would need to map world velocity or abstract tilt into firmware units.
4. Optional supervised application of participant control follows the flight sequence: takeoff,
   settled hover, transfer to participant control with a continuous command at
   the switch, bounded one-axis step, landing and analysis.
   Use the supplied run card/launcher and a separate stop operator. Preserve
   estimator frame and the [mass/preflight record](practical_workflow.md#hardware-preflight-mass-and-controller-assumptions).
   The core C7 result is the recorded-data sign/frame comparison and handover notes.

That optional participant-controller experiment is distinct from the scheduled
first hover, which uses cfclient and the unchanged qualified onboard controller.
It needs separate time and qualification; it does not replace the hover session.

**Deliverable.** Saved replay outputs and sign/frame checks. Note your baseline
and candidate, active interface and what Days 2-3 still need. Keep any flight
outcomes and outstanding checks with the B1-B6 preflight record.

**Completion check.** Another team can reproduce the replay and explain which
loop would own each command. Keep onboard attitude stabilization and the tested
stop path. Missing calibration, frame mapping, visibility or qualification means
qualified fallback or no flight, not weaker limits.

## Submission and closing discussion

Share today's section of your team notebook or notes file: at most one page,
one useful comparison plot, one failure/diagnosis and your controller choice.
Link saved outputs and the commands/settings needed to reproduce them; identify
supplied data and missing checks. No separate report, `manifest.json` or manually
compiled `metrics.csv` is required. The [short template](templates/controller_report.md)
is optional. Keep automatic output files and the preflight record; there is no
need to copy their contents into the notes.

If useful for your analysis, an optional `metrics.csv` can contain:
`run_id,config_id,scenario,seed,interval,dt_s,controller_layer,error_units,
rms_error,max_error,command_kind,command_effort,command_effort_units,
saturation_fraction,rise_time_s,overshoot_fraction,settling_time_s,recovery_time_s,
qualified_for_hardware`. Use blanks for unavailable/inapplicable quantities and
explain failures. Velocity-squared effort is m2/s; tilt-squared effort is deg2*s;
acceleration-squared effort in the race extension is m2/s3. None is electrical energy.

Two teams volunteer for five-minute presentations; everyone can join the
discussion. What did we expect, what happened, and what would we use next?
Connect the results to interferometer feedback where helpful, without overstating
what the simulation or recording proves. We can compare other results during
practical work or in parallel groups.

## Stretch exercises

| Extension | Task and additional evidence |
| --- | --- |
| Moving reference and feed-forward | Use `day1.feedforward` to compare P/PD/reference-acceleration feed-forward under matched limits. Derive its acceleration-command plant, gain units and `a_ref + Kp*e_p + Kv*e_v` before comparing with the Day 1 velocity interface. Separate startup, the 7-10 s disturbance, and recovery; nominal RMS includes startup. |
| System identification | Fit tilt/acceleration and lag/drag models to compatible, provenance-labelled data. Check the near-level 5.84 deg per m/s2 scale, frame signs, differentiation/smoothing and held-out fit uncertainty. See the software guide; this is additional work, not part of the 25-minute C4 core. |
| Loop analysis | Derive the appropriate velocity/tilt plant, Bode response and margins, then compare sampling/lag and nonlinear limiting effects. |
| Estimated-state control | Replace ideal state inputs with the Day 2 estimate; compare bias, noise/dropout, estimator lag and actual closed-loop truth. This needs an explicit coupled harness; fixed replay is not a controller-performance test. |
| [LQR/LQG](glossary.md#lqr-and-lqg) or [MPC](glossary.md#mpc) | Define model, cost units and command interface; compare effort, constraints, timing and failure rates against the simpler controller without equating Kalman Q/R with controller costs. |

These extensions bridge to continuous racing trajectories. They are additional
work alongside the scheduled C1-C7 exercises. The
[race freeze/fallback plan](day_3_race.md#freeze-points-and-fallback) still applies.
