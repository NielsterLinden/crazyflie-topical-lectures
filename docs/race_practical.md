# Race practical tasks and deliverables

We bring together the controller and estimator work from Days 1-2 for a fast,
repeatable race. We measure the gates with the supplied survey tool and check its results.
The [glossary](glossary.md) explains the control, estimation and flight terminology.
Each team checks the specific drone, software and course before flying and
records its decision, with a second member cross-checking flight-critical items.

## Scope and schedule

R1-R7 form the scheduled race practical. Use the linked course-interface,
survey, trajectory and analysis guides for those activities. The longer
theory notes are background; writing a survey method, building an integrated
simulator, and higher-order trajectory/controller extensions are optional
projects. Course validation, preflight checks and flight limits apply to every
physical attempt, including a supplied fallback.

We prepare and qualify in the morning, then race and analyse in the afternoon.
Share survey, coding and analysis roles. Each flight has one named emergency-stop
operator, separate from the pilot.

Add at most one page to your shared team notebook or notes file, with links to
saved outputs. The deliverables below are practical checkpoints, not seven
separate reports. Preflight checks and attempt logs remain necessary safety records.

| Exercise | Suggested minutes | Result |
| --- | --- | --- |
| R1: define the race and handover | 15 | Freeze interpretation of rules, interfaces, and limits. |
| R2: measure and check the gates | 40 | Use the supplied survey code, then check the geometry; retain 20 minutes for retries. |
| R3: prepare two racing strategies | 35 | Produce conservative and candidate trajectories. |
| R4: review baseline validation | 30 | Reuse Day 1/2 closed-loop evidence and check course limits; full race-harness integration is optional. |
| R5: qualify and freeze | 30 | Complete team cross-checks and a low-speed test for a specific configuration. |
| R6: run the timed competition | Shared 100-minute window | Record all attempts under common rules, including recovery and battery changes. |
| R7: analyse and report | 35 | Explain performance, spread, failures, and next steps. |

Do not shorten safety checks to meet these times. Share flight slots fairly,
including battery changes and recovery. Stop racing at 15:40 to leave 35 minutes
for R7. Up to five teams volunteer for five-minute presentations, followed by
20 minutes of discussion; we can compare other results during practical work
or in parallel groups.

R1-R7 require evidence, including a diagnosed unsuccessful attempt where needed.
Hardware flights depend on passing team checks and equipment availability; a
team that cannot qualify uses the documented fallback or completes the analysis
with shared recordings. We can learn from that analysis without completing a lap.

## Preparation

The [Day 3 notebook](../code/day3/day3_race.ipynb) uses the trajectory planner and
post-flight analysis tools for the same exercises. Use the team's measured
course and gates, and save the notebook with the exported plan. Its offline
default demonstrates the format only; the flight checks and separate flight
procedure below remain unchanged.

Use the native environment and the controller/estimator configurations selected
on Days 1 and 2. Confirm the team's assigned [radio URI](glossary.md#radio-uri), firmware/decks, sensor
calibration, code revision, and available logging channels. Keep simulation and
flight commands distinct. The protected fallback must remain unchanged.
The [course format and flight-tool guide](course_interface.md) describes the public format,
synthetic example, command units, operator launch interface and telemetry return.
Confirm the shared run card/tool handoff before R2 acquisition. The 30-minute
R4 slot reuses earlier experiments; it does not include building a new simulator.

The [starter guide](../code/day2/README.md) describes the model and APIs;
the [equations of motion](quadrotor_equations_of_motion.md) explain the dynamics. The
[survey guide](hoop_survey_challenge.md) gives the supplied measurement procedure.
No new reconstruction code is needed; teams can share the survey data and checking.

## R1: define the race and accept the handover

**Task.** Begin with a shared understanding of the common race rules,
so the team can compare routes on the same basis.

1. Record gate identities, crossing order/direction, number of laps, required
   start/finish conditions, official timing method, and validity/penalty rules.
   Resolve ambiguous rules together across teams before timed attempts.
2. Record common flight-volume, altitude, speed, acceleration, bank, and
   [clearance](glossary.md#clearance-and-gate-normal)
   limits. Identify supports and floor changes as well as the nominal openings.
3. Select Day 1/2 baseline configurations and the intended flight interface.
   Identify which estimates actually drive control and which run in shadow mode.
4. Assign pilot, stop operator and run recorder. The pilot and stop operator both
   confirm readiness before takeoff. Anyone can call a stop; restart only after
   the team has checked and recorded the cause and corrective action.

**Deliverable.** A note of the rules, limits, roles,
configuration IDs, interface diagram, and unresolved dependencies.

**Completion check.** All team members agree what counts as a valid lap and how
it is timed. The code's requested trajectory duration is not an official measured
finish time. No team assumes different safety limits to gain an advantage.

## R2: measure and check the gates

**Task.** Follow the [supplied survey procedure](hoop_survey_challenge.md).
Share collection/checking roles and survey files rather than queueing for a
separate complete survey by every team.

1. Place the drone on a stable surface, clear of propellers, and run
   `scripts/measure_hoop.py` on the assigned URI. Confirm `STOP AND SURVEY` and
   wait for **STOP VERIFIED**: the tool sets `stabilizer.stop=1` and verifies the
   lock and zero motor outputs before you pick up the drone.
   Trace the inner opening using
   a position-referenced pointer/jig and retain the fitted JSON and sample points.
2. Check centre, normal sign and aperture against manual measurements or a
   second pass excluded from the fit. Account for pointer offsets and uncertainty;
   repeated measurements in one Lighthouse frame cannot reveal all systematics.
3. Export a course with physical-gate IDs, crossing directions and laps. Keep
   full rectangle geometry separately when using its conservative inscribed circle.
   Plot the path, and leave flight disabled pending qualification.

The [supplied trajectory tool](trajectory_construction.md) handles course
construction and plots; writing another planner is optional.

**Deliverable.** Survey JSONs, a brief independent check, geometry/path plot and
runner-compatible course. Shared measurements are welcome; note their source.
No survey report or new fitting code is required.

**Completion check.** Another team can reproduce the geometry and the team can
explain its accuracy and limits. Two team members check the physical setup and
margin before any powered crossing. A small fit residual or successful JSON
parsing alone does not qualify a survey. A moved gate or changed Lighthouse
frame requires revalidation.

## R3: prepare a conservative and a candidate strategy

**Task.** Explore one change alongside a conservative central-crossing baseline:
for example a speed policy, smoother approach, or different crossing line.
The comparison includes a justification for the proposed change.

Use the gate approach to connect the earlier results: Day 1's lag, overshoot and
tilt demand affect tracking margin; Day 2's uncertainty and sensor availability
affect confidence in clearance. Refer to those saved comparisons when choosing
the single change, then check the prediction against the existing race telemetry.
This uses R3's hypothesis and R7's analysis, with no extra experiment or report.

1. Export both configurations and draw the full planned trajectories with the
   measured gates, directions, start/finish, and flight volume. Account for
   repeated/reverse crossings and a horizontal gate if present.
2. Budget aperture for drone size, survey uncertainty, estimator uncertainty,
   tracking performance, and a safety margin. State assumptions and correlations;
   do not silently double-count error terms or claim an exact collision
   probability from a worst-axis sigma multiplier.
3. Check approach/exit alignment, braking distance, speed, acceleration, bank,
   and, where modelled, jerk. A path through each centre may still be dynamically
   infeasible or intersect a support between gates.
4. Define behaviour when clearance is non-positive or telemetry becomes stale.
   Do not enforce a minimum forward speed through a failed clearance check.
   State a hypothesis and the performance metric the candidate should improve.

**Deliverable.** Baseline/candidate course and policy configurations,
planned-path plots, margin budget, limit checks, and the written hypothesis.

**Completion check.** Both strategies obey identical physical limits. The team
explains where any predicted speed advantage comes from and identifies a case
where slowing down or declining to cross is necessary. Limits are checked over
the trajectory, not just at waypoints.

## R4: review baseline validation and course checks

**Task.** Use the 30 minutes for evidence review and motor-free checks. A full
closed-loop race simulator is a separate extension. Use the unchanged qualified
fallback if a candidate lacks the evidence needed for hardware approval.

1. Reuse the Day 1 nominal/disturbed controller comparison and held-out C6 runs,
   plus the Day 2 dropout/outlier results. Identify exact code/configuration,
   reference, limits, seeds and metrics. If earlier work is incomplete, rerun
   the supplied baselines or analyse instructor-provided outputs and label that
   substitution. Andreas's Day 1 tests use step references and simulated (optionally
   noisy) feedback in velocity/tilt-loop models; the estimation baseline prescribes
   truth motion. Their interfaces differ from the position-setpoint race runner.
   They are separate
   tests, not a coupled race simulation.
2. Summarize maximum/[RMS](glossary.md#rms-and-rmse) tracking error, saturation and failures from those runs,
   drawing all available trajectories for the compared conditions. Distinguish
   uncertainty/innovation evidence from closed-loop tracking evidence. Do not
   infer performance of a changed racing line from an unchanged reference.
3. Review R3's planned trajectories and full-path limits, aperture budgets and
   ordering for both strategies. Run the supplied course dry-run interface
   with motors disconnected. Save its output. This tests configuration
   and planned feasibility, not physical accuracy or closed-loop robustness.
4. Document software-only stale-data/clearance abort evidence from an available
   tested runner/test harness, running it without a radio/motors. If unavailable,
   record an unresolved hardware-qualification
   requirement. No live fault injection, weaker limits, or assumed pass.

**Deliverable.** Links to reused/supplied runs and all-run plots/metrics,
candidate/baseline evidence table, dry-run outputs, fault-handling evidence or
outstanding requirement, and a flight/fallback/no-flight recommendation. Include
an explicit list of what has **not** been simulated or validated.

**Completion check.** The team distinguishes estimator-only replay, fixed-reference
closed-loop tests, planned-course checks and actual race validation. Missing
integration is a documented limitation. Incomplete flight checks mean a
qualified fallback or no flight,
even when the analysis is complete.

**Optional integrated-race extension.** Only if a course-driven closed-loop harness
was completed and tested earlier, compare nominal, dropout, outlier and lag/
disturbance cases using development seeds 0-4 and held-out seeds 20-39. Check ordered
directional aperture crossings and abort transitions, retain every trajectory,
and report valid completions, times, clearance and tracking/saturation metrics.
Building that harness is additional work, not part of the 30-minute core slot.
The starter's `recommended_speed` does not alter prescribed truth motion; do not
claim faster laps unless the speed policy actually changes the simulated motion.

## R5: qualify and freeze the race configuration

**Task.** Check the exact system that will fly, record the result and make the
team's go/no-go decision. The pilot and a second team member cross-check the
flight-critical items before starting; all group members agree before each
stage transition.

1. For hand-held measurements, use the [firmware-stop preparation](sensor_recording_contract.md#prepare-the-drone-for-hand-held-recording).
   Check the assigned radio identity, connection, frames,
   units, sensor calibration, stable estimation, command interface, telemetry,
   and emergency-stop procedure. Confirm raw/derived channel labels.
   Recheck the [flight-ready mass and controller assumptions](practical_workflow.md#hardware-preflight-mass-and-controller-assumptions)
   for the race battery/decks/cage, including any applicable persistent mass
   setting and the controller actually selected by the flight script.
2. Review survey validation, full path, limit checks, logs, and simulation
   failures together within the team. Keep onboard attitude stabilization active.
3. Once these checks pass and before racing, try a short, low-speed test flight in the
   Lighthouse flight area. After survey/hand-held work, place the drone down and
   power-cycle it to clear the firmware lock, then repeat the preflight checks.
   Check tracking and landing. Record whether actual
   crossings and tracking agree with the planned course. Do not tune unstable
   gains during flight.
4. Freeze code/configuration IDs and select full, estimation-only, or conservative
   operation. Archive the checked settings and a tested route back to fallback.

**Deliverable.** The team's preflight record: checklist, who checked and
cross-checked it, time, exact code/configuration IDs, evidence/logs, active profile,
operator roles, stop conditions and the go/no-go decision.
Include the mass/configuration record, controller/parameter applicability, and
reboot/readback evidence for any applicable persistent mass setting.

**Completion check.** The checks apply to the recorded configuration. A material
software, geometry, frame or limit change needs repeated checks and testing,
as does a battery/deck/cage/payload change that affects
mass or control authority. The intended flight interface is verified; simulator
gain numbers are not copied into unrelated onboard PID settings.

### Freeze points and fallback

Let T be the scheduled start of the timed competition, not an earlier supervised
qualification manoeuvre. The instructor confirms T before morning work begins.

- **T minus 90 minutes:** freeze estimator development; select full or shadow mode.
- **T minus 60 minutes:** freeze controller development; retain the qualified
  onboard-position-control fallback unless the participant outer loop is already qualified.
- **T minus 30 minutes:** stop algorithm changes and perform final safety checks.
- **Any failed safety check:** no timed start; use a qualified fallback or stand down.

If the survey fails the setup deadline, instructors may supply validated fallback
geometry. Label it explicitly and keep the original survey attempt/diagnosis;
discuss the supplied geometry's validation rather than claiming your own acquisition. Unfinished estimation can be
evaluated offline on fallback telemetry. The same physical checks apply to the
fallback; it is not permission to fly an unsafe or unqualified system.

## R6: execute and record the timed runs

**Task.** Fly the configuration that passed the team's checks under the common competition rules.

1. Before each attempt, record a run ID, configuration/profile, battery condition,
   operators, course/geometry version and preflight outcome. Apply the
   [battery checks](day_1_baseline_checks.md#battery-checks-for-every-flight).
   Confirm the area is
   clear and the stop operator is ready.
2. Use a distinct telemetry filename per attempt and retain console events and
   the official timing record. Mark qualification, timed, and aborted runs
   separately. Never overwrite an unsuccessful log with the next attempt.
3. Have the designated observer or agreed video/timing system establish actual
   physical crossing validity, order, direction, contacts, and boundary violations.
   The crossing monitor uses the onboard estimate; its message is not independent
   proof that the physical drone passed through the aperture.
4. Use the allocated repeat opportunities fairly. Compare baseline and candidate
   physically only if both are qualified and time allows; otherwise retain the
   simulation comparison and race the qualified choice. Any adjustment follows
   the shared freeze points and repeated-check requirements.

**Deliverable.** Per-attempt logs/configuration references, official times,
observer/recording evidence, penalties, validity decision, and abort/failure cause.

**Completion check.** Every attempted start is accounted for. Invalid/aborted runs
are not assigned a finish time of zero or silently discarded. The official
ranking follows the published rules, not a team-specific post-hoc scoring formula.

## R7: compare planned, estimated, and physical performance

**Task.** Bring together all attempts to understand performance beyond the
fastest valid run. From the repository root with the native environment activated:
The single-line command below works in Bash and PowerShell.

```bash
python scripts/plot_race.py path/to/run.csv --course path/to/approved-course.json --phase race --output plots/run --no-show
```

1. Inspect the 3D/projection overlays, gate positions, and signed x/y/z tracking
   errors. Preserve the course/planner version used for the run. Optional survey
   outlines and plotting limitations are documented in the
   [plotter instructions](post_flight_analysis.md).
2. Mark race start/finish, gate passages, dropouts, limit events, and aborts.
   Separate takeoff/landing from the race interval. Host-aligned estimated
   position minus sent setpoint includes transport delay and is not independent
   position truth. Logs lacking sent-command events need a documented alternative;
   the plotter does not currently support advanced-runner logs without them.
3. Compare official completion times and valid-run spread, valid crossings,
   clearance estimates, tracking error, and completion rate. State sample counts
   and uncertainty; a handful of clean runs cannot establish a rare-failure rate.
4. Investigate one unsuccessful or degraded case. Distinguish surveying error,
   estimator bias, controller lag/saturation, Lighthouse visibility, and raised-floor
   range/flow effects using available evidence. State competing explanations and
   a safe discriminating test rather than asserting causality from one plot.

**Deliverable.** Figures, run-summary table, baseline/candidate comparison,
failure diagnosis, and one prioritized improvement supported by evidence.

**Completion check.** Distinguish planned duration, estimated tracking error,
observed physical crossing, and official race time. Report what the available
measurements cannot establish. Explain whether the fastest run was repeatable
and whether predicted margins were credible, without treating the estimator's
own output as an independent calibration reference.

## Submission and closing discussion

Share today's section of the same team notebook or notes file: at most one page,
your survey/strategy, one trajectory plot, the outcome and one thing you would
change. Link saved course/settings, telemetry and survey observations. The
[short template](templates/race_report.md) is optional; no separate report or
`manifest.json` is required.

Keep one shared attempt log with run ID, checked configuration/course, operators,
battery condition, validity, official time/penalty, telemetry/observer links and
abort reason. Include failed starts. A team recorder can maintain it; there is
no need to repeat it in the notes. Keep the R5 preflight record with this log.

If automating analysis, an optional `runs.csv` can use:
`run_id,run_type,config_id,profile,course_id,survey_source,
preflight_passed,official_valid,completed_laps,required_laps,valid_crossings,
required_crossings,official_time_s,penalty,telemetry_path,observer_record,
tracking_rms_m,tracking_max_m,min_estimated_clearance_m,abort_reason`.
Define penalty units/rules in R1. Use empty fields for unavailable/inapplicable
quantities, not zero. Simulation results remain separate, with seeds/scenarios
and truth-based versus estimated metrics labelled.

Discuss the survey, strategy, results and a failure or unresolved question.
What limited repeatability? Which margins were credible? Links to detector
acceptance, calibration and feedback are welcome, but an in-loop residual is
not an independent performance measurement.

## Stretch exercises

Once the baseline is qualified, these extensions offer further avenues to
explore before the relevant freeze:

| Extension | Task | Additional evidence and completion check |
| --- | --- | --- |
| Jerk-continuous reference | Follow the [optional planner extension](trajectory_construction.md#stretch-goal-continuous-and-limited-jerk): replace C2 quintics with a C3 reference and an explicit jerk bound. | Left/right derivative tests, acceleration/jerk plots and planned duration under matched limits; no tracking claim without closed-loop evidence. Retain the supplied planner and qualified fallback. |
| Racing line | Optimize off-centre crossing points or approach geometry. | Same course and limits; continuous-path clearance and dynamics, repeat-run benefit, and a retained central-crossing fallback. |
| Adaptive speed | Combine uncertainty, curvature, and braking constraints. | Closed-loop nominal/dropout trials and non-positive-clearance behaviour; no lap-time claims from a speed recommendation that does not affect motion. |
| Survey uncertainty | Propagate geometry errors into clearance decisions. | Document shared frame/calibration errors and correlations; compare predictions with independent or simulated perturbed geometry. |
| Banked turns | Use bank/attitude constraints in planning and clearance. | Swept-volume/thrust checks and attitude-aware simulation before a team-checked low-speed physical test. |
| Joint control/estimation | Compare an advanced controller/observer with the simpler choice. | Matched limits, complete trial ensembles, tracking/saturation/effort and completion-rate evidence, and measured computation/timing cost. |

These extensions are optional, not prerequisites or guarantees of the best
physical lap. Qualification takes priority over additional algorithm complexity.
