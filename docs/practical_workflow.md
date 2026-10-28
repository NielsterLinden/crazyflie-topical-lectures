# Shared practical conventions

We use these conventions across the [controller](controller_practical.md),
[Kalman-filter](kalman_filter_practical.md) and [race](race_practical.md) practicals
so we can compare results and help each other reproduce them.

Keep **one shared team notebook or notes file**, with at most one page per day
plus links to plots and saved outputs. No separate exercise reports, custom
manifest or manually compiled metrics tables are required. The short daily
templates offer optional prompts for that same notebook. Most checks can be
shown or discussed while we work. Preflight, calibration and flight records are
working/safety data and remain separate from this short write-up.

## Responsibility for the experiment

Each group of **four participants per drone** owns its checks, configuration
changes and decision to fly. Work through the checklist together, recording what
was checked, who checked it, the result and any uncertainty. A different member
cross-checks each flight-critical item. **All four agree that the checks for a
stage are complete before moving to the next stage.** An unresolved check stays
open; record it and investigate, use a checked fallback or stop that activity.

For a flight, assign pilot, separate emergency-stop operator, telemetry/checklist
monitor and observer/recorder. Rotate these roles between attempts so everyone
contributes to operation and diagnosis. Everyone can call a stop; the pilot and
stop operator both confirm readiness immediately before takeoff.

Use the shared Lighthouse flight area, agreed course limits and allocated slots.
Organizers coordinate access to that area and equipment; teams perform the
technical checks themselves. Questions and diagnoses can be discussed with
other teams and instructors, without assuming anyone has a ready-made solution.
If a fault remains unresolved, keep the evidence and switch to checked hardware,
a tested fallback or recorded-data analysis. A failed safety check means no
flight with that configuration.

Here, **qualification** means the team's documented checks and low-speed test
for a specific configuration, followed by the group's agreement to proceed.
Repeat the relevant checks after changes. Venue access rules and shared
flight-area restrictions still apply.

## Conduct each experiment

Prefer live measurements, or a recording just made by the team, when the supplied
helper supports the needed channels. Reuse a capture for matched comparisons.
Spend at most five minutes resolving unexpected acquisition/data behaviour within
the existing slot, then use the checked recording supplied for that exercise.
Keep the original data and one note of the issue for later investigation; do not
change calibration, inflate uncertainties or relax safety checks just to continue.
Controlled simulations remain useful where truth or a known injected fault is
needed. Label each result as live, recorded or synthetic, with unperformed hardware
checks left open. Prepared data are a reproducible fallback, not a reason to hide
unexpected features of the live data.

1. Before changing a parameter, predict what will happen; run the comparison,
   then explain what agrees or differs. Discuss this within the team; the existing
   plots and short notes are enough.
2. Keep the original baseline. Compare with the same limits, scenarios,
   references and random draws; save the changed settings.
3. Development and validation use separate data. Unless an exercise specifies
   otherwise, seeds 0-4 are for development and 20-39 for final simulation validation.
   Calibration and tuning must not use the final held-out recordings.
4. Keep failed, rejected, aborted and non-settling runs, with sample counts and
   uncertainty. A smooth plot or one fast lap can hide a lot.
5. Share modelling, acquisition, coding and analysis. For each flight, name the
   pilot and a separate emergency-stop operator.

For diagnosis, change one cause at a time: a bias, noise level, lag, outlier or
missing measurement. Use the existing comparisons and intervals before combining
effects. Keep physical mass errors as preflight checks; explore unsafe conditions
only in simulation or recordings.

Across the three days, use an approach to a gate as the connecting example:
Day 1 asks how feedback reaches and settles at a target, Day 2 asks how well
we know the position and velocity, and Day 3 asks whether the approach leaves
enough clearance and braking distance. Reuse the existing outputs to discuss
this; the supplied simulations retain their own references and interfaces.

## Keep evidence interpretable

- Each result includes units, frames, physical reference points, acquisition
  times, delays, sampling rates, invalid-sample handling, and numerical
  integration conventions.
- Separate labels identify raw sensors, onboard estimates, offboard estimates,
  reference commands, simulated truth, and independent observations. An estimator
  output or in-loop residual is not independent ground truth.
- Innovation records include values before rejection, separated by sensor/update
  dimension. Accepted-only residuals are selected data; mixed-dimensional maxima
  do not form one chi-square-distributed sample.
- A lap-time improvement claim needs a model in which the proposed speed policy
  changes motion. Replaying a fixed flight log does not show how
  an alternative controller would have changed that flight.
- Blank metric fields mark unavailable/inapplicable values; zero is not a
  missing-value marker. The report includes explanations of unsuccessful or
  non-settling runs.

## Baseline verification before optimization

The [Day 1 cfclient baseline-check guide](day_1_baseline_checks.md) covers physical
condition, identity/settings, logging, frame and timing measurements, battery and
scheduled first-hover outcome, limits and readiness decision. Record the unchanged baseline and acceptance
criteria first. Check and document corrections to setup faults; optimize only in a
separate, controlled experiment. A failed check or missing measurement is not a pass.
The mass-specific procedure below is part of that guide and applies on later days too.

## Hardware preflight: mass and controller assumptions

Before powered flight, each team checks the actual flight configuration against
the controller model, with motors stopped and a second team member cross-checking.
Repeat this check after changing the battery, decks, cage, or other payload.
The [battery checks](day_1_baseline_checks.md#battery-checks-for-every-flight)
apply before and during every powered attempt: at least 4.10 V before takeoff,
and controlled landing at 3.50 V under load or the first low-battery warning.

1. Weigh the complete flight-ready drone, including battery, decks, cage, cables,
   and attachments. Record the drone ID, configuration/photo, scale resolution,
   and measured total mass. Convert grams to kilograms; for example,
   **35.63 g = 0.03563 kg** for the complete assembly.
2. Read `stabilizer.controller` in cfclient: `1` is PID and `2` is Mellinger.
   Check the controller intended by the flight script too: a script can change
   selection after connecting. Record readback during the run where available;
   inspecting the `ctrlMel` parameter group does not mean it is active.
3. For Mellinger, compare `ctrlMel.mass` with the measured total in kilograms.
   Use the cfclient procedure below to correct and persist it, retaining a record
   of the previous current/stored values. PID does **not** use `ctrlMel.mass`;
   record this as not applicable rather than tuning it to fix PID flight. Other
   controllers need their own documented mass/force-conversion check.
4. Explain the consequence using `T_hover = m*g` and, for level-altitude banking,
   `T = m*g/cos(tilt)`. In an acceleration-command model mass may cancel from the
   ideal plant; it still matters where force or motor commands are generated.
   Keep measured physical mass separate from thrust-conversion calibration.
5. Reconnect after a power-cycle and verify the setting. Before any
   flight, also check battery charge, stable estimation, free/undamaged propellers,
   cage clearance, the qualified flight volume, and the emergency-stop operator.
   Correct mass is not flight qualification. Do not increase mass artificially,
   change `ctrlMel.massThrust` without a separate calibration, or relax safety
   limits to compensate for a failed takeoff. Diagnose low lift or high hover
   command before attempting faster manoeuvres.

### Set and persist mass in cfclient

With the assigned drone connected and motors stopped:

1. Open **Parameters** (enable it via **View > Tabs** if hidden), expand
   **ctrlMel**, and select **mass**. Confirm the **Persistent** column says **Yes**.
2. In the parameter details, enter the measured total in kilograms in the
   current-value field and click **Set**. Verify the current-value readback;
   `0.03563` is the example above, not a common value for every drone.
3. Click **Store** in that parameter's persistent-value section. If it instead
   says **Clear**, an earlier value is stored: record it, click **Clear** for
   this parameter only, then **Store** to save the new current value. Do not use
   the global clear-all-stored-parameters control.
4. Confirm **Stored value** matches the intended mass (allowing normal floating
   point rounding). Power-cycle the drone, reconnect, and verify **Current value**
   still matches. **Set** alone changes RAM; **Store** saves for subsequent boots.
   If persistence is unavailable or verification fails, record the failure and
   resolve it or choose another checked configuration before flying.

This follows Bitcraze's [persistent-parameter workflow](https://www.bitcraze.io/documentation/repository/crazyflie-firmware/master/userguides/logparam/#persistent-parameters).
The stored value stays with that drone across power cycles.

**Evidence for C1 and preflight.** Record measured mass in g/kg, scale resolution,
hardware configuration, controller selection, previous/new/current/stored parameter
values, post-reboot verification, and a prediction for gravity compensation.
Link any hover observations/logs; distinguish motor-command
percentage from measured thrust and record battery conditions. Powered flight is
not needed to complete this exercise. Without hardware, use instructor-supplied
mass/configuration data, mark the source and unperformed checks, and retain them
as outstanding preflight requirements rather than claiming hardware verification.

## Reproducibility manifest

In your notes, link the code/settings, commands, data source and seed needed to
repeat the comparison. Keep calibration values, units, frames and uncertainty
with the data, and note supplied recordings or missing measurements. Reuse the
runner's automatic settings/metadata files rather than transcribing them.

No `manifest.json` is required. If useful for a larger experiment, the following
fields can form an optional manifest; no submission tool parses it:

| Group | Required information where applicable |
| --- | --- |
| Identity | Team, run IDs, configuration IDs, active profile, and readiness decision. |
| Source | Code revision and any dirty diff; simulation, reference, plant, or analysis modifications needed to reproduce the result. |
| Runtime | Python, dependency versions, and reproduction commands from a stated working directory. |
| Experiment | Scenario/data IDs, dt, acquisition/sample rates, metric windows/conventions, seeds, noise/disturbance/delay settings, and configuration paths. |
| Hardware | Drone/radio identity and URI, firmware/decks/cage/battery configuration, measured total mass in g/kg and scale resolution, selected controller and mass-setting applicability, previous/current/stored mass values and reboot verification, calibration ID, world-frame/geometry version, survey provenance, and operator roles. |
| Data | Raw logs/observations or stable accessible locations, origin, SHA-256 hashes, and whether acquired by the team or supplied by instructors. |
| Qualification | Exact checked code/configuration/course IDs, team members/time, completed checks, readiness decision, remaining limitations, and fallback/abort conditions. |

Keep raw recordings unchanged in a shared location; large files need not go into
Git or the notes. Swap one comparison with another team and try to reproduce it.

## Completion and safety

We compare results to understand what works and why, alongside lap performance.
Use shared data, offline replay or a qualified fallback if equipment or time
runs short. Note the substitution, what you tried and what remains unresolved.
Supplied geometry is useful for continuing, but is not your team's own survey.

Simulation, code tests, and a parsed configuration do not authorize flight.
Apply the [race qualification/freeze policy](race_practical.md#r5-qualify-and-freeze-the-race-configuration),
retain onboard attitude stabilization, and use a named emergency-stop operator.
Any failed safety check means a qualified fallback or no flight, not weaker limits.
