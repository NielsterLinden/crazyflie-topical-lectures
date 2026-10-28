# Sensor bias and calibration exercise

## Purpose

Before tuning the filter, we check its sensors. [Calibration](glossary.md#bias-and-calibration) estimates systematic
offsets; the measurement covariance R describes the remaining random variation.

For a sensor model

\[
z_k = h(x_k) + b + v_k,
\]

estimate the bias from a known reference condition:

\[
\hat b = \frac{1}{N}\sum_{k=1}^{N}\left[z_k-h(x_{\rm ref})\right].
\]

After subtracting \(\hat b\), estimate the measurement covariance from the
remaining residuals. Keep calibration and validation recordings separate.

## Safety and preparation

Before recording, agree the [channel, timestamp and replay contract](sensor_recording_contract.md)
within the team. It lists cfclient log blocks, raw/derived labels, supported
subsets, and the metadata accompanying supplied recordings. Day 1 state-estimate
logs alone are not the raw-sensor acquisition required here.

- Motors must remain disabled throughout measurement acquisition. Remove
  propellers or use a secure restraint that prevents propeller contact.
- Inventory every available sensor and record its units, axes, frame, update
  rate, and exact firmware log name from the cfclient [Log TOC](glossary.md#telemetry-and-log-toc).
- Record firmware, deck configuration, surface, lighting, temperature, and
  sampling period with every data set.
- Label `stateEstimate.*` as derived cross-checks: these variables are outputs
  of the onboard estimator.

## E2 core: 45 minutes

The scheduled exercise is a stationary IMU check and one held-out bias/noise
comparison. Inventory all installed sensors, but choose the gyro for the
quantitative comparison. Use accelerometer data to check the stationary gravity
signal. If position is available in the same session, estimate its stationary
spread as a starting point for E3; otherwise use the supplied position-noise
assumption and label it. Full calibration of every sensor is further work.

| Minutes | Activity | Result |
| --- | --- | --- |
| 0-5 | Inventory the installed sensors and identify gyro/acceleration channels, units, frame and existing onboard corrections. | A short channel/reference list; mark other sensors as checked, supplied-data only, unavailable or not studied. |
| 5-15 | With motors disabled, record 60 s at rest for calibration and a separate 60 s at rest for validation, using the same mounting and conditions. | Two labelled recordings, or the checked supplied pair if acquisition is blocked. |
| 15-30 | Inspect the stationary IMU signals; estimate per-axis gyro bias and covariance from the calibration recording. Predict whether an additional correction is justified. | Bias/noise estimates, one comparison plot and a stated correction decision. |
| 30-40 | Apply that decision to the separate validation recording without refitting. Compare its residual mean and spread before/after. | Held-out evidence, including a failed correction if that is the result. |
| 40-45 | Save the values and limitations, agree which data/model to use for E3, and move on. | The completion check below and links in the team's shared notes. |

Prefer measurements from the team's drone. Keep the checked calibration and
separate validation recordings available before starting, so a troublesome
connection, missing channel or unexplained timing/frame feature can be left for
later. If such an issue blocks progress for five minutes, continue the same
analysis on the supplied recordings. Routine bias/noise estimation is the
exercise itself; a nonzero offset is a result to investigate, not automatically
a reason to abandon live acquisition. Preserve unexpected data and note which
source supplied each result. A calibration from another recording does not
establish calibration of a drone that the team has not checked.

### Stationary checks and analysis

Use the [IMU logging block](sensor_recording_contract.md#cfclient-channel-inventory-and-logging-blocks)
in cfclient; the Day 1 logging instructions explain where to find its CSVs.
Split collection, inspection and calculation within the team. Mark the stationary
intervals and keep the calibration and validation files separate.

1. Plot gyro axes and check that the drone was at rest. Their reference angular
   rate is zero. Convert degrees/s to rad/s before saving values for processing.
   Estimate the mean residual \(\hat b\), standard deviation and 3x3 covariance
   from the calibration interval. A split-half comparison can flag obvious drift;
   a detailed warm-up study is optional.
2. Check that the stationary accelerometer magnitude is consistent with 1 g
   (9.80665 m/s2), allowing for observed noise and mounting uncertainty. A tilted
   drone has nonzero horizontal gravity components. One pose does not identify
   per-axis accelerometer bias or scale; mark those as unknown and retain the
   existing correction. Six-orientation fitting belongs to the optional studies.
3. Subtract the proposed **additional** gyro bias from the held-out recording,
   or test the existing calibration unchanged if a new correction is unjustified.
   Compare before/after residual means and spreads. Do not subtract an onboard
   correction a second time. If the held-out result fails, preserve it and record
   the limitation rather than refitting repeatedly during this slot.
4. If stationary onboard position is available, label it as fused and estimate
   its sample covariance after centring on its calibration-recording mean.
   Check the held-out spread without retuning. This measures short-term variation,
   not absolute Lighthouse bias or accuracy: an absolute correction needs an
   independent surveyed reference. Correlations and shared IMU inputs also limit
   its interpretation as a measurement-noise R.

For an N-by-3 array of calibration residuals in consistent units, the numerical
steps are `bias = residuals.mean(axis=0)`,
`covariance = np.cov(residuals, rowvar=False, ddof=1)` and
`std = np.sqrt(np.diag(covariance))`. Use the fixed `bias` on validation samples;
do not estimate it again from that validation recording. The model/reference,
decision to correct and interpretation of the validation remain the team's work.

### Completion check and handover to E3

Keep the following with today's shared notes; one plot with several panels is
enough, and no separate calibration report is needed:

- The sensor inventory, recording IDs and conditions, with each channel's
  raw/derived status, units/frame and existing onboard correction.
- Per-axis gyro residual means and covariance, the accelerometer gravity check,
  and one held-out raw/corrected (or justified unchanged) comparison.
- A small JSON/CSV containing the tested values, units, frame, data sources and
  the decision to apply or retain each correction. Mark unmeasured parameters
  explicitly; keep onboard and offline corrections separate.
- The position-noise assumption for E3: an empirical starting value from the
  available recording, or the declared supplied/default assumption when a suitable
  position recording is unavailable. Gyro covariance is a sensor diagnostic here;
  it is not the position R used by the six-state filter.

E2 is complete when the team can explain the gyro result, the gravity check,
what the reference identifies and how the held-out comparison supports its
decision. Accept demonstrated improvement **or justified retention of the existing
calibration**. Record an unidentifiable bias as unknown. A checked supplied
calibration/validation pair completes the same analysis if live acquisition
fails; retain its source and label actual-drone checks as unperformed.

At 45 minutes, save unresolved questions and continue E3 on the supported
recording subset or controlled fallback. Optional studies below do not block
offline tuning. A failed sensor/frame check does block using that affected live
stream until resolved; another drone's calibration is not evidence about the
team's hardware. Powered flight also requires the
[team's flight checks](practical_workflow.md#responsibility-for-the-experiment)
and a suitable verified onboard configuration.

## Optional sensor characterization

Choose a study if time and suitable references are available, or return to it
after the scheduled practical. These are extensions, including when supplied
data are used, rather than additional E2 completion requirements.

| Sensor | Reference | Further characterization |
| --- | --- | --- |
| Gyroscope | Zero angular rate | Longer recordings for warm-up drift, cross-axis correlations and repeatability across power cycles. |
| Accelerometer | Gravity vector in the body frame | Six stationary orientations, with each axis approximately along \(+g\) and \(-g\); estimate per-axis offset/scale and check cross-axis coupling. |
| Optical flow | Zero image motion while stationary | Estimate offset/covariance across floor textures, illumination, heights and sensor-quality values. A condition-dependent offset needs a condition-dependent interpretation. |
| Downward range | Surveyed sensor-to-surface distance | Offset/scale, repeatability, usable range, invalid returns and surface/incidence-angle dependence. |
| Barometer | Fixed altitude | Relative zero, noise, warm-up drift and airflow/temperature sensitivity; absolute altitude needs an external reference. |
| Lighthouse position | Surveyed stationary positions | Check position offset, scale, repeatability and spatial dependence throughout the flight volume, with independently checked base-station geometry and world frame. |

For each chosen extension, define the reference and conditions, collect or select
calibration and held-out data, and test the proposed model/correction. Residual
histograms, drift fits and sensor/axis correlations can help distinguish noise,
changing bias and frame/model errors. Keep significant covariance terms and
uncertainties where identifiable. Data from more surfaces, poses or locations
support a broader claim than a single stationary recording; specify that scope.
If revising a model after looking at validation results, reserve new data for
the next independent check.
