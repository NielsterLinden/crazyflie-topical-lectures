# Sensor bias and calibration exercise

## Purpose

Before tuning the filter, we check its sensors. [Calibration](glossary.md#bias-and-calibration)
estimates systematic offsets; measurement covariance \(V_k\) describes the
remaining random variation, following Frühwirth's notation.

For a measurement model

\[
m_k=h_k(x_k)+b+\epsilon_k,\qquad \operatorname{cov}(\epsilon_k)=V_k,
\]

\(h_k\) predicts the sensor reading from the state. A range sensor measures
distance; a gyro measures angular velocity. At a known reference condition,
estimate a constant additive sensor bias using

\[
\hat b=\frac{1}{N}\sum_{k=1}^{N}[m_k-h_k(x_{\rm ref})].
\]

Estimate the remaining covariance from the centered residuals. Their standard
deviation is the square root of the variance. This estimates sensor noise when
the reference is stable and its uncertainty is negligible; reference motion or
uncertainty also contributes to the observed scatter. Keep calibration and
validation recordings separate.

For a filter prediction, the predicted residual is
\(r_k^{k-1}=m_k-h_k(x_k^{k-1})\), with covariance
\(R_k^{k-1}=V_k+H_k C_k^{k-1}H_k^T\), where
\(H_k=\partial h_k/\partial x\). For a scalar measurement its pull is
\(r_k^{k-1}/\sqrt{R_k^{k-1}}\). With an unbiased model and correctly specified
uncertainties, pulls have mean zero and standard deviation one. Vector
measurements use [whitening](glossary.md#whitening-and-cholesky-factorization).

## Safety and preparation

Before recording, check the [channel, timestamp and replay instructions](sensor_recording_contract.md)
within the team. It lists cfclient log blocks, raw/derived labels, supported
subsets, and the metadata accompanying supplied recordings. Day 1 state-estimate
logs alone are not the raw-sensor acquisition required here.

- Before cfclient acquisition, place the drone down and connect cfclient. In
  **Parameters > stabilizer > stop**, enter `1`, press Enter and check the displayed
  value and stationary propellers. Leave this firmware stop engaged throughout
  acquisition; keep clear of fitted propellers. See the [recording preparation](sensor_recording_contract.md#prepare-the-drone-for-hand-held-recording),
  including the power-cycle and preflight checks before returning to flight.
- Inventory every available sensor and record its units, axes, frame, update
  rate, and exact firmware log name from the cfclient [Log TOC](glossary.md#telemetry-and-log-toc).
- Record firmware, deck configuration, surface, lighting, temperature, and
  sampling period with every data set.
- Label `stateEstimate.*` as derived cross-checks: these variables are outputs
  of the onboard estimator.

## E2 core: 45 minutes

Reuse Day 1's sensor checks. Compare stationary gyro bias/noise on calibration
and held-out data, and check the accelerometer gravity signal. E3 applies the
method to acceleration/ToF; E6 tests frozen choices. Gyro covariance remains a
diagnostic. Broader sensor calibration is optional.

| Minutes | Activity | Result |
| --- | --- | --- |
| 0-5 | Reuse Day 1's drone identity, axes, units and drift observations. Check changes and identify sensor-level gyro/acceleration channels and onboard corrections. | Update the existing notes with the IMU channels and their stationary references. |
| 5-15 | With `stabilizer.stop=1` set and checked in cfclient, record 60 s at rest for calibration and a separate 60 s at rest for validation, using the same mounting and conditions. | Two labelled recordings, or the checked supplied pair if acquisition is blocked. |
| 15-30 | Inspect the stationary IMU signals; estimate per-axis gyro bias and covariance from the calibration recording. Predict whether an additional correction is justified. | Bias/noise estimates, one comparison plot and a stated correction decision. |
| 30-40 | Apply that decision to the separate validation recording without refitting. Compare its residual mean and spread before/after. | Held-out evidence, including a failed correction if that is the result. |
| 40-45 | Save the values and limitations, agree which data/model to use for E3, and move on. | The completion check below and links in the team's shared notes. |

Prefer measurements from the team's drone. Keep the checked calibration and
separate validation recordings available before starting, so a troublesome
connection, missing channel or unexplained timing/frame feature can be left for
later. If such an issue blocks progress for five minutes, continue the same
analysis on a checked pair, or set `use_simulated_imu = True` in the notebook's
E2C input cell. This generates `E2-simulated/calibration.csv` and
`validation.csv` under the run's output folder, with independent noise and
simulation labels. Use them for the same analysis and held-out check; calibration
of the actual drone remains outstanding. Routine bias/noise estimation is the
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
   In the same comparison, inspect the standardized histograms. Calibration
   samples use their own mean and RMS, so their mean is zero and RMS is one by
   construction; their shape can still show tails or asymmetry. Held-out samples
   use those frozen calibration values, so their mean and width test transfer
   to a separate recording under the same stationary conditions. Explain this
   distinction for one channel. Shared systematic errors can remain hidden.
4. If stationary onboard position is available, label it as fused and estimate
   its sample covariance after centring on its calibration-recording mean.
   Check the held-out spread without retuning. This measures short-term variation,
   not absolute Lighthouse bias or accuracy: an absolute correction needs an
   independent surveyed reference. Correlations and shared IMU inputs also limit
   its interpretation as a measurement-noise V.

For an N-by-3 array of calibration residuals in consistent units, the numerical
steps are `bias = residuals.mean(axis=0)`,
`covariance = np.cov(residuals, rowvar=False, ddof=1)` and
`std = np.sqrt(np.diag(covariance))`. Use the fixed `bias` on validation samples;
do not estimate it again from that validation recording. The model/reference,
decision to correct and interpretation of the validation remain the team's work.

### Completion check and handover to E3

Keep the notebook's calibration summary and one held-out comparison, including
source recordings, units/frame and the correction decision. E2 is complete when
the team can explain the gyro result, the 1 g check and the reference's limits.
Demonstrated improvement or justified retention of the existing calibration are
both useful outcomes. Record an unidentifiable bias as unknown; label actual-drone
checks as unperformed when using supplied or simulated data.

At 45 minutes, save unresolved questions and continue E3 with supported data.
E3 obtains its acceleration/range calibration separately; gyro covariance
describes a different sensor. A failed sensor/frame check prevents using that
affected stream until resolved. Powered flight still requires the
[team's flight checks](practical_workflow.md#responsibility-for-the-experiment).

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
