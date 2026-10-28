# Drone estimation and control in detector language

**Further reading.** These analogies offer another way to interpret the lecture
and experimental results. The worksheets provide the models and checks used in
the scheduled exercises; choose the comparisons here that are useful to you.

This note translates the Crazyflie exercises into terminology familiar from
experimental particle-physics detectors. The mathematical structures often
match directly; their physical interpretation depends on the system.

For the reading sequence, see the [documentation guide](README.md). Follow the
[Day 2 guide](day_2_kalman.md) and [estimation worksheet](kalman_filter_practical.md)
for the practical sessions. The discussion prompts below offer optional further exploration.

For a tracking-specific treatment, including surface-bound states, material
effects, predicted/filtered/smoothed states, combinatorial finding, alignment,
and weak modes, see [Drone Kalman filtering in collider-tracking
language](collider_tracking_kalman_analogies.md).

## Translation table

| Drone problem | Detector-system analogue | Important qualification |
|---|---|---|
| True position and velocity | Latent particle state or track parameters | Drone forces are actively controlled; particle motion follows the fields and material encountered. |
| IMU propagation | Track extrapolation between detector surfaces | IMU bias and attitude errors replace material and field-model uncertainties. |
| Lighthouse position sample | Spatial hit or reconstructed space point | Its interpretation depends on whether it comes from raw observations or onboard reconstruction. |
| Optical-flow velocity | Complementary subdetector measurement | Response depends on altitude, texture, attitude, and calibration. |
| Range measurement | One-dimensional detector constraint | Geometry and incidence angle affect the measurement model. |
| Kalman prediction | Propagation of track parameters and covariance | Here the transition includes commanded or measured acceleration. |
| [Innovation](glossary.md#innovation-and-nis) \(y=z-H\hat x^-\) | Hit residual against the predicted track | Evaluated before incorporating the current measurement. |
| Innovation covariance \(S\) | Predicted residual covariance | It combines propagated uncertainty and measurement resolution. |
| [NIS](glossary.md#innovation-and-nis) \(y^TS^{-1}y\) | Chi-square contribution of one measurement block | Its degrees of freedom equal the dimension of that update. |
| Normalized scalar residual | [Pull](glossary.md#pull-and-coverage) | Unit width requires an unbiased model and correct covariance. |
| NIS gate | Outlier or hit-rejection cut | Rejection can hide a bad propagation model, not only bad data. |
| Process covariance \(Q\) | Multiple scattering, energy-loss fluctuations, or model uncertainty | \(Q\) describes uncertainty accumulated during propagation. |
| Measurement covariance \(R\) | Detector resolution and channel correlations | Correlated measurements require off-diagonal terms. |
| State covariance \(P\) | Track-parameter covariance matrix | It is an uncertainty claim requiring coverage validation. |
| Sensor dropout | Dead region, masked module, or missing detector layer | Covariance should grow while information is absent. |
| Accelerometer bias | Calibration or alignment nuisance parameter | Include it only if the data make it observable. |
| Acceleration disturbance | Slowly varying latent force or coherent model bias | Enters the dynamics; common-mode readout noise enters the measurement model. |
| Hoop aperture | Geometrical acceptance | The drone also has finite size and tracking error. |
| Successful repeated crossings | Efficiency at a selected working point | Speed, clearance, and contact rate define the operating point. |
| Monte Carlo race trials | Toy Monte Carlo or pseudo-experiments | Results depend on simulation of important tails. |
| Offline smoothing | Bidirectional track fit using later measurements | It improves reconstruction but cannot support causal control. |
| Online filtering | Trigger or online reconstruction | It must be causal and satisfy latency constraints. |

## Prediction, residual, and chi-square

A detector track fit and the drone filter use the same update algebra:

\[
y_k=z_k-H_k\hat x_k^-,\qquad
S_k=H_kP_k^-H_k^T+R_k,
\]

\[
\chi_k^2=y_k^TS_k^{-1}y_k,\qquad
\hat x_k^+=\hat x_k^-+K_ky_k.
\]

In detector jargon, \(y_k\) is the innovation or predicted hit residual and
\(\chi_k^2\) is that measurement block's chi-square contribution. In control
jargon the same scalar is normalized innovation squared (NIS). They are
numerically identical when the same residual and covariance are used. A total
fit chi-square combines contributions according to the full fit definition.

For a scalar residual, the pull is

\[
p_k=\frac{y_k}{\sqrt{S_k}}.
\]

For a vector update, dividing each component only by its diagonal standard
deviation ignores correlations. Use NIS or [whiten the innovation](glossary.md#whitening-and-cholesky-factorization)
with a factorization of \(S_k\). Validate distributions over many events or race runs,
not from one visually smooth trajectory.

## Process noise versus detector resolution

As in the translation table, \(Q\) describes propagation uncertainty, \(R\)
measurement uncertainty, and \(P\) the resulting state uncertainty.

Increasing \(R\) says that a sensor is less precise. Increasing \(Q\) says that
the propagation model is less predictive. Both can reduce apparent residual
tension, but imply different failure modes and different behaviour during
missing measurements.

For the drone, unmodelled aerodynamic acceleration resembles a stochastic
propagation effect. Lighthouse resolution belongs in \(R\). A fixed Lighthouse
frame offset is closer to detector misalignment: it calls for a bias estimate
or calibration, with uncertainty attached to the correction.

## Common-mode noise and acceleration disturbance

Detector common-mode noise is normally a measurement-domain nuisance shared by
several channels:

\[
z_i=h_i(x)+c+v_i,
\]

where \(c\) is the coherent offset. The acceleration-disturbance state in the
advanced drone model instead enters the dynamics:

\[
v_{k+1}=v_k+\Delta t\,(a_{\mathrm{cmd},k}+d_k)+w_k.
\]

Both \(c\) and \(d\) are latent nuisance parameters inferred from correlated
residual patterns. They differ in where they enter the model. A closer detector
analogue for \(d\) is a slowly varying propagation bias, field-map error, or
coherent mechanical displacement. If several sensors acquire a simultaneous
offset, model an explicit common-mode measurement state instead.

## Observability and nuisance parameters

Adding a nuisance parameter is useful only if the measurement sequence can
distinguish it from the physical state. This is the same identifiability issue
encountered in alignment and calibration fits.

- Constant horizontal acceleration eventually produces a velocity and position
  signature, so absolute position measurements can constrain it.
- Accelerometer bias and a small attitude error can be strongly degenerate while
  the drone is stationary.
- Absolute yaw is not constrained by gravity-only accelerometer measurements.
- Global translation cannot be recovered from purely relative optical flow.

Monitor covariance eigenvalues and parameter correlations. A numerical estimate
with a small reported error is not evidence that an unobservable mode became
measurable.

## LQG as online reconstruction plus active correction

The advanced solution uses [LQG](glossary.md#lqr-and-lqg), combining a Kalman
observer with an LQR controller:

\[
a_{\mathrm{cmd}}=a_{\mathrm{ref}}-K(\hat x-x_{\mathrm{ref}})-\hat d.
\]

The observer resembles online reconstruction: it estimates latent state and
nuisance parameters from noisy data. The controller has no direct equivalent in
a passive tracker because it changes the system being observed. Relevant
instrumentation analogues include beam steering, active detector alignment,
temperature stabilization, high-voltage regulation, and interferometer length
and suspension control.

The LQR state cost weights physical deviations from the reference; its control
cost penalizes actuator use. These weights express the control objective and
its units. The estimator covariance \(P\) instead describes confidence in the
reconstructed state. Each choice therefore needs its own physical justification.

## Validation language

Use familiar detector-performance discipline:

- quote efficiency and failure rate over repeated trials;
- report residual means, widths, tails, and correlations;
- perform pull and coverage tests, not only RMS comparisons;
- separate calibration, validation, and final performance samples;
- scan operating points rather than tuning on the fastest single run;
- compare data with simulation and assign margins for mismodelled tails;
- record rejected measurements and saturation, analogous to masked hits and
  clipped channels;
- treat a safety abort as a classified outcome, not missing data.

The race winner should therefore be selected at a declared working point: high
completion efficiency, controlled tail risk, and then minimum time. This is
closer to optimizing detector performance than selecting the smallest residual
from one event.

## Suggested discussion prompts

1. Which drone residual corresponds most closely to an unbiased hit residual?
2. Is a broad pull caused by underestimated \(Q\), underestimated \(R\), bias, or
   non-Gaussian tails? What additional plot distinguishes them?
3. When does NIS gating become equivalent to discarding inconvenient hits?
4. Which drone parameters resemble alignment constants, and which vary too fast
   to calibrate offline?
5. What is the drone equivalent of training and evaluating on the same events?
6. Which safety margin corresponds to detector acceptance, resolution, and
   systematic uncertainty?
7. Why can an offline smoother outperform the live filter but still be useless
   for feedback control?
