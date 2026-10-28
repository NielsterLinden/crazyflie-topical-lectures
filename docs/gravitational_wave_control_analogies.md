# Drone control in gravitational-wave detector language

**Further reading.** This optional note connects the Day 1 concepts to
interferometer feedback. Its detailed examples and discussion questions offer
further exploration alongside the scheduled controller exercises.

This note connects the Crazyflie control exercises to feedback systems used in
interferometric gravitational-wave detectors. Both are real-time, noisy,
[multi-input multi-output (MIMO)](glossary.md#mimo) systems. Their operating points and scientific goals
are very different, so the analogy must be used with care.

See the [documentation guide](README.md) for the reading sequence and the
[Day 1 guide](day_1_control.md) and [controller worksheet](controller_practical.md)
for the practical experiments. The loop-analysis and other extensions below
offer optional further exploration.

## Translation table

| Drone control | Interferometer control analogue | Qualification |
|---|---|---|
| Position and attitude | Cavity lengths, mirror angles, suspension states | Detector coordinates describe optical and mechanical degrees of freedom. |
| Desired trajectory | Resonant operating point or commanded alignment | Science operation usually holds an operating point instead of following a race trajectory. |
| Position error | Optical error signal | Relating photodetector response to displacement requires a sensing calibration. |
| Drone dynamics | Suspended-optic, seismic-isolation, and optical plant | Radiation pressure and optical response can be part of the plant. |
| Motor command | Coil-magnet, electrostatic, piezoelectric, or radiation-pressure actuation | [Actuator](glossary.md#actuator) range, noise, and dynamics differ strongly by stage and frequency. |
| Outer-loop [LQG](glossary.md#lqr-and-lqg) | Digital length/alignment controller plus state estimation | Interferometer controllers use several estimation and filter-design approaches. |
| Onboard attitude loop | Fast inner damping or local-control loop | Nested loops require bandwidth separation and coupling analysis. |
| Takeoff and preflight | Lock acquisition and transition to low-noise operation | Both move through operating regimes where one linear controller is insufficient. |
| Flight-volume limit | Actuator range, linear sensing range, or safe operating envelope | Leaving the linear range can cause loss of lock rather than spatial collision. |
| Command saturation | Actuator saturation or digital overflow | Integrator wind-up and long recovery can occur in both systems. |
| Wind disturbance | Seismic, acoustic, thermal, or technical disturbance | Detector disturbances are strongly frequency dependent. |
| Tracking error spectrum | Residual displacement or angular noise spectrum | Spectral density is usually more informative than one RMS value for detectors. |
| Race completion | Lock robustness and observing duty factor | Scientific sensitivity must also be retained while locked. |
| Emergency landing | Controlled loss-of-lock response and interlock | Hardware protection and deterministic state transitions remain essential. |

## One feedback-loop language

Write the loop using sensor \(H(s)\), controller \(C(s)\), actuator \(A(s)\),
and plant \(P(s)\). For the scalar loop with the signal path
\(e\rightarrow C\rightarrow A\rightarrow P\rightarrow y\rightarrow H\),

\[
L(s)=P(s)A(s)C(s)H(s).
\]

The sensitivity and complementary-sensitivity functions are

\[
S(s)=\frac{1}{1+L(s)},\qquad
T(s)=\frac{L(s)}{1+L(s)}.
\]

Large loop gain suppresses disturbances entering through the plant by roughly
\(S\), but sensing noise and some control paths couple through \(T\). Increasing
gain trades residual motion against
sensor-noise injection, actuator effort, stability margin, and robustness to
plant uncertainty.

The drone time-domain exercise can be complemented by loop analysis.

For Day 1, the position loop commands velocity and the next loop commands tilt;
identify the matching velocity-lag or tilt/drag plant before forming L. The
acceleration/feed-forward race extension closes a different boundary. The
[Day 1 guide](day_1_software.md) gives the controller equations and units.
The plots below are optional loop-analysis extensions:

- identify unity-gain frequency and phase margin;
- inspect resonant modes and delays;
- compare disturbance rejection below bandwidth with noise amplification or
  poor robustness near and above bandwidth;
- verify gain and phase margins for plausible plant variations.

For the drone, RMS tracking error and hoop clearance dominate the immediate
score. For an interferometer, amplitude spectral density and coupling into the
gravitational-wave readout are central. The same controller can improve RMS
motion while degrading sensitivity in a scientifically important band.

## Operating point and lock acquisition

The drone controller is approximately linear only near the trajectory and away
from saturation. An interferometric error signal is likewise useful only over a
limited region around its intended optical operating point. Lock acquisition
brings the system into that region through a sequence of sensing and control
configurations.

Useful parallels are:

1. verify sensors, actuators, and calibration before closing the loop;
2. acquire a broad, robust operating region with conservative control;
3. hand off between sensors, actuators, or controllers as the state approaches
   the target;
4. ramp gains and offsets rather than creating discontinuities;
5. enable the final low-noise configuration only after consistency checks;
6. return through a defined recovery state after saturation or loss of lock.

This is analogous to the course fallback ladder: conservative position control
first, advanced estimator/controller second, aggressive racing only after
qualification.

## MIMO sensing and control

A coupled interferometer has several physical degrees of freedom and several
sensor channels. In linear form,

\[
z(s)=M_s(s)x(s)+n(s),\qquad
f(s)=M_a(s)u(s),
\]

where \(M_s\) is a sensing matrix and \(M_a\) an actuation matrix. Input and
output matrices are used to construct approximately decoupled control channels.
Residual off-diagonal terms allow one loop's sensing or actuation noise to enter
another degree of freedom, including the gravitational-wave readout.

Common-arm and differential-arm length are coordinate combinations of physical
arm lengths. Electronic common-mode noise describes a disturbance shared by
measurement channels; its coupling depends on the sensing electronics.
The differential-arm channel carries the gravitational-wave signal, while
common-arm and auxiliary degrees of freedom must remain controlled without
injecting excessive noise into it.

The drone equivalent is coupling between lateral acceleration, bank angle,
altitude, yaw, thrust reserve, and sensor frames. Independent scalar PID loops
can appear adequate at hover yet interact during a banked turn. The advanced
exercise should therefore inspect the full closed-loop response, not only tune
each Cartesian axis separately.

Practical questions shared by both systems:

- Is the sensing matrix valid over the full operating range?
- Does diagonalization remain valid as optical power, attitude, or speed changes?
- Which actuator has authority in each frequency band?
- Does a nominally auxiliary loop inject noise into the science or race channel?
- Are delays and sample rates represented in the MIMO model?

## Feedback, trajectory feed-forward, and noise feed-forward

The continuous-trajectory race extension uses reference feed-forward:

\[
a_{\mathrm{cmd}}=a_{\mathrm{ref}}-K(\hat x-x_{\mathrm{ref}})-\hat d.
\]

The known trajectory acceleration is applied before an error develops. The
feedback term corrects state error, while the disturbance estimate compensates
persistent model mismatch.

In a gravitational-wave detector, "feed-forward" often means measuring an
auxiliary disturbance or control channel and filtering it to cancel its known
coupling into a target channel. For witness \(w\) and target \(y\),

\[
y_{\mathrm{corrected}}(s)=y(s)-F_{wy}(s)w(s).
\]

Both uses of feed-forward anticipate a known contribution: one follows a
trajectory, while the other cancels a measured disturbance coupling.
A poor feed-forward model can inject witness noise, become unstable through an
unmodelled feedback path, or fail when the coupling changes.

## Estimation and LQG

The advanced drone solution estimates position, velocity, and persistent
acceleration disturbance, then applies LQR feedback. This supplies a compact LQG
example. For estimator-noise weighting versus physical-error and actuator-effort
costs, see the [detector bridge's LQG discussion](detector_system_analogies.md#lqg-as-online-reconstruction-plus-active-correction).

The separation principle and its limits remain important:

- it designs estimator and controller independently for an ideal linear,
  Gaussian, unsaturated model;
- the combined implementation must still be tested against delays, saturation,
  changing couplings, and non-Gaussian disturbances.

For interferometers, state estimation can reconstruct poorly measured
mechanical states, combine sensors across frequency bands, or support damping.
Many operational loops use explicitly shaped digital filters. LQG provides one
design framework alongside these frequency-domain approaches.

The drone acceleration-disturbance state has analogues in slowly varying force,
actuator-calibration, or plant-model errors. Seismic noise has its own broadband
and structured spectrum; sensing noise enters through the measurement path.

## Control noise and calibration

Closing a loop suppresses physical motion in the error signal. The external
disturbance or scientific signal is then distributed between residual error and
applied control. Reconstruction may require both:

\[
\text{external motion estimate}
\sim \text{calibrated residual} + \text{calibrated actuation}.
\]

For the drone, record estimated state, reference, residual, control command,
saturation, and estimator innovations. For an interferometer, equivalent error
and control channels are essential for calibration and diagnosing control-noise
couplings. A small in-loop error signal alone can be misleading because the loop
may suppress it or because the sensor noise is being followed.

Calibration lines or known excitations measure sensing, actuation, and loop
response. The drone analogue is injecting bounded chirps or steps during a safe
tethered or propeller-free system-identification test, then checking the measured
closed-loop transfer function against the model.

## Noise budgets and controller optimization

Replace "fastest nominal lap" with a frequency-dependent and robustness-aware
cost. Candidate controllers should be compared using:

- residual displacement or tracking-error spectrum;
- actuator command spectrum, RMS, and peak range;
- sensing-noise and auxiliary-loop coupling into the target channel;
- gain margin, phase margin, and delay sensitivity;
- saturation fraction and recovery time;
- robustness to plant, calibration, and sensing-matrix variation;
- lock retention or race completion across repeated disturbances;
- final scientific sensitivity or hoop-clearance distribution.

An optimized controller is acceptable only if it improves the declared target
without moving unacceptable noise elsewhere. This is the feedback-control
equivalent of inspecting a complete detector noise budget instead of optimizing
one subsystem metric.

## Suggested practical extensions

1. Plot open-loop gain, sensitivity, and complementary sensitivity for the drone
   linear model and identify the bandwidth and margins.
2. Inject white sensor noise and low-frequency force disturbance separately.
   Verify their different closed-loop transfer functions.
3. Add one lightly damped structural mode and retune without exciting it.
4. Introduce actuator delay and find when the nominal LQG controller loses
   adequate phase margin.
5. Couple horizontal acceleration to altitude during banked turns. Compare
   independent-axis tuning with a MIMO model.
6. Implement a lock-acquisition-style state machine: preflight, conservative
   acquisition, controller handoff, race, recovery, and abort.
7. Reconstruct an injected disturbance from both the residual and control
   command; demonstrate why the in-loop residual alone is insufficient.
8. Build a noise budget showing sensor noise, disturbance, model mismatch, and
   control effort at the output.

## References

- [Advanced LIGO length sensing and control](https://dcc.ligo.org/LIGO-P1500277-v4/public)
- [Advanced LIGO real-time digital control and data acquisition](https://dcc.ligo.org/public/0167/P2000107/006/advligorts.pdf)
- [LIGO calibration and feedback overview](https://ligo.org/science-summaries/gw150914calibration/)
- [Interferometer topology and control systems](https://dcc.ligo.org/public/0065/M930006/000/M930006.pdf)
