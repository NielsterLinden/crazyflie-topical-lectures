# Drone Kalman filtering in collider-tracking language

**Further reading.** This optional note connects the Day 2 concepts to tracking
detectors. Its detailed examples and discussion questions are for further
exploration, rather than additional exercise deliverables.

This note maps the Crazyflie estimation exercises onto charged-particle tracking
in collider detectors. The Kalman equations are shared, while each system has
its own state definition, propagation physics, measurement geometry and
pattern-recognition problem.

Start with the [documentation guide](README.md) and the shorter
[detector-system bridge](detector_system_analogies.md) for shared terminology.
The [Day 2 guide](day_2_kalman.md) and [estimation worksheet](kalman_filter_practical.md)
define the practical; the extensions below are optional specialist explorations.

## Translation table

| Drone estimation | Collider tracking analogue | Qualification |
|---|---|---|
| Drone position and velocity | Local or curvilinear track parameters | Track states often contain surface coordinates, direction, and charge over momentum. |
| Time step | Propagation between detector surfaces | Track propagation is usually ordered by geometry or path length. |
| Acceleration input | Deterministic magnetic-field propagation | A charged particle follows the magnetic field and material interactions. |
| Process noise | Multiple scattering and energy-loss fluctuations | Material effects are momentum-, particle-, and path-dependent. |
| Lighthouse position | Silicon hit, drift measurement, or space point | Collider measurements are usually local coordinates on a detector surface. |
| Optical-flow velocity | Direction-sensitive complementary measurement | Direct velocity measurements are uncommon in tracking detectors. |
| Range measurement | One-dimensional strip or drift constraint | The measurement projection depends on detector geometry. |
| Stationary update | Beam-spot, vertex, mass, or geometric constraint | A zero-velocity constraint has no direct charged-track equivalent. |
| Predicted residual | Predicted hit residual | Use the state before adding the current hit. |
| Predicted residual covariance | Residual covariance | It combines extrapolation uncertainty and hit resolution. |
| [chi-square](glossary.md#predicted-residual-and-chi-square) | Incremental hit chi-square | Degrees of freedom equal the measurement dimension. |
| chi-square gate | Hit compatibility cut | In tracking this also affects pattern recognition and fake rate. |
| Sensor dropout | Inefficient module or inactive detector region | Missing hits may be expected from geometry and detector status. |
| Estimator reset | New track seed or refit from a changed hypothesis | Seed bias can influence finding efficiency and convergence. |
| Forward live estimate | Filtered track state | It uses measurements only up to the current surface. |
| Offline backward pass | Smoothed track state | It uses measurements on both sides of the surface. |
| Persistent acceleration disturbance | Propagation-model nuisance or coherent bias | The appropriate track-model analogue depends on the source of the bias. |

## Track states and reference surfaces

The drone exercise uses a Cartesian state

\[
x=(p_x,p_y,p_z,v_x,v_y,v_z)^T.
\]

Collider tracking commonly defines a state on a reference surface. One possible
bound parameterization is

\[
x=(\ell_0,\ell_1,\phi,\theta,q/p,t)^T,
\]

where \((\ell_0,\ell_1)\) are local surface coordinates, \(\phi\) and \(\theta\)
describe direction, and \(q/p\) determines curvature in a magnetic field.
Conventions differ between frameworks and detectors.

This surface dependence matters. A covariance is meaningful only together with
its parameterization and reference surface. Comparing covariance elements from
different surfaces without transport is like comparing drone body-frame and
world-frame uncertainties without a rotation.

Perigee parameters such as transverse and longitudinal impact parameters are
particularly useful near the beam line. Surface-local parameters are often more
convenient for hit updates. Transport between representations requires both the
state transformation and its Jacobian.

## Propagation and material effects

The generic prediction is

\[
x_{k+1}^-=f_k(x_k^+),\qquad
P_{k+1}^-=F_kP_k^+F_k^T+Q_k,
\]

with transport [Jacobian](glossary.md#jacobian)

\[
F_k=\left.\frac{\partial f_k}{\partial x}\right|_{x_k^+}.
\]

For the drone, \(f_k\) is a time-domain kinematic model driven by acceleration.
For a track, it transports a charged-particle state through magnetic field and
detector geometry to another surface.

The process covariance \(Q_k\) represents stochastic material interactions,
most notably multiple scattering and energy-loss fluctuations. Its size and
orientation follow the
traversed material, incidence angle, momentum, and particle hypothesis.

Mean energy loss can be part of deterministic propagation while fluctuations
enter \(Q_k\). Strongly non-Gaussian processes, especially bremsstrahlung for
electrons, violate the single-Gaussian Kalman approximation. Gaussian-sum
filters approximate the evolving state with a weighted Gaussian mixture rather
than forcing the full tail into one covariance.

The drone analogue is aggressive flight where attitude error, clipping, or
aerodynamic effects produce non-Gaussian prediction errors. Inflating a Gaussian
\(Q\) may improve robustness but does not make the model physically Gaussian.

## Measurement update on a detector surface

For a measurement \(z_k\),

\[
y_k=z_k-h_k(x_k^-),\qquad
H_k=\left.\frac{\partial h_k}{\partial x}\right|_{x_k^-},
\]

\[
S_k=H_kP_k^-H_k^T+R_k,
\]

\[
K_k=P_k^-H_k^TS_k^{-1},\qquad
x_k^+=x_k^-+K_ky_k.
\]

The hit usually measures one or two local coordinates. The measurement
covariance \(R_k\) represents intrinsic hit resolution, cluster reconstruction
and known coordinate correlations. Misalignment enters as a coherent geometry
nuisance, shared across the affected measurements.

The incremental compatibility statistic is

\[
\Delta\chi_k^2=y_k^TS_k^{-1}y_k.
\]

This is the same expression called chi-square in the drone exercise. For a correct
linear-Gaussian model it follows a chi-square distribution with dimension equal
to that hit update. The final track-fit chi-square depends on the full fit
definition. Conditioning and selection can correlate successive residuals.

## Predicted, filtered, and smoothed states

At a measurement surface distinguish:

- **predicted state:** propagated from earlier surfaces, excluding the current hit;
- **filtered state:** predicted state updated with the current hit;
- **smoothed state:** estimate using hits both before and after the surface.

The drone's live controller requires a causal filtered estimate. Collider
reconstruction can run a backward smoother after reaching the last surface.
Smoothing improves parameter precision and gives states appropriate for offline
residual, vertex, and alignment studies, but cannot be used for a real-time
decision made before future measurements exist.

Residual terminology must state which state is used. A predicted residual is the
predicted residual. A residual against a filtered state is biased toward the included
measurement and has a different covariance. An unbiased residual for alignment
or resolution studies may require removing the hit or using a suitable smoothed
construction.

## Track finding and fitting

The drone knows which telemetry stream belongs to its own state. Collider events
contain many hits from many particles, noise, secondary interactions, and pileup.
Track finding associates measurements with candidate particles; a Kalman fitter
estimates the state for a given candidate and its assigned measurements.

A typical combinatorial workflow is:

1. create seeds from compatible measurements and constraints;
2. propagate each candidate to the next detector layer or surface;
3. find compatible hits using spatial windows or chi-square cuts;
4. branch candidates when several hits are compatible;
5. allow expected holes for inefficient or inactive modules;
6. score and prune candidates to control combinatorics;
7. resolve shared-hit ambiguities and duplicate tracks;
8. refit accepted candidates and optionally smooth them.

Tight compatibility cuts reduce combinatorics and fakes but can reject real
tracks with poor seeds, material interactions, or miscalibrated uncertainties.
Loose cuts preserve efficiency but increase CPU cost and fake candidates. This
resembles choosing a drone chi-square gate, except collider gating also changes the
discrete pattern-recognition hypothesis.

## Outliers, holes, and model failure

A large residual can indicate:

- a noise hit or wrong hit assignment;
- an inefficient or malformed cluster;
- detector misalignment or calibration bias;
- incorrect material or magnetic-field model;
- a particle interaction, decay, or abrupt energy loss;
- a poor seed or wrong mass/charge hypothesis;
- underestimated propagated or measurement covariance.

Rejecting every high-chi-square hit can make the surviving residual distribution
look healthy while reducing efficiency or hiding model defects. Record rejected
hits, holes, and candidate branches. Validate against truth in simulation and
against independent detector constraints in data.

The drone equivalent is logging every rejected Lighthouse, flow, or range update
and checking whether rejection correlates with manoeuvres, geometry, or sensor
quality. Gating classifies measurements and therefore affects which data enter the fit.

## Alignment, calibration, and weak modes

Track-based alignment estimates detector geometry parameters from many track-hit
residuals. Track states are local nuisance parameters; alignment constants are
shared global parameters. Their correlations matter.

Interpreting a residual mean requires the geometry and parameter correlations. Coherent
deformations can leave track-fit chi-square nearly unchanged while biasing
momentum, impact parameters, or vertices. These weak modes require survey
constraints, beam-spot or vertex constraints, mass constraints, cosmic tracks,
overlaps, or other complementary data.

The drone analogy is calibrating Lighthouse geometry or sensor-frame alignment
from many trajectories. Simultaneously shifting the reconstructed world frame
and trajectory may preserve residuals while changing physical hoop coordinates.
External geometry constraints fix the gauge freedom.

For global alignment, correlations between states at different surfaces and
between residuals cannot generally be discarded. A sequential fit is
computationally convenient, but downstream calibration may need the global
covariance implied by the smoother.

## Pulls, coverage, and performance

For scalar parameter or residual error \(e\) with predicted standard deviation
\(\sigma\), the pull is

\[
p=e/\sigma.
\]

An approximately standard-normal pull distribution is expected only after
accounting for bias, selection, non-Gaussian tails, and the correct covariance
definition. Check means, widths, tails, and correlations versus momentum,
pseudorapidity, occupancy, detector region, and hit count.

Collider tracking performance requires more than fit chi-square:

- reconstruction efficiency for reconstructable particles;
- fake and duplicate rate;
- hit efficiency and hole rate;
- momentum, direction, and impact-parameter resolution;
- parameter scale and bias;
- pull width and confidence-interval coverage;
- CPU time and candidate multiplicity;
- robustness versus pileup, material, alignment, and detector conditions.

The drone analogue should similarly report race completion efficiency, contacts,
tracking error, clearance tails, rejected measurements, saturation, uncertainty
coverage, and runtime. Optimizing only the best lap resembles reporting track
resolution after silently discarding difficult particles.

## Limits of the analogy

- The drone is actively controlled; the tracker passively observes particles.
- Drone filtering is time-causal; offline tracking can smooth in both directions.
- Collider propagation follows detector geometry and magnetic field rather than
  a commanded acceleration.
- Collider reconstruction includes a hard combinatorial association problem.
- Track material effects depend strongly on momentum and particle species.
- The drone may repeatedly observe one object; each collider track is usually a
  short, one-pass trajectory through the detector.

## Suggested practical extensions

1. Label every drone estimate as predicted, filtered, or smoothed and plot the
   corresponding residuals separately.
2. Whiten vector predicted residuals with the full \(S\) matrix and compare their squared
   norm (the chi-square) with a chi-square distribution with the measurement dimension
   as its degrees of freedom. The norm itself follows a chi distribution under
   the same unbiased Gaussian assumptions.
3. Introduce a sensor-frame misalignment. Show that increasing \(R\) does not
   correctly remove the resulting coherent residual bias.
4. Add a non-Gaussian acceleration impulse and compare a standard Kalman filter,
   robust update, and Gaussian-mixture approximation.
5. Remove position updates over one course region and treat the gap like an
   inactive detector module. Verify covariance growth and recovery.
6. Create ambiguous synthetic measurements and implement candidate branching and
   pruning. Compare efficiency, fake rate, and CPU cost versus the gate.
7. Run a backward smoother offline and explain why its improved trajectory cannot
   be fed back into the earlier live flight.
8. Fit a shared geometry offset from many runs, then study an unconstrained weak
   mode and add an external survey constraint.

## References

- [ACTS track-fitting documentation](https://acts.readthedocs.io/en/stable/core/reconstruction/track_fitting.html)
- [CMS track and primary-vertex reconstruction performance](https://cms-results.web.cern.ch/cms-results/public-results/publications/TRK-11-001/index.html)
- [Global covariance of Kalman-fitted tracks and detector alignment](https://cds.cern.ch/record/1134267)
- [LHCb tracking-station alignment with Kalman-fitted tracks](https://cds.cern.ch/record/1142717)
