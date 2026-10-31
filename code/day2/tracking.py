"""Two-dimensional tracking through thin measurement/scattering planes."""

import argparse
import csv
from dataclasses import asdict, dataclass
import json
from pathlib import Path

import numpy as np


@dataclass(frozen=True)
class Detector:
    planes: int = 10
    spacing_m: float = 1.0
    active_height_m: float = 1.0
    measurement_std_m: float = 0.01
    scattering_slope_std: float = 0.02
    initial_position_std_m: float = 0.1
    initial_slope_std: float = 0.1
    fit_position_std_m: float = 1.0
    fit_slope_std: float = 1.0

    def __post_init__(self):
        if not isinstance(self.planes, int) or self.planes < 2:
            raise ValueError("Use at least two detector planes.")
        if any(not np.isfinite(value) or value <= 0
               for name, value in asdict(self).items() if name != 'planes'):
            raise ValueError("Detector lengths and standard deviations must be finite and positive.")

    @property
    def z(self):
        return np.arange(1, self.planes + 1) * self.spacing_m

    @property
    def prior_covariance(self):
        return np.diag([self.fit_position_std_m**2, self.fit_slope_std**2])


def transport(distance_m):
    """A slope kick at the departure plane precedes straight propagation."""
    if not np.isfinite(distance_m) or distance_m < 0:
        raise ValueError("Propagation distance must be finite and nonnegative.")
    F = np.array([[1., distance_m], [0., 1.]])
    G = np.array([distance_m, 1.])
    return F, G


def simulate(detector, rng):
    # Start at z=0; no material is crossed before the first measurement plane.
    truth = np.empty((detector.planes, 2))
    starting_C = np.diag([detector.initial_position_std_m**2, detector.initial_slope_std**2])
    initial_truth = rng.multivariate_normal(np.zeros(2), starting_C)
    kicks = rng.normal(0, detector.scattering_slope_std, detector.planes)
    F, G = transport(detector.spacing_m)
    truth[0] = F @ initial_truth
    for index in range(1, detector.planes):
        truth[index] = F @ truth[index-1] + G * kicks[index-1]
    measurements = truth[:, 0] + rng.normal(0, detector.measurement_std_m, detector.planes)
    active = np.abs(truth[:, 0]) <= detector.active_height_m / 2
    outgoing_truth = truth[-1] + [0, kicks[-1]]
    return dict(initial_truth=initial_truth, truth=truth, outgoing_truth=outgoing_truth,
                measurements=measurements, active=active)


def fit_track(track, detector, *, finite_aperture=False):
    x, C = np.zeros(2), detector.prior_covariance
    F, G = transport(detector.spacing_m)
    Q = np.outer(G, G) * detector.scattering_slope_std**2
    V = detector.measurement_std_m**2
    predicted, predicted_C, filtered, filtered_C = [], [], [], []
    observed = track['active'] if finite_aperture else np.ones(detector.planes, dtype=bool)
    for index, measurement in enumerate(track['measurements']):
        # Q at a plane is diag(0, sigma_slope**2). Transporting that kick
        # to the next plane creates position variance and cross-covariance.
        x, C = F @ x, F @ C @ F.T
        if index:
            C = C + Q
        predicted.append(x.copy())
        predicted_C.append(C.copy())
        if observed[index]:
            residual = measurement - x[0]
            R = C[0, 0] + V
            K = C[:, 0] / R
            x = x + K * residual
            A = np.eye(2) - np.outer(K, [1., 0.])
            C = A @ C @ A.T + np.outer(K, K) * V
        filtered.append(x.copy())
        filtered_C.append(C.copy())
    return dict(predicted=np.asarray(predicted), predicted_C=np.asarray(predicted_C),
                filtered=np.asarray(filtered), filtered_C=np.asarray(filtered_C), observed=observed.copy())


def batch_solution(measurements, detector):
    """Independent joint-Gaussian solution at the last plane, before its kick."""
    z = detector.z
    # Latent variables: initial x/slope, then kicks at the first n-1 measurement planes.
    design = np.zeros((detector.planes, detector.planes + 1))
    design[:, 0], design[:, 1] = 1, z
    design[:, 2:] = np.maximum(z[:, None] - z[None, :-1], 0)
    latent_C = np.diag([detector.fit_position_std_m**2, detector.fit_slope_std**2]
                       + [detector.scattering_slope_std**2] * (detector.planes-1))
    final = np.vstack((np.r_[1, z[-1], z[-1]-z[:-1]], np.r_[0, 1, np.ones(detector.planes-1)]))
    cross = final @ latent_C @ design.T
    measurement_C = design @ latent_C @ design.T + np.eye(detector.planes) * detector.measurement_std_m**2
    K = np.linalg.solve(measurement_C, cross.T).T
    return K @ measurements, final @ latent_C @ final.T - K @ cross.T


def ensemble(tracks=1000, seed=7, detector=None, *, finite_aperture=False, outgoing=False):
    if not isinstance(tracks, int) or tracks < 2:
        raise ValueError("Use at least two independent tracks.")
    detector = detector or Detector()
    rng = np.random.default_rng(seed)
    rows, errors, covariances = [], [], []
    example = None
    for index in range(tracks):
        track = simulate(detector, rng)
        fit = fit_track(track, detector, finite_aperture=finite_aperture)
        state = fit['filtered'][-1].copy()
        C = fit['filtered_C'][-1].copy()
        truth = track['truth'][-1]
        if outgoing:
            # The final kick is unobserved: mean unchanged, slope variance larger.
            C[1, 1] += detector.scattering_slope_std**2
            truth = track['outgoing_truth']
        error = state - truth
        pull = error / np.sqrt(np.diag(C))
        white = np.linalg.solve(np.linalg.cholesky(C), error)
        rows.append([index, *state, *truth, C[0, 0], C[0, 1], C[1, 1], *pull,
                     *white, float(white @ white), int(np.sum(fit['observed'])),
                     int(track['active'].all()), int(fit['observed'][-1])])
        errors.append(error)
        covariances.append(C)
        if example is None or (not example[0]['active'].all() and track['active'].all()):
            example = track, fit
    rows = np.asarray(rows)
    summary = dict(tracks=tracks, seed=seed, detector=asdict(detector),
                   side_of_last_plane='outgoing' if outgoing else 'incoming',
                   slope_kick_variance=detector.scattering_slope_std**2,
                   active_area_mode='finite' if finite_aperture else 'Gaussian all-plane control',
                   full_active_area_tracks=int(rows[:, 14].sum()),
                   mean_pull=rows[:, 8:10].mean(axis=0).tolist(),
                   std_pull=rows[:, 8:10].std(axis=0, ddof=1).tolist(),
                   marginal_95_percent_coverage=(np.abs(rows[:, 8:10]) <= 1.96).mean(axis=0).tolist(),
                   mean_state_error_chi2=float(rows[:, 12].mean()),
                   assumptions='Start at z=0 with a broad fit prior; planes at z=1,...,10 m by default. Thin slope kicks after each measurement; material extends beyond the active readout. Finite-mode KF skips absent hits and ignores information in the absence itself. A broad prior or acceptance selection can affect pull calibration.')
    return dict(rows=rows, errors=np.asarray(errors), covariance=np.asarray(covariances),
                summary=summary, example=example, detector=detector)


def draw_track(track, fit, detector):
    import matplotlib.pyplot as plt

    figure, axis = plt.subplots(figsize=(8, 4.5))
    for index, z in enumerate(detector.z):
        axis.plot([z, z], [-detector.active_height_m/2, detector.active_height_m/2], color='.8', lw=2)
        if fit['observed'][index]:
            axis.errorbar(z, track['measurements'][index], yerr=detector.measurement_std_m,
                          fmt='ko', markersize=3, capsize=2)
        distance = np.linspace(0, detector.spacing_m if index < detector.planes-1 else .6*detector.spacing_m, 20)
        state, C = fit['filtered'][index], fit['filtered_C'][index].copy()
        C[1, 1] += detector.scattering_slope_std**2
        mean = state[0] + distance * state[1]
        variance = C[0, 0] + 2*distance*C[0, 1] + distance**2*C[1, 1]
        sigma = np.sqrt(variance)
        axis.fill_between(z+distance, mean-sigma, mean+sigma, color='#9cdd88', alpha=.7)
        axis.plot(z+distance, mean, color='#469552', lw=.8)
    extent = np.array([0, detector.z[-1] + .6*detector.spacing_m])
    axis.plot(extent, track['initial_truth'][0]+extent*track['initial_truth'][1],
              ':', color='#e26845', label='Initial direction without scattering')
    axis.plot(0, track['initial_truth'][0], 's', color='#e9a02d', markersize=4)
    # Zoom for the fan shapes; grey lines still represent the 1 m active planes.
    lower = min(track['truth'][:, 0].min(), fit['filtered'][:, 0].min()) - .10
    upper = max(track['truth'][:, 0].max(), fit['filtered'][:, 0].max()) + .10
    axis.set(xlabel='z [m]', ylabel='x [m]', ylim=(lower, upper))
    axis.set_title('Measured hits and forward predictions with +/-1-sigma bands')
    axis.legend(fontsize=8)
    figure.tight_layout()
    return figure


def save(result, output):
    import matplotlib.pyplot as plt

    output = Path(output)
    output.mkdir(parents=True, exist_ok=False)
    with (output/'tracks.csv').open('x', newline='', encoding='utf-8') as stream:
        writer = csv.writer(stream)
        writer.writerow(['track', 'fit_x_m', 'fit_slope', 'true_x_m', 'true_slope',
                         'C_xx_m2', 'C_xt_m', 'C_tt', 'pull_x', 'pull_slope',
                         'whitened_x', 'whitened_slope', 'state_error_chi2',
                         'observed_planes', 'all_inside_active_area', 'last_plane_observed'])
        writer.writerows(result['rows'])
    (output/'summary.json').write_text(json.dumps(result['summary'], indent=2, allow_nan=False)+'\n')
    figure = draw_track(*result['example'], result['detector'])
    figure.savefig(output/'track.png', dpi=140)
    with plt.rc_context({'svg.hashsalt': 'crazyflie-tracking'}):
        figure.savefig(output/'track.svg', metadata={'Date': None, 'Creator': 'Crazyflie Topical Lectures'})
    plt.close(figure)
    figure, axes = plt.subplots(1, 3, figsize=(12, 3.5))
    for column, label, color, axis in zip((8, 9), ('Position pull', 'Slope pull'), ('C0', 'C1'), axes):
        axis.hist(result['rows'][:, column], bins=35, density=True, color=color, alpha=.65)
        grid = np.linspace(-4, 4, 200)
        axis.plot(grid, np.exp(-grid**2 / 2) / np.sqrt(2*np.pi),
                  'k--', label='Unit Gaussian reference')
        axis.set(xlabel=label, ylabel='Density')
        axis.legend(fontsize=7)
    axes[2].hist(result['rows'][:, 12], bins=35, density=True, color='C2', alpha=.65)
    grid = np.linspace(0, max(12, result['rows'][:, 12].max()), 200)
    axes[2].plot(grid, .5*np.exp(-grid/2), 'k--', label='Chi-square, 2 dof reference')
    axes[2].set(xlabel='Whitened state-error chi-square', ylabel='Density')
    axes[2].legend(fontsize=7)
    figure.suptitle(result['summary']['active_area_mode'] + ': ' + str(result['summary']['tracks']) + ' tracks')
    figure.tight_layout()
    figure.savefig(output/'pulls.png', dpi=140)
    plt.close(figure)


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--tracks', type=int, default=1000)
    parser.add_argument('--seed', type=int, default=7)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--finite-aperture', action='store_true')
    parser.add_argument('--outgoing', action='store_true', help='Estimate state after the final unobserved scattering kick')
    args = parser.parse_args(argv)
    if args.tracks < 2:
        parser.error('--tracks must be at least two')
    result = ensemble(args.tracks, args.seed, finite_aperture=args.finite_aperture, outgoing=args.outgoing)
    save(result, args.output)
    print(json.dumps(result['summary'], indent=2))
    print('Saved tracks.csv, summary.json, track.png/.svg and pulls.png to', args.output)
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
