"""Position PID for the send_hover_setpoint layer, plus offline helpers.

The position-loop exercise (see docs/controller_practical.md) keeps the velocity and height loops onboard and lets
Python own only the horizontal position loop. Its output is a velocity command
vx, vy that gets streamed with send_hover_setpoint(vx, vy, 0, z_hold).

This module holds the controller and the offline plant so it can be reused
unchanged across the three development stages:

  Stage 0 (offline): run it on a recorded flight and on a toy integrator model to
    confirm it is stable and the sign is right, no drone.
  Stage 1 (motors off): read live state, compute the command, log it but do not
    send it, and move the drone by hand to confirm the command points the
    correcting way.
  Stage 3 (flight): the same controller, its output finally streamed.

Deliberately no cflib import here: Stage 0 runs in an environment without the
radio stack, and keeping the controller free of hardware calls is what makes the
offline check meaningful. The cflib glue lives in the Stage 1 notebook.

Units: positions and targets in metres, velocities in m/s, time in seconds. The
controller is single axis; use one AxisPID per axis (x and y are independent),
which also matches the plan of bringing up one axis before both.
"""
import csv

import numpy as np


def _clamp(value, lo, hi):
    return lo if value < lo else hi if value > hi else value


class AxisPID:
    """One horizontal axis: position error in, velocity command out.

    The proportional term pulls toward the target, the derivative term damps using
    the measured EKF velocity directly (not a differenced error), which avoids
    derivative kick on a target step and is the clean velocity the plan calls for.
    The integral term rejects a modelled steady disturbance. Propulsion faults
    still require repair before flight.

    output = Kp (target - pos)  -  Kd * vel  +  Ki * integral(target - pos) dt

    clamped to +/- v_max. Because the D term uses measured velocity, with a
    constant target this is a PD on position with velocity feedback plus a slow
    integral trim, which is the standard outer loop for a velocity commanded plant.
    """

    def __init__(self, kp, kd, ki=0.0, v_max=0.3, i_max=0.15):
        self.kp = kp
        self.kd = kd
        self.ki = ki
        self.v_max = v_max          # velocity command clamp, m/s
        self.i_max = i_max          # clamp on the integral CONTRIBUTION, m/s
        self.reset()

    def reset(self):
        self._i = 0.0               # integral of position error, m*s

    def update(self, target, pos, vel, dt):
        """Return (cmd, terms) for one step. terms carries p, d, i and error.

        cmd is the clamped velocity command. Integration uses conditional
        anti-windup: the integral state is clamped so its contribution stays
        within i_max, and it does not wind further into an already saturated
        output. Pass the measured EKF velocity as vel.
        """
        error = target - pos
        p = self.kp * error
        d = -self.kd * vel

        if not np.isfinite(dt) or dt <= 0:
            raise ValueError('dt must be finite and positive')
        # Keep the previous value so rejection restores it exactly, even if
        # the candidate integral was clipped at its own contribution limit.
        previous_integral = self._i if self.ki > 0 else 0.0
        self._i = self._i + error * dt if self.ki > 0 else 0.0
        if self.ki > 0.0:
            lim = self.i_max / self.ki
            self._i = _clamp(self._i, -lim, lim)
        i = self.ki * self._i

        raw = p + d + i
        cmd = _clamp(raw, -self.v_max, self.v_max)

        # Conditional anti-windup: if the output is saturated and this step pushed
        # the integral further into the saturated direction, undo that step.
        if (raw - cmd) * error > 0:
            self._i = previous_integral
            i = self.ki * self._i
            raw = p + d + i
            cmd = _clamp(raw, -self.v_max, self.v_max)

        return cmd, {'p': p, 'd': d, 'i': i, 'error': error}


class HoverPID:
    """Two independent AxisPID loops (x and y) with a shared gain set.

    Convenience wrapper so a notebook can drive both axes with one call while the
    axes stay genuinely independent, which lets Stage 3 hold one axis at zero and
    step the other.
    """

    def __init__(self, kp, kd, ki=0.0, v_max=0.3, i_max=0.15):
        self.x = AxisPID(kp, kd, ki, v_max, i_max)
        self.y = AxisPID(kp, kd, ki, v_max, i_max)

    def reset(self):
        self.x.reset()
        self.y.reset()

    def update(self, target_xy, pos_xy, vel_xy, dt):
        vx, tx = self.x.update(target_xy[0], pos_xy[0], vel_xy[0], dt)
        vy, ty = self.y.update(target_xy[1], pos_xy[1], vel_xy[1], dt)
        return (vx, vy), {'x': tx, 'y': ty}


def simulate_axis(pid, target, dt, n_steps, x0=0.0, v0=0.0, tau=0.30, v_bias=0.0,
                  position_noise_std=0.0, velocity_noise_std=0.0, seed=0):
    """Closed loop of one AxisPID against a toy velocity tracking plant.

    Plant (a point mass whose velocity chases the command with a first order lag,
    the lag standing in for the onboard velocity loop, plus a constant drift
    v_bias representing a modelled velocity disturbance):

        v[k+1] = v[k] + (cmd - v[k]) * dt / tau
        x[k+1] = x[k] + (v[k] + v_bias) * dt

    target may be a scalar (held constant) or an array of length n_steps. With
    v_bias non zero a pure PD settles to a steady offset and only the integral term
    removes it, which is the point of the integral demo. Returns a dict of arrays:
    t, x, v, cmd, p, d, i, error. Used in Stage 0 to show the loop settles and
    points the right way before any hardware.
    """
    pid.reset()
    random = np.random.default_rng(seed)
    target = np.asarray(target, dtype=float)
    if target.ndim == 0:
        target = np.full(n_steps, float(target))

    t = np.arange(n_steps) * dt
    x = np.zeros(n_steps)
    v = np.zeros(n_steps)
    cmd = np.zeros(n_steps)
    p = np.zeros(n_steps)
    d = np.zeros(n_steps)
    i = np.zeros(n_steps)
    err = np.zeros(n_steps)

    xk, vk = x0, v0
    measured_x, measured_v = np.zeros(n_steps), np.zeros(n_steps)
    for k in range(n_steps):
        x[k] = xk
        v[k] = vk
        measured_x[k] = xk + random.normal(0, position_noise_std)
        measured_v[k] = vk + random.normal(0, velocity_noise_std)
        c, terms = pid.update(target[k], measured_x[k], measured_v[k], dt)
        cmd[k] = c
        p[k], d[k], i[k], err[k] = terms['p'], terms['d'], terms['i'], terms['error']
        vk = vk + (c - vk) * dt / tau
        xk = xk + (vk + v_bias) * dt

    return {'t': t, 'x': x, 'v': v, 'cmd': cmd,
            'p': p, 'd': d, 'i': i, 'error': err, 'target': target,
            'measured_x': measured_x, 'measured_v': measured_v}


def replay_axis(pid, target, pos, vel, t):
    """Run one AxisPID over recorded state, computing the command it WOULD issue.

    Open loop on logged data: the drone was flown by the onboard controller, and
    this asks what velocity command our PID would have produced from the same
    recorded position and velocity. It does not close the loop (the recording
    cannot react), so it is a sign and scale check, and the recorded EKF velocity
    is the onboard controller's answer to compare against.

    target, pos, vel and t are arrays of equal length; t is seconds. Returns a
    dict of arrays: cmd, p, d, i, error (aligned to the input samples, first sample
    uses dt from the second).
    """
    n = len(t)
    target = np.broadcast_to(np.asarray(target, dtype=float), (n,))
    cmd = np.zeros(n)
    p = np.zeros(n)
    d = np.zeros(n)
    i = np.zeros(n)
    err = np.zeros(n)
    pid.reset()
    for k in range(n):
        dt = t[k] - t[k - 1] if k > 0 else (t[1] - t[0] if n > 1 else 0.02)
        c, terms = pid.update(target[k], pos[k], vel[k], dt)
        cmd[k] = c
        p[k], d[k], i[k], err[k] = terms['p'], terms['d'], terms['i'], terms['error']
    return {'cmd': cmd, 'p': p, 'd': d, 'i': i, 'error': err}


def load_log(path):
    """Load one FlightLog csv into a dict of numpy arrays keyed by header name.

    Stdlib csv on the read side keeps the dotted column names (stateEstimate.x)
    intact, unlike numpy genfromtxt which mangles them. Missing or blank cells
    become NaN.
    """
    with open(path, newline='', encoding='utf-8-sig') as f:
        reader = csv.reader(f)
        header = next(reader)
        cols = {name: [] for name in header}
        for row in reader:
            for name, cell in zip(header, row):
                try:
                    cols[name].append(float(cell))
                except ValueError:
                    cols[name].append(np.nan)
    return {name: np.asarray(vals) for name, vals in cols.items()}


def nearest_merge(t_ref, t_src, *arrays):
    """Resample src arrays onto the t_ref timeline by nearest source sample.

    The state and sensor groups log as separate configs with slightly offset
    timestamps, so to feed x, y (state) and vx, vy (sensor) to one controller they
    have to be put on a common timeline. Returns the arrays sampled at the source
    index nearest each t_ref, in the order given.
    """
    t_ref = np.asarray(t_ref)
    t_src = np.asarray(t_src)
    idx = np.searchsorted(t_src, t_ref)
    idx = np.clip(idx, 1, len(t_src) - 1)
    left = t_src[idx - 1]
    right = t_src[idx]
    idx = np.where(t_ref - left <= right - t_ref, idx - 1, idx)
    return tuple(np.asarray(a)[idx] for a in arrays)
