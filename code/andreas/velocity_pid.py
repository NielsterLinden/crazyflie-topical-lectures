"""Velocity PID for the send_zdistance_setpoint layer, plus offline helpers.

Layer 2 of the bring-up order in docs/controller_practical.md:
Python owns the horizontal position loop and the velocity loop, while the
attitude loops and the height hold stay onboard. The position loop reuses
hover_pid.AxisPID unchanged (position error in, velocity setpoint out); the
loop here turns the velocity error into a tilt command in degrees, to be
streamed with send_zdistance_setpoint(roll, pitch, yawrate, zdistance).

Axis convention: each axis commands "tilt that accelerates toward the positive
axis", in degrees. How that maps onto the firmware's signed roll and pitch is
measured from the recorded stage 3 flights in stage0_velocity_pid_offline.ipynb
and confirmed in Stage 1 before any flight uses it.

Units: velocities in m/s, tilt in degrees, time in seconds. Deliberately no
cflib import, as in hover_pid.py: Stage 0 runs anywhere.
"""
import numpy as np

from . import hover_pid

G = 9.81   # m/s^2


def _clamp(value, lo, hi):
    return lo if value < lo else hi if value > hi else value


class AxisVelocityPID:
    """One horizontal axis: velocity error in, tilt command out, degrees.

    output = Kv (target_v - vel)  +  Kvi integral(target_v - vel) dt

    clamped to +/- tilt_max. Proportional plus optional integral: holding a
    steady velocity against drag needs a steady tilt, which a pure P loop can
    only supply with a steady velocity error; the integral provides the
    holding tilt instead (and absorbs a tilt bias from the airframe). No
    derivative term: the onboard attitude loop already damps the tilt
    response, and differentiating the EKF velocity would only add noise.
    """

    def __init__(self, kv, kvi=0.0, tilt_max=6.0, i_max=3.0):
        self.kv = kv                # deg per m/s of velocity error
        self.kvi = kvi              # deg per m/s per s
        self.tilt_max = tilt_max    # tilt command clamp, deg
        self.i_max = i_max          # clamp on the integral CONTRIBUTION, deg
        self.reset()

    def reset(self):
        self._i = 0.0               # integral of velocity error, m

    def update(self, target_v, vel, dt):
        """Return (tilt_cmd, terms) for one step. terms carries p, i and error.

        Same conditional anti-windup as hover_pid.AxisPID: the integral
        contribution is clamped to i_max and does not wind further into an
        already saturated output. Pass the measured EKF velocity as vel.
        """
        error = target_v - vel
        p = self.kv * error

        if not np.isfinite(dt) or dt <= 0:
            raise ValueError('dt must be finite and positive')
        previous_integral = self._i if self.kvi > 0 else 0.0
        self._i = self._i + error * dt if self.kvi > 0 else 0.0
        if self.kvi > 0.0:
            lim = self.i_max / self.kvi
            self._i = _clamp(self._i, -lim, lim)
        i = self.kvi * self._i

        raw = p + i
        cmd = _clamp(raw, -self.tilt_max, self.tilt_max)

        if (raw - cmd) * error > 0:
            self._i = previous_integral
            i = self.kvi * self._i
            raw = p + i
            cmd = _clamp(raw, -self.tilt_max, self.tilt_max)

        return cmd, {'p': p, 'i': i, 'error': error}


class VelocityPID:
    """Two independent AxisVelocityPID loops (x and y) with a shared gain set."""

    def __init__(self, kv, kvi=0.0, tilt_max=6.0, i_max=3.0):
        self.x = AxisVelocityPID(kv, kvi, tilt_max, i_max)
        self.y = AxisVelocityPID(kv, kvi, tilt_max, i_max)

    def reset(self):
        self.x.reset()
        self.y.reset()

    def update(self, target_v_xy, vel_xy, dt):
        tx, terms_x = self.x.update(target_v_xy[0], vel_xy[0], dt)
        ty, terms_y = self.y.update(target_v_xy[1], vel_xy[1], dt)
        return (tx, ty), {'x': terms_x, 'y': terms_y}


class CascadePID:
    """Position to velocity to tilt, both horizontal axes.

    Position loop: hover_pid.HoverPID with the flown layer 1 gains, its output
    still clamped at v_max as in the flown layer. Velocity loop: VelocityPID.
    update returns the tilt commands plus the intermediate velocity setpoints
    so a flight loop can log both.
    """

    def __init__(self, position_gains, velocity_gains):
        self.position = hover_pid.HoverPID(**position_gains)
        self.velocity = VelocityPID(**velocity_gains)

    def reset(self):
        self.position.reset()
        self.velocity.reset()

    def update(self, target_xy, pos_xy, vel_xy, dt):
        v_set, pos_terms = self.position.update(target_xy, pos_xy, vel_xy, dt)
        tilt, vel_terms = self.velocity.update(v_set, vel_xy, dt)
        return tilt, {'v_set': v_set, 'position': pos_terms,
                      'velocity': vel_terms}


def tilt_plant_step(vel, tilt_actual, tilt_cmd, dt, tau_att, drag):
    """One step of the toy tilt plant, shared by the two simulators below.

    The actual tilt chases the command with a first order lag (standing in for
    the onboard attitude loop), the tilt accelerates the mass, and a linear
    drag opposes the velocity:

        tilt[k+1] = tilt[k] + (cmd - tilt[k]) * dt / tau_att
        v[k+1]    = v[k] + (G tan(tilt[k+1]) - drag v[k]) * dt

    The defaults are measured from the flown stage 3 runs in
    stage0_velocity_pid_offline.ipynb: drag 0.25 1/s from the tilt fit, and
    tau_att 0.2 s from the lag between velocity error and attitude response.
    Returns (vel, tilt_actual) advanced by one step.
    """
    tilt_actual = tilt_actual + (tilt_cmd - tilt_actual) * dt / tau_att
    accel = G * np.tan(np.radians(tilt_actual)) - drag * vel
    return vel + accel * dt, tilt_actual


def simulate_velocity_axis(pid, target_v, dt, n_steps, tau_att=0.2, drag=0.25,
                           v0=0.0, velocity_noise_std=0.0, seed=0):
    """Closed loop of one AxisVelocityPID against the toy tilt plant.

    target_v may be a scalar or an array of length n_steps. Returns a dict of
    arrays: t, v, tilt_cmd, tilt, p, i, error, target.
    """
    pid.reset()
    random = np.random.default_rng(seed)
    target_v = np.asarray(target_v, dtype=float)
    if target_v.ndim == 0:
        target_v = np.full(n_steps, float(target_v))

    t = np.arange(n_steps) * dt
    v = np.zeros(n_steps)
    tilt_cmd = np.zeros(n_steps)
    tilt = np.zeros(n_steps)
    p = np.zeros(n_steps)
    i = np.zeros(n_steps)
    err = np.zeros(n_steps)

    vk, tiltk = v0, 0.0
    measured_v = np.zeros(n_steps)
    for k in range(n_steps):
        v[k] = vk
        measured_v[k] = vk + random.normal(0, velocity_noise_std)
        c, terms = pid.update(target_v[k], measured_v[k], dt)
        tilt_cmd[k] = c
        p[k], i[k], err[k] = terms['p'], terms['i'], terms['error']
        tilt[k] = tiltk
        vk, tiltk = tilt_plant_step(vk, tiltk, c, dt, tau_att, drag)

    return {'t': t, 'v': v, 'tilt_cmd': tilt_cmd, 'tilt': tilt,
            'p': p, 'i': i, 'error': err, 'target': target_v, 'measured_v': measured_v}


def simulate_cascade_axis(pos_pid, vel_pid, target, dt, n_steps, tau_att=0.2,
                          drag=0.25, x0=0.0, v0=0.0, position_noise_std=0.0,
                          velocity_noise_std=0.0, seed=0):
    """Closed loop of one axis of the full cascade against the toy tilt plant.

    Position error -> velocity setpoint (pos_pid, a hover_pid.AxisPID) ->
    tilt command (vel_pid, an AxisVelocityPID) -> toy tilt plant. Returns a
    dict of arrays: t, x, v, v_set, tilt_cmd, tilt, target.
    """
    pos_pid.reset()
    vel_pid.reset()
    random = np.random.default_rng(seed)
    target = np.asarray(target, dtype=float)
    if target.ndim == 0:
        target = np.full(n_steps, float(target))

    t = np.arange(n_steps) * dt
    x = np.zeros(n_steps)
    v = np.zeros(n_steps)
    v_set = np.zeros(n_steps)
    tilt_cmd = np.zeros(n_steps)
    tilt = np.zeros(n_steps)

    xk, vk, tiltk = x0, v0, 0.0
    measured_x, measured_v = np.zeros(n_steps), np.zeros(n_steps)
    for k in range(n_steps):
        x[k] = xk
        v[k] = vk
        measured_x[k] = xk + random.normal(0, position_noise_std)
        measured_v[k] = vk + random.normal(0, velocity_noise_std)
        s, _ = pos_pid.update(target[k], measured_x[k], measured_v[k], dt)
        v_set[k] = s
        c, _ = vel_pid.update(s, measured_v[k], dt)
        tilt_cmd[k] = c
        tilt[k] = tiltk
        vk, tiltk = tilt_plant_step(vk, tiltk, c, dt, tau_att, drag)
        xk = xk + vk * dt

    return {'t': t, 'x': x, 'v': v, 'v_set': v_set,
            'tilt_cmd': tilt_cmd, 'tilt': tilt, 'target': target,
            'measured_x': measured_x, 'measured_v': measured_v}


def replay_velocity_axis(pid, target_v, vel, t):
    """Run one AxisVelocityPID over recorded data, open loop.

    target_v is the recorded velocity setpoint stream (the cmd column of a
    stage 3 run: what the position loop asked for in flight), vel the recorded
    EKF velocity, t seconds. The recording cannot react, so as in
    hover_pid.replay_axis this is a sign and scale check: the recorded
    attitude is the onboard velocity loop's answer to the same request, to
    compare against. Returns a dict of arrays: tilt_cmd, p, i, error.
    """
    n = len(t)
    target_v = np.broadcast_to(np.asarray(target_v, dtype=float), (n,))
    tilt_cmd = np.zeros(n)
    p = np.zeros(n)
    i = np.zeros(n)
    err = np.zeros(n)
    pid.reset()
    for k in range(n):
        dt = t[k] - t[k - 1] if k > 0 else (t[1] - t[0] if n > 1 else 0.02)
        c, terms = pid.update(target_v[k], vel[k], dt)
        tilt_cmd[k] = c
        p[k], i[k], err[k] = terms['p'], terms['i'], terms['error']
    return {'tilt_cmd': tilt_cmd, 'p': p, 'i': i, 'error': err}
