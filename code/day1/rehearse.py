"""Race rehearsal without the track: fly a planned trajectory.csv (planner output) with position setpoints,
shifted so it starts where the drone stands, optionally scaled down to fit the room. Each real flight needs a go.

    PY rehearse.py check plans/race1/trajectory.csv room=3.0          # no radio: fits? speeds? predicted lag
    PY rehearse.py fly plans/race1/trajectory.csv room=3.0            # FLIES
    PY rehearse.py fly plans/race1/trajectory.csv room=3.0 laps=2     # FLIES two laps, straight back in between
    PY rehearse.py fly race2/plan_fast/trajectory.csv absolute=1 lead=0.5   # FLIES race 2 fast path
    PY rehearse.py fly <plan>/trajectory.csv absolute=1 tuning=<submission>/optimized/tuning.json   # FLIES with it

`room` is the free length in metres in front of the drone along +x (and +-room/2 sideways). The path is scaled
uniformly in x/y and in time if it does not fit, so the speeds stay the race speeds; z is kept, clipped
to 0.3-1.5 m. Works on the flow deck (relative frame) or with Lighthouse.
"""
import sys
import time

import numpy as np

import crazyflie
import drone_pipeline as dp
import flip

LAG_S = 0.50         # measured position-loop lag of drone 4 (1/xKp)


def load(path, room=3.0, z_min=0.3, z_max=1.5, absolute=False):
    d = np.genfromtxt(path, delimiter=',', names=True)
    t = d['time_s'] - d['time_s'][0]
    p = np.c_[d['x_m'], d['y_m'], d['z_m']]
    if absolute:                        # Lighthouse: fly the planned coordinates as they are
        return t, p, 1.0
    xy = p[:, :2] - p[0, :2]
    # rotate so the path's overall direction (start -> furthest point) is +x
    far = xy[np.argmax(np.linalg.norm(xy, axis=1))]
    a = -np.arctan2(far[1], far[0])
    xy = xy @ np.array([[np.cos(a), np.sin(a)], [-np.sin(a), np.cos(a)]])
    need = max(xy[:, 0].max(), 0.01), max(np.abs(xy[:, 1]).max() * 2, 0.01)
    s = min(1.0, room / need[0], room / need[1], room / max(-xy[:, 0].min(), 1e-9) if xy[:, 0].min() < 0 else 1.0)
    return t * s, np.c_[xy * s, np.clip(p[:, 2], z_min, z_max)], s      # time scaled too: race speeds kept


def check(path, room=3.0, absolute=False, lead=0.0):
    t, p, s = load(path, room, absolute=absolute)
    vp = np.gradient(p, t, axis=0)
    v = np.linalg.norm(np.diff(p, axis=0), axis=1) / np.diff(t)
    q, err = p[0].copy(), []
    for i in range(1, len(t)):
        q += (p[i] + lead * vp[i] - q) * (t[i] - t[i - 1]) / LAG_S
        err.append(np.linalg.norm(p[i] - q))
    print('%s: %.0f s, scale %.2f, x %.2f..%.2f m, y %.2f..%.2f m, z %.2f..%.2f m, peak speed %.2f m/s, '
          'predicted max lag error %.2f m' % (path, t[-1], s, p[:, 0].min(), p[:, 0].max(), p[:, 1].min(),
                                              p[:, 1].max(), p[:, 2].min(), p[:, 2].max(), v.max(), max(err)))
    return dict(duration_s=t[-1], scale=s, peak_speed=v.max(), lag_error=max(err))


def repeat(t, p, laps=1, v_back=0.3):
    """laps > 1: straight transit back to the start at v_back (linear interp between end and start), then again."""
    T, P = t, p
    for _ in range(int(laps) - 1):
        T = np.r_[T, T[-1] + max(np.linalg.norm(p[-1] - p[0]) / v_back, 0.5) + t]
        P = np.r_[P, p]
    return T, P


def apply_tuning(cf, path):
    """Set a submission tuning.json's parameters (baseline x factor) the way the runner does; reboot restores."""
    sys.path.insert(0, str(__import__('pathlib').Path(__file__).resolve().parents[1]))
    from race_common.tuning import load_tuning, parameter_value, selected_parameters
    tun = load_tuning(path, dp.URI)
    for name in selected_parameters(tun):
        if abs(parameter_value(cf, name) - tun['baseline'][name]) > 1e-6:
            raise crazyflie.DroneError('%s on the drone differs from the tuning baseline' % name)
        print('tuning: %s %.3f -> %.3f' % (name, tun['baseline'][name],
                                          parameter_value(cf, name, expected=tun['baseline'][name] * tun['factor'])))


def fly(path, room=3.0, vbat_min=3.9, laps=1, absolute=False, backflip=0, flip_z=1.7, tuning=None, lead=0.0, arm='', hold=2.0, lh_std=0.0):
    """FLIES the trajectory `laps` times. Enter lands. Lands too when the estimate leaves the path's box by 0.3 m.
    backflip=1 (race 2 finale, Lighthouse only): after the path (also after a fence trip), 0.3 m/s to (0, 0, flip_z), hold 2 s, flip.fly's
    gates + flip, land. Skipped only after Enter; the hover-thrust gate stands in for vbat in flight."""
    if backflip and not (absolute and 1.2 <= flip_z <= 2.2):
        raise crazyflie.DroneError('backflip needs absolute=1 and 1.2 <= flip_z <= 2.2')
    t, p, s = load(path, room, absolute=absolute)
    t, p = repeat(t, p, laps)
    vp = np.gradient(p, t, axis=0)     # lead > 0: command p + lead*v so the lagging drone sits on the path
    if np.linalg.norm(np.diff(p, axis=0), axis=1).max() / np.diff(t).min() > 0.8:
        raise crazyflie.DroneError('planned speed above 0.8 m/s')
    dp.preflight(vbat_min=vbat_min)
    rows = []
    with crazyflie.connect(dp.URI) as scf:
        cf = scf.cf
        if tuning:
            apply_tuning(cf, tuning)
        if lh_std:                      # trust Lighthouse more than the height sensor (it sees hoop rims); reboot restores
            cf.param.set_value('lighthouse.sweepStd', lh_std)
            cf.param.set_value('lighthouse.sweepStd2', 2.5 * lh_std)
            time.sleep(0.3)
            print('lighthouse sweepStd', cf.param.get_value('lighthouse.sweepStd'), 'sweepStd2', cf.param.get_value('lighthouse.sweepStd2'))
        if arm:                         # checked and connected; take off within ~2 s once the file `arm` appears
            go, t_arm = __import__('pathlib').Path(arm), time.monotonic()
            go.unlink(missing_ok=True)
            print('ARMED: waiting for', go, flush=True)
            while not go.exists():          # no abort window yet: it would take the keyboard while armed
                if time.monotonic() - t_arm > 900:
                    print('not flying: 15 min without go')
                    return None
                time.sleep(0.05)
            go.unlink()
        abort = crazyflie.start_abort('rehearsal: Enter lands')
        crazyflie.reset_estimator(scf, settle=1.5)
        conf = flip._log(scf, rows)
        time.sleep(0.3)
        o = np.zeros(3) if absolute else np.array([rows[-1]['stateEstimate.x'], rows[-1]['stateEstimate.y'], 0.0])
        ref = []
        if backflip:
            cf.commander.send_setpoint(0, 0, 0, 0)     # unlock the rpyt commander on the ground, as flip.fly does
        try:
            for zi in np.linspace(0.1, p[0, 2], int(p[0, 2] / 0.4 * 100)):
                cf.commander.send_position_setpoint(o[0], o[1], float(zi), 0.0)
                time.sleep(0.01)
            if not flip._hold(cf, o[0] + p[0, 0], o[1] + p[0, 1], p[0, 2], hold, abort):
                return flip._finish(cf, conf, rows, o[0], o[1], 'rehearsal')
            t0 = time.monotonic()
            while (tn := time.monotonic() - t0) < t[-1]:
                sp = o + np.array([np.interp(tn, t, p[:, i]) + lead * np.interp(tn, t, vp[:, i]) for i in range(3)])
                cf.commander.send_position_setpoint(*map(float, sp), 0.0)
                ref.append((time.monotonic(), *sp))
                e = rows[-1]
                rel = np.array([e['stateEstimate.x'], e['stateEstimate.y']]) - o[:2]
                if np.any(rel < p[:, :2].min(0) - 0.3) or np.any(rel > p[:, :2].max(0) + 0.3):   # path box + 0.3 m
                    print('FENCE: estimate at x %+.2f y %+.2f from start, leaving the path' % tuple(rel))
                    break
                abort.pump()
                if abort.stop.is_set():
                    break
                time.sleep(0.01)
            flip._hold(cf, *map(float, sp), 1.0, abort)
            if backflip and not abort.stop.is_set():   # fence trip too: the gates at the centre still decide
                c = np.array([0.0, 0.0, flip_z])
                for q in np.linspace(sp, c, max(2, int(np.linalg.norm(c - sp) / 0.3 * 100))):   # 0.3 m/s line
                    cf.commander.send_position_setpoint(*map(float, q), 0.0)
                    abort.pump()
                    if abort.stop.is_set():
                        break
                    time.sleep(0.01)
                if not abort.stop.is_set() and flip._hold(cf, 0.0, 0.0, flip_z, 2.0, abort):
                    flip._maneuver(cf, rows, 0.0, 0.0, flip_z, 'flip', abort)
                sp = [rows[-1]['stateEstimate.x'], rows[-1]['stateEstimate.y']]     # land where it is
            name = flip._finish(cf, conf, rows, float(sp[0]), float(sp[1]), 'rehearsal')
        except BaseException:
            e = rows[-1] if rows else {'stateEstimate.x': o[0], 'stateEstimate.y': o[1], 'stateEstimate.z': 0.3}
            flip._land(cf, e['stateEstimate.x'], e['stateEstimate.y'], max(0.1, e['stateEstimate.z']))
            raise
        finally:
            abort.close()
    R = np.array(ref)
    if len(R):
        est = np.array([[r['t'], r['stateEstimate.x'], r['stateEstimate.y'], r['stateEstimate.z']] for r in rows])
        m = np.c_[[np.interp(R[:, 0], est[:, 0], est[:, i]) for i in (1, 2, 3)]].T
        err = np.linalg.norm(m - R[:, 1:], axis=1)
        print('tracking error vs setpoint: mean %.3f m, max %.3f m (model %.2f m)' % (err.mean(), err.max(),
                                                                                    check(path, room, absolute, lead)['lag_error']))
    return name


if __name__ == '__main__':
    if len(sys.argv) < 3 or sys.argv[1] not in ('check', 'fly'):
        sys.exit(__doc__)
    kw = {k: dp._arg(v) for k, v in (a.split('=', 1) for a in sys.argv[3:])}
    print(globals()[sys.argv[1]](sys.argv[2], **kw))
