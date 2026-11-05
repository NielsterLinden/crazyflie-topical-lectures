"""Race 2 hand-made path: no stops, 3 -> 2 -> 1 -> 3 -> 2 -> 1 -> 3, straight through every hoop.

    PY race2/fast_path.py [v_max=0.7] [v_hoop=0.5] [a=0.9]      # writes race2/plan_fast/trajectory.csv + checks

Piecewise-cubic Hermite through hand-picked waypoints; at each hoop a straight pre -> centre -> post line along
the hoop normal. Speed profile: v_max on straights, v_hoop on the hoop lines, lateral and tangential
acceleration <= a. Checks the planned path AND the first-order lag model of drone 4 flown with lead (what
rehearse.py lead=0.5 sends) for hoop-frame clearance, bounds and speed.
"""
import json
import sys
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve().parent
LAG_S = 0.50                 # drone 4 position-loop lag (1/xKp)
DRONE_R, MARGIN = 0.12, 0.05
BOX = np.array([[-1.3, -1.8, 0.3], [1.7, 1.4, 1.5]])     # where we have flown so far, +-0.3 m


def hoops():
    c = json.loads((HERE / 'plan_cons/course.json').read_text())
    by = {h['name'].split('-pass')[0]: h for h in c['hoops']}
    return {k: (np.array(h['centre']), np.array(h['normal']) / np.linalg.norm(h['normal']), h['radius_m'])
            for k, h in by.items()}


def waypoints(H, d=0.35):
    """[(point, tangent or None, on_hoop_line)]: tangent None = Catmull-Rom.
    Flight 13:42 lessons: hoop 3 needs a longer straight run-in (tight turn put it 0.35 m off centre), and the
    climb out of hoop 1 must leave the space above hoop 1 quickly and low (estimate jumps there, overshoot to 1.59 m)."""
    def through(k, pre, post, shift=0.0):
        c, n, _ = H[k]
        if shift:                       # move the pass line `shift` m in the hoop plane, towards hoop 3
            u = H['hoop-3'][0] - c
            u -= (u @ n) * n
            c = c + shift * u / np.linalg.norm(u)
        return [(c - pre * n, n, True), (c, n, True), (c + post * n, n, True)]
    after3 = [(np.array([0.20, 0.95, 0.70]), None, False), (np.array([-0.95, 0.05, 0.43]), None, False)]
    after2 = [(np.array([0.80, -1.35, 0.45]), None, False)]
    after1 = [(np.array([1.15, -0.55, 1.10]), None, False)]
    lap = through('hoop-3', 0.65, d) + after3 + through('hoop-2', d, d) + after2 + through('hoop-1', 0.40, 0.25, shift=0.04) + after1
    c3, n3, _ = H["hoop-3"]   # start low at z 0.3: diagonal climb, no hover
    return ([(np.array([0.0, 0.0, 0.3]), np.zeros(3), False), (np.array([0.75, -0.25, 0.82]), None, False)]
            + lap + lap + through('hoop-3', 0.65, d) + [(c3 + 0.75 * n3, np.zeros(3), False)])


def curve(W, n=4000):
    """Dense samples of the Hermite curve and a per-sample 'on hoop line' flag."""
    P = [w[0] for w in W]
    T = []
    for i, (p, t, _) in enumerate(W):
        if t is None:
            t = P[i + 1] - P[i - 1]
        T.append(t / max(np.linalg.norm(t), 1e-9))
    pts, flag = [], []
    for i in range(len(W) - 1):
        L = np.linalg.norm(P[i + 1] - P[i])
        u = np.linspace(0, 1, max(20, int(L * 400)), endpoint=False)[:, None]
        h00, h10, h01, h11 = 2*u**3 - 3*u**2 + 1, u**3 - 2*u**2 + u, -2*u**3 + 3*u**2, u**3 - u**2
        pts.append(h00 * P[i] + h10 * L * T[i] + h01 * P[i + 1] + h11 * L * T[i + 1])
        flag += [W[i][2] and W[i + 1][2]] * len(u)
    pts.append(P[-1][None])
    flag.append(False)
    return np.vstack(pts), np.array(flag)


def timing(p, hoop, v_max, v_hoop, a):
    s = np.r_[0, np.cumsum(np.linalg.norm(np.diff(p, axis=0), axis=1))]
    d1 = np.gradient(p, s, axis=0)
    d2 = np.gradient(d1, s, axis=0)
    kappa = np.linalg.norm(np.cross(d1, d2), axis=1) / np.maximum(np.linalg.norm(d1, axis=1) ** 3, 1e-9)
    v = np.minimum(np.where(hoop, v_hoop, v_max), np.sqrt(a / np.maximum(kappa, 1e-6)))
    v[0] = v[-1] = 0.0
    ds = np.diff(s)
    for i in range(1, len(v)):                       # forward: accelerate within a
        v[i] = min(v[i], np.sqrt(v[i - 1] ** 2 + 2 * a * ds[i - 1]))
    for i in range(len(v) - 2, -1, -1):              # backward: brake within a
        v[i] = min(v[i], np.sqrt(v[i + 1] ** 2 + 2 * a * ds[i]))
    t = np.r_[0, np.cumsum(ds / np.maximum((v[1:] + v[:-1]) / 2, 1e-3))]
    return t, s


def resample(t, p, rate=50.0):
    T = np.linspace(0, t[-1], int(t[-1] * rate) + 1)
    P = np.c_[[np.interp(T, t, p[:, i]) for i in range(3)]].T
    V = np.gradient(P, T, axis=0)
    A = np.gradient(V, T, axis=0)
    return T, P, V, A


def lag_sim(T, P, V, lead):
    """Drone 4 as a first-order position lag, commanded p + lead*v (rehearse.py lead=)."""
    q, out = P[0].copy(), []
    for i in range(len(T)):
        if i:
            q += (P[i] + lead * V[i] - q) * (T[i] - T[i - 1]) / LAG_S
        out.append(q.copy())
    return np.array(out)


def clearance(X, H):
    """Worst margin to each hoop frame (m, >0 = clear) and the number of plane crossings inside it."""
    res = {}
    for k, (c, n, r) in H.items():
        s = (X - c) @ n
        rho = np.linalg.norm((X - c) - s[:, None] * n, axis=1)
        near = np.abs(s) < 0.05 + DRONE_R                   # within half frame depth + drone radius
        inside = r - DRONE_R - rho                           # margin when passing through
        outside = rho - (r + 0.03 + DRONE_R)                 # margin when passing beside (frame tube ~3 cm)
        m = np.where(rho < r, inside, outside)[near]
        cross = np.sum((np.sign(s[1:]) != np.sign(s[:-1])) & (rho[1:] < r))
        res[k] = (float(m.min()) if m.size else 9.0, int(cross))
    return res


def main(v_max=0.7, v_hoop=0.5, a=0.9, lead=0.5, out='plan_fast'):
    H = hoops()
    p, hoop = curve(waypoints(H))
    t, _ = timing(p, hoop, v_max, v_hoop, a)
    T, P, V, A = resample(t, p)
    sim = lag_sim(T, P, V, lead)
    ok = True
    print('duration %.1f s, peak speed %.2f m/s, peak accel %.2f m/s^2' % (
        T[-1], np.linalg.norm(V, axis=1).max(), np.linalg.norm(A, axis=1).max()))
    for name, X in (('plan', P), ('drone model (lead %.1f)' % lead, sim)):
        cl = clearance(X, H)
        print('%-22s' % name, '  '.join('%s margin %+.3f m, %d crossings' % (k, *cl[k]) for k in sorted(cl)),
              ' max dev %.3f m' % np.linalg.norm(X - P, axis=1).max())
        inbox = np.all((X >= BOX[0] - 1e-9) & (X <= BOX[1] + 1e-9))
        print('%-22s in box %s: %s (x %.2f..%.2f y %.2f..%.2f z %.2f..%.2f)' % (
            '', BOX.tolist(), inbox, *np.ravel(np.c_[X.min(0), X.max(0)])))
        ok &= inbox and all(m > 0.03 for m, _ in cl.values())
        ok &= [cl[k][1] for k in ('hoop-1', 'hoop-2', 'hoop-3')] == [2, 2, 3]
    d = HERE / out
    d.mkdir(exist_ok=True)
    np.savetxt(d / 'trajectory.csv', np.c_[T, P, V, A], delimiter=',', comments='',
               header='time_s,x_m,y_m,z_m,vx_m_s,vy_m_s,vz_m_s,ax_m_s2,ay_m_s2,az_m_s2')
    print('wrote', d / 'trajectory.csv', 'CHECKS OK' if ok else 'CHECKS FAILED')
    return ok


if __name__ == '__main__':
    kw = {k: float(v) for k, v in (x.split('=') for x in sys.argv[1:])}
    sys.exit(0 if main(**kw) else 1)
