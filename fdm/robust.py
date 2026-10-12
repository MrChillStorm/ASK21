"""Robustness: parked in strong wind from all directions; random attitudes/rates/wind in the air (no NaN, no blow-up)."""
import numpy as np, sys, harness as H, takeoff as T


def ground_wind(aero, jsb=None, kt=25.0, frm=180.0, T_s=30.0):
    f = T.setup(aero, jsb, front=176, rear=0)
    f['atmosphere/wind-north-fps'] = -kt * 1.6878 * np.cos(np.radians(frm))
    f['atmosphere/wind-east-fps'] = -kt * 1.6878 * np.sin(np.radians(frm))
    th0 = f['attitude/theta-deg']; thx = 0.0; phx = 0.0; hmax = 0.0; ok = True
    n = int(T_s / f['simulation/dt'])
    for i in range(n):
        f['fcs/elevator-cmd-norm'] = 0.0
        f.run()
        th = f['attitude/theta-deg']; ph = f['attitude/phi-deg']
        if not np.isfinite(th) or not np.isfinite(f['velocities/vt-fps']): ok = False; break
        thx = max(thx, abs(th - th0)); phx = max(phx, abs(ph)); hmax = max(hmax, f['position/h-agl-ft'])
    return ok, thx, phx, hmax


def fuzz(aero, jsb=None, n=60, seed=1):
    rng = np.random.default_rng(seed); bad = 0; worst = 0.0
    for k in range(n):
        f = H.mk(aero=aero, jsb=jsb, front_lb=float(rng.uniform(120, 240)), rear_lb=float(rng.uniform(0, 240)), tail_lb=float(rng.uniform(0, 26)))
        f['ic/h-sl-ft'] = 8000; f['ic/terrain-elevation-ft'] = 0
        f['ic/vt-fps'] = float(rng.uniform(20, 200)); f['ic/alpha-deg'] = float(rng.uniform(-180, 180)); f['ic/beta-deg'] = float(rng.uniform(-80, 80))
        f['ic/theta-deg'] = float(rng.uniform(-89, 89)); f['ic/phi-deg'] = float(rng.uniform(-180, 180)); f['ic/psi-true-deg'] = float(rng.uniform(0, 360))
        for ax in 'pqr': f['ic/%s-rad_sec' % ax] = float(rng.uniform(-3, 3))
        f.run_ic()
        f['atmosphere/wind-north-fps'] = float(rng.uniform(-60, 60)); f['atmosphere/wind-east-fps'] = float(rng.uniform(-60, 60))
        for i in range(int(25 / f['simulation/dt'])):
            if i % 60 == 0:
                f['fcs/elevator-cmd-norm'] = float(rng.uniform(-1, 1)); f['fcs/aileron-cmd-norm'] = float(rng.uniform(-1, 1)); f['fcs/rudder-cmd-norm'] = float(rng.uniform(-1, 1))
                f['/controls/engines/engine/throttle'] = float(rng.choice([0.0, 1.0]))
            f.run()
            w = np.sqrt(f['velocities/p-rad_sec'] ** 2 + f['velocities/q-rad_sec'] ** 2 + f['velocities/r-rad_sec'] ** 2)
            if not np.isfinite(w) or not np.isfinite(f['velocities/vt-fps']):
                bad += 1; break
            worst = max(worst, w)
            if f['position/h-agl-ft'] < 300: break
    return bad, worst * 57.3


if __name__ == '__main__':
    aero = sys.argv[1]
    for kt in (15, 25, 35):
        res = []
        for frm in (0, 45, 90, 135, 180):
            ok, thx, phx, hmax = ground_wind(aero, kt=kt, frm=frm)
            res.append('%3d deg: %s pitch %4.1f bank %4.1f h %.1f' % (frm, 'ok ' if ok else 'NaN', thx, phx, hmax))
        print('parked, wind %d kt from  ' % kt + ' | '.join(res), flush=True)
    bad, worst = fuzz(aero)
    print('fuzz: %d / 60 runs non-finite; max angular rate %.0f deg/s' % (bad, worst))
