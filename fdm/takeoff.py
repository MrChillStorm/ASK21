"""Aerotow and winch take-off with the stick released (elevator cmd 0, trim neutral).

Aerotow: nose hook, 40 m textile rope (k 3000 N/m, c 600 N s/m) to a tug that accelerates at full power
(a = a0 (1 - V/Vmax) - F_rope/m_tug) and rotates at V_tug_lo.  Wings level / heading held with aileron and rudder.
Winch (Gaeb & Santel 2010 reference): cable force 2500 N, raised to 7500 N within 5 s after lift-off,
surfaces fixed at trim until the safety altitude; 1000 m cable to the winch.
"""
import numpy as np, harness as H, sys

N2LB = 0.224809


def setup(aero, jsb=None, front=176.0, rear=176.0, tail=0.0, trim=0.0, tag=None):
    f = H.mk(aero=aero, jsb=jsb, front_lb=front, rear_lb=rear, tail_lb=tail, tag=tag)
    f['ic/terrain-elevation-ft'] = 0; f['ic/h-agl-ft'] = 2.25; f['ic/vt-fps'] = 0.0; f['ic/psi-true-deg'] = 0.0
    f['ic/theta-deg'] = 1.8; f['ic/phi-deg'] = 0
    f.run_ic()
    f['fcs/pitch-trim-cmd-norm'] = trim
    for _ in range(240): f.run()          # settle on the wheels
    return f


def rot(f):
    ph, th, ps = np.radians([f['attitude/phi-deg'], f['attitude/theta-deg'], f['attitude/psi-deg']])
    cph, sph, cth, sth, cps, sps = np.cos(ph), np.sin(ph), np.cos(th), np.sin(th), np.cos(ps), np.sin(ps)
    # body -> local NED
    return np.array([[cth * cps, sph * sth * cps - cph * sps, cph * sth * cps + sph * sps],
                     [cth * sps, sph * sth * sps + cph * cps, cph * sth * sps - sph * cps],
                     [-sth, sph * cth, cph * cth]])


def hook_pos(f, hook, xg):
    """hook position in a runway frame (x north, z up) ; hook = body offset from CG (m, fwd/right/down)."""
    R = rot(f)
    d = R @ hook
    return np.array([xg + d[0], d[1], f['position/h-agl-ft'] * 0.3048 - d[2]])


def body_hook(f, name):
    x = f['external_reactions/%s/location-x-in' % name] if False else None
    return x


def aerotow(aero, jsb=None, front=176.0, rear=176.0, tail=0.0, trim=0.0, stick=0.0, rope=40.0, a0=2.6, vmax=40.0,
            m_tug=950.0, v_tug_lo=85.0, tug_hook_h=0.75, t_end=40.0, verbose=False, tag=None):
    f = setup(aero, jsb, front, rear, tail, trim, tag)
    dt = f['simulation/dt']
    # nose hook (structural x -3.42633, z 0.2176) relative to the CG, body axes (fwd, right, down)
    cgx, cgz = f['inertia/cg-x-in'] * 0.0254, f['inertia/cg-z-in'] * 0.0254
    hook = np.array([-(-3.42633 - cgx), 0.0, -(0.2176 - cgz)])
    xg = 0.0; vt = 0.0; t = 0.0
    xt = hook_pos(f, hook, xg)[0] + rope          # tug hook straight ahead, rope just taut
    k_r, c_r = 3000.0, 600.0
    last_s = 0.0
    ev = dict(nose_off=None, main_off=None, airborne=None, tug_lo=None)
    off_t = None; L = []
    while t < t_end:
        hp = hook_pos(f, hook, xg)
        tug = np.array([xt, 0.0, tug_hook_h if ev['tug_lo'] is None else tug_hook_h + 1.5 * (t - ev['tug_lo'])])
        dvec = tug - hp; dist = np.linalg.norm(dvec); s = dist - rope; sd = (s - last_s) / dt; last_s = s
        F = max(0.0, k_r * s + c_r * sd) if s > 0 else 0.0
        F = min(F, 6000.0)
        # tug dynamics
        vt += (a0 * (1 - vt / vmax) - F / m_tug) * dt; vt = max(vt, 0.0); xt += vt * dt
        if ev['tug_lo'] is None and vt * 3.6 >= v_tug_lo: ev['tug_lo'] = t
        # force direction in body axes (NED -> body): u = R^T * unit(NED)
        u_ned = np.array([dvec[0], dvec[1], -dvec[2]]) / max(dist, 1e-6)
        u_b = rot(f).T @ u_ned
        f['external_reactions/aerotow_hitch/magnitude'] = F * N2LB
        f['external_reactions/aerotow_hitch/x'] = float(u_b[0]); f['external_reactions/aerotow_hitch/y'] = float(u_b[1]); f['external_reactions/aerotow_hitch/z'] = float(u_b[2])
        # pilot: elevator released (stick), wings level with aileron, heading with rudder
        f['fcs/elevator-cmd-norm'] = stick
        f['fcs/aileron-cmd-norm'] = float(np.clip(-0.05 * f['attitude/phi-deg'] - 0.03 * f['velocities/p-rad_sec'] * 57.3, -1, 1))
        dpsi = (f['attitude/psi-deg'] + 180) % 360 - 180
        f['fcs/rudder-cmd-norm'] = float(np.clip(0.04 * dpsi + 0.05 * f['velocities/r-rad_sec'] * 57.3, -1, 1))
        f.run(); t += dt
        xg += f['velocities/v-north-fps'] * 0.3048 * dt
        V = f['velocities/vc-kts'] * 1.852
        wow = [f['gear/unit[%d]/WOW' % i] for i in range(3)]
        if ev['nose_off'] is None and wow[0] < 0.5 and V > 5: ev['nose_off'] = (t, V)
        if wow[1] < 0.5 and V > 20:
            off_t = off_t if off_t is not None else (t, V)
            if ev['main_off'] is None: ev['main_off'] = (t, V)
            if ev['airborne'] is None and t - off_t[0] > 1.0 and f['position/h-agl-ft'] > 3.0: ev['airborne'] = off_t
        else:
            off_t = None
        if int(round(t / dt)) % 12 == 0:
            L.append((t, V, vt * 3.6, f['attitude/theta-deg'], f['aero/alpha-deg'], f['position/h-agl-ft'] * 0.3048 - 0.69, F, wow[0], wow[1], wow[2], 0.0))
        if ev['airborne'] and t > ev['airborne'][0] + 4: break
    return ev, np.array(L)


def winch(aero, jsb=None, front=176.0, rear=176.0, tail=0.0, trim=0.0, stick=0.0, F0=2500.0, F1=7500.0, cable=1000.0,
          t_end=12.0, tag=None):
    f = setup(aero, jsb, front, rear, tail, trim, tag)
    dt = f['simulation/dt']
    cgx, cgz = f['inertia/cg-x-in'] * 0.0254, f['inertia/cg-z-in'] * 0.0254
    hook = np.array([-(-1.94837 - cgx), 0.0, -(0.2361 - cgz)])
    xg = 0.0; t = 0.0; ev = dict(nose_off=None, main_off=None, airborne=None); off_t = None; L = []
    win = np.array([cable, 0.0, 0.5])
    while t < t_end:
        hp = hook_pos(f, hook, xg); dvec = win - hp; dist = np.linalg.norm(dvec)
        F = F0 if ev['airborne'] is None else F0 + (F1 - F0) * min(1.0, (t - ev['airborne'][0]) / 5.0)
        F = min(F, F0 + (F1 - F0) * min(1.0, t / 2.0)) if False else F
        F = F * min(1.0, t / 1.0)
        u_ned = np.array([dvec[0], dvec[1], -dvec[2]]) / dist
        u_b = rot(f).T @ u_ned
        f['external_reactions/winch_hitch/magnitude'] = F * N2LB
        f['external_reactions/winch_hitch/x'] = float(u_b[0]); f['external_reactions/winch_hitch/y'] = float(u_b[1]); f['external_reactions/winch_hitch/z'] = float(u_b[2])
        f['fcs/elevator-cmd-norm'] = stick
        f['fcs/aileron-cmd-norm'] = float(np.clip(-0.05 * f['attitude/phi-deg'] - 0.03 * f['velocities/p-rad_sec'] * 57.3, -1, 1))
        dpsi = (f['attitude/psi-deg'] + 180) % 360 - 180
        f['fcs/rudder-cmd-norm'] = float(np.clip(0.04 * dpsi + 0.05 * f['velocities/r-rad_sec'] * 57.3, -1, 1))
        f.run(); t += dt
        xg += f['velocities/v-north-fps'] * 0.3048 * dt
        V = f['velocities/vc-kts'] * 1.852
        wow = [f['gear/unit[%d]/WOW' % i] for i in range(3)]
        if ev['nose_off'] is None and wow[0] < 0.5 and V > 5: ev['nose_off'] = (t, V)
        if wow[1] < 0.5 and V > 20:
            off_t = off_t if off_t is not None else (t, V)
            if ev['main_off'] is None: ev['main_off'] = (t, V)
            if ev['airborne'] is None and t - off_t[0] > 1.0 and f['position/h-agl-ft'] > 3.0: ev['airborne'] = off_t
        else:
            off_t = None
        if int(round(t / dt)) % 12 == 0:
            L.append((t, V, f['attitude/theta-deg'], f['aero/alpha-deg'], f['position/h-agl-ft'] * 0.3048 - 0.69, F, wow[0], wow[1], wow[2], f['velocities/q-rad_sec'] * 57.3))
    return ev, np.array(L)


if __name__ == '__main__':
    aero = sys.argv[1]
    for fr, re in ((176, 176), (176, 0), (230, 230)):
        for trim in (0.0,):
            ev, L = aerotow(aero, front=fr, rear=re, trim=trim)
            fmt = lambda e: '-' if e is None else '%.1f s / %.0f km/h' % e
            tug = ev['tug_lo']; vt_lo = None
            print('aerotow front %d rear %d trim %+.1f: nose wheel off %s | main wheel off %s | airborne %s | tug lift-off at t=%s' % (
                fr, re, trim, fmt(ev['nose_off']), fmt(ev['main_off']), fmt(ev['airborne']), '-' if tug is None else '%.1f s' % tug))
