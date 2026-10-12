"""Flight-test campaign: the same programme flown with each model, results stored as JSON for the report.

usage: campaign.py <model-key> [workers]      model keys: see MODELS
"""
import sys, os, json, time, numpy as np, csv
from concurrent.futures import ProcessPoolExecutor
from scipy.optimize import brentq, fsolve
import sim, harness as H, takeoff as T
from static import Static

HERE = os.path.dirname(os.path.abspath(__file__))
MODELS = {
    'fg': dict(name='FlightGear ASK 21 (FGAddon rev. 8166)', aero='fg_aero.xml', test='fg_test.xml', ship='fg_jsb.xml'),
    'v9': dict(name='ASK 21 v9 component model', aero='v9_aero.xml', test='jsb_test.xml', ship='v9_jsb.xml'),
}
KT = 0.514444
REC_LOADS = [21, 11, 19, 16]
REC_CASES = ('continue', 'fm', 'rudder', 'neutral', 'forward', 'ail_against')
STALL_LOADS = (0, 9, 17, 22)


def P(x): return os.path.join(HERE, x)


def noadot(aero):
    """copy of an aero file whose alpha-dot input is a free property (test/adot): static coefficients without the
    spurious alpha-dot that unbalanced forces would otherwise produce in a frozen-integrator evaluation."""
    out = aero.replace('.xml', '_noadot.xml')
    if not os.path.exists(out):
        s = open(aero).read().replace('<property>aero/alphadot-rad_sec</property>', '<property>test/adot</property>')
        open(out + '.tmp', 'w').write(s); os.replace(out + '.tmp', out)
    return out


def mkstatic(aero, jsb, front, rear, tail=0.0, ckpt=0.0, h_ft=3000.0):
    return Static(aero, jsb=jsb, front=front, rear=rear, tail=tail, ckpt=ckpt, h_ft=h_ft)


def arr_dec(F, k=4):
    L = F.arr()
    return L[::k].round(3).tolist()


# ----------------------------------------------------------------------------------------------- jobs
def job(args):
    kind, M = args[0], args[1]
    try:
        return (args, JOBS[kind](M, *args[2:]))
    except Exception as e:
        import traceback
        return (args, dict(error=repr(e), tb=traceback.format_exc()[-600:]))


def j_massprops(M):
    out = []
    for i, row in enumerate(sim.A1):
        fr, re, tl, ck = sim.a1_load(row)
        f = H.mk(aero=P(M['aero']), jsb=P(M['test']), front_lb=fr, rear_lb=re, tail_lb=tl, ckpt_lb=ck)
        f['ic/vt-fps'] = 80; f.run_ic(); f.run()
        k = 32.174 * 144 / 1e6     # slug ft2 -> 1e6 lb in2
        out.append(dict(idx=i, cg_afftc=row[0], W_afftc=row[5], Iyy_afftc=row[6], cg=H.cg_report_in(f), W=f['inertia/weight-lbs'],
                        Ixx=f['inertia/ixx-slugs_ft2'] * k, Iyy=f['inertia/iyy-slugs_ft2'] * k, Izz=f['inertia/izz-slugs_ft2'] * k,
                        Ixz=f['inertia/ixz-slugs_ft2'] * k))
    return out


def j_polar(M, mass):
    loads = {470: (197, 45), 570: (117, 346)}
    fr, re = loads[mass]
    st = Static(P(M['aero']), jsb=P(M['ship']), front=fr, rear=re, h_ft=0.0)
    res = []; g = (6, -0.3, -2)
    for v in list(range(62, 100, 2)) + list(range(100, 285, 5)):
        t = st.trim_glide(v / 3.6, guess=g)
        if t['ok']:
            g = (t['alpha'], t['elev'], t['gamma'])
            res.append((v, t['sink'], t['alpha'], t['elev']))
    # scale to the exact FM mass (weight differs by the empty-mass assumption): V ~ sqrt(W), w ~ sqrt(W)
    k = np.sqrt(mass * 2.20462 / st.W)
    return dict(mass=mass, W=st.W, cg=st.cg_in(), pts=[(v * k, s * k, a, e) for v, s, a, e in res])


def j_curves(M):
    aero = noadot(P(M['aero']))
    st = mkstatic(aero, P(M['ship']), 176, 176)
    V = 25.0; out = dict(cg=st.cg_in(), W=st.W, alpha=[], CL=[], CD=[], Cm=[], Cm_up=[], Cm_dn=[], beta=[], CY=[], Cl=[], Cn=[])
    for a in list(np.arange(-30, 30, 1.0)) + list(np.arange(30, 91, 2.5)):
        c = st.coeffs(V, float(a)); cu = st.coeffs(V, float(a), elev=-1.0); cd = st.coeffs(V, float(a), elev=1.0)
        out['alpha'].append(float(a)); out['CL'].append(c['CL']); out['CD'].append(c['CD']); out['Cm'].append(c['Cm'])
        out['Cm_up'].append(cu['Cm']); out['Cm_dn'].append(cd['Cm'])
    for b in np.arange(0, 61, 2.0):
        c = st.coeffs(V, 4.0, beta=float(b))
        out['beta'].append(float(b)); out['CY'].append(c['CY']); out['Cl'].append(c['Cl']); out['Cn'].append(c['Cn'])
    return out


def trimmed_clmax(st, V=25.0, sb=0.0):
    best = (0.0, None, None)
    for a in np.arange(2, 30.1, 0.5):
        try: el = brentq(lambda e: st.coeffs(V, float(a), elev=e, sb=sb)['Cm'], -1, 1)
        except ValueError: continue
        c = st.coeffs(V, float(a), elev=el, sb=sb)['CL']
        if c > best[0]: best = (c, float(a), el)
    return best


def j_clmax(M, idx):
    row = sim.A1[idx]; fr, re, tl, ck = sim.a1_load(row)
    st = mkstatic(noadot(P(M['aero'])), P(M['test']), fr, re, tl, ck)
    c, a, el = trimmed_clmax(st)
    return dict(idx=idx, cg=st.cg_in(), W=st.W, CLmax=c, alpha=a, elev=el, elev_limited=(el is None or el <= -0.98))


def j_stallspeeds(M):
    out = {}
    for lab, (fr, re), mass in (('single', (176, 0), 470), ('dual', (176, 176), 600)):
        st = mkstatic(noadot(P(M['aero'])), P(M['ship']), fr, re)
        for sb in (0.0, 1.0):
            c, a, el = trimmed_clmax(st, sb=sb)
            out['%s_sb%d' % (lab, sb)] = dict(cg=st.cg_in(), CLmax=c, alpha=a, elev=el,
                                              Vs=float(np.sqrt(2 * mass * 9.80665 / (1.225 * 17.95 * c)) * 3.6) if c > 0 else None)
    return out


def j_stall1g(M, idx):
    o = sim.stall_ramp(P(M['aero']), sim.a1_load(sim.A1[idx]), jsb=P(M['test']))
    return o


def j_stall1g_th(M, idx):
    """same as stall_ramp but return the time history"""
    fr, re, tl, ck = sim.a1_load(sim.A1[idx])
    F = sim.Flight(P(M['aero']), jsb=P(M['test']), front=fr, rear=re, tail=tl, ckpt=ck, V_kmh=90.0); f = F.f
    F.run_controls(1.0); sh = sim.SpeedHold(F, 90.0); t0 = F.t; st = dict(t_full=None, amax=-1)
    def cb(F):
        tt = F.t - t0
        if st['t_full'] is None:
            sh.V = 90.0 - 1.852 * tt; el = sh()
            if el <= -0.999 or (st['amax'] > 10 and f['aero/alpha-deg'] < st['amax'] - 1.5): st['t_full'] = tt
            st['amax'] = max(st['amax'], f['aero/alpha-deg'])
        else:
            f['fcs/elevator-cmd-norm'] = -1.0
        f['fcs/aileron-cmd-norm'] = float(np.clip(-0.04 * f['attitude/phi-deg'] - 0.02 * f['velocities/p-rad_sec'] * 57.3, -1, 1))
        f['fcs/rudder-cmd-norm'] = float(np.clip(-0.04 * f['aero/beta-deg'], -1, 1))
        return st['t_full'] is not None and tt > st['t_full'] + 12
    F.run_controls(90.0, cb=cb)
    return dict(cg=F.cg, t_full=st['t_full'], th=arr_dec(F, 2))


def j_pullup(M, idx, theta_cmd=30.0):
    fr, re, tl, ck = sim.a1_load(sim.A1[idx])
    F = sim.Flight(P(M['aero']), jsb=P(M['test']), front=fr, rear=re, tail=tl, ckpt=ck, V_kmh=100.0); f = F.f
    F.run_controls(1.0); h0 = f['position/h-sl-ft']; t_start = F.t
    F.hold_pitch_to_stall(theta_cmd, kth=0.08, kq=0.04, ramp_rate=12.0)
    th_st = f['attitude/theta-deg']; kias_st = f['velocities/vc-kts']; t0 = F.t
    mn = dict(th=99, phi=0, a=0, kias=99)
    def cb(F):
        mn['th'] = min(mn['th'], f['attitude/theta-deg']); mn['phi'] = max(mn['phi'], abs(f['attitude/phi-deg']))
        mn['a'] = max(mn['a'], f['aero/alpha-deg']); mn['kias'] = min(mn['kias'], f['velocities/vc-kts'])
        f['fcs/aileron-cmd-norm'] = float(np.clip(-0.04 * f['attitude/phi-deg'], -1, 1)); f['fcs/rudder-cmd-norm'] = 0.0
        return F.t - t0 > 4
    F.run_controls(4.5, el=-1.0, cb=cb, ramp=0.2)
    # recovery: release, wings level, pull out to level flight
    hmin = [f['position/h-sl-ft']]
    def rec(F):
        hmin[0] = min(hmin[0], f['position/h-sl-ft']); th = f['attitude/theta-deg']
        if f['velocities/vc-kts'] > 45:
            f['fcs/elevator-cmd-norm'] = float(np.clip(F.trim['elev'] - 0.05 * (0.0 - th) + 0.03 * f['velocities/q-rad_sec'] * 57.3, -1, 1))
        else:
            f['fcs/elevator-cmd-norm'] = float(np.clip(F.trim['elev'] + 0.03 * f['velocities/q-rad_sec'] * 57.3, -1, 1))
        f['fcs/aileron-cmd-norm'] = float(np.clip(-0.04 * f['attitude/phi-deg'], -1, 1)); f['fcs/rudder-cmd-norm'] = 0.0
        return F.t - t0 > 8 and f['velocities/v-down-fps'] < 3
    F.run_controls(14.0, cb=rec)
    L = F.arr(); pre = L[(L[:, 0] >= t_start) & (L[:, 0] <= t0)]
    return dict(cg=F.cg, kias_stall=kias_st, kias_min=min(mn['kias'], float(pre[:, 1].min())), g_break=th_st - mn['th'],
                phi_max=mn['phi'], alpha_max=mn['a'], h_loss=float(h0 - hmin[0]) if False else float(L[L[:, 0] >= t0][:, 7].max() - L[:, 7].min()),
                th=arr_dec(F, 2))


def j_spin(M, idx, ail, thermal):
    row = sim.A1[idx]
    F = sim.spin_entry(P(M['aero']), sim.a1_load(row), jsb=P(M['test']), rud=-1.0, ail=ail, hold=40.0, thermal=thermal)
    c = sim.classify(F, F.stall_t)
    m = sim.spin_metrics(F, F.stall_t + 12) if c['cls'] == 'sustained' else None
    if m: m = {k: float(v) for k, v in m.items()}
    return dict(idx=idx, cg=F.cg, I=row[6], ail=ail, thermal=thermal, cls=c['cls'], turns=c['turns'], m=m, stall_t=F.stall_t,
                ok=bool(np.isfinite(F.arr()).all()), th=arr_dec(F, 4))


def j_rec(M, idx, case):
    row = sim.A1[idx]
    r = sim.recovery(P(M['aero']), sim.a1_load(row), case, jsb=P(M['test']))
    return dict(idx=idx, cg=r['cg'], I=row[6], case=case, spinning=bool(r['spinning']), stopped=r['stopped'])


def j_roll45(M, coord):
    F = sim.Flight(P(M['aero']), jsb=P(M['ship']), front=176, rear=176, V_kmh=90.0); f = F.f; sh = sim.SpeedHold(F, 90.0)
    def to_bank(F):
        sh(); e = f['attitude/phi-deg'] + 45
        f['fcs/aileron-cmd-norm'] = float(np.clip(-0.05 * e - 0.03 * f['velocities/p-rad_sec'] * 57.3, -1, 1))
        f['fcs/rudder-cmd-norm'] = float(np.clip(-0.04 * f['aero/beta-deg'], -1, 1)); return False
    F.run_controls(12.0, cb=to_bank)
    t0 = F.t; res = {}; bm = [0.0]
    def rev(F):
        sh(); f['fcs/aileron-cmd-norm'] = 1.0
        f['fcs/rudder-cmd-norm'] = float(np.clip(-0.08 * f['aero/beta-deg'] - 0.3, -1, 1)) if coord else 0.0
        ph = f['attitude/phi-deg']; bm[0] = max(bm[0], abs(f['aero/beta-deg']))
        if ph >= 45: res['t'] = F.t - t0; return True
        return False
    F.run_controls(15.0, cb=rev)
    return dict(coord=coord, t=res.get('t'), beta_max=bm[0], th=[r for r in arr_dec(F, 2) if r[0] >= t0 - 1])


def j_roll360(M, fr, re, V, coord):
    F = sim.Flight(P(M['aero']), jsb=P(M['ship']), front=fr, rear=re, V_kmh=V); f = F.f; F.run_controls(1.0)
    t0 = F.t; marks = {}; st = dict(ang=0.0, last=f['attitude/phi-deg'])
    def cb(F):
        f['fcs/aileron-cmd-norm'] = 1.0
        f['fcs/rudder-cmd-norm'] = float(np.clip(-0.08 * f['aero/beta-deg'] - 0.2, -1, 1)) if coord else 0.0
        ph = f['attitude/phi-deg']; d = (ph - st['last'] + 180) % 360 - 180; st['ang'] += d; st['last'] = ph
        for m in (90, 180, 270, 360):
            if m not in marks and st['ang'] >= m: marks[m] = F.t - t0
        return 360 in marks or F.t - t0 > 14
    F.run_controls(14.0, cb=cb)
    return dict(fr=fr, re=re, V=V, coord=coord, marks={str(k): v for k, v in marks.items()}, v_end=f['velocities/vc-kts'] * 1.852)


def j_dive(M, sb):
    # 45-deg dive = flight path 45 deg below the horizon (JAR 22 airbrake requirement): pitch = alpha - 45
    st = Static(P(M['aero']), jsb=P(M['ship']), front=264.5, rear=264.5)       # 1323 lb = 600 kg
    W = st.W
    def res(x):
        V, a, el = x
        th = np.radians(a - 45.0)
        r = st.eval(V, a, elev=el, theta=a - 45.0, sb=sb)
        return [(r['X'] - W * np.sin(th)) / W, (r['Z'] + W * np.cos(th)) / W, r['M'] / (W * 3)]
    best = None
    for V0 in ((64.0, 70.0, 80.0) if sb else (150.0, 200.0, 250.0)):
        sol, info, ier, msg = fsolve(res, (V0, -3.0, 0.2), full_output=True)
        e = float(np.abs(info['fvec']).max())
        if best is None or e < best[1]: best = (sol, e)
    sol, e = best
    st.eval(sol[0], sol[1], elev=sol[2], theta=sol[1] - 45.0, sb=sb)
    vc = st.f['velocities/vc-kts'] * 1.852
    # scale to 600 kg (EAS ~ sqrt(W))
    return dict(sb=sb, W=W, TAS=float(sol[0] * 3.6), CAS=vc, CAS600=vc * np.sqrt(600 * 2.20462 / W), alpha=float(sol[1]), elev=float(sol[2]), resid=e)


def j_slip(M, mode):
    F = sim.Flight(P(M['aero']), jsb=P(M['ship']), front=176, rear=176, V_kmh=90.0); f = F.f; F.run_controls(1.0)
    sh = sim.SpeedHold(F, 90.0); psi0 = f['attitude/psi-deg']; t0 = F.t; L = []
    def ctl(F):
        tt = F.t - t0; sh()
        if mode == 'brakes':
            f['/controls/engines/engine/throttle'] = 0.0 if tt > 2 else 1.0
            f['fcs/rudder-cmd-norm'] = float(np.clip(-0.03 * f['aero/beta-deg'], -1, 1))
            f['fcs/aileron-cmd-norm'] = float(np.clip(-0.05 * f['attitude/phi-deg'] - 0.02 * f['velocities/p-rad_sec'] * 57.3, -1, 1))
        else:
            rud = 1.0 if 2 < tt < 30 else 0.0; f['fcs/rudder-cmd-norm'] = rud
            dpsi = (f['attitude/psi-deg'] - psi0 + 180) % 360 - 180
            phic = float(np.clip(-2.0 * dpsi, -30, 30)) if tt < 30 else 0.0
            f['fcs/aileron-cmd-norm'] = float(np.clip(-0.06 * (f['attitude/phi-deg'] - phic) - 0.03 * f['velocities/p-rad_sec'] * 57.3, -1, 1))
        if int(round(F.t / F.dt)) % 24 == 0:
            L.append((tt, f['velocities/vc-kts'] * 1.852, f['aero/beta-deg'], f['attitude/phi-deg'], (f['attitude/psi-deg'] - psi0 + 180) % 360 - 180,
                      -f['velocities/v-down-fps'] * 0.3048, f['fcs/aileron-cmd-norm'], f['fcs/rudder-cmd-norm'], f['fcs/elevator-cmd-norm'],
                      f['velocities/vt-fps'] * 0.3048 * 3.6))
        return False
    F.run_controls(36.0, cb=ctl)
    return dict(mode=mode, th=np.array(L).round(3).tolist())


def j_inverted(M):
    st = mkstatic(noadot(P(M['aero'])), P(M['test']), 176, 0)
    best = (0, None, None)
    for a in np.arange(-3, -30, -0.5):
        try: el = brentq(lambda e: st.coeffs(25.0, float(a), elev=e)['Cm'], -1, 1)
        except ValueError: continue
        c = st.coeffs(25.0, float(a), elev=el)['CL']
        if c < best[0]: best = (c, float(a), el)
    W = st.W * 4.448
    return dict(cg=st.cg_in(), W=st.W, CLmin=best[0], alpha=best[1], elev=best[2],
                Vs_kias=float(np.sqrt(2 * W / (1.225 * 17.95 * abs(best[0]))) / KT) if best[0] < 0 else None)


def j_aerotow(M, fr, re, stick):
    ev, L = T.aerotow(P(M['aero']), jsb=P(M['ship']), front=fr, rear=re, stick=stick)
    return dict(fr=fr, re=re, stick=stick, ev={k: v for k, v in ev.items()}, th=np.array(L)[:, :10].round(3).tolist())


def j_winch(M, fr, re):
    ev, L = T.winch(P(M['aero']), jsb=P(M['ship']), front=fr, rear=re)
    return dict(fr=fr, re=re, ev=ev, th=np.array(L).round(3).tolist())


def j_trimspeed(M, fr, re):
    st = Static(P(M['aero']), jsb=P(M['ship']), front=fr, rear=re)
    try:
        a = brentq(lambda a: st.coeffs(25.0, a, elev=0.0)['Cm'], -4, 16)
        cl = st.coeffs(25.0, a, elev=0.0)['CL']; V = float(np.sqrt(2 * st.W * 4.448 / (1.225 * 17.95 * cl)) * 3.6)
    except ValueError:
        a, V = None, None
    return dict(fr=fr, re=re, cg=st.cg_in(), W=st.W, alpha=a, V=V)


def j_modes(M):
    """small-perturbation modes about a trimmed 90 km/h glide (2 x 80 kg): dimensional body-axis derivatives by
    central differences of the model's own forces (alpha-dot handled as a separate input), then eigenvalues."""
    aero = noadot(P(M['aero']))
    st = mkstatic(aero, P(M['ship']), 176, 176)
    V0 = 25.0; tr = st.trim_glide(V0); f = st.f
    a0 = np.radians(tr['alpha']); th0 = np.radians(tr['alpha'] + tr['gamma']); el0 = tr['elev']
    m = st.W / 32.174; g = 32.174
    Ixx, Iyy, Izz, Ixz = (f['inertia/%s-slugs_ft2' % k] for k in ('ixx', 'iyy', 'izz', 'ixz'))
    u0, w0 = V0 / 0.3048 * np.cos(a0), V0 / 0.3048 * np.sin(a0)
    def F(u=0.0, v=0.0, w=0.0, p=0.0, q=0.0, r=0.0, adot=0.0, el=0.0):
        U, Vv, Wv = u0 + u, v, w0 + w
        Vt = np.sqrt(U * U + Vv * Vv + Wv * Wv); al = np.degrees(np.arctan2(Wv, U)); be = np.degrees(np.arcsin(Vv / Vt))
        f['test/adot'] = adot
        r_ = st.eval(Vt * 0.3048, al, beta=be, p=p, q=q, r=r, elev=el0 + el, theta=np.degrees(th0), phi=0.0)
        f['test/adot'] = 0.0
        return np.array([r_['X'], r_['Y'], r_['Z'], r_['L'], r_['M'], r_['N']])
    du, dr, dad = 1.0, 0.02, 0.02
    D = {}
    for nm, kw, h in (('u', 'u', du), ('v', 'v', du), ('w', 'w', du), ('p', 'p', dr), ('q', 'q', dr), ('r', 'r', dr), ('ad', 'adot', dad)):
        D[nm] = (F(**{kw: h}) - F(**{kw: -h})) / (2 * h)
    X, Y, Z, L, Mm, N = range(6)
    # alpha-dot -> w-dot: adot = wdot / u0
    Zwd, Mwd = D['ad'][Z] / u0, D['ad'][Mm] / u0
    # longitudinal: x = [u, w, q, theta]
    E = np.diag([m, m - Zwd, Iyy, 1.0]); E[2, 1] = -Mwd
    A = np.array([[D['u'][X], D['w'][X], D['q'][X] - m * w0, -m * g * np.cos(th0)],
                  [D['u'][Z], D['w'][Z], D['q'][Z] + m * u0, -m * g * np.sin(th0)],
                  [D['u'][Mm], D['w'][Mm], D['q'][Mm], 0.0],
                  [0.0, 0.0, 1.0, 0.0]])
    Alon = np.linalg.solve(E, A)
    # lateral: x = [v, p, r, phi]
    E2 = np.array([[m, 0, 0, 0], [0, Ixx, -Ixz, 0], [0, -Ixz, Izz, 0], [0, 0, 0, 1.0]])
    A2 = np.array([[D['v'][Y], D['p'][Y] + m * w0, D['r'][Y] - m * u0, m * g * np.cos(th0)],
                   [D['v'][L], D['p'][L], D['r'][L], 0.0],
                   [D['v'][N], D['p'][N], D['r'][N], 0.0],
                   [0.0, 1.0, np.tan(th0), 0.0]])
    Alat = np.linalg.solve(E2, A2)
    def modes(Am):
        ev = np.linalg.eigvals(Am); out = []
        for e in ev:
            if abs(e.imag) > 1e-6:
                if e.imag < 0: continue
                wn = abs(e); z = -e.real / wn
                out.append(dict(re=float(e.real), im=float(e.imag), wn=float(wn), zeta=float(z), period=float(2 * np.pi / e.imag)))
            else:
                out.append(dict(re=float(e.real), im=0.0, tau=float(-1 / e.real) if e.real != 0 else None,
                                t_half_or_double=float(np.log(2) / abs(e.real)) if e.real != 0 else None))
        return out
    # nondimensional derivatives for the table (stability axes approx. at small alpha)
    qS = st.eval(V0, np.degrees(a0), elev=el0, theta=np.degrees(th0))['qbar'] * 193.21
    b = 55.774; c = 1.121 / 0.3048; Vf = V0 / 0.3048
    ca, sa = np.cos(a0), np.sin(a0)
    def lift(Fv): return -Fv[Z] * ca + Fv[X] * sa
    nd = dict(CLa=lift(D['w']) * Vf / qS, Cma=D['w'][Mm] * Vf / (qS * c), Cmq=D['q'][Mm] * 2 * Vf / (qS * c * c),
              Cmad=D['ad'][Mm] * 2 * Vf / (qS * c * c), CLq=lift(D['q']) * 2 * Vf / (qS * c),
              CYb=D['v'][Y] * Vf / qS, Clb=D['v'][L] * Vf / (qS * b), Cnb=D['v'][N] * Vf / (qS * b),
              Clp=D['p'][L] * 2 * Vf / (qS * b * b), Cnp=D['p'][N] * 2 * Vf / (qS * b * b),
              Clr=D['r'][L] * 2 * Vf / (qS * b * b), Cnr=D['r'][N] * 2 * Vf / (qS * b * b))
    return dict(trim=dict(alpha=float(np.degrees(a0)), gamma=tr['gamma'], elev=el0, cg=st.cg_in(), W=st.W),
                inertia=dict(Ixx=Ixx, Iyy=Iyy, Izz=Izz, Ixz=Ixz), nd={k: float(v) for k, v in nd.items()},
                lon=modes(Alon), lat=modes(Alat))


def j_robust(M):
    import robust
    bad, worst = robust.fuzz(P(M['aero']), jsb=P(M['ship']), n=60)
    return dict(bad=bad, worst=worst)


def j_ground(M):
    """resting attitude on the wheels, 2 x 80 kg and solo"""
    out = []
    for fr, re in ((176, 176), (176, 0)):
        f = T.setup(P(M['aero']), P(M['ship']), fr, re)
        for _ in range(1200): f.run()
        out.append(dict(fr=fr, re=re, theta=f['attitude/theta-deg'], wow=[f['gear/unit[%d]/WOW' % i] for i in range(3)]))
    return out


def j_wind(M, kt, frm):
    import robust
    ok, thx, phx, hmax = robust.ground_wind(P(M['aero']), jsb=P(M['ship']), kt=kt, frm=frm)
    return dict(kt=kt, frm=frm, ok=ok, dtheta=thx, bank=phx, hmax=hmax)


def j_phugoid(M, V):
    """stick-fixed phugoid from a +1.5 m/s speed disturbance, 2 x 80 kg: period and amplitude ratio per cycle"""
    from scipy.signal import find_peaks
    F = sim.Flight(P(M['aero']), jsb=P(M['ship']), front=176, rear=176, V_kmh=V, h_ft=3000); f = F.f; el = F.trim['elev']
    f['ic/vt-fps'] = (V / 3.6 + 1.5) / 0.3048; f.run_ic(); t = []; v = []
    for i in range(int(150 / F.dt)):
        f['fcs/elevator-cmd-norm'] = el; f['fcs/aileron-cmd-norm'] = 0; f['fcs/rudder-cmd-norm'] = 0; f.run()
        if i % 12 == 0: t.append(i * F.dt); v.append(f['velocities/vt-fps'] * 0.3048)
    t = np.array(t); v = np.array(v); pk, _ = find_peaks(v, distance=40); vp = v[pk] - np.mean(v[len(v) // 3:])
    if len(pk) < 3 or vp[0] <= 0 or vp[-1] <= 0: return dict(V=V, period=None, ratio=None, zeta=None)
    r = float((vp[-1] / vp[0]) ** (1 / (len(vp) - 1)))
    return dict(V=V, period=float(np.diff(t[pk]).mean()), ratio=r, zeta=float(-np.log(r) / (2 * np.pi)))


def j_brakeglide(M, V_tas_kmh):
    """steady glide with full airbrakes at a given true airspeed (2 x 80 kg, 6000 ft): for the slip comparison"""
    st = Static(P(M['aero']), jsb=P(M['ship']), front=176, rear=176, h_ft=6000.0)
    tb = st.trim_glide(V_tas_kmh / 3.6, sb=1.0, guess=(4.0, -0.2, -6.0))
    return dict(V_tas=V_tas_kmh, sink=tb['sink'], ok=bool(tb['ok']))


JOBS = dict(massprops=j_massprops, polar=j_polar, curves=j_curves, clmax=j_clmax, stallspeeds=j_stallspeeds,
            stall1g=j_stall1g, stall1g_th=j_stall1g_th, pullup=j_pullup, spin=j_spin, rec=j_rec, roll45=j_roll45,
            roll360=j_roll360, dive=j_dive, slip=j_slip, inverted=j_inverted, aerotow=j_aerotow, winch=j_winch,
            trimspeed=j_trimspeed, modes=j_modes, robust=j_robust, ground=j_ground, wind=j_wind,
            phugoid=j_phugoid, brakeglide=j_brakeglide)


def jobs_for(key):
    M = MODELS[key]; J = []
    J += [('spin', M, i, a, th) for i in range(len(sim.A1)) for (a, th) in ((0.0, False), (1.0, False), (0.0, True))]
    J += [('rec', M, i, c) for i in REC_LOADS for c in REC_CASES]
    J += [('stall1g', M, i) for i in STALL_LOADS] + [('stall1g_th', M, i) for i in (0, 17)]
    J += [('pullup', M, i) for i in (0, 17)]
    J += [('robust', M)]
    J += [('massprops', M), ('polar', M, 470), ('polar', M, 570), ('curves', M), ('stallspeeds', M), ('inverted', M), ('modes', M)]
    J += [('clmax', M, i) for i in range(len(sim.A1))]
    J += [('roll45', M, True), ('roll45', M, False)]
    J += [('roll360', M, fr, re, V, c) for (fr, re, V) in ((176, 176, 165), (176, 0, 150)) for c in (True, False)]
    J += [('dive', M, 1.0), ('dive', M, 0.0), ('slip', M, 'slip'), ('slip', M, 'brakes')]
    J += [('aerotow', M, fr, re, 0.0) for fr, re in ((176, 176), (176, 0), (230, 230), (230, 0))]
    J += [('winch', M, 176, 176), ('winch', M, 176, 0)]
    J += [('trimspeed', M, fr, re) for fr, re in ((176, 176), (176, 0), (230, 230), (230, 0), (140, 0))]
    J += [('ground', M)] + [('wind', M, kt, frm) for kt in (15, 25, 35) for frm in (0, 45, 90, 135, 180)]
    J += [('phugoid', M, V) for V in (80, 90, 100, 110, 120, 140)]
    J += [('brakeglide', M, V) for V in range(85, 116, 5)]
    return J


if __name__ == '__main__':
    key = sys.argv[1]; workers = int(sys.argv[2]) if len(sys.argv) > 2 else 8
    only = sys.argv[3].split(',') if len(sys.argv) > 3 else None
    J = jobs_for(key)
    noadot(P(MODELS[key]['aero']))
    if only: J = [j for j in J if j[0] in only]
    t = time.time(); out = []
    with ProcessPoolExecutor(workers) as ex:
        for k, (args, r) in enumerate(ex.map(job, J)):
            out.append(dict(kind=args[0], args=[a for a in args[2:]], res=r))
            if isinstance(r, dict) and 'error' in r: print('ERROR', args[0], args[2:], r['error'], flush=True)
            if k % 10 == 0: print('%d / %d  %.0f s' % (k, len(J), time.time() - t), flush=True)
    fn = 'results_%s%s.json' % (key, '_' + '_'.join(only) if only else '')
    json.dump(dict(model=MODELS[key], date=time.strftime('%Y-%m-%d %H:%M'), results=out), open(fn, 'w'))
    print('wrote', fn, '%.0f s' % (time.time() - t))
