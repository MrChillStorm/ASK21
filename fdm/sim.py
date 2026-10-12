"""Dynamic flight tests: trimmed start, AFFTC spin entries, recoveries, classification."""
import numpy as np, harness as H
from static import Static

KT = 0.5144444
INTEG = ('simulation/integrator/rate/rotational', 'simulation/integrator/rate/translational',
         'simulation/integrator/position/rotational', 'simulation/integrator/position/translational')

# AFFTC Table A1 loadings: (cg_in, front, rear, n_tail_weights, n_cockpit_weights, gross, Iyy_e6)
A1 = [(12.36, 176, 230, 0, 0, 1257, 8.021), (12.47, 230, 176, 8, 6, 1288, 8.980), (12.94, 230, 176, 9, 5, 1288, 9.046),
      (13.11, 176, 230, 2, 0, 1262, 8.197), (13.46, 176, 230, 3, 0, 1264, 8.277), (13.49, 176, 217, 3, 1, 1253, 8.280),
      (13.53, 230, 176, 10, 3, 1286, 9.097), (13.87, 230, 176, 11, 3, 1288, 9.176), (13.96, 176, 230, 10, 14, 1310, 9.045),
      (14.01, 176, 0, 0, 11, 1051, 7.986), (14.03, 217, 176, 10, 4, 1275, 9.048), (14.20, 230, 176, 12, 3, 1290, 9.256),
      (14.34, 176, 230, 6, 1, 1273, 8.530), (14.49, 217, 230, 12, 0, 1325, 9.194), (14.99, 139, 176, 0, 0, 1166, 7.791),
      (15.00, 176, 0, 0, 3, 1038, 7.891), (15.50, 176, 0, 0, 2, 1031, 7.843), (15.84, 176, 0, 0, 0, 1027, 7.811),
      (15.92, 122, 176, 0, 0, 1149, 7.702), (16.27, 176, 217, 12, 3, 1277, 9.019), (16.74, 176, 0, 2, 0, 1032, 7.978),
      (17.42, 230, 0, 12, 0, 1108, 9.035), (17.56, 176, 0, 4, 0, 1036, 8.129), (18.37, 176, 0, 6, 0, 1041, 8.279)]


def cg_sust(I): return 15.0 - (1.5 / 1.3) * (I - 7.8)      # AFFTC: 15.0 in at low inertia, 13.5 in at high
def cg_inc(I): return 13.0 - (0.5 / 1.3) * (I - 7.8)       # AFFTC: 13.0 in low, 12.5 in high


def expected(cg, I):
    if cg >= cg_sust(I): return 'sustained'
    if cg >= cg_inc(I): return 'incipient'
    return 'none'


def a1_load(row):
    cg, fr, re, nt, nc, gw, I = row
    return fr, re, (2.2 * nt + (1.0 if nt else 0.0)), 2.2 * nc


class Flight:
    def __init__(s, aero, jsb=None, front=176, rear=0, tail=0, ckpt=0, V_kmh=90.0, h_ft=6000.0, tag=None):
        st = Static(aero, jsb=jsb, front=front, rear=rear, tail=tail, ckpt=ckpt, h_ft=h_ft, tag=tag)
        tr = st.trim_glide(V_kmh / 3.6)
        s.trim = tr; s.cg = st.cg_in(); s.W = st.W
        f = st.f
        s.Iyy = f['inertia/iyy-slugs_ft2'] * 32.174 * 144 / 1e6
        for k, v in zip(INTEG, (1, 3, 1, 4)):       # JSBSim defaults
            f[k] = v
        f['ic/vt-fps'] = V_kmh / 3.6 / 0.3048; f['ic/alpha-deg'] = tr['alpha']; f['ic/beta-deg'] = 0
        f['ic/gamma-deg'] = tr['gamma']; f['ic/phi-deg'] = 0; f['ic/psi-true-deg'] = 0
        f['ic/p-rad_sec'] = 0; f['ic/q-rad_sec'] = 0; f['ic/r-rad_sec'] = 0
        f['ic/terrain-elevation-ft'] = -4000; f['ic/h-sl-ft'] = h_ft
        f['fcs/elevator-cmd-norm'] = tr['elev']; f['fcs/aileron-cmd-norm'] = 0; f['fcs/rudder-cmd-norm'] = 0
        f.run_ic()
        s.f = f; s.t = 0.0; s.dt = f['simulation/dt']
        s.psi_unw = 0.0; s.last_psi = f['attitude/psi-deg']; s.h0 = f['position/h-sl-ft']
        s.log = []

    def step(s, n=1):
        f = s.f
        for _ in range(n):
            f.run(); s.t += s.dt
            psi = f['attitude/psi-deg']; d = (psi - s.last_psi + 180) % 360 - 180; s.psi_unw += d; s.last_psi = psi

    def rec(s):
        f = s.f
        s.log.append((s.t, f['velocities/vc-kts'], f['aero/alpha-deg'], f['attitude/theta-deg'], f['attitude/phi-deg'],
                      f['velocities/r-rad_sec'] * 57.3, s.psi_unw, f['position/h-sl-ft'] - s.h0, f['aero/beta-deg'],
                      -f['accelerations/n-pilot-z-norm'], f['velocities/p-rad_sec'] * 57.3, f['velocities/q-rad_sec'] * 57.3,
                      f['velocities/vt-fps'] * 0.3048 * 3.6, f['fcs/elevator-cmd-norm'], f['fcs/rudder-cmd-norm'], f['fcs/aileron-cmd-norm']))

    def hold_pitch_to_stall(s, theta_cmd=10.0, tmax=40.0, kth=0.05, kq=0.03, ail=0.0, ramp_rate=2.0, amax_trigger=None):
        """AFFTC entry: raise the nose at ramp_rate deg/s to theta_cmd and hold it with the elevator (PD) while the
        speed bleeds off; the stall is taken when the elevator reaches full aft or alpha passes its peak."""
        f = s.f; e_tr = f['fcs/elevator-cmd-norm']; t0 = s.t; amax = -99; th0 = f['attitude/theta-deg']; ei = 0.0
        while s.t - t0 < tmax:
            thr = min(theta_cmd, th0 + ramp_rate * (s.t - t0))
            th = f['attitude/theta-deg']; q = f['velocities/q-rad_sec'] * 57.3
            ei = float(np.clip(ei + 0.06 * (th - thr) * s.dt, -2.0, 2.0))      # integral: keep pulling as the speed bleeds off
            el = float(np.clip(e_tr + kth * (th - thr) + kq * q + ei, -1, 1))
            f['fcs/elevator-cmd-norm'] = el
            f['fcs/aileron-cmd-norm'] = float(np.clip(-0.04 * f['attitude/phi-deg'] - 0.02 * f['velocities/p-rad_sec'] * 57.3 + ail, -1, 1))
            f['fcs/rudder-cmd-norm'] = float(np.clip(0.05 * f['aero/beta-deg'], -0.3, 0.3))
            s.step()
            if int(round(s.t / s.dt)) % 6 == 0: s.rec()
            a = f['aero/alpha-deg']; amax = max(amax, a)
            if el <= -0.999 or (amax > 10 and a < amax - 0.7) or a > 16 or (amax_trigger is not None and a > amax_trigger):
                return s.t
        return None

    def run_controls(s, T, el=None, rud=None, ail=None, ramp=1.0, cb=None):
        """hold controls for T seconds (ramping from the current values over `ramp` s); cb(self) per step."""
        f = s.f; e0, r0, a0 = f['fcs/elevator-cmd-norm'], f['fcs/rudder-cmd-norm'], f['fcs/aileron-cmd-norm']
        t0 = s.t
        while s.t - t0 < T:
            w = min(1.0, (s.t - t0) / max(ramp, 1e-6))
            if el is not None: f['fcs/elevator-cmd-norm'] = e0 + w * (el - e0)
            if rud is not None: f['fcs/rudder-cmd-norm'] = r0 + w * (rud - r0)
            if ail is not None: f['fcs/aileron-cmd-norm'] = a0 + w * (ail - a0)
            if cb and cb(s): return True
            s.step()
            if int(round(s.t / s.dt)) % 6 == 0: s.rec()
            if f['position/h-agl-ft'] < 500 or not np.isfinite(f['velocities/vt-fps']): return False
        return False

    def arr(s):
        return np.array(s.log)


def spin_entry(aero, load, jsb=None, rud=-1.0, ail=0.0, hold=40.0, tag=None, theta_cmd=10.0, entry_ail=0.0, thermal=False):
    """AFFTC wings-level entry. rud=-1 (JSBSim) = right pedal = spin to the right. returns Flight.
    thermal=True: AFFTC 'thermal entry' - near the stall full aileron toward the intended spin without rudder,
    a 1.5 s pause (adverse yaw), then full pro-spin rudder, full aft stick, aileron neutral."""
    fr, re, tl, ck = load
    F = Flight(aero, jsb=jsb, front=fr, rear=re, tail=tl, ckpt=ck, tag=tag)
    F.run_controls(2.0)
    if thermal:
        st = F.hold_pitch_to_stall(theta_cmd, ail=entry_ail, amax_trigger=9.0)
        F.run_controls(1.5, ail=-rud, rud=0.0, ramp=0.3)
    else:
        st = F.hold_pitch_to_stall(theta_cmd, ail=entry_ail)
    F.stall_t = st if st is not None else F.t
    F.psi_stall = F.psi_unw; F.h_stall = F.f['position/h-sl-ft']
    F.run_controls(hold, el=-1.0, rud=rud, ail=ail, ramp=1.0)
    return F


def classify(F, t_from, a_spin=25.0, r_spin=30.0):
    """AFFTC classes: none (<1 spinning turn: spiral, sideslip or recovery), incipient (>=1 turn, self-recovers
    with pro-spin controls held), sustained (>=5 turns, still spinning at the end)."""
    L = F.arr(); w = L[L[:, 0] >= t_from]
    if len(w) < 3: return dict(cls='none', turns=0.0, recovered=True)
    dpsi = np.abs(np.diff(w[:, 6])); dt = np.diff(w[:, 0])
    rate = dpsi / np.maximum(dt, 1e-6)
    spinning = (w[1:, 2] > a_spin) & (rate > r_spin)
    turns = float(np.sum(dpsi[spinning]) / 360.0)
    tail = w[:, 0][1:] > w[-1, 0] - 8
    still = spinning[tail].mean() > 0.5 if tail.any() else False
    if turns < 1.0: cls = 'none'
    elif still and turns >= 5.0: cls = 'sustained'
    else: cls = 'incipient'
    return dict(cls=cls, turns=turns, recovered=not still)


def spin_metrics(F, t_from):
    L = F.arr(); w = L[L[:, 0] >= t_from]
    if len(w) < 10: return None
    turns = abs(w[-1, 6] - w[0, 6]) / 360.0
    dt = w[-1, 0] - w[0, 0]
    dh = -(w[-1, 7] - w[0, 7])
    return dict(theta_mean=w[:, 3].mean(), theta_min=w[:, 3].min(), theta_max=w[:, 3].max(), theta_sd=w[:, 3].std(),
                rate_mean=turns * 360 / dt if dt > 0 else 0, rate_max=np.abs(w[:, 5]).max(), rate_min=np.abs(w[:, 5]).min(),
                ft_per_turn=dh / max(turns, 0.1), kias_mean=w[:, 1].mean(), kias_min=w[:, 1].min(), kias_max=w[:, 1].max(),
                bank_mean=np.abs(w[:, 4]).mean(), bank_max=np.abs(w[:, 4]).max(), beta_max=np.abs(w[:, 8]).max(),
                nz_mean=w[:, 9].mean(), nz_max=w[:, 9].max(), alpha_mean=w[:, 2].mean(), turns=turns, s_per_turn=dt / max(turns, 0.1))


class SpeedHold:
    """pilot model: pitch attitude loop with an outer airspeed loop (elevator)."""
    def __init__(s, F, V_kmh, kv=0.5, kth=0.05, kq=0.035, ki=0.004):
        s.F = F; s.V = V_kmh; s.kv = kv; s.kth = kth; s.kq = kq; s.ki = ki
        s.th0 = F.f['attitude/theta-deg']; s.e0 = F.f['fcs/elevator-cmd-norm']; s.iv = 0.0
    def __call__(s):
        f = s.F.f
        v = f['velocities/vc-kts'] * 1.852; err = v - s.V
        s.iv = float(np.clip(s.iv + err * s.F.dt, -60, 60))
        thc = s.th0 + s.kv * err + 0.05 * s.iv
        th = f['attitude/theta-deg']; q = f['velocities/q-rad_sec'] * 57.3
        el = s.e0 - s.kth * (thc - th) + s.kq * q - s.ki * 0
        el = float(np.clip(el, -1, 1)); f['fcs/elevator-cmd-norm'] = el
        return el


def recovery(aero, load, case, jsb=None, t_dev=14.0, tag=None, rud=-1.0):
    """Develop a spin (t_dev s after the stall with pro-spin controls), then apply a recovery input.
    case: 'fm' (opposite rudder, 1/2 turn pause, stick to neutral), 'rudder' (opposite rudder, stick held aft),
    'neutral' (all controls centred), 'forward' (full forward stick, rudder kept pro-spin), 'ail_against',
    'ail_with', 'continue'.  Returns turns until the rotation stops (None = not within 20 s), height loss (ft)."""
    fr, re, tl, ck = load
    F = Flight(aero, jsb=jsb, front=fr, rear=re, tail=tl, ckpt=ck, tag=tag)
    F.run_controls(2.0)
    st = F.hold_pitch_to_stall(10.0)
    F.run_controls(t_dev, el=-1.0, rud=rud, ail=0.0, ramp=1.0)
    L = F.arr(); w = L[L[:, 0] > F.t - 6]
    spinning = (w[:, 2].mean() > 25) and (np.abs(np.diff(w[:, 6])).sum() / 360 > 1.0)
    f = F.f; psi0 = F.psi_unw; h0 = f['position/h-sl-ft']; t0 = F.t
    state = dict(stopped=None, turns_at=0.0, h_at=0.0)
    def cb(F):
        tt = F.t - t0
        turns = abs(F.psi_unw - psi0) / 360
        if case == 'fm':
            f['fcs/rudder-cmd-norm'] = -rud
            f['fcs/elevator-cmd-norm'] = -1.0 if turns < 0.5 else max(-1.0, -1.0 + (tt) * 2.0) if False else (-1.0 if turns < 0.5 else 0.0)
            f['fcs/aileron-cmd-norm'] = 0.0
        elif case == 'rudder':
            f['fcs/rudder-cmd-norm'] = -rud; f['fcs/elevator-cmd-norm'] = -1.0; f['fcs/aileron-cmd-norm'] = 0.0
        elif case == 'neutral':
            f['fcs/rudder-cmd-norm'] = 0.0; f['fcs/elevator-cmd-norm'] = 0.0; f['fcs/aileron-cmd-norm'] = 0.0
        elif case == 'forward':
            f['fcs/rudder-cmd-norm'] = rud; f['fcs/elevator-cmd-norm'] = 1.0; f['fcs/aileron-cmd-norm'] = 0.0
        elif case == 'ail_against':
            f['fcs/rudder-cmd-norm'] = rud; f['fcs/elevator-cmd-norm'] = -1.0; f['fcs/aileron-cmd-norm'] = rud      # rud -1 (right spin) -> aileron left
        elif case == 'ail_with':
            f['fcs/rudder-cmd-norm'] = rud; f['fcs/elevator-cmd-norm'] = -1.0; f['fcs/aileron-cmd-norm'] = -rud
        elif case == 'continue':
            f['fcs/rudder-cmd-norm'] = rud; f['fcs/elevator-cmd-norm'] = -1.0; f['fcs/aileron-cmd-norm'] = 0.0
        # rotation stopped: earth-axis turn rate small and wing unstalled
        om = np.sqrt(f['velocities/p-rad_sec'] ** 2 + f['velocities/r-rad_sec'] ** 2) * 57.3
        calm = om < 25 and f['aero/alpha-deg'] < 16
        if calm:
            state.setdefault('calm_since', tt)
            if state['stopped'] is None and tt - state['calm_since'] > 1.0:
                tc = state['calm_since']; state['stopped'] = (tc, state['turns_at'], state['h_at'])
        else:
            state.pop('calm_since', None); state['turns_at'] = abs(F.psi_unw - psi0) / 360; state['h_at'] = h0 - f['position/h-sl-ft']
        return state['stopped'] is not None and tt > state['stopped'][0] + 1.5
    F.run_controls(25.0, cb=cb)
    return dict(spinning=spinning, stopped=state['stopped'], cg=F.cg)


def stall_ramp(aero, load, jsb=None, decel=1.0, hold=12.0, tag=None, V_kmh=90.0):
    """1-g wings-level stall as flown: the speed is bled off at `decel` kt/s with the attitude loop (SpeedHold with a
    falling target) until the stick reaches full aft or the wing stalls, then the stick is held full aft for `hold` s;
    wings level with aileron, ball centred with rudder."""
    fr, re, tl, ck = load
    F = Flight(aero, jsb=jsb, front=fr, rear=re, tail=tl, ckpt=ck, V_kmh=V_kmh, tag=tag); f = F.f
    F.run_controls(1.0)
    sh = SpeedHold(F, V_kmh); t0 = F.t; st = dict(t_full=None, amax=-1, t_amax=0)
    def cb(F):
        tt = F.t - t0
        if st['t_full'] is None:
            sh.V = V_kmh - decel * 1.852 * tt
            el = sh()
            if el <= -0.999 or (st['amax'] > 10 and f['aero/alpha-deg'] < st['amax'] - 1.5):
                st['t_full'] = tt
        else:
            f['fcs/elevator-cmd-norm'] = -1.0
        a = f['aero/alpha-deg']
        if a > st['amax'] and st['t_full'] is None: st['amax'] = a; st['t_amax'] = tt
        f['fcs/aileron-cmd-norm'] = float(np.clip(-0.04 * f['attitude/phi-deg'] - 0.02 * f['velocities/p-rad_sec'] * 57.3, -1, 1))
        f['fcs/rudder-cmd-norm'] = float(np.clip(-0.04 * f['aero/beta-deg'], -1, 1))
        return st['t_full'] is not None and tt > st['t_full'] + hold
    F.run_controls(90.0, cb=cb)
    L = F.arr(); w = L[L[:, 0] >= t0]
    tf = st['t_full'] if st['t_full'] is not None else w[-1, 0] - t0
    pre = w[w[:, 0] <= t0 + tf + 0.01]
    ia = int(np.argmax(pre[:, 2])) if len(pre) else 0
    after = w[(w[:, 0] >= pre[ia, 0]) & (w[:, 0] <= pre[ia, 0] + 4)]
    gbreak = float(pre[ia, 3] - after[:, 3].min()) if len(after) else 0.0
    hold_w = w[(w[:, 0] >= t0 + tf + 1) & (w[:, 0] <= t0 + tf + 6)]
    drop3 = w[(w[:, 0] >= t0) & (w[:, 0] <= t0 + tf + 3)]
    elev_lim = (pre[-1, 13] <= -0.99) and (pre[ia, 2] >= pre[:, 2].max() - 0.3) and gbreak < 1.5
    return dict(cg=F.cg, W=F.W, amax=float(pre[ia, 2]), gbreak=gbreak, kias_min=float(w[:, 1].min()),
                kias_min_pre=float(pre[:ia + 1, 1].min()),
                kias_stall=float(pre[ia, 1]), theta_osc=float(hold_w[:, 3].max() - hold_w[:, 3].min()) if len(hold_w) else 0.0,
                alpha_hold=(float(hold_w[:, 2].min()), float(hold_w[:, 2].max())) if len(hold_w) else (0, 0),
                wing_drop=float(np.abs(drop3[:, 4]).max()), theta_max=float(w[:, 3].max()), elev_limited=bool(elev_lim),
                full_aft_reached=bool(pre[-1, 13] <= -0.99))
