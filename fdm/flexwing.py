"""Wing bending of the ASK 21: each wing panel as a beam clamped in the fuselage, in its first bending mode.

The beam:
  - mass: 95 kg per panel (TCDS EASA.A.221: 600 kg maximum, of which 410 kg non-lifting parts; the Mi's 705/510 kg
    give the same), distributed with the square of the chord;
  - stiffness: the spar and the shell.
    The spar is sized to one strain allowable at the limit load, the usual sizing of a glass-fibre spar cap:
    EI(y) = M_limit(y) h(y) / (2 eps), with the limit load factor of the aerobatic category (+6.5 at VA, 600 kg), the
    lifting line's spanwise load less the panel's own inertia, and spar depth h = 0.9 t/c c (FX S 02-196 to the kink,
    blending to FX 60-126 at the tip).
    The shell's glass faces, 0.18 + 0.6 mm per surface (the outboard shell of D. Eller, U. Ringertz, "Aeroelastic
    Simulations of a Sailplane", KTH 2005) of +-45 deg fabric (10 GPa spanwise) over 80 % of the chord at +-h/2, carry
    the bending where the spar runs out;
  - stiffness level: the strain eps that gives the measured first symmetric wing bending of the whole glider hanging on
    soft springs, 2.95 Hz, with the rest of the 380 kg test glider as a rigid mass at the wing roots (KTH's ground
    vibration test of an ASK 21: U. Nilsson, M. Norsell, U. Carlsson, TRITA/AVE 2003:32; listed by Eller and Ringertz,
    table 1). It comes out at 0.9 %. The same beam gives 6.0 Hz for the first antisymmetric bending (test: 6.50 Hz,
    with the fuselage's own bending in it); without the shell it would be 5.2 Hz;
  - structural damping 2 % (glass-fibre wings, ground vibration tests).
That gives a first bending frequency of 2.6 Hz with the fuselage held, a tip deflection of about 0.17 m in level flight
at 500 kg and a sag of 8 cm under the wing's own weight on the ground. The panels' mass matters most for those
deflections: 100 to 80 kg panels give 0.16 to 0.21 m, a mass that follows the chord instead of its square 0.10 m.

In flight the mode is driven by the strips' own normal forces and by the inertia of the panel in the accelerating
airframe (load factor, rotation), and it acts back on the flight model: each strip moves with the
bending (plunge velocity, local dihedral, height) and the airframe feels the panels' acceleration. The tip skids and the
ground-handling wing supports follow the tip; the wing cameras ride on the wing. python3 flexwing.py [--write]  (prints the mode; --write the system file and the animation)"""
import os, sys, json, math
import numpy as np
import geometry as G
import liftingline as LL
from scipy.linalg import eigh
from scipy.optimize import brentq

HERE = os.path.dirname(os.path.abspath(__file__))
AIRCRAFT = os.path.join(os.path.dirname(HERE), 'ASK21_v9')
g = 9.80665
LBF = 4.4482216                   # N per lbf
FT = 0.3048
M_WING = 95.0                     # kg per panel
N_LIM, W_LIM = 6.5, 600.0         # limit load factor (aerobatic category, at VA) and mass
GVT_HZ = 2.95                     # first symmetric wing bending, whole glider on soft springs (KTH GVT)
M_GVT = 380.0                     # the tested glider without payload (kg)
ZETA = 0.02                       # structural damping ratio
E_SHELL, T_SHELL = 10e9, 0.78e-3  # the shell faces: spanwise modulus (N/m2) and thickness per surface (m)
IC_FRAMES = 12                    # frames of static deflection after an initial condition (0.1 s at 120 Hz)
TIP = 8.5
# the 3-D model's wing hinges (Models/ask21.xml: wing.L1..L4, |y| in m) and the tip skids' span station
HINGES = [0.29524, 2.33212, 4.32656, 7.93317]
SKID = (-1.25364, 8.43947, 1.22131)     # the tip skids' contacts (ask21-jsb.xml, structural frame m, |y|) and their
SKID_Y = SKID[1]
SKID_K, SKID_C = 396.90, 198.45         # spring (lbf/ft) and damper (lbf/ft/s)
HELPER_Z = 0.05                         # the ground-handling wing supports (assist gear, same place and spring), contact z
# the left below-wing and right wingtip cameras (ask21-main.xml): view, |x| (span, m) and height (y-offset-m, up)
VIEWS = {'left': (100, 2.58654, 0.7), 'right': (101, 8.34341, 1.5)}


def section(y):
    """spar depth and chord (m) at |y|"""
    c = G.chord(y)
    tc = np.where(y < G.Y_KINK, 0.196, 0.196 + (0.126 - 0.196) * (y - G.Y_KINK) / (TIP - G.Y_KINK))
    return 0.9 * tc * c, c


def structure(eps, n=200):
    """stations, mass per length (kg/m) and EI (N m2) for spar cap strain eps at the limit load"""
    y = np.linspace(G.Y_FUS, TIP, n)
    h, c = section(y)
    m = c ** 2
    m *= M_WING / np.trapezoid(m, y)
    # spanwise lift shape from the lifting line (root at 8 deg, the rigging's twist)
    rows = LL.wing_layout()
    cl, ai, CL = LL.solve_linear(rows, LL.f_matrix(rows), np.array([8.0 + r['inc'] - G.I_ROOT for r in rows]))
    ys = np.array([r['ys'] for r in rows]); cs = np.array([r['c'] for r in rows]); right = ys > 0
    lift = np.interp(y, ys[right], (cl * cs)[right])
    lift *= N_LIM * W_LIM * g / 2 / np.trapezoid(lift, y)
    w = lift - N_LIM * g * m
    M = np.array([np.trapezoid(w[i:] * (y[i:] - y[i]), y[i:]) for i in range(n)])
    EI = M * h / (2 * eps) + E_SHELL * 2 * 0.8 * c * T_SHELL * (h / 2) ** 2
    return y, m, EI


def beam(eps, N=80):
    """the panel in N beam elements from the root: stiffness and mass matrices (deflection and slope per node), nodes"""
    y, mpl, EI = structure(eps)
    ye = np.linspace(y[0], y[-1], N + 1); Le = np.diff(ye); ym = 0.5 * (ye[1:] + ye[:-1])
    EIe, me = np.interp(ym, y, EI), np.interp(ym, y, mpl)
    K = np.zeros((2 * N + 2,) * 2); Mm = np.zeros_like(K)
    for e in range(N):
        L = Le[e]
        k = EIe[e] / L ** 3 * np.array([[12, 6 * L, -12, 6 * L], [6 * L, 4 * L * L, -6 * L, 2 * L * L],
                                        [-12, -6 * L, 12, -6 * L], [6 * L, 2 * L * L, -6 * L, 4 * L * L]])
        m = me[e] * L / 420 * np.array([[156, 22 * L, 54, -13 * L], [22 * L, 4 * L * L, 13 * L, -3 * L * L],
                                        [54, 13 * L, 156, -22 * L], [-13 * L, -3 * L * L, -22 * L, 4 * L * L]])
        K[2 * e:2 * e + 4, 2 * e:2 * e + 4] += k; Mm[2 * e:2 * e + 4, 2 * e:2 * e + 4] += m
    return K, Mm, ye


def mode(eps):
    """first cantilever bending mode: frequency (Hz), shape phi (tip = 1) and slope on the beam's nodes"""
    K, Mm, ye = beam(eps)
    w2, V = eigh(K[2:, 2:], Mm[2:, 2:])
    v = np.concatenate([[0.0, 0.0], V[:, 0]])
    w = v[0::2] / v[-2]; s = v[1::2] / v[-2]
    return math.sqrt(w2[0]) / (2 * math.pi), ye, w, s


def gvt_hz(eps):
    """first symmetric wing bending of the whole glider hanging free: the rest of the test glider a rigid mass at the
    roots, the root slope zero by symmetry"""
    K, Mm, ye = beam(eps)
    Mm[0, 0] += (M_GVT - 2 * M_WING) / 2
    keep = [0] + list(range(2, len(K)))
    w2 = eigh(K[np.ix_(keep, keep)], Mm[np.ix_(keep, keep)], eigvals_only=True)
    return math.sqrt(w2[1]) / (2 * math.pi)          # (w2[0]: the rigid heave)


def gvt_asym_hz(eps):
    """first antisymmetric wing bending of the whole glider hanging free: the panels roll about the centre line with the
    fuselage between them (its roll inertia is small beside theirs)"""
    K, Mm, ye = beam(eps)
    T = np.zeros((len(K), len(K) - 1)); T[0, 0] = ye[0]; T[1, 0] = 1.0
    T[2:, 1:] = np.eye(len(K) - 2)
    w2 = eigh(T.T @ K @ T, T.T @ Mm @ T, eigvals_only=True)
    return math.sqrt(w2[1]) / (2 * math.pi)          # (w2[0]: the rigid roll)


def limit_strain():
    """the spar cap strain at the limit load that reproduces the ground vibration test"""
    return brentq(lambda e: gvt_hz(e) - GVT_HZ, 0.001, 0.05)


def tip_load(eps, F):
    """static tip deflection (m) of the panel held at the root under a force F (N) at the tip"""
    K, Mm, ye = beam(eps)
    f = np.zeros(len(K) - 2); f[-2] = F
    return float(np.linalg.solve(K[2:, 2:], f)[-2])


def model():
    """everything the flight model and the animation need, SI units"""
    eps = limit_strain()
    f, ye, phi, slope = mode(eps)
    y, mpl, EI = structure(eps)
    ph = np.interp(y, ye, phi)
    xs = G.XS0 + G.x_le(y) + 0.40 * G.chord(y)            # mass centroid at 40 % chord (structural frame, aft +)
    mg = float(np.trapezoid(mpl * ph ** 2, y))
    om = 2 * math.pi * f
    Pt = float(np.trapezoid(mpl * ph, y))
    Py = float(np.trapezoid(mpl * ph * y, y))
    Px = float(np.trapezoid(mpl * ph * xs, y))
    at = lambda yy: float(np.interp(abs(yy), ye, phi, left=0.0))
    sl = lambda yy: float(np.interp(abs(yy), ye, slope, left=0.0))
    # static deflections (level flight from the lifting line's load at 500 kg; ground: own weight)
    rows = LL.wing_layout()
    cl, ai, CL = LL.solve_linear(rows, LL.f_matrix(rows), np.array([4.0 + r['inc'] - G.I_ROOT for r in rows]))
    q1 = sum(at(r['ys']) * cl[k] * r['area'] for k, r in enumerate(rows) if r['ys'] > 0) / sum(cl[k] * r['area'] for k, r in enumerate(rows))
    # (the share of the lift that the mode feels, per newton of total lift)
    k_g = mg * om ** 2
    d_flight = (q1 * 500 * g - Pt * g) / k_g
    d_ground = -Pt * g / k_g
    # 3-D hinge angles per metre of tip deflection: the bending line through the hinges, piecewise straight
    pts = HINGES + [TIP]
    seg = [(at(pts[i + 1]) - at(pts[i])) / (pts[i + 1] - pts[i]) for i in range(len(HINGES))]
    hinge = [math.degrees(seg[0])] + [math.degrees(seg[i] - seg[i - 1]) for i in range(1, len(seg))]
    return dict(freq_hz=f, mg=mg, omega=om, k=k_g, c=2 * ZETA * om * mg, Pt=Pt, Py=Py, Px=Px, x_c=Px / Pt, y_c=Py / Pt,
                d_flight_m=d_flight, d_ground_m=d_ground, slope_tip_deg=math.degrees(sl(TIP)), hinge_deg_per_m=hinge,
                skid=at(SKID_Y), skid_xyz=SKID, skid_a=at(SKID_Y) * math.cos(math.radians(G.DIHEDRAL)), view={k: at(v[1]) for k, v in VIEWS.items()}, phi=at, slope=sl,
                EI_root=float(EI[0]), mass=M_WING, eps=eps, gvt_hz=gvt_hz(eps), gvt_asym_hz=gvt_asym_hz(eps),
                tip_10kg_m=tip_load(eps, 10 * g))


def fcs_xml(D):
    """JSBSim system: the two panels' modal states, their generalized forces, the tip skids, outputs"""
    out = ['<?xml version="1.0"?>',
           '<!-- generated by fdm/flexwing.py: the wing panels\' first bending mode (tip deflection, m, up +).',
           '     %.2f Hz clamped, generalized mass %.2f kg, %.0f %% structural damping; %.2f m at 1 g (500 kg), %.2f m on the ground -->'
           % (D['freq_hz'], D['mg'], 100 * ZETA, D['d_flight_m'], D['d_ground_m']),
           '<system name="ASK21 wing bending">',
           '  <property value="1">flex/enable</property>']
    out.append('  <property value="0">flex/ic-frames</property>')
    for s in ('L', 'R'):
        out += ['  <property value="0">flex/q%s-m</property>' % s, '  <property value="0">flex/qd%s-mps</property>' % s,
                '  <property value="0">flex/qdd%s-mps2</property>' % s]
    out.append('  <channel name="wing bending">')
    dt = '<property>simulation/dt</property>'
    # frames since the last initial condition: JSBSim runs the IC frame with dt = 0 (sim time goes on through a reset)
    out.append('    <fcs_function name="flex/ic-frames-calc"><function><ifthen><lt>%s<value>1e-9</value></lt><value>0</value>'
               '<min><value>1000</value><sum><property>flex/ic-frames</property><value>1</value></sum></min></ifthen></function>'
               '<output>flex/ic-frames</output></fcs_function>' % dt)
    fn = lambda name, body, outp: out.append('    <fcs_function name="%s"><function>%s</function><output>%s</output></fcs_function>' % (name, body, outp))
    val = lambda x: '<value>%.6g</value>' % x
    dts = '<max><value>1e-6</value>%s</max>' % dt
    ph, a = D['skid'], D['skid_a']                      # the mode at the skid; skid height per metre of tip deflection
    ks, cs = SKID_K * LBF / FT, SKID_C * LBF / FT        # the skid's spring and damper (N/m, N s/m)
    for s, unit, helper, Py in (('L', 3, 5, -D['Py']), ('R', 4, 6, D['Py'])):
        q, qd = '<property>flex/q%s-m</property>' % s, '<property>flex/qd%s-mps</property>' % s
        # the tip rests on its skid, or on the ground-handling wing support when that is out (1.2 m lower, so never both)
        gear = lambda prop: ('<ifthen><gt><property>gear/unit[%d]/WOW</property><value>0</value></gt><property>gear/unit[%d]/%s</property>'
                             '<property>gear/unit[%d]/%s</property></ifthen>' % (helper, helper, prop, unit, prop))
        wow = '<max><property>gear/unit[%d]/WOW</property><property>gear/unit[%d]/WOW</property></max>' % (unit, helper)
        Cm = '<product><value>0.3048</value>%s</product>' % gear('compression-ft')
        Cdm = '<product><value>0.3048</value>%s</product>' % gear('compression-velocity-fps')
        # inertia of the panel in the airframe: load factor, and the rotation's z acceleration of a point (x, y) of the
        # panel, (pdot + q r) y + (p r - qdot) x (body axes from the CG; x arm from the structural frame, aft +)
        nz = '<property>accelerations/Nz</property>'
        nz0 = '<max><value>-1</value><min><value>3</value><property>accelerations/Nz</property></min></max>'   # (the first frames on the gear spike)
        inert = ('<sum><product><value>%.6g</value>NZ</product>'
                 '<product><value>%.6g</value><sum><property>accelerations/pdot-rad_sec2</property>'
                 '<product><property>velocities/q-rad_sec</property><property>velocities/r-rad_sec</property></product></sum></product>'
                 '<product><value>%.6g</value><difference><product><property>velocities/p-rad_sec</property><property>velocities/r-rad_sec</property></product>'
                 '<property>accelerations/qdot-rad_sec2</property></difference>'
                 '<difference><product><value>%.6g</value><property>inertia/cg-x-in</property></product><value>%.6g</value></difference></product></sum>'
                 % (-D['Pt'] * g, Py, D['Pt'], 0.0254, D['Px'] / D['Pt']))
        start = '<lt><property>flex/ic-frames</property><value>%d</value></lt>' % IC_FRAMES
        # the forces on the mode without the skid: air, inertia, and the structure at the last state (N)
        fn('flex/F%s-calc' % s, '<difference><sum><property>aero/v9/flex/Q%s-N</property>%s</sum><sum><product>%s%s</product><product>%s%s</product></sum></difference>'
           % (s, inert.replace('NZ', nz), val(D['c']), qd, val(D['k']), q), 'flex/F%s-N' % s)
        F = '<property>flex/F%s-N</property>' % s
        # the tip's contact on the ground (its spring and damper, JSBSim's BOGEY contact, which runs after this system) is
        # stiff for the panel's small generalized mass, so it acts implicitly: its force with the new state,
        #   ks (C - a dt qd') + cs (Cdot - a qd'),   C, Cdot from the last frame's gear with the contact at a q
        # and the free velocity otherwise (semi-implicit Euler, as without contact)
        fn('flex/qd%s-free' % s, '<sum>%s<product>%s%s%s</product></sum>' % (qd, dts, F, val(1 / D['mg'])), 'flex/qd%s-free' % s)
        fn('flex/qd%s-contact' % s, '<quotient><sum>%s<product>%s%s<sum>%s<product>%s<sum><product>%s%s</product><product>%s%s</product></sum></product></sum></product></sum>'
           '<sum><value>1</value><product>%s%s<sum>%s<product>%s%s</product></sum></product></sum></quotient>'
           % (qd, dts, val(1 / D['mg']), F, val(ph), val(ks), Cm, val(cs), Cdm,
              val(ph * a / D['mg']), dts, val(cs), val(ks), dts), 'flex/qd%s-contact' % s)
        qdc = '<property>flex/qd%s-contact</property>' % s
        fn('flex/skid%s-calc' % s, '<sum><product>%s<difference>%s<product>%s%s%s</product></difference></product><product>%s<difference>%s<product>%s%s</product></difference></product></sum>'
           % (val(ks), Cm, val(a), dts, qdc, val(cs), Cdm, val(a), qdc), 'flex/skid%s-N' % s)
        contact = '<and><gt>%s<value>0</value></gt><gt><property>flex/skid%s-N</property><value>0</value></gt></and>' % (wow, s)
        # flex/enable 0 holds the panel still: no deflection (below), no motion, so no reaction on the airframe
        fn('flex/qd%s-new' % s, '<product><property>flex/enable</property><ifthen>%s<value>0</value><ifthen>%s%s<property>flex/qd%s-free</property></ifthen></ifthen></product>'
           % (start, contact, qdc, s), 'flex/qd%s-new' % s)
        fn('flex/qdd%s-calc' % s, '<ifthen>%s<value>0</value><quotient><difference><property>flex/qd%s-new</property>%s</difference>%s</quotient></ifthen>' % (start, s, qd, dts),
           'flex/qdd%s-mps2' % s)
        fn('flex/qd%s-calc' % s, '<property>flex/qd%s-new</property>' % s, 'flex/qd%s-mps' % s)
        # the airframe gets JSBSim's contact force, which sees the airframe's motion only: the difference to the contact's
        # force on the tip goes on the airframe as well (up +, lbf; both forces push only)
        fn('flex/skid%s-dF' % s, '<product><property>flex/enable</property>%s%s<difference><max><value>0</value><ifthen>%s<property>flex/skid%s-N</property><value>0</value></ifthen></max>'
           '<max><value>0</value><sum><product>%s%s</product><product>%s%s</product></sum></max></difference></product>'
           % (val(1 / LBF), wow, contact, s, val(ks), Cm, val(cs), Cdm), 'flex/skid%s-dF-lbs' % s)
        # after an initial condition: the static deflection for the loads of that moment, on the contact if it carries the tip;
        # then the integration (held within +-2 m)
        Q0 = '<sum><property>aero/v9/flex/Q%s-N</property>%s</sum>' % (s, inert.replace('NZ', nz0))
        free = '<quotient>%s%s</quotient>' % (Q0, val(D['k']))
        onskid = ('<quotient><sum>%s<product>%s<sum>%s<product>%s%s</product></sum></product></sum>%s</quotient>'
                  % (Q0, val(ph * ks), Cm, val(a), q, val(D['k'] + ph * ks * a)))
        static = '<max><value>-2</value><min><value>2</value><ifthen><gt>%s<value>0</value></gt><max>%s%s</max>%s</ifthen></min></max>' % (wow, free, onskid, free)
        integ = '<max><value>-2</value><min><value>2</value><sum>%s<product>%s%s</product></sum></min></max>' % (q, dt, qd)
        fn('flex/q%s-calc' % s, '<product><property>flex/enable</property><ifthen>%s%s%s</ifthen></product>' % (start, static, integ), 'flex/q%s-m' % s)
        # the tip skid and the wing support follow the tip (structural frame inches, z up)
        for name, u, z in (('skid', unit, SKID[2]), ('helper', helper, HELPER_Z)):
            fn('flex/%s%s-z' % (name, s), '<sum><value>%.4f</value><product>%s%s</product></sum>' % (z / 0.0254, val(a / 0.0254), q),
               'gear/unit[%d]/z-position' % u)
    out += ['  </channel>', '</system>', '']
    return '\n'.join(out)


def views(D, path=os.path.join(AIRCRAFT, 'Systems', 'wingflex_view.xml')):
    """property rules that move the wing cameras up and down with the wing at their span station"""
    out = ['<?xml version="1.0"?>', '', '<PropertyList>', '',
           '\t<!-- generated by fdm/flexwing.py: the below-wing and wingtip cameras ride on the wing, at %.0f and %.0f %% of the'
           % (100 * D['view']['left'], 100 * D['view']['right']),
           '\t     tip deflection (sim/model/wingflex/*-m, from the flight model) -->']
    for side in ('left', 'right'):
        n, x, y0 = VIEWS[side]
        out += ['\t<filter>', '\t\t<type>gain</type>', '\t\t<gain>1</gain>',
                '\t\t<enable><condition><equals><property>/sim/current-view/view-number-raw</property><value>%d</value></equals></condition></enable>' % n,
                '\t\t<input><expression><sum><value>%.4f</value><product><value>%.4f</value><property>/sim/model/wingflex/%s-m</property></product></sum></expression></input>'
                % (y0, D['view'][side], side),
                '\t\t<output>/sim/current-view/y-offset-m</output>', '\t</filter>']
    out += ['', '</PropertyList>', '']
    open(path, 'w').write('\n'.join(out))


def animate(D, path=os.path.join(AIRCRAFT, 'Models', 'ask21.xml')):
    """the 3-D model's eight hinge rotations from the mode: deg per metre of each side's tip deflection"""
    import re
    s = open(path).read()
    a = s.index('<!--	WINGFLEXER	-->'); b = s.index('<!--	FLIGHT CONTROL SURFACES	-->')
    blk = s[a:b]
    anims = re.findall(r'<animation>.*?</animation>', blk, re.S)
    assert len(anims) == 8, len(anims)
    new = []
    for k, an in enumerate(anims):
        side = 'left' if k < 4 else 'right'
        fac = D['hinge_deg_per_m'][k % 4] * (1 if side == 'left' else -1)
        an2 = re.sub(r'<property>[^<]*</property>', '<property>sim/model/wingflex/%s-m</property>' % side, an)
        an2 = re.sub(r'<factor>[^<]*</factor>', '<factor>%.4f</factor>' % fac, an2)
        new.append(an2)
    head = ('<!--	WINGFLEXER	-->\n\t<!-- the wing\'s bending line through the hinges, from the flight model\'s first bending mode '
            '(fdm/flexwing.py): deg per metre of tip deflection, up + -->\n\t')
    s = s[:a] + head + '\n\t'.join(new) + '\n\n\t' + s[b:]
    open(path, 'w').write(s)


if __name__ == '__main__':
    os.chdir(HERE)
    D = model()
    print('spar strain at the limit load %.2f %%; whole glider free: symmetric %.2f Hz (test %.2f), antisymmetric %.2f Hz (test 6.50)'
          % (100 * D['eps'], D['gvt_hz'], GVT_HZ, D['gvt_asym_hz']))
    print('first bending (clamped) %.2f Hz; generalized mass %.2f kg; root EI %.3g N m2' % (D['freq_hz'], D['mg'], D['EI_root']))
    print('10 kg on the tip, panel held at the root: %.1f cm' % (100 * D['tip_10kg_m']))
    print('tip deflection: level flight 500 kg %+.2f m (tip slope %.1f deg), ground %+.2f m' %
          (D['d_flight_m'], D['slope_tip_deg'] * D['d_flight_m'], D['d_ground_m']))
    print('participation: sum m phi %.2f kg, centroid y %.2f m x %.2f m' % (D['Pt'], D['y_c'], D['x_c']))
    print('3-D hinges (deg per m):', ' '.join('%.3f' % h for h in D['hinge_deg_per_m']), '| skid phi %.3f | views' % D['skid'], D['view'])
    if '--write' in sys.argv:
        for d in (os.path.join(AIRCRAFT, 'Systems'), os.path.join(HERE, 'root', 'aircraft', 'ask21-jsb', 'Systems')):    # aircraft, test root
            open(os.path.join(d, 'ask21-flex.xml'), 'w').write(fcs_xml(D))
        animate(D)
        views(D)
        json.dump({k: v for k, v in D.items() if not callable(v)}, open(os.path.join(HERE, 'flex_mode.json'), 'w'), indent=1)
        print('wrote Systems/ask21-flex.xml (aircraft and test root), Models/ask21.xml animations, Systems/wingflex_view.xml, flex_mode.json')
