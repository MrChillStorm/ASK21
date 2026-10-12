"""Generate the v9 ASK 21 aerodynamics (component build-up) as a JSBSim <aerodynamics> file.

All forces are computed per component in body axes from the local air velocity at the component
(translation + rotation about the current CG), summed on axes X/Y/Z, and moments are taken about the
aerodynamic reference point (JSBSim transfers the force sum from there to the CG).
"""
import numpy as np, geometry as G, liftingline as LL, airfoils as AF, json, sys, os

FT = 1 / 0.3048
R2D = 57.29577951308232

DEFAULTS = dict(
    elev0=-6.0,          # elevator angle (deg, TE down +), stick centred + trim neutral: not documented; set so that the
                         # stick-free take-off (trim neutral, FM) lifts off at ~75-80 km/h with 2 x 80 kg, before the tug
    k_ind=1.07,          # induced-angle scale (22-strip lifting line -> converged e)
    a_sec=6.7,           # section lift slope used in the pre-solved lifting-line operator (1/rad)
    k_cd=1.0,            # wing profile-drag factor (surface quality, bugs, gaps); polar fit -> 1.00
    cd_misc=0.0017,      # misc. drag coefficient (canopy joints, vents, gaps, antennas) on S
    cdc=1.0,             # fuselage cross-flow drag coefficient (Allen & Perkins)
    k2k1=0.94,           # fuselage apparent-mass factor (fineness ratio 9.6)
    kpot_aft=0.5,        # fraction of the potential (Munk) load carried by the aft body (separation)
    cf_fus=0.0026,       # fuselage skin friction (Re ~ 1.5e7 turbulent 0.0028, laminar forebody part)
    wet_fus=12.33,       # fuselage surface (Mi-WHB 12.33 m2; 3-view integration 13.5 incl. junctions)
    ff_fus=1.09,         # fuselage form factor (fineness 9.6)
    gear_DS=0.010,       # gear drag area (m2): main wheel half exposed, nose and tail wheel
    eps_k=1.0,           # downwash scale
    wake_k=1.0,          # wing-wake q-deficit scale at the T-tail
    wake_kv=0.6,         # same at the fin/rudder
    ht_e=0.741, vt_e=0.693, vt_ar=2.35,   # one-mode lifting line (Helmbold-consistent), fin with T-tail end plate
    sb_dcl=0.55, sb_dcd=0.578, sb_dcm=0.04,   # sb_dcd calibrated: 45-deg dive with brakes 232 km/h at 600 kg (FM)   # airbrake section increments (per unit brake-covered span)
    ge_k=16.0,           # ground-effect parameter (McCormick)
    cn90_scale=1.0,      # post-stall normal-force scale (all wing strips)
    wing_clmax_k=0.85,   # section clmax relative to NeuralFoil (root/inner wing)
    tip_clmax_k=0.85,
    a_peak_root=15.0, a_peak_tip=15.0, stall_drop=0.22,
    k_sep=0.05,          # separation drag per unit lift removed by the stall shaping
    re_exp=-0.22,        # profile drag ~ Re^re_exp (NeuralFoil: -0.16 FX 60-126 .. -0.29 FX S 02-196)
    flap_eff=0.90,       # plain-flap effectiveness vs NeuralFoil (sealed gaps, Datcom-type correction)
    CN90_wing=1.30,
    boff=8.0, dstall=15.0, xcp90=0.5, CN90_ht=1.20, CN90_vt=1.15,
    w_ca=6.0, ca_sep=0.02,   # post-stall loss of leading-edge suction (chordwise force)
    vt_cn_scale=1.0,
    fin_blank=0.35,       # fin q loss in the separated cross-flow wake of the fuselage at large fin sideslip
    fin_blank_b0=15.0, fin_blank_b1=40.0,
)

ARP = (-1.34, 0.0, 0.79163)        # metrics AERORP of ask21-jsb.xml (m)


def v(x): return '<value>%.6g</value>' % x
def p(n): return '<property>%s</property>' % n
def tag(t, *a): return '<%s>%s</%s>' % (t, ''.join(a), t)
def S(*a): return tag('sum', *a)
def D(a, b): return tag('difference', a, b)
def M(*a): return tag('product', *a)
def Q(a, b): return tag('quotient', a, b)
def SQ(a): return tag('sqrt', a)
def SIN(a): return tag('sin', a)
def COS(a): return tag('cos', a)
def ATAN2(y, x): return tag('atan2', y, x)
def EXP(a): return tag('exp', a)
def ABS(a): return tag('abs', a)
def MAX(*a): return tag('max', *a)
def MIN(*a): return tag('min', *a)


def T1(var, xs, ys):
    rows = '\n'.join('%g %.6g' % (x, y) for x, y in zip(xs, ys))
    return '<table><independentVar lookup="row">%s</independentVar><tableData>\n%s\n</tableData></table>' % (var, rows)


def T2(rowvar, colvar, xs, cols, data):
    head = '      ' + ' '.join('%g' % c for c in cols)
    rows = '\n'.join('%g ' % x + ' '.join('%.5g' % d for d in data[i]) for i, x in enumerate(xs))
    return ('<table><independentVar lookup="row">%s</independentVar><independentVar lookup="column">%s</independentVar>'
            '<tableData>\n%s\n%s\n</tableData></table>') % (rowvar, colvar, head, rows)


class Gen:
    def __init__(s, P):
        s.P = dict(DEFAULTS); s.P.update(P or {})
        s.f = []          # pre-functions (xml strings)
        s.parts = dict(X=[], Y=[], Z=[], L=[], Mm=[], N=[])

    def fn(s, name, body, desc=None):
        d = '<description>%s</description>' % desc if desc else ''
        s.f.append('<function name="%s">%s%s</function>' % (name, d, body))
        return p(name)

    # ------------------------------------------------------------- kinematics
    def kin(s):
        s.U, s.V, s.W = p('velocities/u-aero-fps'), p('velocities/v-aero-fps'), p('velocities/w-aero-fps')
        s.Pp, s.Qq, s.Rr = p('velocities/p-aero-rad_sec'), p('velocities/q-aero-rad_sec'), p('velocities/r-aero-rad_sec')
        s.RHO = p('atmosphere/rho-slugs_ft3')
        s.cgx = s.fn('aero/v9/cg-x-ft', Q(p('inertia/cg-x-in'), v(12)))
        s.cgy = s.fn('aero/v9/cg-y-ft', Q(p('inertia/cg-y-in'), v(12)))
        s.cgz = s.fn('aero/v9/cg-z-ft', Q(p('inertia/cg-z-in'), v(12)))
        s.arx = s.fn('aero/v9/rp-x-ft', Q(p('metrics/aero-rp-x-in'), v(12)))
        s.ary = s.fn('aero/v9/rp-y-ft', Q(p('metrics/aero-rp-y-in'), v(12)))
        s.arz = s.fn('aero/v9/rp-z-ft', Q(p('metrics/aero-rp-z-in'), v(12)))
        s.V2 = s.fn('aero/v9/V2', S(M(s.U, s.U), M(s.V, s.V), M(s.W, s.W), v(1e-4)))
        s.Vt = s.fn('aero/v9/Vt', SQ(s.V2))
        s.qinf = s.fn('aero/v9/qinf', M(v(0.5), s.RHO, s.V2))
        # ground effect on the induced angles (McCormick), h/b of the wing
        hb = np.array([0, 0.02, 0.04, 0.06, 0.08, 0.1, 0.15, 0.2, 0.3, 0.5, 1.0])
        k = s.P['ge_k']; phi = (k * hb) ** 2 / (1 + (k * hb) ** 2); phi[-1] = 1.0
        s.ge = s.fn('aero/v9/ge-phi', T1('aero/h_b-mac-ft', hb, phi), 'ground effect factor on the wing induced angles (McCormick)')
        hbt = np.array([0.0, 0.035, 0.047, 0.059, 0.088, 0.118, 0.176, 0.235, 0.353, 0.5, 0.706, 1.0, 2.0])
        rt = np.array([0.15, 0.21, 0.27, 0.33, 0.455, 0.555, 0.70, 0.79, 0.89, 0.94, 0.97, 0.985, 1.0])
        s.get = s.fn('aero/v9/ge-tail', T1('aero/h_b-mac-ft', hbt, rt), 'ground effect on the tail downwash (image vortices, ge_tail.py)')
        # wake of an aircraft ahead (the tug): written by Nasal/propwash.nas, declared in the FCS system files.
        # Positions in the structural frame (ft: y right, z up); all zero = no wake.
        s.wsh = {k: p('aero/v9/wash/' + k) for k in (
            'y-ft', 'z-ft', 'r-ft', 'u-fps', 'swirl-fps', 'turb-u-fps', 'turb-v-fps', 'turb-w-fps', 'turb-roll-fps',
            'vr-y-ft', 'vr-z-ft', 'vl-y-ft', 'vl-z-ft', 'gamma-ft2_sec', 'core-ft')}
        s.wcache = {}

    def wash(s, ys, zs):
        """air velocity (body axes: x forward, y right; and up) of the wake ahead at structural point (ft).
        Slipstream: a Gaussian jet of radius r (extra speed aft, swirl turning clockwise seen from behind,
        turbulence); wing wake: the tug's two trailing vortices (Lamb-Oseen cores), the right one turning
        anticlockwise seen from behind, the left one clockwise, with the downwash between them."""
        key = (round(ys, 3), round(zs, 3))
        if key in s.wcache:
            return s.wcache[key]
        W = s.wsh; nm = 'aero/v9/wash/p%d' % len(s.wcache)
        dy = s.fn(nm + '/dy', D(v(ys), W['y-ft'])); dz = s.fn(nm + '/dz', D(v(zs), W['z-ft']))
        R2 = S(M(W['r-ft'], W['r-ft']), v(1e-3))
        g = s.fn(nm + '/g', EXP(Q(M(v(-1), S(M(dy, dy), M(dz, dz))), R2)), 'slipstream profile at this station')

        def vortex(side):
            vy = s.fn(nm + '/v%s-dy' % side, D(v(ys), W['v%s-y-ft' % side]))
            vz = s.fn(nm + '/v%s-dz' % side, D(v(zs), W['v%s-z-ft' % side]))
            r2 = S(M(vy, vy), M(vz, vz), v(0.01))
            k = s.fn(nm + '/v%s-k' % side, Q(M(v(1 / (2 * np.pi)), W['gamma-ft2_sec'],
                     D(v(1), EXP(Q(M(v(-1), r2), S(M(W['core-ft'], W['core-ft']), v(1e-3)))))), r2),
                     'vortex induced speed / distance (1/s)')
            return vy, vz, k
        ry_, rz_, kr = vortex('r'); ly_, lz_, kl = vortex('l')
        rr = S(W['r-ft'], v(1e-3))
        ax = s.fn(nm + '/ax', M(v(-1), g, S(W['u-fps'], W['turb-u-fps'])))
        ay = s.fn(nm + '/ay', S(M(g, S(Q(M(W['swirl-fps'], dz), rr), W['turb-v-fps'])),
                                M(v(-1), kr, rz_), M(kl, lz_)))
        az = s.fn(nm + '/az-up', S(M(g, S(Q(M(v(-1), W['swirl-fps'], dy), rr), W['turb-w-fps'],
                                          Q(M(W['turb-roll-fps'], dy), rr))),
                                   M(kr, ry_), M(v(-1), kl, ly_)))
        s.wcache[key] = (ax, ay, az)
        return s.wcache[key]

    def vel(s, xs, ys, zs):
        """local air velocity (u,v,w) body axes at structural point (ft), including the wake of an aircraft ahead"""
        rx = D(s.cgx, v(xs)); ry = D(v(ys), s.cgy); rz = D(s.cgz, v(zs))
        ax, ay, az = s.wash(ys, zs)
        u = S(s.U, M(s.Qq, rz), M(v(-1), s.Rr, ry), M(v(-1), ax))
        vv = S(s.V, M(s.Rr, rx), M(v(-1), s.Pp, rz), M(v(-1), ay))
        w = S(s.W, M(s.Pp, ry), M(v(-1), s.Qq, rx), az)
        return u, vv, w

    def arms(s, xs, ys, zs):
        return D(s.arx, v(xs)), D(v(ys), s.ary), D(s.arz, v(zs))

    def add_force(s, tagn, xs, ys, zs, Fx, Fy, Fz, Mx=None, My=None, Mz=None):
        """register body forces (property strings or None) acting at structural point (ft)."""
        rx, ry, rz = s.arms(xs, ys, zs)
        if Fx: s.parts['X'].append(Fx)
        if Fy: s.parts['Y'].append(Fy)
        if Fz: s.parts['Z'].append(Fz)
        L = []; Mm = []; N = []
        if Fz: L.append(M(ry, Fz)); Mm.append(M(v(-1), rx, Fz))
        if Fy: L.append(M(v(-1), rz, Fy)); N.append(M(rx, Fy))
        if Fx: Mm.append(M(rz, Fx)); N.append(M(v(-1), ry, Fx))
        if Mx: L.append(Mx)
        if My: Mm.append(My)
        if Mz: N.append(Mz)
        for ax, lst in (('L', L), ('Mm', Mm), ('N', N)):
            if lst:
                s.parts[ax].append(s.fn('aero/v9/%s/%s' % ({'L': 'roll', 'Mm': 'pitch', 'N': 'yaw'}[ax], tagn), S(*lst) if len(lst) > 1 else lst[0]))

    # ------------------------------------------------------------- controls
    def controls(s):
        s.ailL = s.fn('aero/v9/ail-left-deg', T1('fcs/left-aileron-pos-norm', [-1, 0, 1], [-26.5, 0, 10.8]),
                      'left aileron, TE down +  (rigging 110/45 mm at R 240: 26.5 up / 10.8 down)')
        s.ailR = s.fn('aero/v9/ail-right-deg', T1('fcs/right-aileron-pos-norm', [-1, 0, 1], [10.8, 0, -26.5]),
                      'right aileron, TE down +')
        e0 = s.P['elev0']
        s.elev = s.fn('aero/v9/elev-deg', T1('fcs/elevator-pos-norm', [-1, 0, 1], [-G.ELEV_UP, e0, G.ELEV_DN]),
                      'elevator, TE down +  (rigging 90/65 mm at R 230: 22.6 up / 16.2 down; stick centred, trim neutral = %.1f deg)' % e0)
        s.rudf = s.fn('aero/v9/rud-fin-deg', M(p('fcs/rudder-pos-norm'), v(-G.RUD_MAX)),
                      'rudder as a fin flap (+ = force to the left); JSBSim rudder + = TE left')
        s.sb = p('fcs/speedbrake-pos-norm')

    # ------------------------------------------------------------- wing
    def wing(s, T):
        P = s.P
        rows = LL.wing_layout(); n = len(rows)
        f = LL.f_matrix(rows)
        c = np.array([r['c'] for r in rows])
        F = f * (0.5 * c)[None, :] * P['k_ind']
        a = P['a_sec']
        Kf = np.linalg.solve(np.eye(n) + a * F, F)          # alpha_i = Kf @ (cl_geo * V_j) / V_k
        gam = np.radians(G.DIHEDRAL)
        s.wrows = rows
        sb = s.sb
        info = []
        for k, r in enumerate(rows):
            side = r['side']; nm = 'aero/v9/w%s%d' % ('L' if side < 0 else 'R', r['i'])
            xs34 = (r['x_c34'] + G.XS0) * FT; xs4 = (r['x_c4'] + G.XS0) * FT
            ys = r['ys'] * FT; zs = (r['z'] + G.ZS0) * FT
            ny, nz = side * np.sin(gam), np.cos(gam)
            if r['fus']: ny, nz = 0.0, 1.0
            u, vv, w = s.vel(xs34, ys, zs)
            U = s.fn(nm + '/u', u)
            Wn = s.fn(nm + '/wn', S(M(vv, v(ny)), M(w, v(nz))) if ny else M(w, v(nz)))
            V2 = s.fn(nm + '/V2', S(M(U, U), M(Wn, Wn), v(1e-4)))
            Vl = s.fn(nm + '/V', SQ(V2))
            phi = s.fn(nm + '/phi', ATAN2(Wn, U))
            ag = s.fn(nm + '/ag', M(S(phi, v(np.radians(r['inc']))), v(R2D)), 'geometric section alpha (deg)')
            # Kutta-Joukowski: the section force is normal to the local velocity at the bound vortex (c/4); the
            # 3/4-chord point only sets the lift magnitude (it differs from c/4 by q c/2 under a pitch rate)
            u4, v4, w4 = s.vel(xs4, ys, zs)
            phi4 = s.fn(nm + '/phi4', ATAN2(S(M(v4, v(ny)), M(w4, v(nz))) if ny else M(w4, v(nz)), u4), 'flow angle at c/4 (rad)')
            # section tables
            if r['ail']:
                key = 'wout%d' % (1 + min(3, int(r['frac'] / 0.25)))
                if r['frac'] > 0.9: key = 'wout4'
                tb = T[key]; dvar = 'aero/v9/ail-left-deg' if side < 0 else 'aero/v9/ail-right-deg'
                def tab(arr, var, tb=tb, dvar=dvar):
                    return T2(var, dvar, tb['alpha'], tb['delta'], arr)
                j0 = list(tb['delta']).index(0.0)
            else:
                if r['frac'] < 0.05: key = 'wroot'
                else: key = 'wout%d' % (1 + min(3, int(r['frac'] / 0.25)))
                tb = T[key]; j0 = list(tb['delta']).index(0.0)
                def tab(arr, var, tb=tb, j0=j0):
                    return T1(var, tb['alpha'], arr[:, j0])
            cn = P['cn90_scale']
            post = np.clip((np.abs(tb['alpha']) - 25) / 20, 0, 1)[:, None]
            clT = tb['cl'] * (1 + (cn - 1) * post); cmT = tb['cm'] * (1 + (cn - 1) * post)
            kcd = 0.0 if r['fus'] else P['k_cd']
            pre = 1 - np.clip((np.abs(tb['alpha']) - 15) / 10, 0, 1)[:, None]
            cdT = tb['cd'] * (kcd * pre + (1 - pre) * (cn if not r['fus'] else 1.0))
            clg = s.fn(nm + '/clg', tab(clT, nm + '/ag'))
            s.fn(nm + '/g', M(clg, Vl))
            info.append(dict(nm=nm, U=U, Wn=Wn, V2=V2, Vl=Vl, phi=phi, phi4=phi4, ag=ag, tab=tab, clT=clT, cdT=cdT, cmT=cmT,
                             xs4=xs4, ys=ys, zs=zs, ny=ny, nz=nz, r=r))
        fade = T1('aero/v9/dummy', [0], [1])
        for k, d in enumerate(info):
            terms = [M(v(Kf[k, j]), p(info[j]['nm'] + '/g')) for j in range(n) if abs(Kf[k, j]) > 1e-7]
            fadeT = T1(d['nm'] + '/ag', [-180, -70, -45, -25, 25, 45, 70, 180], [0, 0, 0.5, 1, 1, 0.5, 0, 0])
            d['ai'] = s.fn(d['nm'] + '/ai', M(Q(S(*terms), d['Vl']), s.ge, fadeT), 'induced angle (rad), explicit lifting line')
        for k, d in enumerate(info):
            nm, r = d['nm'], d['r']
            al = s.fn(nm + '/a', D(d['ag'], M(d['ai'], v(R2D))), 'effective section alpha (deg)')
            cl = d['tab'](d['clT'], nm + '/a'); cd = d['tab'](d['cdT'], nm + '/a'); cm = d['tab'](d['cmT'], nm + '/a')
            if r['brake'] > 0:
                fa = T1(nm + '/a', [-180, -20, -8, 12, 20, 40, 180], [0, 0.3, 1, 1, 0.5, 0.2, 0])
                fd = T1(nm + '/a', [-180, -30, -10, 15, 30, 60, 180], [0.3, 0.6, 1, 1, 0.6, 0.3, 0.3])
                kb = r['brake']
                cl = S(cl, M(v(-s.P['sb_dcl'] * kb), sb, fa)); cd = S(cd, M(v(s.P['sb_dcd'] * kb), sb, fd))
                cm = S(cm, M(v(s.P['sb_dcm'] * kb), sb, fa))
            fre = s.fn(nm + '/fRe', tag('pow', MAX(Q(M(s.RHO, d['Vl']), v(0.0023769 * 19.0 * FT)), v(0.2)), v(s.P['re_exp'])), 'profile drag Reynolds factor')
            pre = T1(nm + '/a', [-180, -25, -15, 15, 25, 180], [0, 0, 1, 1, 0, 0])
            cd = M(cd, S(v(1.0), M(D(fre, v(1.0)), pre)))
            cl = s.fn(nm + '/cl', cl); cd = s.fn(nm + '/cd', cd); cm = s.fn(nm + '/cm', cm)
            phie = s.fn(nm + '/phie', D(d['phi4'], d['ai']))
            qA = s.fn(nm + '/qA', M(v(0.5 * r['area'] * FT * FT), s.RHO, d['V2']))
            Fx = s.fn(nm + '/Fx', M(qA, D(M(cl, SIN(phie)), M(cd, COS(phie)))))
            Fn = s.fn(nm + '/Fn', M(qA, v(-1), S(M(cl, COS(phie)), M(cd, SIN(phie)))))
            Ms = s.fn(nm + '/Msec', M(qA, v(r['c'] * FT), cm))
            Fy = s.fn(nm + '/Fy', M(Fn, v(d['ny']))) if d['ny'] else None
            Fz = s.fn(nm + '/Fz', M(Fn, v(d['nz'])))
            # section moment vector = Msec * (n x xhat) = (0, nz, -ny) * Msec
            s.add_force(nm[8:], d['xs4'], d['ys'], d['zs'], Fx, Fy, Fz, None,
                        M(Ms, v(d['nz'])), (M(Ms, v(-d['ny'])) if d['ny'] else None))
        # wing lift coefficient (for downwash), inner-wing section drag (for the wake)
        terms = [M(p(d['nm'] + '/cl'), d['V2'], v(d['r']['area'] / G.S_REF)) for d in info]
        s.CLw = s.fn('aero/v9/CLw', Q(S(*terms), s.V2), 'wing lift coefficient (strip sum)')
        s.fn('aero/force/Lift_alpha', M(v(-1), S(*[p(d['nm'] + '/Fz') for d in info])), 'wing lift (lbf), read by Systems/jsb_wingflex.xml')
        inner = [d for d in info if d['r']['i'] in (1, 2, 3)]
        s.cdw = s.fn('aero/v9/cd-inner', M(v(1.0 / len(inner)), S(*[p(d['nm'] + '/cd') for d in inner])), 'inner wing section drag (wake)')
        s.winfo = info

    # ------------------------------------------------------------- tailplane
    def tailplane(s, T):
        P = s.P; tb = T['ht']; hts = G.ht_strips()
        L = LL.compute(verbose=False)
        dh = L['ht_dh']; Dm = L['ht_eps_per_CL']
        # wake centre line: leaves the root TE (x 1.5, z -0.065) along alpha - eps (eps ~ 0.28 (alpha + 7 deg))
        x_te, z_te = 1.50, -0.065
        al = p('aero/alpha-rad')
        ae = M(v(1 - 0.28 * P['eps_k']), S(al, v(-np.radians(7.0 * 0.28 / (1 - 0.28)))))   # alpha - eps, eps = 0.28 (alpha + 7 deg)
        tan_ae = Q(SIN(ae), MAX(COS(ae), v(0.2)))
        s.tan_wake = s.fn('aero/v9/wake-tan', tan_ae, 'slope of the wing wake relative to body x (up +)')
        cd = MAX(s.cdw, v(0.004))
        info = []; ARt = G.HT_B ** 2 / G.HT_S
        alphadot = p('aero/alphadot-rad_sec')
        for side in (-1, 1):
            for h in hts:
                nm = 'aero/v9/h%s%d' % ('L' if side < 0 else 'R', h['i'])
                xs34 = (h['x_c34'] + G.XS0) * FT; xs4 = (h['x_c4'] + G.XS0) * FT
                ys = side * h['y'] * FT; zs = (h['z'] + G.ZS0) * FT
                dist = h['x_c4'] - x_te
                hgt = s.fn(nm + '/hw', D(v(h['z'] - z_te), M(v(dist), s.tan_wake)), 'height above the wing wake (m)')
                eps_tab = T1(nm + '/hw', dh, Dm[h['i']] * P['eps_k'])
                tau = dist * FT
                eps = s.fn(nm + '/eps', D(M(eps_tab, s.CLw, s.get), M(v(0.28 * P['eps_k'] * tau), Q(alphadot, MAX(s.Vt, v(10))))), 'downwash (rad), lag lt/V')
                xc = dist / 1.45
                dq0 = 2.42 / (xc + 0.3); bw = 0.68 * 1.45 * np.sqrt(xc + 0.15)
                # gaussian wake: deficit = dq0*sqrt(cd)*exp(-0.69 (h/(bw sqrt cd))^2)
                defi = M(v(dq0 * P['wake_k']), SQ(cd), tag('exp', M(v(-0.69 / bw ** 2), Q(M(hgt, hgt), cd))))
                eta = s.fn(nm + '/eta', MAX(D(v(1.0), defi), v(0.15)), 'dynamic pressure ratio (wing wake)')
                u, vv, w = s.vel(xs34, ys, zs)
                U = s.fn(nm + '/u', u)
                Wn = s.fn(nm + '/wn', D(w, M(eps, s.Vt)))
                V2 = s.fn(nm + '/V2', M(S(M(U, U), M(Wn, Wn), v(1e-4)), eta))
                phi = s.fn(nm + '/phi', ATAN2(Wn, U))
                ag = s.fn(nm + '/ag', M(S(phi, v(np.radians(G.HT_INC))), v(R2D)))
                u4, v4, w4 = s.vel(xs4, ys, zs)
                phi4 = s.fn(nm + '/phi4', ATAN2(D(w4, M(eps, s.Vt)), u4), 'flow angle at c/4 (rad), force direction')
                tab = lambda arr, var: T2(var, 'aero/v9/elev-deg', tb['alpha'], tb['delta'], arr)
                clg = s.fn(nm + '/clg', tab(tb['cl'], nm + '/ag'))
                info.append(dict(nm=nm, U=U, Wn=Wn, V2=V2, phi=phi, phi4=phi4, ag=ag, clg=clg, tab=tab, xs4=xs4, ys=ys, zs=zs, h=h))
        CLg = s.fn('aero/v9/ht-CLgeo', S(*[M(d['clg'], v(d['h']['area'] / G.HT_S)) for d in info]))
        ai = s.fn('aero/v9/ht-ai', M(Q(CLg, v(np.pi * ARt * P['ht_e'] + 6.7)),
                                    T1('aero/v9/hL0/ag', [-180, -70, -45, -25, 25, 45, 70, 180], [0, 0, 0.5, 1, 1, 0.5, 0, 0])), 'tailplane induced angle (rad)')
        for d in info:
            nm, h = d['nm'], d['h']
            al_ = s.fn(nm + '/a', D(d['ag'], M(ai, v(R2D))))
            cl = s.fn(nm + '/cl', d['tab'](tb['cl'], nm + '/a')); cd_ = s.fn(nm + '/cd', d['tab'](tb['cd'], nm + '/a'))
            cm = s.fn(nm + '/cm', d['tab'](tb['cm'], nm + '/a'))
            phie = s.fn(nm + '/phie', D(d['phi4'], ai))
            qA = s.fn(nm + '/qA', M(v(0.5 * h['area'] * FT * FT), s.RHO, d['V2']))
            Fx = s.fn(nm + '/Fx', M(qA, D(M(cl, SIN(phie)), M(cd_, COS(phie)))))
            Fz = s.fn(nm + '/Fz', M(qA, v(-1), S(M(cl, COS(phie)), M(cd_, SIN(phie)))))
            Ms = s.fn(nm + '/Msec', M(qA, v(h['c'] * FT), cm))
            s.add_force(nm[8:], d['xs4'], d['ys'], d['zs'], Fx, None, Fz, None, Ms, None)
        s.CLt = s.fn('aero/v9/ht-CL', S(*[M(p(d['nm'] + '/cl'), v(d['h']['area'] / G.HT_S)) for d in info]))

    # ------------------------------------------------------------- fin
    def fin(s, T):
        P = s.P; tb = T['vt']; vts = G.vt_strips()
        info = []
        cd = MAX(s.cdw, v(0.004))
        x_te, z_te = 1.50, -0.065
        for st in vts:
            nm = 'aero/v9/v%d' % st['i']
            xs34 = (st['x_c34'] + G.XS0) * FT; xs4 = (st['x_c4'] + G.XS0) * FT
            zs = (st['z'] + G.ZS0) * FT
            dist = st['x_c4'] - x_te
            hgt = s.fn(nm + '/hw', D(v(st['z'] - z_te), M(v(dist), s.tan_wake)))
            xc = dist / 1.45; dq0 = 2.42 / (xc + 0.3); bw = 0.68 * 1.45 * np.sqrt(xc + 0.15)
            defi = M(v(dq0 * P['wake_kv']), SQ(cd), tag('exp', M(v(-0.69 / bw ** 2), Q(M(hgt, hgt), cd))))
            u, vv, w = s.vel(xs34, 0.0, zs)
            U = s.fn(nm + '/u', u); Vn = s.fn(nm + '/vn', vv)
            bf = M(ABS(ATAN2(Vn, ABS(U))), v(R2D))
            b0, b1, kb = P['fin_blank_b0'], P['fin_blank_b1'], P['fin_blank']
            bl = T1('aero/v9/dummyb', [0], [1]) if kb == 0 else None
            blank = tag('table', '<independentVar lookup="row">%s/bf</independentVar><tableData>\n0 1\n%g 1\n%g %g\n90 %g\n</tableData>' % (nm, b0, b1, 1 - kb, 1 - kb))
            s.fn(nm + '/bf', bf, 'fin local sideslip (deg)')
            eta = s.fn(nm + '/eta', M(MAX(D(v(1.0), defi), v(0.15)), blank))
            V2 = s.fn(nm + '/V2', M(S(M(U, U), M(Vn, Vn), v(1e-4)), eta))
            phi = s.fn(nm + '/phi', ATAN2(Vn, U))
            ag = s.fn(nm + '/ag', M(phi, v(R2D)))
            u4, v4, w4 = s.vel(xs4, 0.0, zs)
            phi4 = s.fn(nm + '/phi4', ATAN2(v4, u4), 'flow angle at c/4 (rad), force direction')
            tab = lambda arr, var: T2(var, 'aero/v9/rud-fin-deg', tb['alpha'], tb['delta'], arr)
            clg = s.fn(nm + '/clg', tab(tb['cl'], nm + '/ag'))
            info.append(dict(nm=nm, U=U, V2=V2, phi=phi, phi4=phi4, ag=ag, clg=clg, tab=tab, xs4=xs4, zs=zs, st=st))
        CYg = s.fn('aero/v9/vt-CLgeo', S(*[M(d['clg'], v(d['st']['area'] / G.VT_S)) for d in info]))
        ai = s.fn('aero/v9/vt-ai', M(Q(CYg, v(np.pi * P['vt_ar'] * P['vt_e'] + 6.7)),
                                    T1('aero/v9/v1/ag', [-180, -70, -45, -30, 30, 45, 70, 180], [0, 0, 0.5, 1, 1, 0.5, 0, 0])), 'fin induced angle (rad)')
        cs = P['vt_cn_scale']; post = np.clip((np.abs(tb['alpha']) - 25) / 20, 0, 1)[:, None]
        for d in info:
            nm, st = d['nm'], d['st']
            s.fn(nm + '/a', D(d['ag'], M(ai, v(R2D))))
            cl = s.fn(nm + '/cl', d['tab'](tb['cl'] * (1 + (cs - 1) * post), nm + '/a'))
            cd_ = s.fn(nm + '/cd', d['tab'](tb['cd'] * (1 + (cs - 1) * post), nm + '/a'))
            phie = s.fn(nm + '/phie', D(d['phi4'], ai))
            qA = s.fn(nm + '/qA', M(v(0.5 * st['area'] * FT * FT), s.RHO, d['V2']))
            Fx = s.fn(nm + '/Fx', M(qA, D(M(cl, SIN(phie)), M(cd_, COS(phie)))))
            Fy = s.fn(nm + '/Fy', M(qA, v(-1), S(M(cl, COS(phie)), M(cd_, SIN(phie)))))
            s.add_force(nm[8:], d['xs4'], 0.0, d['zs'], Fx, Fy, None)
        s.CYv = s.fn('aero/v9/vt-CL', S(*[M(p(d['nm'] + '/cl'), v(d['st']['area'] / G.VT_S)) for d in info]))

    # ------------------------------------------------------------- fuselage
    def fuselage(s):
        P = s.P; Fu = G.fuselage(); L = LL.compute(verbose=False); Uw = L['fus_upwash_per_CL']
        xs_, hs, ws = Fu['xs'], Fu['hs'], Fu['ws']
        Sv = np.pi * ws ** 2 / 4; Sl = np.pi * hs ** 2 / 4
        xmax = xs_[np.argmax(Fu['A'])]
        for st, up in zip(Fu['stations'], Uw):
            nm = 'aero/v9/f%d' % st['i']
            x0, x1 = st['x'] - st['dx'] / 2, st['x'] + st['dx'] / 2
            dSv = np.interp(x1, xs_, Sv) - np.interp(x0, xs_, Sv)
            dSl = np.interp(x1, xs_, Sl) - np.interp(x0, xs_, Sl)
            kp = P['k2k1'] * (1.0 if st['x'] < xmax else P['kpot_aft'])
            xs = (st['x'] + G.XS0) * FT; zs = (st['z'] + G.ZS0) * FT
            u, vv, w = s.vel(xs, 0.0, zs)
            U = s.fn(nm + '/u', u); Vv = s.fn(nm + '/v', vv)
            Wv = s.fn(nm + '/w', S(w, M(v(up), s.CLw, s.Vt))) if abs(up) > 1e-6 else s.fn(nm + '/w', w)
            Vc = s.fn(nm + '/Vc', SQ(S(M(Vv, Vv), M(Wv, Wv), v(1e-4))))
            # Munk (potential) load per station: rho u w dS/dx, i.e. q sin(2 alpha_t) resolved into the cross-flow plane;
            # the sideslip component must not enter the vertical term (and vice versa)
            Fz = s.fn(nm + '/Fz', S(M(v(-kp * dSv * FT * FT), s.RHO, U, Wv),
                                     M(v(-0.5 * P['cdc'] * st['plan'] * FT * FT), s.RHO, Wv, Vc)))
            Fy = s.fn(nm + '/Fy', S(M(v(-kp * dSl * FT * FT), s.RHO, U, Vv),
                                     M(v(-0.5 * P['cdc'] * st['side'] * FT * FT), s.RHO, Vv, Vc)))
            s.add_force(nm[8:], xs, 0.0, zs, None, Fy, Fz)
        # zero-lift drag of fuselage + gear + misc, along the relative wind
        D0 = (P['cf_fus'] * P['ff_fus'] * P['wet_fus'] + P['gear_DS'] + P['cd_misc'] * G.S_REF) * FT * FT
        s.D0 = D0
        kx = s.fn('aero/v9/d0/k', M(v(-0.5 * D0), s.RHO, s.Vt), 'fuselage+gear+misc zero-lift drag: F = k * (u,v,w)')
        xs = (0.3 + G.XS0) * FT; zs = (-0.15 + G.ZS0) * FT
        s.add_force('d0', xs, 0.0, zs, s.fn('aero/v9/d0/Fx', M(kx, s.U)), s.fn('aero/v9/d0/Fy', M(kx, s.V)), s.fn('aero/v9/d0/Fz', M(kx, s.W)))

    # ------------------------------------------------------------- output
    def build(s):
        T = AF.build_all(wing_clmax_k=s.P['wing_clmax_k'], tip_clmax_k=s.P['tip_clmax_k'], a_peak_root=s.P['a_peak_root'], a_peak_tip=s.P['a_peak_tip'], drop=s.P['stall_drop'], CN90_wing=s.P['CN90_wing'], flap_eff=s.P['flap_eff'], k_sep=s.P['k_sep'], boff=s.P['boff'], dstall=s.P['dstall'], xcp90=s.P['xcp90'], CN90_ht=s.P['CN90_ht'], CN90_vt=s.P['CN90_vt'], w_ca=s.P['w_ca'], ca_sep=s.P['ca_sep'])
        s.kin(); s.controls(); s.wing(T); s.tailplane(T); s.fin(T); s.fuselage()
        ax = []
        names = dict(X='X', Y='Y', Z='Z', L='ROLL', Mm='PITCH', N='YAW')
        tot = {}
        for k in ('X', 'Y', 'Z', 'L', 'Mm', 'N'):
            tot[k] = s.fn('aero/v9/total/%s' % k, S(*s.parts[k]))
        for k in ('X', 'Y', 'Z', 'L', 'Mm', 'N'):
            ax.append('<axis name="%s"><function name="aero/v9/axis/%s">%s</function></axis>' % (names[k], k, tot[k]))
        head = '<?xml version="1.0"?>\n<!-- ASK 21 aerodynamics v9: component build-up generated by genxml.py; parameters: %s -->\n' % json.dumps(s.P)
        return head + '<aerodynamics>\n' + '\n'.join(s.f) + '\n' + '\n'.join(ax) + '\n</aerodynamics>\n'


def write(path, P=None):
    g = Gen(P); xml = g.build()
    open(path, 'w').write(xml)
    return g


if __name__ == '__main__':
    out = sys.argv[1] if len(sys.argv) > 1 else 'ask21-aero-v9.xml'
    P = json.loads(sys.argv[2]) if len(sys.argv) > 2 else None
    g = write(out, P)
    print('wrote', out, '%.0f kB' % (os.path.getsize(out) / 1024), 'functions', len(g.f), 'D0 ft2 %.3f' % g.D0)
