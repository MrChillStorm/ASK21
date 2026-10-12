"""Section tables (cl, cd, cm about c/4) over -180..180 deg for the ASK 21 airfoils.

Pre-stall: NeuralFoil (xxlarge) on the UIUC coordinates, flap deflections applied to the geometry.
Post-stall: smooth blend into a flat-plate model  cn = CN90*sin(a + k*delta),  centre of pressure
moving from c/4 (small a) to c/2 (90 deg) to 3c/4 (reverse flow).
"""
import numpy as np, aerosandbox as asb, neuralfoil as nf, os, pickle, warnings
warnings.filterwarnings('ignore')
HERE = os.path.dirname(os.path.abspath(__file__))

ALPHA = np.r_[np.arange(-180, -40, 10.0), np.arange(-40, -25, 3.0), np.arange(-25, 30, 1.0),
              np.arange(30, 45, 3.0), np.arange(45, 181, 10.0)]
ALPHA = np.unique(np.r_[ALPHA, -180, 180])


def smooth(x):
    x = np.clip(x, 0, 1)
    return x * x * (3 - 2 * x)



def cap_clmax(an, cl, k, a_peak=None, drop=0.22, w=5.0, knee=0.80):
    """trailing-edge stall shape: NeuralFoil below knee*clmax_t, then a smooth monotone rise
    cl = ct - (ct-ck)(1-t)^n to the peak ct = k*clmax_NF at a_peak (zero slope), then an exponential
    lift drop of `drop` over `w` deg (the flat-plate blend follows further out)."""
    ip = np.where((an >= 0) & (an <= 29))[0]
    ct = k * cl[ip].max()
    i_k = [i for i in ip if cl[i] >= knee * ct][0]
    ak, ck = an[i_k], cl[i_k]
    sk = (cl[i_k] - cl[i_k - 2]) / (an[i_k] - an[i_k - 2])
    if a_peak is None: a_peak = ak + 2.0 * (ct - ck) / sk
    h = a_peak - ak; n = max(sk * h / (ct - ck), 1.5)
    out = cl.copy()
    for i in range(i_k, len(an)):
        x = an[i]
        if x <= a_peak:
            t = (x - ak) / h; out[i] = ct - (ct - ck) * (1 - t) ** n
        else:
            out[i] = ct - drop * (1 - np.exp(-(x - a_peak) / w))
    return out


def section(af_name, Re, hinge=None, deltas=(0.0,), blend=None, CN90=1.30, kflap=0.35, dstall=15.0, ncrit=9.0, dref=12.0, clmax_k=1.0, boff=8.0, a_peak=None, drop=0.22, flap_eff=1.0, k_sep=0.10, xcp90=0.5, w_ca=6.0, ca_sep=0.02):
    """Return dict(alpha, delta, cl[a,d], cd[a,d], cm[a,d], meta)."""
    base = asb.Airfoil(af_name)
    if blend is not None:
        base = base.blend_with_another_airfoil(asb.Airfoil(blend[0]), blend[1])
    an = np.arange(-25, 30.01, 0.5)
    CL = np.zeros((len(ALPHA), len(deltas))); CD = CL.copy(); CM = CL.copy(); meta = []
    def nfrun(d):
        af = base if (hinge is None or d == 0) else base.add_control_surface(deflection=d, hinge_point_x=hinge)
        r = nf.get_aero_from_airfoil(af, alpha=an, Re=Re, model_size='xxlarge', n_crit=ncrit)
        return np.asarray(r['CL']), np.asarray(r['CD']), np.asarray(r['CM'])
    clean = nfrun(0.0)
    for j, d in enumerate(deltas):
        if abs(d) <= dref + 1e-9:
            cl, cd, cm = nfrun(d)
        else:
            # NeuralFoil loses the lift slope at large deflections: use the increment curve of the
            # reference deflection (+-dref), scaled to NeuralFoil's increment at alpha = 0
            sgn = np.sign(d); clr, cdr, cmr = nfrun(sgn * dref); cld, cdd, cmd = nfrun(d)
            i0 = np.argmin(abs(an))
            sc = (cld[i0] - clean[0][i0]) / (clr[i0] - clean[0][i0])
            cl = clean[0] + sc * (clr - clean[0]); cm = clean[2] + sc * (cmr - clean[2])
            cd = clean[1] + sc ** 2 * (cdr - clean[1]) + max(0.0, (cdd[i0] - clean[1][i0]) - sc ** 2 * (cdr[i0] - clean[1][i0]))
        if flap_eff != 1.0 and d != 0.0:
            cl = clean[0] + flap_eff * (cl - clean[0]); cm = clean[2] + flap_eff * (cm - clean[2])
        if clmax_k < 1.0:
            ap_d = None if a_peak is None else a_peak - 0.55 * d   # TE-down flap stalls earlier
            cl_nf = cl.copy()
            cl = cap_clmax(an, cl, clmax_k, a_peak=ap_d, drop=drop)
            # lift removed by (earlier) separation shows up as pressure drag
            cd = cd + k_sep * np.clip(cl_nf - cl, 0, None) * np.clip(np.sin(np.radians(an)) * 4, 0, 1)
        # stall points
        ip = np.where((an >= 4) & (an <= 28))[0]; ap = an[ip[np.argmax(cl[ip])]]
        im = np.where((an <= -4) & (an >= -24))[0]; am = an[im[np.argmin(cl[im])]]
        cn_fp = lambda a: CN90 * np.sin(np.radians(a + kflap * d * np.cos(np.radians(a)) ** 2))
        def fp(a):
            cn = cn_fp(a); ar = np.radians(a)
            aa = np.abs(a)
            xcp = 0.25 + (xcp90 - 0.25) * aa / 90.0 if aa <= 90 else xcp90 + (0.75 - xcp90) * (aa - 90) / 90.0
            return cn * np.cos(ar), 0.012 + cn * np.sin(ar), -cn * (xcp - 0.25)
        def nfv(a):
            ac = np.clip(a, -25, 29.5)
            return np.interp(ac, an, cl), np.interp(ac, an, cd), np.interp(ac, an, cm)
        for i, a in enumerate(ALPHA):
            c1, d1, m1 = nfv(a)
            c2, d2, m2 = fp(a)
            if a >= ap + boff:
                w = smooth((a - ap - boff) / dstall)
            elif a <= am - boff:
                w = smooth((am - boff - a) / dstall)
            else:
                w = 0.0
            CL[i, j] = (1 - w) * c1 + w * c2
            CD[i, j] = (1 - w) * d1 + w * max(d2, d1)
            CM[i, j] = (1 - w) * m1 + w * m2
        # stalled flow carries no leading-edge suction: the chordwise force CA = cd cos a - cl sin a goes from its
        # value at the lift peak to ca_sep (aft) within w_ca deg after the stall; the normal force is kept.
        cl_j, cd_j = CL[:, j].copy(), CD[:, j].copy()
        r = np.radians(ALPHA); CN_ = cl_j * np.cos(r) + cd_j * np.sin(r); CA_ = cd_j * np.cos(r) - cl_j * np.sin(r)
        ipos = (ALPHA >= 0) & (ALPHA <= 40); ipk = np.where(ipos)[0][np.argmax(cl_j[ipos])]; apk = ALPHA[ipk]; capk = CA_[ipk]
        ineg = (ALPHA <= 0) & (ALPHA >= -40); ink = np.where(ineg)[0][np.argmin(cl_j[ineg])]; ank = ALPHA[ink]; cank = CA_[ink]
        for i, a in enumerate(ALPHA):
            if apk < a <= 90:
                w = smooth((a - apk) / w_ca); CA_[i] = (1 - w) * (capk + (CA_[i] - capk) * 0) + w * max(CA_[i], ca_sep * np.cos(r[i]))
            elif -90 <= a < ank:
                w = smooth((ank - a) / w_ca); CA_[i] = (1 - w) * cank + w * max(CA_[i], ca_sep * np.cos(r[i]))
        CL[:, j] = CN_ * np.cos(r) - CA_ * np.sin(r); CD[:, j] = CN_ * np.sin(r) + CA_ * np.cos(r)
        a0 = np.interp(0, cl[(an > -10) & (an < 6)], an[(an > -10) & (an < 6)])
        meta.append(dict(delta=d, a0=a0, clmax=cl.max(), a_stall=ap, clmin=cl.min(), a_stall_neg=am,
                         cla=np.polyfit(an[(an >= -2) & (an <= 4)], cl[(an >= -2) & (an <= 4)], 1)[0] * 57.2958,
                         cm0=np.interp(0, an, cm), cdmin=cd[(an > -6) & (an < 10)].min()))
    return dict(alpha=ALPHA.copy(), delta=np.array(deltas, float), cl=CL, cd=CD, cm=CM, meta=meta, Re=Re)


def build_all(wing_clmax_k=0.90, tip_clmax_k=None, a_peak_root=14.0, a_peak_tip=14.0, drop=0.22, tail_clmax_k=1.0, CN90_wing=1.30,
              flap_eff=0.9, k_sep=0.10, boff=8.0, dstall=15.0, xcp90=0.5, CN90_ht=1.20, CN90_vt=1.15, w_ca=6.0, ca_sep=0.02, force=False):
    tip_clmax_k = wing_clmax_k if tip_clmax_k is None else tip_clmax_k
    key = 'sec_' + '_'.join('%.4g' % x for x in (wing_clmax_k, tip_clmax_k, a_peak_root, a_peak_tip, drop, tail_clmax_k, CN90_wing,
                                                 flap_eff, k_sep, boff, dstall, xcp90, CN90_ht, CN90_vt, w_ca, ca_sep)) + '.pkl'
    cache = os.path.join(HERE, 'cache', key)
    os.makedirs(os.path.dirname(cache), exist_ok=True)
    if os.path.exists(cache) and not force:
        try:
            return pickle.load(open(cache, 'rb'))
        except Exception:
            pass
    T = {}
    kw = dict(CN90=CN90_wing, drop=drop, k_sep=k_sep, boff=boff, dstall=dstall, xcp90=xcp90, w_ca=w_ca, ca_sep=ca_sep)
    # wing: FX S 02-196 inner panel (no flap), outer panel blends to FX 60-126 at the tip
    T['wroot'] = section('fxs02196', 1.8e6, clmax_k=wing_clmax_k, a_peak=a_peak_root, **kw)
    ail = (-27.0, -20.0, -13.0, -6.0, 0.0, 5.0, 11.0)          # TE down positive (rigging: 26.5 up / 10.8 down)
    for k, frac in ((1, 0.15), (2, 0.45), (3, 0.75), (4, 0.95)):
        Re = (1.6e6, 1.25e6, 0.95e6, 0.7e6)[k - 1]
        kk = wing_clmax_k + (tip_clmax_k - wing_clmax_k) * frac
        ap = a_peak_root + (a_peak_tip - a_peak_root) * frac
        T['wout%d' % k] = section('fxs02196', Re, hinge=0.76, deltas=ail, blend=('fx60126', frac), dref=13.0, clmax_k=kk,
                                  a_peak=ap, flap_eff=flap_eff, **kw)
    # tailplane FX 71-L-150/30, 30 % elevator (rigging 22.6 up / 16.2 down)
    de = (-24.0, -18.0, -12.0, -6.0, 0.0, 6.0, 12.0, 18.0)
    T['ht'] = section('fx711530', 0.9e6, hinge=0.70, deltas=de, CN90=CN90_ht, clmax_k=tail_clmax_k, flap_eff=flap_eff, xcp90=xcp90, w_ca=w_ca, ca_sep=ca_sep)
    # fin FX 71-L-150/30, 31 % rudder (+-27.8 deg)
    dr = (-30.0, -22.0, -15.0, -8.0, 0.0, 8.0, 15.0, 22.0, 30.0)
    T['vt'] = section('fx711530', 1.4e6, hinge=0.69, deltas=dr, CN90=CN90_vt, clmax_k=tail_clmax_k, flap_eff=flap_eff, xcp90=xcp90, w_ca=w_ca, ca_sep=ca_sep)
    tmp = cache + '.%d' % os.getpid()
    pickle.dump(T, open(tmp, 'wb')); os.replace(tmp, cache)
    return T


if __name__ == '__main__':
    T = build_all(force=True)
    for k, v in T.items():
        print(k, 'Re %.2g' % v['Re'])
        for m in v['meta']:
            print('   d %+5.1f  a0 %+5.2f  cla %.2f/rad  clmax %.2f @%4.1f  clmin %+.2f @%5.1f  cm0 %+.3f  cdmin %.5f' % (
                m['delta'], m['a0'], m['cla'], m['clmax'], m['a_stall'], m['clmin'], m['a_stall_neg'], m['cm0'], m['cdmin']))
