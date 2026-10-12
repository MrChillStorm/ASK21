"""Quasi-steady evaluation of the JSBSim model with frozen integrators; trim and polar tools."""
import numpy as np, harness as H
from scipy.optimize import fsolve

KT = 0.5144444; FPS = 0.3048


class Static:
    def __init__(s, aero, jsb=None, front=176.0, rear=176.0, tail=0.0, ckpt=0.0, h_ft=3000.0, tag=None):
        s.f = f = H.mk(aero=aero, jsb=jsb, front_lb=front, rear_lb=rear, tail_lb=tail, ckpt_lb=ckpt, tag=tag)
        try: f['fcs/stick-map-enable'] = 0
        except Exception: pass
        for k in ('simulation/integrator/rate/rotational', 'simulation/integrator/rate/translational',
                  'simulation/integrator/position/rotational', 'simulation/integrator/position/translational'):
            f[k] = 0
        f['ic/h-sl-ft'] = h_ft; f['ic/terrain-elevation-ft'] = -4000
        s.h = h_ft
        f['ic/vt-fps'] = 80; f['ic/alpha-deg'] = 2; f.run_ic(); f.run()
        s.W = f['inertia/weight-lbs']

    def eval(s, V_mps, alpha, beta=0.0, p=0.0, q=0.0, r=0.0, elev=0.0, ail=0.0, rud=0.0, sb=0.0, theta=None, phi=0.0, h_agl_ft=None):
        f = s.f
        f['fcs/elevator-cmd-norm'] = elev; f['fcs/aileron-cmd-norm'] = ail; f['fcs/rudder-cmd-norm'] = rud
        f['fcs/pitch-trim-cmd-norm'] = 0; f['fcs/roll-trim-cmd-norm'] = 0; f['fcs/yaw-trim-cmd-norm'] = 0
        f['/controls/engines/engine/throttle'] = 1.0 - 0.0 * sb
        f['ic/vt-fps'] = V_mps / FPS; f['ic/alpha-deg'] = alpha; f['ic/beta-deg'] = beta
        f['ic/theta-deg'] = alpha if theta is None else theta; f['ic/phi-deg'] = phi
        f['ic/p-rad_sec'] = p; f['ic/q-rad_sec'] = q; f['ic/r-rad_sec'] = r
        if h_agl_ft is not None:
            f['ic/terrain-elevation-ft'] = 0; f['ic/h-agl-ft'] = h_agl_ft
        else:
            f['ic/terrain-elevation-ft'] = -4000; f['ic/h-sl-ft'] = s.h
        f.run_ic()
        if sb:
            f['/controls/engines/engine/throttle'] = 1.0 - sb
        f.run(); f.run()
        def g(k):
            try: return f[k]
            except Exception: return float('nan')
        return dict(X=g('forces/fbx-aero-lbs'), Y=g('forces/fby-aero-lbs'), Z=g('forces/fbz-aero-lbs'),
                    L=g('moments/l-aero-lbsft'), M=g('moments/m-aero-lbsft'), N=g('moments/n-aero-lbsft'),
                    qbar=g('aero/qbar-psf'), alpha=g('aero/alpha-deg'), beta=g('aero/beta-deg'),
                    CLw=g('aero/v9/CLw'), CLt=g('aero/v9/ht-CL'), elev_deg=g('aero/v9/elev-deg'))

    def coeffs(s, V_mps, alpha, **kw):
        r = s.eval(V_mps, alpha, **kw)
        Sft = 193.21; b = 55.774; c = 1.121 / FPS
        qS = r['qbar'] * Sft
        a = np.radians(r['alpha'])
        bb = np.radians(r['beta'])
        L = -r['Z'] * np.cos(a) + r['X'] * np.sin(a)
        Dg = -(r['X'] * np.cos(a) * np.cos(bb) + r['Y'] * np.sin(bb) + r['Z'] * np.sin(a) * np.cos(bb))
        return dict(CL=L / qS, CD=Dg / qS, CY=r['Y'] / qS, Cl=r['L'] / (qS * b), Cm=r['M'] / (qS * c), Cn=r['N'] / (qS * b), raw=r)

    def trim_glide(s, V_mps, sb=0.0, guess=(4.0, -0.3, -2.0)):
        """steady straight glide at true airspeed V: unknowns alpha, elevator norm, gamma."""
        W = s.W
        def res(x):
            al, el, ga = x
            r = s.eval(V_mps, al, elev=el, theta=al + ga, sb=sb)
            th = np.radians(al + ga)
            return [(r['X'] - W * np.sin(th)) / W, (r['Z'] + W * np.cos(th)) / W, r['M'] / (W * 3.0)]
        sol, info, ier, msg = fsolve(res, guess, full_output=True)
        al, el, ga = sol
        V = V_mps
        return dict(alpha=al, elev=el, gamma=ga, sink=-V * np.sin(np.radians(ga)), ok=ier == 1 and np.abs(info['fvec']).max() < 1e-4,
                    resid=np.abs(info['fvec']).max())

    def cg_in(s):
        return H.cg_report_in(s.f)
