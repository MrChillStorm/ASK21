"""Discrete lifting-line influence data for the strip model.

Wing: strips with horseshoe vortices (bound at c/4, trailing legs at the strip edges, straight aft).
  downwash at strip k (velocity / V) = sum_j f_kj * Gamma_j / V_k,   Gamma_j = 0.5 * V_j * c_j * cl_j
Tail: downwash at the tailplane per unit wing CL as a function of the tail height above the wake plane.
Fuselage: up/downwash along the body per unit wing CL (Multhopp-type flow field for the body moment).
"""
import numpy as np, geometry as G


def wing_layout():
    W = G.wing_strips()
    scale = G.S_REF / (2 * sum(o['area'] for o in W))      # match the TCDS area
    rows = []
    for side in (-1, 1):
        for o in (W[::-1] if side < 0 else W):
            y0, y1 = (o['y0'], o['y1']) if side > 0 else (-o['y1'], -o['y0'])
            rows.append(dict(o, side=side, ya=y0, yb=y1, ys=side * o['y'], c=o['c'] * scale, area=o['area'] * scale))
    return rows


def f_matrix(rows):
    n = len(rows); f = np.zeros((n, n))
    for k, rk in enumerate(rows):
        yk = rk['ys']
        for j, rj in enumerate(rows):
            # horseshoe j: + circulation, trailing leg at yb (right) and -leg at ya (left)
            f[k, j] = (1 / (4 * np.pi)) * (1.0 / (rj['yb'] - yk) + 1.0 / (yk - rj['ya']))
    return f


def solve_linear(rows, f, alpha_geo_deg, cla=6.7):
    """steady linear lifting line: cl_k = cla*(a_geo_k - a_i_k), a_i = f @ (0.5 c cl)."""
    c = np.array([r['c'] for r in rows]); n = len(rows)
    A = np.eye(n) + cla * f * (0.5 * c)[None, :]
    cl = np.linalg.solve(A, cla * np.radians(alpha_geo_deg))
    ai = f @ (0.5 * c * cl)
    area = np.array([r['area'] for r in rows])
    CL = np.sum(cl * area) / G.S_REF
    return cl, ai, CL


def trailing_downwash(rows, gam, xp, yp, zp, x_bound, z_wake):
    """downwash (positive down, per V) at point from horseshoes: bound + trailing legs (straight aft at height z_wake)."""
    w = 0.0
    for r, g in zip(rows, gam):
        for ye, s in ((r['yb'], 1.0), (r['ya'], -1.0)):
            # semi-infinite vortex along +x starting at (x_bound, ye, z_wake), strength s*g
            dy = yp - ye; dz = zp - z_wake; dx = xp - x_bound
            rr = dy * dy + dz * dz
            if rr < 1e-6: continue
            # induced velocity of semi-infinite line: (G/4pi r) (1 + cos theta), direction about x axis
            fac = s * g / (4 * np.pi * rr) * (1 + dx / np.sqrt(dx * dx + rr))
            # velocity = fac * (x_hat cross r_perp) ; r_perp=(0,dy,dz): x_hat x (0,dy,dz) = (0,-dz,dy)
            # downwash positive down (-z): w_down = -(fac*dy)
            w += -fac * dy          # right leg (+) induces downwash inboard (dy<0)
        # bound vortex (finite segment along y at x_bound) - small at the tail, include for upwash ahead
        ya, yb = r['ya'], r['yb']
        dx = xp - x_bound; dz = zp - 0.0
        rho = np.hypot(dx, dz)
        if rho > 1e-6:
            c1 = (yb - yp) / np.sqrt((yb - yp) ** 2 + rho ** 2); c2 = (ya - yp) / np.sqrt((ya - yp) ** 2 + rho ** 2)
            vb = g / (4 * np.pi * rho) * (c1 - c2)
            # bound vortex along +y with circulation g (lift up): induces downwash behind (dx>0), upwash ahead
            w += vb * dx / rho
    return w


def compute(verbose=True):
    rows = wing_layout(); f = f_matrix(rows); n = len(rows)
    out = dict(rows=rows, f=f)
    # lift slope & span efficiency with section slope 6.7/rad (NF ~6.6-6.8)
    cl, ai, CL1 = solve_linear(rows, f, np.ones(n) * 1.0)
    cl0, _, _ = solve_linear(rows, f, np.zeros(n))
    CLa = CL1 / np.radians(1.0)
    area = np.array([r['area'] for r in rows]); c = np.array([r['c'] for r in rows]); y = np.array([r['ys'] for r in rows])
    CDi = np.sum(cl * area * np.radians(ai)) / G.S_REF if False else np.sum(cl * area * ai) / G.S_REF
    AR = G.B ** 2 / G.S_REF; e = CL1 ** 2 / (np.pi * AR * CDi)
    out.update(CLa=CLa, e=e)
    # fine reference (many strips) for comparison
    if verbose:
        print('strip lifting line: CLa %.3f /rad  e %.3f  (AR %.2f)' % (CLa, e, AR))
    # roll damping check: alpha_geo = p*y/V -> Clp
    cl, ai, _ = solve_linear(rows, f, np.degrees(y / (G.B / 2)))   # pbar*... alpha = p y / V with p b/2V = 1
    Cl = -np.sum(cl * area * y) / (G.S_REF * G.B)
    out['Clp'] = Cl
    if verbose: print('Clp (per p b/2V) %.3f' % Cl)
    # downwash at the tail per unit CL vs tail height above the wake plane (uniform alpha loading)
    cl, ai, CL = solve_linear(rows, f, np.ones(n) * 5.0)
    gam = 0.5 * c * cl / CL                                         # per unit CL, per V
    xb = np.mean([r['x_c4'] for r in rows])
    hts = G.ht_strips()
    dh = np.array([-1.5, -1.0, -0.6, -0.3, 0.0, 0.3, 0.6, 1.0, 1.5, 2.0, 2.5])
    D = np.zeros((len(hts), len(dh)))
    for i, h in enumerate(hts):
        for k, z in enumerate(dh):
            D[i, k] = np.mean([trailing_downwash(rows, gam, h['x_c4'], yy, z, xb, 0.0) for yy in (h['y'], -h['y'])])
    out.update(ht_dh=dh, ht_eps_per_CL=D)
    if verbose:
        print('tail downwash per unit CL (rad) vs height above wake plane:')
        for i in range(len(hts)): print('   strip %d: %s' % (i, ' '.join('%.3f' % v for v in D[i])))
        CLa_ = CLa
        print('   d eps/d alpha at h=1.3 m: %.3f' % (np.interp(1.3, dh, D[0]) * CLa_))
    # fuselage stations: up/downwash per unit CL (on the centre line, at the station height)
    F = G.fuselage()
    U = []
    for st in F['stations']:
        if -0.1 < st['x'] < 1.6:     # inside the wing root: local flow ~ wing flow, no extra up/downwash
            U.append(0.0); continue
        U.append(-trailing_downwash(rows, gam, st['x'], 0.0, st['z'] + 0.0, xb, 0.0))
    out['fus_upwash_per_CL'] = np.array(U)
    if verbose: print('fuselage upwash per unit CL (rad):', ' '.join('%+.3f' % u for u in U))
    return out


if __name__ == '__main__':
    compute()
