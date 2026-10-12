"""Spin descent-rate consistency: vertical-force coefficient implied by height loss per turn and rotation rate."""
import numpy as np, json, sys
sys.path.insert(0, '../v9'); import geometry as G
ys = np.linspace(-8.5, 8.5, 2001); c = G.chord(ys) * (17.95 / np.trapezoid(G.chord(ys), ys))
I2 = np.trapezoid(c * ys ** 2, ys)                       # m^4
def sigma(h_ft):
    T = 288.15 - 0.0019812 * h_ft; return (T / 288.15) ** 4.2559
W = 1150 * 4.44822; S = 17.95; rho0 = 1.225
out = {}
sys.path.insert(0, '.')
from summarize import load
from mktables import spin_stats
cs = spin_stats(load('results_v9.json')[1])
for lab, dh_ft, rate, h_ft in (('AFFTC nominal', 200, 105, 8500), ('AFFTC best case', 250, 140, 8500), ('AFFTC worst case', 150, 80, 8500),
                               ('v9 (sim at 6000 ft)', cs['ft_per_turn'][0], cs['rate_mean'][0], 6000)):
    Om = np.radians(rate); Vz = dh_ft * 0.3048 * rate / 360.0          # m/s TAS
    sg = sigma(h_ft); Ve = Vz * np.sqrt(sg); kappa = Om ** 2 * I2 / (S * Vz ** 2)
    CR = 2 * W / (rho0 * S * Ve ** 2)
    out[lab] = dict(Vz_tas=Vz, Ve_kt=Ve / 0.514444, kappa=kappa, C_vert=CR, C_vert_rot=CR / (1 + kappa))
    print('%-22s dh %3d ft, %3d deg/s, %5d ft: Vz %.1f m/s TAS = %.1f kt EAS, kappa %.2f, C_vert %.2f (rotation-corrected %.2f)' % (
        lab, dh_ft, rate, h_ft, Vz, Ve / 0.514444, kappa, CR, CR / (1 + kappa)))
# what the v9 airframe produces statically at spin incidence (resultant force coefficient)
R = json.load(open('results_v9.json'))
for r in R['results']:
    if r['kind'] == 'curves':
        c_ = r['res']; a = np.array(c_['alpha']); CL = np.array(c_['CL']); CD = np.array(c_['CD'])
        CRs = np.hypot(CL, CD)
        for aa in (40, 45, 50, 60, 90): print('v9 static resultant coefficient at alpha %d: %.2f' % (aa, np.interp(aa, a, CRs)))
        out['v9_CR_45'] = float(np.interp(45, a, CRs)); out['v9_CR_50'] = float(np.interp(50, a, CRs))
out['I2'] = float(I2); out['flatplate_2d_45'] = 1.98 * np.sin(np.radians(45)); out['flatplate_2d_50'] = 1.98 * np.sin(np.radians(50))
json.dump(out, open('tab/paradox.json', 'w'), indent=1)
print('I2 = %.0f m^4; 2-D flat plate CN at 45/50 deg: %.2f / %.2f' % (I2, out['flatplate_2d_45'], out['flatplate_2d_50']))
