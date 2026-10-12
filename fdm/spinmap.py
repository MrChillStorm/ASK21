"""AFFTC spin map: every Table A1 loading, AFFTC wings-level entry (rudder + full aft stick, aileron neutral and
aileron with the spin), classification vs the report's boundaries, spin characteristics of sustained spins."""
import numpy as np, sys, os, json
from concurrent.futures import ProcessPoolExecutor
import sim

HERE = os.path.dirname(os.path.abspath(__file__))
JSB_TEST = os.path.join(HERE, 'jsb_test.xml')
ORDER = {'none': 0, 'incipient': 1, 'sustained': 2}


def one(args):
    aero, idx, ail, hold = args
    row = sim.A1[idx]
    try:
        F = sim.spin_entry(aero, sim.a1_load(row), jsb=JSB_TEST, rud=-1.0, ail=ail, hold=hold, tag='sm%d_%d' % (idx, int(ail * 10)))
        c = sim.classify(F, F.stall_t)
        m = sim.spin_metrics(F, F.stall_t + 12) if c['cls'] == 'sustained' else None
        L = F.arr(); w = L[L[:, 0] >= F.stall_t]
        return dict(idx=idx, cg=row[0], I=row[6], ail=ail, cls=c['cls'], turns=c['turns'], m=m, amax=float(w[:, 2].max()) if len(w) else 0,
                    rmax=float(np.abs(w[:, 5]).max()) if len(w) else 0, ok=bool(np.isfinite(L).all()))
    except Exception as e:
        return dict(idx=idx, cg=row[0], I=row[6], ail=ail, cls='error', turns=0, m=None, err=str(e)[:80], ok=False)


def run(aero, ails=(0.0, 1.0), hold=45.0, workers=8, idxs=None):
    idxs = range(len(sim.A1)) if idxs is None else idxs
    args = [(aero, i, a, hold) for i in idxs for a in ails]
    with ProcessPoolExecutor(workers) as ex:
        res = list(ex.map(one, args))
    by = {}
    for r in res:
        k = r['idx']
        if k not in by or ORDER.get(r['cls'], -1) > ORDER.get(by[k]['cls'], -1):
            by[k] = r
    return res, by


def score(by):
    """mismatch count against the AFFTC boundaries (most spin-prone entry per loading)."""
    out = []
    for k, r in sorted(by.items()):
        exp = sim.expected(r['cg'], r['I'])
        out.append((r['cg'], r['I'], exp, r['cls'], r['turns']))
    bad = sum(1 for o in out if o[2] != o[3])
    return bad, out


def characteristics(res):
    ms = [r['m'] for r in res if r.get('m')]
    if not ms: return None
    keys = ms[0].keys()
    return {k: (float(np.mean([m[k] for m in ms])), float(np.min([m[k] for m in ms])), float(np.max([m[k] for m in ms]))) for k in keys}


if __name__ == '__main__':
    aero = sys.argv[1]
    res, by = run(aero)
    bad, out = score(by)
    for cg, I, exp, got, turns in out:
        print('cg %5.2f I %.3f  expected %-9s got %-9s (%.1f turns) %s' % (cg, I, exp, got, turns, '' if exp == got else '  <-- MISMATCH'))
    print('mismatches %d / %d' % (bad, len(out)))
    ch = characteristics(res)
    if ch:
        for k, (m, lo, hi) in ch.items(): print('  %-12s mean %7.1f  (%7.1f .. %7.1f)' % (k, m, lo, hi))
