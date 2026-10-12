"""Spin objective: AFFTC spin map, spin characteristics, recoveries. Lower is better."""
import numpy as np, os, json, sys
from concurrent.futures import ProcessPoolExecutor
import sim, spinmap, genxml

REC_LOADS = [21, 11, 19, 16]       # 17.42/9.04, 14.20/9.26, 16.27/9.02, 15.50/7.84
REC_CASES = ('continue', 'fm', 'rudder', 'neutral', 'forward', 'ail_against')


def _spin(args):
    aero, idx, ail, entry_ail = args
    row = sim.A1[idx]
    try:
        F = sim.spin_entry(aero, sim.a1_load(row), jsb=spinmap.JSB_TEST, rud=-1.0, ail=ail, hold=40.0, entry_ail=0.0, thermal=(entry_ail != 0.0),
                           tag=None)
        c = sim.classify(F, F.stall_t)
        m = sim.spin_metrics(F, F.stall_t + 12) if c['cls'] == 'sustained' else None
        ok = bool(np.isfinite(F.arr()).all())
        return dict(idx=idx, ail=ail, cls=c['cls'], turns=c['turns'], m=m, ok=ok)
    except Exception as e:
        return dict(idx=idx, ail=ail, cls='error', turns=0, m=None, ok=False)


def _rec(args):
    aero, idx, case = args
    row = sim.A1[idx]
    try:
        r = sim.recovery(aero, sim.a1_load(row), case, jsb=spinmap.JSB_TEST, tag=None)
        return dict(idx=idx, case=case, **r)
    except Exception as e:
        return dict(idx=idx, case=case, spinning=False, stopped=None, err=str(e))


def _stall(args):
    aero, idx = args
    try:
        o = sim.stall_ramp(aero, sim.a1_load(sim.A1[idx]), jsb=spinmap.JSB_TEST)
        return dict(idx=idx, **o)
    except Exception as e:
        return dict(idx=idx, err=str(e))


def _vs(args):
    aero, fr, re = args
    from static import Static
    from scipy.optimize import brentq
    st = Static(aero, front=fr, rear=re); W = st.W; best = 0.0
    for a in np.arange(6, 22.1, 0.5):
        try: el = brentq(lambda e: st.coeffs(25.0, a, elev=e)['Cm'], -1, 1)
        except ValueError: continue
        best = max(best, st.coeffs(25.0, a, elev=el)['CL'])
    return dict(fr=fr, re=re, W=W, cg=st.cg_in(), CL=best)


STALL_LOADS = (0, 9, 17, 22)


ORDER = {'none': 0, 'incipient': 1, 'sustained': 2, 'error': -1}


W_RATE, W_FT, W_KIAS = 1.0, 1.0, 0.6     # weights of the spin-characteristic penalties (stage 2 raises rate / ft per turn)


def evaluate(P, tag='obj', workers=8, verbose=False):
    fn = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'obj_%s.xml' % tag)
    genxml.write(fn, P)
    jobs = [(fn, i, a, ea) for i in range(len(sim.A1)) for (a, ea) in ((0.0, 0.0), (1.0, 0.0), (0.0, -1.0))]
    rjobs = [(fn, i, c) for i in REC_LOADS for c in REC_CASES]
    with ProcessPoolExecutor(workers) as ex:
        fs = [ex.submit(_stall, (fn, i)) for i in STALL_LOADS] + [ex.submit(_vs, (fn, 230, 230)), ex.submit(_vs, (fn, 176, 0))]
        sres = list(ex.map(_spin, jobs)); rres = list(ex.map(_rec, rjobs))
        extra = [x.result() for x in fs]
    stl = extra[:len(STALL_LOADS)]; vs = extra[len(STALL_LOADS):]
    by = {}
    for r in sres:
        if r['idx'] not in by or ORDER[r['cls']] > ORDER[by[r['idx']]['cls']]: by[r['idx']] = r
    loss = 0.0; notes = []
    # 1. spin map
    for i, r in by.items():
        cg, I = sim.A1[i][0], sim.A1[i][6]
        exp = sim.expected(cg, I); got = r['cls']
        if got == 'error': loss += 3; notes.append('E%.2f' % cg); continue
        d = abs(ORDER[exp] - ORDER[got])
        if d:
            # distance to the relevant boundary softens marginal cases
            bd = min(abs(cg - sim.cg_sust(I)), abs(cg - sim.cg_inc(I)))
            loss += d * (0.6 + 0.4 * min(1.0, bd / 0.8)); notes.append('%s%.2f' % (got[0], cg))
    # 2. characteristics of sustained spins (pro-spin aileron neutral entries)
    ms = [r['m'] for r in sres if r['m'] is not None and r['ail'] == 0.0]
    ch = None
    if ms:
        avg = lambda k: float(np.mean([m[k] for m in ms]))
        ch = {k: avg(k) for k in ms[0]}
        pen = lambda x, lo, hi, s: max(0.0, lo - x, x - hi) / s
        loss += 1.0 * pen(ch['theta_mean'], -50, -40, 10) + 0.5 * pen(ch['theta_sd'], 9, 25, 8)
        loss += W_RATE * pen(ch['rate_mean'], 95, 130, 25) + W_FT * pen(ch['ft_per_turn'], 175, 235, 60)
        loss += W_KIAS * pen(ch['kias_mean'], 30, 42, 8) + 0.4 * pen(ch['bank_mean'], 0, 12, 10) + 0.4 * pen(ch['nz_mean'], 0.9, 1.25, 0.4)
        # AFFTC: steep phase to -70, flat phase up to the horizon; no tumbling
        loss += 0.6 * float(np.mean([max(0.0, m['theta_max'] - 5.0) / 15.0 + max(0.0, -85.0 - m['theta_min']) / 10.0 for m in ms]))
        loss += 0.6 * float(np.mean([max(0.0, m['nz_max'] - 2.5) / 1.5 for m in ms]))
    else:
        loss += 6
    # 3. recoveries
    rec = {}
    for r in rres:
        rec[(r['idx'], r['case'])] = r
        s = r.get('stopped'); cg = sim.A1[r['idx']][0]; hiI = sim.A1[r['idx']][6] > 8.9
        if not r.get('spinning'): loss += 0.3; continue
        turns = None if s is None else s[1]; hft = None if s is None else s[2]
        c = r['case']
        if c == 'continue':
            if s is not None: loss += 0.5
        elif c in ('fm', 'rudder'):
            lim = 1.5 if (hiI and 14 <= cg <= 16.3) else 1.0
            if s is None: loss += 2.0
            else:
                loss += max(0.0, turns - lim) * 1.5 + max(0.0, turns - 0.75) * 0.3
                loss += max(0.0, hft - 350) / 300.0
        elif c == 'neutral':
            if s is None: loss += 1.0
        elif c == 'forward':
            if s is None: loss += 0.5
            elif turns > 3.5: loss += 0.3
        elif c == 'ail_against':
            if s is None: loss += 0.4
    # 4. 1-g stalls (AFFTC: elevator-limited forward of ~13 in; aft: mild g-break 2-3 deg, pitch bucking, wings level)
    for o in stl:
        if 'err' in o: loss += 3; continue
        if o['cg'] < 12.8:
            loss += 0.0 if o['elev_limited'] else 0.6
        else:
            loss += 0.6 if (o['elev_limited'] and o['cg'] > 13.5) else 0.0
            loss += 1.0 * max(0.0, o['gbreak'] - 5.0) / 5.0
        loss += 0.8 * max(0.0, o['theta_osc'] - 12.0) / 12.0 + 0.8 * max(0.0, o['alpha_hold'][1] - 26.0) / 10.0
        loss += 0.8 * max(0.0, o['wing_drop'] - 15.0) / 15.0
    # 5. stall speeds (flight manual 65 / 74 km/h at 470 / 600 kg, AFFTC 33-38 KIAS): scale to those weights
    v6 = np.sqrt(2 * 600 * 9.81 / (1.225 * 17.95 * vs[0]['CL'])) * 3.6
    v4 = np.sqrt(2 * 470 * 9.81 / (1.225 * 17.95 * vs[1]['CL'])) * 3.6
    loss += max(0.0, abs(v6 - 72.5) - 1.5) / 2.0 + max(0.0, abs(v4 - 63.5) - 1.5) / 2.0
    if verbose:
        print('loss %.3f  map %s' % (loss, ' '.join(notes)))
        print('   Vs 600 kg %.1f  470 kg %.1f km/h' % (v6, v4))
        for o in stl:
            if 'err' not in o: print('   stall cg %.2f: %.1f KIAS a %.1f g-break %.1f osc %.1f alpha-hold %.0f..%.0f wing drop %.1f %s' % (o['cg'], o['kias_stall'], o['amax'], o['gbreak'], o['theta_osc'], o['alpha_hold'][0], o['alpha_hold'][1], o['wing_drop'], 'ELEV-LIM' if o['elev_limited'] else ''))
        if ch: print('   theta %.0f sd %.0f rate %.0f ft/turn %.0f KIAS %.0f bank %.0f nz %.2f alpha %.0f' % (ch['theta_mean'], ch['theta_sd'], ch['rate_mean'], ch['ft_per_turn'], ch['kias_mean'], ch['bank_mean'], ch['nz_mean'], ch['alpha_mean']))
        for i in REC_LOADS:
            print('   rec cg %.2f I %.2f: ' % (sim.A1[i][0], sim.A1[i][6]) + '  '.join('%s %s' % (c, ('-' if not rec[(i, c)].get('spinning') else ('no' if rec[(i, c)]['stopped'] is None else '%.2ft/%.0fft' % (rec[(i, c)]['stopped'][1], rec[(i, c)]['stopped'][2])))) for c in REC_CASES))
    try: os.remove(fn)
    except Exception: pass
    return loss, dict(by=by, ch=ch, rec=rec, notes=notes, stall=stl, vs=(v6, v4))


if __name__ == '__main__':
    P = json.loads(sys.argv[1]) if len(sys.argv) > 1 else {}
    evaluate(P, verbose=True)
