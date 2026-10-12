"""Condensed text summary of a campaign result file (also used to fill the report tables)."""
import json, sys, numpy as np
import sim

ORDER = {'none': 0, 'incipient': 1, 'sustained': 2, 'error': -1}


def load(fn):
    d = json.load(open(fn)); R = {}
    for r in d['results']:
        R.setdefault(r['kind'], []).append((r['args'], r['res']))
    return d, R


def spin_by_loading(R):
    by = {}
    for args, r in R['spin']:
        if 'error' in r: continue
        k = r['idx']
        if k not in by or ORDER[r['cls']] > ORDER[by[k]['cls']]: by[k] = r
    return by


def spin_chars(R):
    ms = [r['m'] for a, r in R['spin'] if 'error' not in r and r['m'] is not None and r['ail'] == 0.0 and not r['thermal']]
    if not ms: return None, 0
    return {k: (float(np.mean([m[k] for m in ms])), float(np.min([m[k] for m in ms])), float(np.max([m[k] for m in ms]))) for k in ms[0]}, len(ms)


def summary(fn):
    d, R = load(fn)
    print('==', d['model']['name'], d['date'])
    for a, r in R.get('polar', []):
        pts = np.array(r['pts']); i = np.argmin(pts[:, 1])
        print('polar %d kg: min sink %.2f m/s at %.0f km/h; ' % (r['mass'], pts[i, 1], pts[i, 0]) +
              ' '.join('%.0f:%.2f' % (v, s) for v, s, _, _ in r['pts'] if int(round(v)) % 20 == 0))
    ss = R['stallspeeds'][0][1]
    print('stall speeds: single %.1f (+brakes %.1f), dual %.1f (+brakes %.1f) km/h  [FM 65/68, 74/77]' % (
        ss['single_sb0']['Vs'], ss['single_sb1']['Vs'], ss['dual_sb0']['Vs'], ss['dual_sb1']['Vs']))
    cl = sorted([r for a, r in R['clmax']], key=lambda r: r['cg'])
    print('trimmed CLmax / elevator-limited vs CG: ' + ' '.join('%.1f:%.2f%s' % (r['cg'], r['CLmax'], '*' if r['elev_limited'] else '') for r in cl))
    by = spin_by_loading(R); bad = 0; line = []
    for k, r in sorted(by.items(), key=lambda kv: sim.A1[kv[0]][0]):
        row = sim.A1[k]; exp = sim.expected(row[0], row[6])
        bad += exp != r['cls']
        line.append('%.2f/%.2f %s%s' % (row[0], row[6], r['cls'][0], '' if exp == r['cls'] else '!' + exp[0]))
    print('spin map (%d mismatches): ' % bad + ' '.join(line))
    ch, n = spin_chars(R)
    if ch:
        print('spin characteristics (%d sustained, neutral-aileron entries): ' % n + ' '.join('%s %.1f [%.1f..%.1f]' % (k, *ch[k]) for k in
              ('rate_mean', 'ft_per_turn', 'theta_mean', 'theta_sd', 'alpha_mean', 'kias_mean', 'nz_mean', 's_per_turn')))
    rec = {}
    for a, r in R['rec']:
        s = r['stopped']; rec[(r['idx'], r['case'])] = '-' if not r['spinning'] else ('no' if s is None else '%.2ft/%.0fft' % (s[1], s[2]))
    for i in sorted(set(k[0] for k in rec)):
        print('  rec cg %.2f I %.2f: ' % (sim.A1[i][0], sim.A1[i][6]) + '  '.join('%s %s' % (c, rec[(i, c)]) for c in
              ('continue', 'fm', 'rudder', 'neutral', 'forward', 'ail_against')))
    for a, o in R['stall1g']:
        if 'error' in o: print('stall1g error', o['error']); continue
        print('  1g stall cg %.2f: %.1f KIAS a %.1f g-break %.1f, held aft: theta osc %.1f alpha %.0f..%.0f wing drop %.1f %s' % (
            o['cg'], o['kias_stall'], o['amax'], o['gbreak'], o['theta_osc'], o['alpha_hold'][0], o['alpha_hold'][1], o['wing_drop'], 'ELEV-LIM' if o['elev_limited'] else ''))
    for a, o in R['pullup']:
        print('  30-deg pull-up cg %.2f: stall %.0f KIAS (min %.0f), nose drop %.0f deg, bank %.0f, alpha max %.0f, height loss %.0f ft' % (
            o['cg'], o['kias_stall'], o['kias_min'], o['g_break'], o['phi_max'], o['alpha_max'], o['h_loss']))
    inv = R['inverted'][0][1]
    print('inverted: CLmin %.2f -> -1 g stall %.1f KIAS [AFFTC 38-40]' % (inv['CLmin'], inv['Vs_kias'] or -1))
    for a, o in R['roll45']: print('roll 45-45 at 90 km/h %s: %s s, max beta %.1f' % ('coord' if o['coord'] else 'ail only', o['t'] and '%.2f' % o['t'], o['beta_max']))
    for a, o in R['roll360']: print('roll %d/%d at %d km/h %s: %s' % (o['fr'], o['re'], o['V'], 'coord' if o['coord'] else 'ail', o['marks']))
    for a, o in R['dive']: print('45-deg dive brakes %d: CAS %.0f km/h (at 600 kg %.0f) [FM 232 with brakes]' % (o['sb'], o['CAS'], o['CAS600']))
    for a, o in R['slip']:
        th = np.array(o['th']); w = th[(th[:, 0] > 15) & (th[:, 0] < 28)]
        print('%s: beta %.1f phi %.1f V %.0f km/h sink %.2f m/s' % (o['mode'], w[:, 2].mean(), w[:, 3].mean(), w[:, 1].mean(), -w[:, 5].mean()))
    for a, o in R['aerotow']:
        ev = o['ev']; f = lambda e: '-' if e is None else '%.0f' % e[1]
        print('aerotow %d/%d hands-free: nose off %s main off %s airborne %s km/h, tug off t=%s' % (o['fr'], o['re'], f(ev['nose_off']), f(ev['main_off']), f(ev['airborne']), ev['tug_lo'] and round(ev['tug_lo'], 1)))
    for a, o in R['trimspeed']: print('stick-free trim %d/%d cg %.1f: %s km/h' % (o['fr'], o['re'], o['cg'], o['V'] and round(o['V'])))
    md = R['modes'][0][1]
    print('modes: trim', {k: round(v, 2) for k, v in md['trim'].items()}, 'nd', {k: round(v, 3) for k, v in md['nd'].items()})
    print('  lon', md['lon']); print('  lat', md['lat'])
    rb = R['robust'][0][1]; print('robust: %s' % rb)
    mp = R['massprops'][0][1]
    print('mass props (cg, Iyy model vs AFFTC): ' + ' '.join('%.2f/%.2f:%.2f/%.2f' % (m['cg'], m['cg_afftc'], m['Iyy'], m['Iyy_afftc']) for m in mp[:6]), '... Ixx %.1f Izz %.1f' % (mp[0]['Ixx'], mp[0]['Izz']))


if __name__ == '__main__':
    for fn in sys.argv[1:]: summary(fn)
