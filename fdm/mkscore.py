"""Summary scorecard (tab/score.tex) with verdicts from explicit rules (listed in the table notes), plus count macros."""
import json, numpy as np, sim
from summarize import load, spin_by_loading
from mktables import spin_stats

Rf, Rv = load('results_fg.json')[1], load('results_v9.json')[1]
P = json.load(open('fig/polar_stats.json'))
OK, PK, NO = '\\ok', '\\pok', '\\nok'
f1 = lambda x: ('%.1f' % x).replace('-', '$-$'); f0 = lambda x: ('%.0f' % x).replace('-', '$-$'); f2 = lambda x: ('%.2f' % x).replace('-', '$-$')
def rg(a, f=f1): return f(min(a)) if f(min(a)) == f(max(a)) else f(min(a)) + '--' + f(max(a))


def metrics(R, key):
    m = {}
    m['rms'] = (P[key]['470']['rms'], P[key]['570']['rms'])
    m['ld'] = (P[key]['470']['ld'], P[key]['570']['ld'])
    ss = R['stallspeeds'][0][1]
    m['vs'] = (ss['single_sb0']['Vs'], ss['dual_sb0']['Vs'])
    m['dvs'] = (ss['single_sb1']['Vs'] - ss['single_sb0']['Vs'], ss['dual_sb1']['Vs'] - ss['dual_sb0']['Vs'])
    m['dive'] = [r for a, r in R['dive'] if r['sb'] == 1.0][0]['CAS']
    md = R['modes'][0][1]; nd = md['nd']; m['np'] = md['trim']['cg'] - nd['Cma'] / nd['CLa'] * 44.13
    st = sorted([r for a, r in R['stall1g']], key=lambda r: r['cg'])
    m['fwd_lim'] = st[0]['elev_limited']; m['fwd_gb'] = st[0]['gbreak']
    m['aft_gb'] = [o['gbreak'] for o in st[1:]]; m['aft_buck'] = [o['theta_osc'] for o in st[1:]]
    pu = [r for a, r in R['pullup']]; m['pull_v'] = [p['kias_min'] for p in pu]; m['pull_drop'] = [p['g_break'] for p in pu]
    m['inv'] = R['inverted'][0][1]['Vs_kias']
    by = spin_by_loading(R)
    m['match'] = sum(1 for k, r in by.items() if sim.expected(sim.A1[k][0], sim.A1[k][6]) == r['cls'])
    cnt = {'WL': 0, 'TH': 0}
    for a, r in R['spin']:
        if r['ail'] == 0.0 and r['cls'] == 'sustained' and sim.expected(sim.A1[r['idx']][0], sim.A1[r['idx']][6]) == 'sustained':
            cnt['TH' if r['thermal'] else 'WL'] += 1
    m['sus'] = cnt
    cs = spin_stats(R); m['spin'] = cs
    rec = {}
    for a, r in R['rec']:
        if r['spinning']: rec.setdefault(r['case'], []).append(None if r['stopped'] is None else r['stopped'][1])
    m['rec'] = rec
    th = np.array([x for a, x in R['slip'] if x['mode'] == 'slip'][0]['th']); w = th[(th[:, 0] > 15) & (th[:, 0] < 28)]
    bg = sorted([x for a, x in R['brakeglide']], key=lambda x: x['V_tas'])
    m['beta'] = w[:, 2].mean(); m['slip'] = -w[:, 5].mean(); m['brake'] = float(np.interp(w[:, 9].mean(), [x['V_tas'] for x in bg], [x['sink'] for x in bg]))
    m['roll'] = [x['marks']['180'] for a, x in R['roll360'] if not x['coord']]
    to = {(x['fr'], x['re']): x for a, x in R['aerotow']}
    m['lo_dual'] = to[(176, 176)]['ev']['airborne'][1] if to[(176, 176)]['ev']['airborne'] else None
    m['lo'] = [to[k]['ev']['airborne'][1] for k in ((176, 176), (176, 0), (230, 0)) if to[k]['ev']['airborne']]
    import re
    txt = open({'fg': 'fg_jsb.xml', 'v9': 'v9_jsb.xml'}[key]).read(); k = 32.174 * 144 / 1e6     # slug ft2 -> 1e6 lb in2
    m['ine'] = tuple(float(re.search(r'<%s unit="SLUG\*FT2">\s*([0-9.]+)' % ax, txt).group(1)) * k for ax in ('ixx', 'iyy', 'izz'))
    ph = {x['V']: x['zeta'] for a, x in R['phugoid']}; m['phug'] = ph
    return m


def rows(mf, mv):
    R = []
    def add(name, src, ref, vf, vv, judge):
        R.append((name, src, ref, vf(mf), judge(mf), vv(mv), judge(mv)))
    add('Glide polar, rms sink error', 'FM p.\\,55', '--', lambda m: rg(m['rms'], f2) + '\\,m/s', lambda m: rg(m['rms'], lambda x: '%.3f' % x) + '\\,m/s',
        lambda m: OK if max(m['rms']) < 0.05 else (PK if max(m['rms']) < 0.10 else NO))
    add('Best glide ratio', 'FM p.\\,55', '33.5', lambda m: rg(m['ld']), lambda m: rg(m['ld']),
        lambda m: OK if all(31.5 <= x <= 35.0 for x in m['ld']) else (PK if all(30 <= x <= 37 for x in m['ld']) else NO))
    add('Stall speed single / dual', 'FM', '65 / 74\\,km/h', lambda m: f1(m['vs'][0]) + ' / ' + f1(m['vs'][1]), lambda m: f1(m['vs'][0]) + ' / ' + f1(m['vs'][1]),
        lambda m: OK if max(abs(m['vs'][0] - 65), abs(m['vs'][1] - 74)) <= 2 else (PK if max(abs(m['vs'][0] - 65), abs(m['vs'][1] - 74)) <= 4 else NO))
    add('Stall-speed increase, airbrakes', 'FM, AFFTC', '3\\,km/h, 2\\,KIAS', lambda m: rg(m['dvs']), lambda m: rg(m['dvs']),
        lambda m: OK if all(2.0 <= x <= 4.5 for x in m['dvs']) else NO)
    add('Airbrake 45$^\\circ$ dive, 600\\,kg', 'FM', '232\\,km/h', lambda m: f0(m['dive']), lambda m: f0(m['dive']),
        lambda m: OK if abs(m['dive'] / 232 - 1) <= 0.05 else (PK if abs(m['dive'] / 232 - 1) <= 0.20 else NO))
    add('Neutral point aft of the aft CG limit', '--', '$>18.46$\\,in', lambda m: f1(m['np']) + '\\,in', lambda m: f1(m['np']) + '\\,in',
        lambda m: OK if m['np'] > 18.46 else NO)
    add('1-$g$ stall: forward CG elevator-limited', 'AFFTC', 'yes', lambda m: 'yes' if m['fwd_lim'] else 'no (%s$^\\circ$ break)' % f1(m['fwd_gb']),
        lambda m: 'yes' if m['fwd_lim'] else 'no (%s$^\\circ$ break)' % f1(m['fwd_gb']), lambda m: OK if m['fwd_lim'] else (PK if m['fwd_gb'] < 3.5 else NO))
    add('1-$g$ stall: aft CG $g$-break + bucking', 'AFFTC', '2--3$^\\circ$, bucking', lambda m: rg(m['aft_gb']) + '$^\\circ$', lambda m: rg(m['aft_gb']) + '$^\\circ$',
        lambda m: OK if all(1.0 <= x <= 4.0 for x in m['aft_gb']) and min(m['aft_buck']) > 3 else (PK if min(m['aft_buck']) > 3 else NO))
    add('Dynamic stall: min.\\ speed / nose drop', 'AFFTC', '20\\,KIAS / $\\le$40$^\\circ$', lambda m: rg(m['pull_v'], f0) + ' / ' + rg(m['pull_drop'], f0) + '$^\\circ$',
        lambda m: rg(m['pull_v'], f0) + ' / ' + rg(m['pull_drop'], f0) + '$^\\circ$',
        lambda m: OK if min(m['pull_v']) <= 25 and max(m['pull_drop']) <= 45 else (PK if min(m['pull_v']) <= 25 else NO))
    add('Inverted $-1g$ stall', 'AFFTC', '38--40\\,KIAS', lambda m: f1(m['inv']), lambda m: f1(m['inv']),
        lambda m: OK if 36 <= m['inv'] <= 42 else (PK if 32 <= m['inv'] <= 46 else NO))
    add('Spin map, 24 loadings', 'AFFTC Fig.~5', '--', lambda m: '%d of 24' % m['match'], lambda m: '%d of 24' % m['match'],
        lambda m: OK if m['match'] >= 20 else (PK if m['match'] >= 16 else NO))
    add('Thermal entry more effective than wings-level', 'AFFTC', 'yes', lambda m: '%d vs %d' % (m['sus']['TH'], m['sus']['WL']),
        lambda m: '%d vs %d' % (m['sus']['TH'], m['sus']['WL']), lambda m: OK if m['sus']['TH'] > m['sus']['WL'] else (PK if m['sus']['TH'] == m['sus']['WL'] > 0 else NO))
    nosp = lambda m: m['spin'] is None
    add('Spin rate (mean)', 'AFFTC', '80--140\\,deg/s', lambda m: 'no spin' if nosp(m) else f0(m['spin']['rate_mean'][0]),
        lambda m: 'no spin' if nosp(m) else f0(m['spin']['rate_mean'][0]), lambda m: NO if nosp(m) else (OK if 80 <= m['spin']['rate_mean'][0] <= 140 else PK))
    add('Spin pitch attitude / oscillation', 'AFFTC', '$-50\\pm15^\\circ$ (low $I$)', lambda m: '--' if nosp(m) else f0(m['spin']['theta_mean_lo']) + ', $\\sigma$ = ' + f0(m['spin']['theta_sd_lo']) + '$^\\circ$',
        lambda m: '--' if nosp(m) else f0(m['spin']['theta_mean_lo']) + ', $\\sigma$ = ' + f0(m['spin']['theta_sd_lo']) + '$^\\circ$',
        lambda m: NO if nosp(m) else (OK if -60 <= m['spin']['theta_mean_lo'] <= -35 and m['spin']['theta_sd_lo'] <= 12 else PK))
    add('Height loss per turn', 'AFFTC', '200 (150--250)\\,ft', lambda m: '--' if nosp(m) else f0(m['spin']['ft_per_turn'][0]),
        lambda m: '--' if nosp(m) else f0(m['spin']['ft_per_turn'][0]),
        lambda m: NO if nosp(m) else (OK if 150 <= m['spin']['ft_per_turn'][0] <= 250 else (PK if m['spin']['ft_per_turn'][0] <= 350 else NO)))
    def recj(case, lim):
        def j(m):
            r = m['rec'].get(case)
            if not r: return NO
            if any(x is None for x in r): return NO
            return OK if max(r) <= lim else PK
        return j
    recv = lambda case: (lambda m: '--' if not m['rec'].get(case) else ('none' if any(x is None for x in m['rec'][case]) else rg(m['rec'][case], f2)))
    add('Recovery, FM procedure (turns)', 'FM, AFFTC', '$<1$ (max $1\\frac12$)', recv('fm'), recv('fm'), recj('fm', 1.5))
    add('Recovery, opposite rudder only (turns)', 'AFFTC', '$\\frac14$--$\\frac34$ (max $1\\frac12$)', recv('rudder'), recv('rudder'), recj('rudder', 1.5))
    add('Forward stick, pro-spin rudder (turns)', 'AFFTC', 'up to 3 more turns', recv('forward'), recv('forward'),
        lambda m: NO if not m['rec'].get('forward') else PK)
    add('Full-rudder sideslip', 'AFFTC', 'up to 50$^\\circ$', lambda m: f0(m['beta']) + '$^\\circ$', lambda m: f0(m['beta']) + '$^\\circ$',
        lambda m: OK if m['beta'] >= 35 else (PK if m['beta'] >= 25 else NO))
    add('Slip sink vs.\\ airbrakes, same speed', 'FM, pilot', 'slip $\\gtrsim$ brakes', lambda m: '%s vs %s\\,m/s' % (f1(m['slip']), f1(m['brake'])),
        lambda m: '%s vs %s\\,m/s' % (f1(m['slip']), f1(m['brake'])), lambda m: OK if m['slip'] >= m['brake'] else NO)
    add('Full-aileron roll, first 180$^\\circ$', 'video', '$\\approx5$\\,s', lambda m: rg(m['roll']) + '\\,s', lambda m: rg(m['roll']) + '\\,s',
        lambda m: OK if all(4.0 <= x <= 6.0 for x in m['roll']) else (PK if all(3.0 <= x <= 7.0 for x in m['roll']) else NO))
    add('Hands-free aerotow lift-off before the tug', 'FM, pilot', '$\\approx75$\\,km/h', lambda m: rg(m['lo'], f0) + '\\,km/h', lambda m: rg(m['lo'], f0) + '\\,km/h',
        lambda m: OK if m['lo_dual'] is not None and m['lo_dual'] < 85 else NO)
    add('Empty inertias', 'AFFTC App.~C', '30.2 / 6.7 / 34.3', lambda m: '%s / %s / %s' % tuple(f1(x) for x in m['ine']), lambda m: '%s / %s / %s' % tuple(f1(x) for x in m['ine']),
        lambda m: OK if abs(m['ine'][0] / 30.2 - 1) < 0.05 and abs(m['ine'][1] / 6.7 - 1) < 0.05 and abs(m['ine'][2] / 34.3 - 1) < 0.05 else NO)
    add('Phugoid damped at 80--140\\,km/h', 'theory', '$\\zeta\\approx0.02$', lambda m: 'min.\\ $\\zeta$ = ' + f2(min(v for v in m['phug'].values() if v is not None)),
        lambda m: 'min.\\ $\\zeta$ = ' + f2(min(v for v in m['phug'].values() if v is not None)),
        lambda m: OK if min(v for v in m['phug'].values() if v is not None) > 0 else NO)
    return R


mf, mv = metrics(Rf, 'fg'), metrics(Rv, 'v9')
R = rows(mf, mv)
s = ['\\begin{tabularx}{\\textwidth}{@{}Xllrlrl@{}}', '\\toprule',
     'Criterion & Source & Reference & \\multicolumn{2}{c}{FlightGear rev.\\ 8166} & \\multicolumn{2}{c}{\\vnine{}} \\\\', '\\midrule']
for name, src, ref, vf, jf, vv, jv in R:
    s.append('%s & %s & %s & %s & %s & %s & %s \\\\' % (name, src, ref, vf, jf, vv, jv))
s += ['\\midrule', '\\multicolumn{7}{p{0.97\\textwidth}}{\\footnotesize Rules: polar rms $<0.05$\\,m/s (\\pok\\ $<0.10$); best glide 31.5--35 (\\pok\\ 30--37); '
      'stall speeds within 2\\,km/h (\\pok\\ 4); airbrake increment 2--4.5\\,km/h; dive within 5\\,\\% (\\pok\\ 20\\,\\%); inverted stall 36--42\\,KIAS (\\pok\\ 32--46); '
      'spin map $\\ge$20 of 24 (\\pok\\ $\\ge$16); spin rate 80--140; pitch mean $-35$ to $-60^\\circ$ with $\\sigma\\le12^\\circ$; height loss 150--250\\,ft (\\pok\\ $\\le$350); '
      'recoveries $\\le1\\frac12$ turns; forward stick recovering at all is too benign (\\pok); sideslip $\\ge35^\\circ$ (\\pok\\ $\\ge25^\\circ$); roll 4--6\\,s (\\pok\\ 3--7); '
      'lift-off below the tug\'s 85\\,km/h with 2\\,$\\times$\\,80\\,kg; inertias within 5\\,\\%; phugoid $\\zeta>0$ at all tested speeds. Stall-break rows: \\ok\\ as described, \\pok\\ bucking present / break $<3.5^\\circ$.} \\\\',
      '\\bottomrule', '\\end{tabularx}']
open('tab/score.tex', 'w').write('\n'.join(s) + '\n')
cnt = lambda i, v: sum(1 for r in R if r[i] == v)
mac = {'scN': len(R), 'scOkFg': cnt(4, OK), 'scPkFg': cnt(4, PK), 'scNoFg': cnt(4, NO), 'scOkNine': cnt(6, OK), 'scPkNine': cnt(6, PK), 'scNoNine': cnt(6, NO)}
open('tab/score_numbers.tex', 'w').write('\n'.join('\\newcommand{\\N%s}{%d}' % (k, v) for k, v in mac.items()) + '\n')
print(mac)
for r in R: print('%-45s FG %-28s %s  v9 %-28s %s' % (r[0][:45], r[3][:28], r[4], r[5][:28], r[6]))
