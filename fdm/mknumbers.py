"""LaTeX macros for every number quoted in the report text (tab/numbers.tex), straight from the campaign results."""
import json, sys, numpy as np, sim
from summarize import load, spin_by_loading
from mktables import spin_stats
Rf, Rv = load('results_fg.json')[1], load('results_v9.json')[1]
P = json.load(open('fig/polar_stats.json')); PX = json.load(open('tab/paradox.json'))
N = {}
def put(k, v): N[k] = v if ('$' in v or v.startswith('--') or not v.startswith('-')) else sgn(v)
f0 = lambda x: '%.0f' % x; f1 = lambda x: '%.1f' % x; f2 = lambda x: '%.2f' % x
def sgn(s):
    return ('$-$' + s[1:]) if s.startswith('-') else s
def rng(a, fm=f1):
    lo, hi = fm(min(a)), fm(max(a))
    if lo == hi: return sgn(lo)
    return sgn(lo) + (' to ' if (lo.startswith('-') or hi.startswith('-')) else '--') + sgn(hi)
# counts
put('nTests', str(sum(len(v) for v in Rv.values())))
for lab, R in (('Fg', Rf), ('Nine', Rv)):
    by = spin_by_loading(R); mism = []
    for k, r in by.items():
        row = sim.A1[k]; e = sim.expected(row[0], row[6])
        if e != r['cls']: mism.append((row[0], row[6], e, r['cls'], min(abs(row[0] - sim.cg_sust(row[6])), abs(row[0] - sim.cg_inc(row[6])))))
    put('mism' + lab, str(len(mism))); put('match' + lab, str(24 - len(mism)))
    if lab == 'Nine':
        put('mismNear', str(sum(1 for m in mism if m[4] <= 0.2)))
        O = {'none': 0, 'incipient': 1, 'sustained': 2}
        put('mismBenign', str(sum(1 for m in mism if O[m[3]] < O[m[2]]))); put('mismProne', str(sum(1 for m in mism if O[m[3]] > O[m[2]])))
# entry techniques (v9): sustained spins where the AFFTC found them
cnt = {}
for a, r in Rv['spin']:
    k = 'TH' if r['thermal'] else ('WLA' if r['ail'] else 'WL'); e = sim.expected(sim.A1[r['idx']][0], sim.A1[r['idx']][6])
    cnt.setdefault(k, 0)
    if r['cls'] == 'sustained' and e == 'sustained': cnt[k] += 1
for k, v in cnt.items(): put('sus' + k, str(v))
put('nExpSus', str(sum(sim.expected(r[0], r[6]) == 'sustained' for r in sim.A1)))
# spin characteristics
cs = spin_stats(Rv)
put('spinN', str(cs['n'])); put('spinRate', f0(cs['rate_mean'][0])); put('spinRateRng', rng(cs['rate_mean'][1:], f0))
put('spinTurn', f1(cs['s_per_turn'][0])); put('spinFt', f0(cs['ft_per_turn'][0])); put('spinTheta', f0(cs['theta_mean'][0]))
put('spinAlpha', f0(cs['alpha_mean'][0])); put('spinKcas', f0(cs['kias_mean'][0])); put('spinRateLo', f0(cs['rate_lo'])); put('spinRateHi', f0(cs['rate_hi']))
put('spinThetaMin', f0(cs['theta_min'][1])); put('spinThetaMax', f0(cs['theta_max'][2]))
put('spinThetaLo', f0(cs['theta_mean_lo'])); put('spinSdLo', f0(cs['theta_sd_lo']))
# recoveries
rec = {}
for a, r in Rv['rec']:
    if r['spinning'] and r['stopped'] is not None: rec.setdefault(r['case'], []).append(r['stopped'])
for c, nm in (('fm', 'Fm'), ('rudder', 'Rud'), ('forward', 'Fwd'), ('neutral', 'Neu')):
    if c in rec: put('rec' + nm, rng([s[1] for s in rec[c]], f2)); put('rec' + nm + 'Ft', rng([s[2] for s in rec[c]], f0))
put('recAilNone', str(sum(1 for a, r in Rv['rec'] if r['case'] == 'ail_against' and r['spinning'] and r['stopped'] is None)))
put('recNspin', str(sum(1 for a, r in Rv['rec'] if r['case'] == 'fm' and r['spinning'])))
# stalls
for lab, R in (('Fg', Rf), ('Nine', Rv)):
    st = sorted([r for a, r in R['stall1g']], key=lambda r: r['cg'])
    put('stallV' + lab, rng([o['kias_min_pre'] for o in st])); put('gbreakAft' + lab, rng([o['gbreak'] for o in st[1:]]))
    put('gbreakFwd' + lab, f1(st[0]['gbreak']))
    pu = sorted([r for a, r in R['pullup']], key=lambda r: r['cg'])
    put('pullV' + lab, rng([p['kias_min'] for p in pu], f0)); put('pullDrop' + lab, rng([p['g_break'] for p in pu], f0)); put('pullH' + lab, rng([p['h_loss'] for p in pu], f0))
    put('inv' + lab, f1(R['inverted'][0][1]['Vs_kias'])); put('invCl' + lab, f2(R['inverted'][0][1]['CLmin']))
# static / performance
ss = lambda R: R['stallspeeds'][0][1]
for lab, R, pk in (('Fg', Rf, 'fg'), ('Nine', Rv, 'v9')):
    s_ = ss(R); put('vsS' + lab, f1(s_['single_sb0']['Vs'])); put('vsD' + lab, f1(s_['dual_sb0']['Vs']))
    put('dvsB' + lab, rng([s_['single_sb1']['Vs'] - s_['single_sb0']['Vs'], s_['dual_sb1']['Vs'] - s_['dual_sb0']['Vs']]))
    put('dive' + lab, f0([r for a, r in R['dive'] if r['sb'] == 1.0][0]['CAS']))
    put('polRms' + lab, f2(P[pk]['470']['rms']) + '--' + f2(P[pk]['570']['rms']) if lab == 'Fg' else '%.3f--%.3f' % (P[pk]['470']['rms'], P[pk]['570']['rms']))
    put('ld' + lab, f1(P[pk]['470']['ld']) + '--' + f1(P[pk]['570']['ld']))
    md = R['modes'][0][1]; nd = md['nd']
    put('np' + lab, f1(md['trim']['cg'] - nd['Cma'] / nd['CLa'] * 44.13)); put('cla' + lab, f1(nd['CLa'])); put('cmad' + lab, f1(nd['Cmad'])); put('cmq' + lab, f1(nd['Cmq']))
    lat = md['lat']; rr = sorted([e for e in lat if not e['im']], key=lambda e: e['re'])
    put('tauRoll' + lab, f2(rr[0]['tau'])); dr = [e for e in lat if e['im']][0]; put('dutch' + lab, f1(dr['period'])); put('dutchZ' + lab, f2(dr['zeta']))
    ph = min([e for e in md['lon'] if e['im']], key=lambda e: e['wn']); put('phT' + lab, f1(ph['period'])); put('phZ' + lab, '%.3f' % ph['zeta'])
    put('phR' + lab, f2(np.exp(-2 * np.pi * ph['zeta'] / np.sqrt(1 - ph['zeta'] ** 2))))
    c = R['curves'][0][1]; a = np.array(c['alpha']); CL = np.array(c['CL']); m = (a > 0) & (a < 25)
    put('clmax' + lab, f2(CL[m].max())); put('aclmax' + lab, f0(a[m][np.argmax(CL[m])])); put('clThirty' + lab, f2(np.interp(30, a, CL)))
    # take-off
    to = {(x['fr'], x['re']): x for a_, x in R['aerotow']}
    lo = [to[k]['ev']['airborne'][1] for k in ((176, 176), (176, 0), (230, 0)) if to[k]['ev']['airborne']]
    put('lo' + lab, rng(lo, f0)); put('loDual' + lab, f0(to[(176, 176)]['ev']['airborne'][1]) if to[(176, 176)]['ev']['airborne'] else '--')
    hv = to[(230, 230)]['ev']['airborne']; put('loHeavy' + lab, '--' if hv is None else f0(hv[1]))
    g = R['ground'][0][1]; put('ground' + lab, rng([x['theta'] for x in g]))
    w = [x for a_, x in R['wind']]; put('windDth' + lab, f1(max(x['dtheta'] for x in w))); put('windOk' + lab, 'all' if all(x['ok'] for x in w) else 'not all')
    rb = R['robust'][0][1]; put('fuzzBad' + lab, str(rb['bad']))
    phs = {x['V']: x for a_, x in R['phugoid']}
    for V in (80, 90, 100, 110, 120, 140):
        z = phs[V]['zeta']; put('phz%s%s' % ({80: 'A', 90: 'B', 100: 'C', 110: 'D', 120: 'E', 140: 'F'}[V], lab), '--' if z is None else '%+.3f' % z)
    # roll
    r45 = {x['coord']: x['t'] for a_, x in R['roll45']}; put('rollRev' + lab, f1(r45[True]) + '--' + f1(r45[False]))
    r3 = [x['marks']['180'] for a_, x in R['roll360'] if not x['coord']]; put('rollHalf' + lab, rng(r3))
    # slip vs brakes at the same speed
    th = np.array([x for a_, x in R['slip'] if x['mode'] == 'slip'][0]['th']); ww = th[(th[:, 0] > 15) & (th[:, 0] < 28)]
    vt = ww[:, 9].mean(); bg = sorted([x for a_, x in R['brakeglide']], key=lambda x: x['V_tas'])
    sb = float(np.interp(vt, [x['V_tas'] for x in bg], [x['sink'] for x in bg]))
    put('slipBeta' + lab, f0(ww[:, 2].mean())); put('slipSink' + lab, f1(-ww[:, 5].mean())); put('slipBrake' + lab, f1(sb)); put('slipV' + lab, f0(ww[:, 1].mean()))
    put('slipRatio' + lab, f1(-ww[:, 5].mean() / sb))
    mp = R['massprops'][0][1]; put('cgDev' + lab, f2(max(abs(x['cg'] - x['cg_afftc']) for x in mp)))
put('paraCv', f2(PX['AFFTC nominal']['C_vert_rot'])); put('paraVz', f1(PX['AFFTC nominal']['Vz_tas'])); put('paraKt', f0(PX['AFFTC nominal']['Ve_kt']))
put('paraKappa', f2(PX['AFFTC nominal']['kappa'])); put('paraCvNine', f2(PX['v9 (sim at 6000 ft)']['C_vert_rot'])); put('paraCrA', f2(PX['v9_CR_45']) + '--' + f2(PX['v9_CR_50']))
PP = json.load(open('v9_params.json'))
put('parCN', f2(PP['CN90_wing'])); put('parCdc', f2(PP['cdc'])); put('parFlap', f2(PP['flap_eff']))
out = ['%% generated by mknumbers.py'] + ['\\newcommand{\\N%s}{%s}' % (k, v) for k, v in sorted(N.items())]
open('tab/numbers.tex', 'w').write('\n'.join(out) + '\n'); print(len(N), 'macros')
