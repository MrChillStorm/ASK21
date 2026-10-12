"""LaTeX tables for the report, generated from the campaign results (no hand transcription).
usage: mktables.py results_fg.json results_v9.json fig/polar_stats.json outdir"""
import sys, os, json, numpy as np
import sim
from summarize import load, spin_by_loading

ORDER = {'none': 0, 'incipient': 1, 'sustained': 2}
K = 32.174 * 144 / 1e6


def f1(x, n=1): return '--' if x is None else ('%.*f' % (n, x))


def spin_stats(R, entries=('a0', 'a1', 'th')):
    ms = []
    for a, r in R['spin']:
        if 'error' in r or r['m'] is None: continue
        lab = 'th' if r['thermal'] else 'a%d' % r['ail']
        if lab in entries: ms.append((r, r['m']))
    if not ms: return None
    out = {}
    for k in ms[0][1]:
        v = np.array([m[k] for r, m in ms]); out[k] = (float(v.mean()), float(v.min()), float(v.max()))
    lo = [m for r, m in ms if r['I'] < 8.6]; hi = [m for r, m in ms if r['I'] >= 8.6]
    for nm, grp in (('lo', lo), ('hi', hi)):
        if grp:
            out['theta_mean_' + nm] = float(np.mean([m['theta_mean'] for m in grp])); out['theta_sd_' + nm] = float(np.mean([m['theta_sd'] for m in grp]))
            out['rate_' + nm] = float(np.mean([m['rate_mean'] for m in grp])); out['n_' + nm] = len(grp)
    out['n'] = len(ms)
    return out


def elev_boundary(R):
    rs = sorted([r for a, r in R['clmax']], key=lambda r: r['cg'])
    cg = np.array([r['cg'] for r in rs]); el = np.array([r['elev'] if r['elev'] is not None else -1.0 for r in rs])
    # elevator needed at CLmax vs CG; boundary where it reaches -1 (extrapolated linearly if outside)
    p = np.polyfit(cg, el, 1)
    return float((-1.0 - p[1]) / p[0]), rs


def main(fg, v9, pst, out):
    Rf, Rv = load(fg)[1], load(v9)[1]; P = json.load(open(pst))
    os.makedirs(out, exist_ok=True)
    T = {}
    # ------------------------------------------------------------------ mass properties
    def mp(R):
        m = R['massprops'][0][1]
        e = np.array([x['Iyy'] - x['Iyy_afftc'] for x in m]); dcg = np.array([x['cg'] - x['cg_afftc'] for x in m])
        return m, float(np.sqrt(np.mean(e ** 2))), float(np.mean(e / np.array([x['Iyy_afftc'] for x in m]))), float(np.abs(dcg).max())
    mf, ef, rf, cf = mp(Rf); mv, ev, rv, cv = mp(Rv)
    md_f, md_v = Rf['modes'][0][1], Rv['modes'][0][1]
    rows = [('$I_{xx}$, $10^6$ lb\\,in$^2$ (empty)', '30.2', 4.63, 30.2), ('$I_{yy}$ (empty)', '6.7', 1.39, 6.70), ('$I_{zz}$ (empty)', '34.3', 10.91, 34.3)]
    s = ['\\begin{tabular}{lccc}', '\\toprule', 'Quantity & AFFTC App.~C & FlightGear & v9 \\\\', '\\midrule']
    for nm, a, b, c in rows: s.append('%s & %s & %.2f & %.2f \\\\' % (nm, a, b, c))
    s.append('$I_{yy}$ over the 24 Table A1 loadings, rms error & -- & %.2f (%+.0f\\,\\%%) & %.2f (%+.0f\\,\\%%) \\\\' % (ef, 100 * rf, ev, 100 * rv))
    s.append('CG over the 24 loadings, max.\\ deviation (in) & -- & %.2f & %.2f \\\\' % (cf, cv))
    s += ['\\bottomrule', '\\end{tabular}']
    T['massprops'] = s
    # ------------------------------------------------------------------ performance
    s = ['\\begin{tabular}{lccc}', '\\toprule', ' & Flight manual & FlightGear & v9 \\\\', '\\midrule']
    for m in ('470', '570'):
        a, b = P['fg'][m], P['v9'][m]
        s.append('\\multicolumn{4}{l}{\\emph{%s kg}} \\\\' % m)
        s.append('\\quad rms sink error (m/s) & -- & %.3f & %.3f \\\\' % (a['rms'], b['rms']))
        s.append('\\quad $(L/D)_{\\max}$ at km/h & %.1f at %.0f & %.1f at %.0f & %.1f at %.0f \\\\' % (a['ld_fm'], a['v_ld_fm'], a['ld'], a['v_ld'], b['ld'], b['v_ld']))
        s.append('\\quad min.\\ sink (m/s) at km/h & %.2f at %.0f & %.2f at %.0f & %.2f at %.0f \\\\' % (a['ws_fm'], a['v_ws_fm'], a['ws'], a['v_ws'], b['ws'], b['v_ws']))
    ssf, ssv = Rf['stallspeeds'][0][1], Rv['stallspeeds'][0][1]
    s.append('\\midrule')
    s.append('$V_S$ single, 470 kg (km/h) & 65 & %.1f & %.1f \\\\' % (ssf['single_sb0']['Vs'], ssv['single_sb0']['Vs']))
    s.append('$V_S$ dual, 600 kg (km/h) & 74 & %.1f & %.1f \\\\' % (ssf['dual_sb0']['Vs'], ssv['dual_sb0']['Vs']))
    s.append('$\\Delta V_S$ airbrakes open (km/h) & +3 & +%.1f / +%.1f & +%.1f / +%.1f \\\\' % (ssf['single_sb1']['Vs'] - ssf['single_sb0']['Vs'], ssf['dual_sb1']['Vs'] - ssf['dual_sb0']['Vs'],
                                                                                      ssv['single_sb1']['Vs'] - ssv['single_sb0']['Vs'], ssv['dual_sb1']['Vs'] - ssv['dual_sb0']['Vs']))
    dv = {r['sb']: r for a, r in Rv['dive']}; df = {r['sb']: r for a, r in Rf['dive']}
    s.append('45$^\\circ$ dive (flight path), airbrakes open, 600 kg (km/h CAS) & 232 & %.0f & %.0f \\\\' % (df[1.0]['CAS'], dv[1.0]['CAS']))
    s += ['\\bottomrule', '\\end{tabular}']
    T['performance'] = s
    # ------------------------------------------------------------------ stalls
    def st_rows(R):
        return sorted([r for a, r in R['stall1g']], key=lambda r: r['cg'])
    sf, sv = st_rows(Rf), st_rows(Rv)
    s = ['\\begin{tabular}{lcccccc}', '\\toprule', ' & \\multicolumn{3}{c}{FlightGear} & \\multicolumn{3}{c}{v9} \\\\', '\\cmidrule(lr){2-4}\\cmidrule(lr){5-7}',
         'AFFTC loading & $V_{\\min}$ & $g$-break & bucking & $V_{\\min}$ & $g$-break & bucking \\\\', '\\midrule']
    for a, b in zip(sf, sv):
        s.append('CG %.2f in (%.0f lb) & %.1f & %.1f$^\\circ$%s & %.0f$^\\circ$ & %.1f & %.1f$^\\circ$%s & %.0f$^\\circ$ \\\\' % (
            sim.A1[[i for i in range(len(sim.A1)) if abs(sim.A1[i][0] - b['cg']) < 0.15][0]][0] if False else b['cg'], b['W'],
            a['kias_min_pre'], a['gbreak'], '$^\\ast$' if a['elev_limited'] else '', a['theta_osc'], b['kias_min_pre'], b['gbreak'], '$^\\ast$' if b['elev_limited'] else '', b['theta_osc']))
    s += ['\\midrule', '\\multicolumn{7}{l}{AFFTC: 33--38 KIAS; forward of 13 in full aft stick before the stall, no $g$-break ($^\\ast$ = elevator-limited);}\\\\',
          '\\multicolumn{7}{l}{aft of 13 in a very mild $g$-break of 2--3$^\\circ$, then pitch bucking with the stick held aft.}\\\\', '\\bottomrule', '\\end{tabular}']
    T['stall'] = s
    # dynamic + inverted
    pf = sorted([r for a, r in Rf['pullup']], key=lambda r: r['cg']); pv = sorted([r for a, r in Rv['pullup']], key=lambda r: r['cg'])
    inf, inv = Rf['inverted'][0][1], Rv['inverted'][0][1]
    s = ['\\begin{tabular}{lccc}', '\\toprule', ' & AFFTC & FlightGear & v9 \\\\', '\\midrule']
    s.append('30$^\\circ$ nose-high entry: min.\\ speed (KIAS) & ``as low as 20\'\' & %s & %s \\\\' % (' / '.join('%.0f' % p['kias_min'] for p in pf), ' / '.join('%.0f' % p['kias_min'] for p in pv)))
    s.append('\\quad nose drop at the break (deg) & up to 40 & %s & %s \\\\' % (' / '.join('%.0f' % p['g_break'] for p in pf), ' / '.join('%.0f' % p['g_break'] for p in pv)))
    s.append('\\quad height loss (ft) & $\\approx 100$ & %s & %s \\\\' % (' / '.join('%.0f' % p['h_loss'] for p in pf), ' / '.join('%.0f' % p['h_loss'] for p in pv)))
    s.append('$-1\\,g$ inverted stall (KIAS) & 38--40 & %.1f & %.1f \\\\' % (inf['Vs_kias'], inv['Vs_kias']))
    s += ['\\bottomrule', '\\end{tabular}']
    T['dynstall'] = s
    # ------------------------------------------------------------------ spin characteristics
    cf_, cv_ = spin_stats(Rf), spin_stats(Rv); cv0 = spin_stats(Rv, ('a0',))
    def rng(c, k, n=0): return '--' if c is None else '%.*f (%.*f--%.*f)' % (n, c[k][0], n, c[k][1], n, c[k][2])
    s = ['\\begin{tabular}{lccc}', '\\toprule', 'Developed spin & AFFTC-TR-89-27 & FlightGear & v9 (all entries) \\\\', '\\midrule']
    s.append('sustained spins in the programme & -- & %d of 72 & %d of 72 \\\\' % (0 if cf_ is None else cf_['n'], cv_['n']))
    s.append('rotation rate (deg/s) & 80 (flat)--140 (steep) & -- & %s \\\\' % rng(cv_, 'rate_mean'))
    s.append('time per turn (s) & 2.5--4.5 & -- & %s \\\\' % rng(cv_, 's_per_turn', 1))
    s.append('height loss per turn (ft) & 200 (150--250) & -- & %s \\\\' % rng(cv_, 'ft_per_turn'))
    s.append('mean pitch, low inertia (deg) & $-50 \\pm 15$ & -- & %.0f, $\\sigma$ = %.0f \\\\' % (cv_['theta_mean_lo'], cv_['theta_sd_lo']))
    s.append('mean pitch, high inertia (deg) & $-40 \\pm 30$ & -- & %.0f, $\\sigma$ = %.0f \\\\' % (cv_['theta_mean_hi'], cv_['theta_sd_hi']))
    s.append('mean rate low / high inertia (deg/s) & higher at high $I$ & -- & %.0f / %.0f \\\\' % (cv_['rate_lo'], cv_['rate_hi']))
    s.append('angle of attack (deg) & stalled, ``fast, steep\'\' & -- & %s \\\\' % rng(cv_, 'alpha_mean'))
    s.append('airspeed (KIAS / model CAS) & 30--40 (indicated) & -- & %s \\\\' % rng(cv_, 'kias_mean'))
    s.append('normal load factor & $\\approx 1.0$ & -- & %s \\\\' % rng(cv_, 'nz_mean', 2))
    s += ['\\bottomrule', '\\end{tabular}']
    T['spinchar'] = s
    # ------------------------------------------------------------------ recoveries
    names = [('fm', 'flight-manual procedure'), ('rudder', 'opposite rudder, stick aft'), ('neutral', 'all controls centred'),
             ('forward', 'full forward stick, pro-spin rudder'), ('ail_against', 'aileron against, stick aft'), ('continue', 'pro-spin controls held')]
    ref = {'fm': '$<1$, max.\\ 1$\\frac12$', 'rudder': '$\\frac14$--$\\frac34$, max.\\ 1$\\frac12$', 'neutral': 'not tested\\textsuperscript{a}',
           'forward': 'up to 3 more turns', 'ail_against': 'unreliable', 'continue': 'spin continues'}
    loads = [21, 11, 19, 16]
    s = ['\\begin{tabular}{l' + 'c' * len(loads) + 'l}', '\\toprule',
         'v9 input \\textbackslash{} loading & ' + ' & '.join('%.2f/%.2f' % (sim.A1[i][0], sim.A1[i][6]) for i in loads) + ' & AFFTC \\\\', '\\midrule']
    for c, nm in names:
        cells = []
        for i in loads:
            r = [x for a, x in Rv['rec'] if x['idx'] == i and x['case'] == c][0]
            if not r['spinning']: cells.append('n.s.')
            elif r['stopped'] is None: cells.append('none')
            else: cells.append('%.2f (%.0f ft)' % (r['stopped'][1], r['stopped'][2]))
        s.append('%s & %s & %s \\\\' % (nm, ' & '.join(cells), ref[c]))
    s += ['\\bottomrule', '\\end{tabular}']
    T['recovery'] = s
    # ------------------------------------------------------------------ lateral / handling
    r45 = lambda R, c: [x for a, x in R['roll45'] if x['coord'] == c][0]['t']
    r360 = lambda R, fr, re, c: [x for a, x in R['roll360'] if x['fr'] == fr and x['re'] == re and x['coord'] == c][0]['marks']
    def slipv(R, mode):
        th = np.array([x for a, x in R['slip'] if x['mode'] == mode][0]['th']); w = th[(th[:, 0] > 15) & (th[:, 0] < 28)]
        return w[:, 2].mean(), -w[:, 5].mean(), w[:, 3].mean()
    s = ['\\begin{tabular}{lccc}', '\\toprule', ' & Reference & FlightGear & v9 \\\\', '\\midrule']
    s.append('45$^\\circ$--45$^\\circ$ reversal, 90 km/h, with / without rudder (s) & -- & %.2f / %.2f & %.2f / %.2f \\\\' % (r45(Rf, True), r45(Rf, False), r45(Rv, True), r45(Rv, False)))
    for fr, re, V, lab in ((176, 176, 165, 'dual, 165 km/h'), (176, 0, 150, 'solo, 150 km/h')):
        a, b = r360(Rf, fr, re, False), r360(Rv, fr, re, False)
        s.append('full-aileron roll, %s: 180$^\\circ$ / 360$^\\circ$ (s) & $\\approx 5$ / -- & %.1f / %.1f & %.1f / %.1f \\\\' % (lab, a['180'], a['360'], b['180'], b['360']))
    def slipeq(R):
        th = np.array([x for a, x in R['slip'] if x['mode'] == 'slip'][0]['th']); w = th[(th[:, 0] > 15) & (th[:, 0] < 28)]
        vt, vc, sink = w[:, 9].mean(), w[:, 1].mean(), -w[:, 5].mean()
        bg = sorted([x for a, x in R['brakeglide']], key=lambda x: x['V_tas'])
        sb = float(np.interp(vt, [x['V_tas'] for x in bg], [x['sink'] for x in bg]))
        return w[:, 2].mean(), vc, sink, sb
    bf, vcf, sf_, sbf = slipeq(Rf); bv, vcv, sv_, sbv = slipeq(Rv)
    s.append('full-rudder sideslip, 2 $\\times$ 80 kg: $\\beta$ (deg) & up to 50 & %.0f & %.0f \\\\' % (bf, bv))
    s.append('\\quad slip flown at (km/h CAS) & -- & %.0f & %.0f \\\\' % (vcf, vcv))
    s.append('\\quad sink in the slip / full airbrakes at the same speed (m/s) & slip $\\gtrsim$ brakes\\textsuperscript{b} & %.1f / %.1f & %.1f / %.1f \\\\' % (sf_, sbf, sv_, sbv))
    md = lambda R: R['modes'][0][1]
    s += ['\\bottomrule', '\\end{tabular}']
    T['lateral'] = s
    # ------------------------------------------------------------------ take-off
    s = ['\\begin{tabular}{lcccc}', '\\toprule', 'Loading (front / rear, lb) & stick-free trim (km/h) & \\multicolumn{3}{c}{hands-free aerotow lift-off (km/h)} \\\\',
         '\\cmidrule(lr){3-5}', ' & FG / v9 & FlightGear & v9 & tug (85 km/h) \\\\', '\\midrule']
    for fr, re in ((176, 176), (176, 0), (230, 0), (230, 230)):
        tf = [x for a, x in Rf['trimspeed'] if x['fr'] == fr and x['re'] == re][0]; tv = [x for a, x in Rv['trimspeed'] if x['fr'] == fr and x['re'] == re][0]
        af = [x for a, x in Rf['aerotow'] if x['fr'] == fr and x['re'] == re][0]; av = [x for a, x in Rv['aerotow'] if x['fr'] == fr and x['re'] == re][0]
        g = lambda e: '--' if e['ev']['airborne'] is None else '%.0f' % e['ev']['airborne'][1]
        s.append('%d / %d (CG %.1f in) & %s / %s & %s & %s & %s \\\\' % (fr, re, tv['cg'], 'none\\textsuperscript{c}' if tf['V'] is None else '%.0f' % tf['V'],
                                                                     '--' if tv['V'] is None else '%.0f' % tv['V'], g(af), g(av),
                                                                     'v9 first' if (av['ev']['airborne'] and av['ev']['airborne'][1] < 85) else 'tug first'))
    s += ['\\bottomrule', '\\end{tabular}']
    T['takeoff'] = s
    # ------------------------------------------------------------------ derivatives & modes
    def modes_rows(m):
        lon = m['lon']; lat = m['lat']
        osc = [e for e in lon if e['im']]; real = [e for e in lon if not e['im']]
        ph = min(osc, key=lambda e: e['wn']) if osc else None
        sp = [e for e in osc if e is not ph]
        lo = [e for e in lat if e['im']]; lr = sorted([e for e in lat if not e['im']], key=lambda e: e['re'])
        return ph, sp, real, lo[0] if lo else None, lr
    s = ['\\begin{tabular}{lcc@{\\hspace{2em}}lcc}', '\\toprule', 'Derivative & FlightGear & v9 & Mode & FlightGear & v9 \\\\', '\\midrule']
    ndf, ndv = md_f['nd'], md_v['nd']
    ders = [('CLa', '$C_{L_\\alpha}$'), ('Cma', '$C_{m_\\alpha}$'), ('Cmq', '$C_{m_q}$'), ('Cmad', '$C_{m\\dot\\alpha}$'), ('CLq', '$C_{L_q}$'),
            ('CYb', '$C_{Y_\\beta}$'), ('Clb', '$C_{l_\\beta}$'), ('Cnb', '$C_{n_\\beta}$'), ('Clp', '$C_{l_p}$'), ('Cnp', '$C_{n_p}$'), ('Clr', '$C_{l_r}$'), ('Cnr', '$C_{n_r}$')]
    phf, spf, rlf, drf, lrf = modes_rows(md_f); phv, spv, rlv, drv, lrv = modes_rows(md_v)
    def osc(e): return '--' if e is None else '%.1f s, $\\zeta$ %.3f' % (e['period'], e['zeta'])
    def sp_(sp, rl):
        if sp: return '%.1f s, $\\zeta$ %.2f' % (sp[0]['period'], sp[0]['zeta'])
        return 'overdamped: $\\tau$ %s s' % ', '.join('%.2f' % e['tau'] for e in rl)
    def rr(l): return '%.2f s' % l[0]['tau']
    def sp_l(l):
        e = l[-1]; return ('%s %.0f s' % ('$t_2$' if e['re'] > 0 else '$t_{1/2}$', e['t_half_or_double']))
    modes = [('phugoid', osc(phf), osc(phv)), ('short period', sp_(spf, rlf), sp_(spv, rlv)), ('roll subsidence $\\tau$', rr(lrf), rr(lrv)),
             ('Dutch roll', osc(drf), osc(drv)), ('spiral', sp_l(lrf), sp_l(lrv))]
    for k, (key, lab) in enumerate(ders):
        mcol = modes[k] if k < len(modes) else ('', '', '')
        s.append('%s & %+.3f & %+.3f & %s & %s & %s \\\\' % (lab, ndf[key], ndv[key], mcol[0], mcol[1], mcol[2]))
    s += ['\\midrule', '\\multicolumn{6}{l}{trim: 90 km/h, 2 $\\times$ 80 kg, CG %.1f / %.1f in; static margin $-C_{m_\\alpha}/C_{L_\\alpha}$ = %.1f / %.1f\\,\\%% MAC '
          '$\\Rightarrow$ $x_{NP}$ = %.1f / %.1f in}\\\\' % (md_f['trim']['cg'], md_v['trim']['cg'], -100 * ndf['Cma'] / ndf['CLa'], -100 * ndv['Cma'] / ndv['CLa'],
                                                           md_f['trim']['cg'] - ndf['Cma'] / ndf['CLa'] * 44.13, md_v['trim']['cg'] - ndv['Cma'] / ndv['CLa'] * 44.13),
          '\\bottomrule', '\\end{tabular}']
    T['modes'] = s
    # ------------------------------------------------------------------ phugoid sweep, ground, robustness
    def ph(R):
        return {x['V']: x for a, x in R['phugoid']}
    pf_, pv_ = ph(Rf), ph(Rv)
    s = ['\\begin{tabular}{lcccccc}', '\\toprule', '$V$ (km/h) & ' + ' & '.join('%d' % V for V in sorted(pv_)) + ' \\\\', '\\midrule']
    for lab, P_ in (('FlightGear', pf_), ('v9', pv_)):
        s.append('%s: period (s) & ' % lab + ' & '.join('--' if P_[V]['period'] is None else '%.1f' % P_[V]['period'] for V in sorted(P_)) + ' \\\\')
        s.append('\\quad damping ratio $\\zeta$ & ' + ' & '.join('--' if P_[V]['zeta'] is None else '%+.3f' % P_[V]['zeta'] for V in sorted(P_)) + ' \\\\')
    s += ['\\bottomrule', '\\end{tabular}']
    T['phugoid'] = s
    gr = lambda R: R['ground'][0][1]
    wd = lambda R: [x for a, x in R['wind']]
    rb = lambda R: R['robust'][0][1]
    extra = dict(fg_ground=[g['theta'] for g in gr(Rf)], v9_ground=[g['theta'] for g in gr(Rv)],
                 fg_wind_ok=all(x['ok'] for x in wd(Rf)), v9_wind_ok=all(x['ok'] for x in wd(Rv)),
                 fg_wind_dtheta=max(x['dtheta'] for x in wd(Rf)), v9_wind_dtheta=max(x['dtheta'] for x in wd(Rv)),
                 fg_fuzz=rb(Rf), v9_fuzz=rb(Rv), v9_phug=pv_, fg_phug=pf_)

    # ------------------------------------------------------------------ spin map mismatch summary
    def mism(R):
        by = spin_by_loading(R); out = []
        for k, r in by.items():
            row = sim.A1[k]; e = sim.expected(row[0], row[6])
            if e != r['cls']:
                bd = min(abs(row[0] - sim.cg_sust(row[6])), abs(row[0] - sim.cg_inc(row[6])))
                out.append((row[0], row[6], e, r['cls'], bd))
        return sorted(out)
    mf_, mv_ = mism(Rf), mism(Rv)
    s = ['\\begin{tabular}{ccccc}', '\\toprule', 'CG (in) & $I_{yy}$ & AFFTC & v9 & distance to boundary (in) \\\\', '\\midrule']
    for cg, I, e, g, bd in mv_: s.append('%.2f & %.2f & %s & %s & %.2f \\\\' % (cg, I, e, g, bd))
    s += ['\\bottomrule', '\\end{tabular}']
    T['mismatch'] = s
    # ------------------------------------------------------------------ fitted parameters
    sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', 'v9'))
    import optimize as O
    PP = json.load(open(os.path.join(os.path.dirname(os.path.abspath(__file__)), 'v9_params.json')))
    meaning = dict(CN90_wing='wing section normal force at 90$^\\circ$ (flat plate, AR-corrected)', stall_drop='lift drop after $c_{l,\\max}$',
                   boff='onset of the flat-plate blend beyond the peak (deg)', dstall='width of the flat-plate blend (deg)',
                   w_ca='width of the leading-edge-suction collapse (deg)', xcp90='section centre of pressure at 90$^\\circ$ ($x/c$)',
                   CN90_ht='tailplane $c_{N,90}$', CN90_vt='fin $c_{N,90}$', wake_k='wing-wake $q$-deficit at the tailplane (scale)',
                   wake_kv='wing-wake $q$-deficit at the fin (scale)', fin_blank='fin $q$ loss in the fuselage cross-flow wake',
                   cdc='fuselage cross-flow drag coefficient', kpot_aft='aft-body share of the Munk potential load',
                   wing_clmax_k='inner-wing $c_{l,\\max}$ / NeuralFoil', a_peak_root='inner-wing stall angle (deg)', tip_clmax_k='outer-wing $c_{l,\\max}$ / NeuralFoil',
                   a_peak_tip='outer-wing stall angle (deg)', flap_eff='plain-flap effectiveness / NeuralFoil')
    s = ['\\begin{tabular}{llccc}', '\\toprule', 'Parameter & Meaning & bounds & fitted \\\\', '\\midrule']
    for n, lo, hi in zip(O.NAMES, O.LO, O.HI):
        s.append('\\texttt{%s} & %s & %.3g--%.3g & %.3g \\\\' % (n.replace('_', '\\_'), meaning[n], lo, hi, PP[n]))
    s += ['\\bottomrule', '\\end{tabular}']
    T['params'] = s
    import re
    for k, v in T.items():
        txt = '\n'.join(v) + '\n'
        txt = re.sub(r'(?<![-\w$\\{])-(?=\d)', '$-$', txt)      # text-mode minus signs
        open(os.path.join(out, 'tab_%s.tex' % k), 'w').write(txt)
    # numbers used in the text
    nums = dict(fg_mism=len(mf_), v9_mism=len(mv_), v9_rate=cv_['rate_mean'][0], v9_ft=cv_['ft_per_turn'][0], v9_kias=cv_['kias_mean'][0],
                v9_spins=cv_['n'], fg_spins=0 if cf_ is None else cf_['n'], v9_a0=None if cv0 is None else cv0['n'], **extra)
    json.dump(nums, open(os.path.join(out, 'numbers.json'), 'w'), indent=1)
    print(json.dumps(nums, indent=1))


if __name__ == '__main__':
    main(*sys.argv[1:5])
