"""Figures for the report (PDF, vector).  usage: figures.py results_fg.json results_v9.json outdir"""
import sys, os, json, csv, numpy as np
import matplotlib
matplotlib.use('pdf')
import matplotlib.pyplot as plt
from matplotlib.patches import Polygon, Rectangle
import sim
sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', 'v9'))

plt.rcParams.update({'font.family': 'serif', 'font.serif': ['STIX Two Text', 'STIXGeneral', 'DejaVu Serif'], 'mathtext.fontset': 'stix',
                     'font.size': 8.5, 'axes.labelsize': 8.5, 'axes.titlesize': 9, 'legend.fontsize': 7.5, 'xtick.labelsize': 7.5,
                     'ytick.labelsize': 7.5, 'axes.linewidth': 0.6, 'lines.linewidth': 1.1, 'grid.linewidth': 0.3, 'grid.alpha': 0.5,
                     'axes.grid': True, 'figure.dpi': 150, 'savefig.bbox': 'tight', 'savefig.pad_inches': 0.02,
                     'axes.spines.top': False, 'axes.spines.right': False})
C_FG, C_V9, C_REF = '#b2182b', '#2166ac', '#222222'
LAB_FG, LAB_V9 = 'FlightGear (rev. 8166)', 'v9 component model'
W1, W2 = 3.35, 6.9          # single / double column width (in)


def load(fn):
    d = json.load(open(fn)); R = {}
    for r in d['results']:
        R.setdefault(r['kind'], []).append((r['args'], r['res']))
    return R


def save(fig, out, name):
    fig.savefig(os.path.join(out, name + '.pdf')); plt.close(fig)


def get(R, kind, **match):
    for a, r in R[kind]:
        if all(r.get(k) == v for k, v in match.items()): return r
    return None


# ----------------------------------------------------------------------------------------------- geometry
def fig_geometry(out):
    import geometry as G
    fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(W2, 5.0), gridspec_kw=dict(height_ratios=[1.25, 1], hspace=0.38))
    # planform (x aft = down in plan view -> plot x horizontally, y vertical)
    for st in G.wing_strips():
        for sgn in (1, -1):
            y0, y1 = st['y0'], st['y1']
            ys = np.array([y0, y1, y1, y0]); xl = np.array([G.x_le(y0), G.x_le(y1), G.x_le(y1) + G.chord(y1), G.x_le(y0) + G.chord(y0)])
            col = '#d9e6f2' if not st['ail'] else '#f2dcd9'
            ax1.add_patch(Polygon(np.c_[sgn * ys, xl], closed=True, fc=col, ec='0.35', lw=0.4))
            ax1.plot(sgn * st['y'], st['x_c34'], '.', color=C_V9, ms=2.5)
            if st['brake'] > 0:
                a, b = max(y0, G.BRAKE_Y[0]), min(y1, G.BRAKE_Y[1])
                ax1.add_patch(Rectangle((sgn * a if sgn > 0 else -b, (G.x_le(a) + 0.40 * G.chord(a)) - 0.03), b - a, 0.06, fc='0.2', ec='none'))
    for h in G.ht_strips():
        for sgn in (1, -1):
            y0, y1 = h['y'] - h['dy'] / 2, h['y'] + h['dy'] / 2
            x0 = h['x_c4'] - 0.25 * h['c']
            ax1.add_patch(Polygon(np.c_[sgn * np.array([y0, y1, y1, y0]), np.array([x0, x0, x0 + h['c'], x0 + h['c']])], fc='#e3f0d9', ec='0.35', lw=0.4))
            ax1.plot(sgn * h['y'], h['x_c34'], '.', color=C_V9, ms=2.5)
    F = G.fuselage()
    ax1.add_patch(Polygon(np.c_[np.r_[F['ws'] / 2, -F['ws'][::-1] / 2], np.r_[F['xs'], F['xs'][::-1]]], fc='#eeeeee', ec='0.25', lw=0.5, zorder=3))
    ax1.set_aspect('equal'); ax1.set_xlim(-8.8, 8.8); ax1.set_ylim(6.1, -2.8)
    ax1.set_xlabel('$y$ (m)'); ax1.set_ylabel('$X$ aft of datum (m)'); ax1.grid(False)
    ax1.set_title('Plan view: 22 wing strips (aileron strips shaded red, airbrakes black), 6 tailplane strips; dots = collocation points (3/4 chord)', fontsize=7.5)
    # side view: fuselage contour, stations, fin strips
    ax2.plot(F['xs'], F['zs'] + F['hs'] / 2, color='0.2', lw=0.8); ax2.plot(F['xs'], F['zs'] - F['hs'] / 2, color='0.2', lw=0.8)
    for x in G.FUS_STATIONS: ax2.axvline(x, color='0.6', lw=0.3)
    for st in F['stations']: ax2.plot(st['x'], st['z'], 's', color=C_FG, ms=2.5)
    for v in G.vt_strips():
        z0, z1 = v['z'] - v['dz'] / 2, v['z'] + v['dz'] / 2; x0 = v['x_c4'] - 0.25 * v['c']
        ax2.add_patch(Polygon(np.c_[[x0, x0 + v['c'], x0 + v['c'], x0], [z0, z0, z1, z1]], fc='#f6ecd2', ec='0.35', lw=0.4))
    ax2.plot([G.HT_X_LE_ROOT, G.HT_X_LE_ROOT + G.HT_CR], [G.HT_Z, G.HT_Z], color='0.1', lw=2)
    c0 = G.chord(0); ax2.plot([0, c0], [0, -c0 * np.tan(np.radians(G.I_ROOT))], color=C_V9, lw=2)
    for nm, (x, y, z) in (('front pilot', G.PILOT_FRONT), ('rear pilot', G.PILOT_REAR), ('tail ballast', G.TAIL_BALLAST), ('cockpit ballast', G.CKPT_BALLAST)):
        ax2.plot(x - G.XS0, z - G.ZS0, 'v', color='k', ms=3.5); ax2.annotate(nm, (x - G.XS0, z - G.ZS0), xytext=(2, -8), textcoords='offset points', fontsize=6)
    ax2.set_aspect('equal'); ax2.set_xlim(-2.8, 6.0); ax2.set_ylim(-0.8, 2.0); ax2.set_xlabel('$X$ aft of datum (m)'); ax2.set_ylabel('$Z$ (m)'); ax2.grid(False)
    ax2.set_title('Side view: 11 slender-body stations (squares), 3 fin strips, T-tail, root chord at its rigging incidence, AFFTC load stations', fontsize=7.5)
    save(fig, out, 'geometry')


def fig_sections(out):
    import airfoils as AF, json as js
    P = js.load(open(os.path.join(os.path.dirname(os.path.abspath(__file__)), 'v9_params.json')))
    T = AF.build_all(wing_clmax_k=P['wing_clmax_k'], tip_clmax_k=P['tip_clmax_k'], a_peak_root=P['a_peak_root'], a_peak_tip=P['a_peak_tip'],
                     drop=P['stall_drop'], CN90_wing=P['CN90_wing'], flap_eff=P['flap_eff'], k_sep=0.05, boff=P['boff'], dstall=P['dstall'],
                     xcp90=P['xcp90'], CN90_ht=P['CN90_ht'], CN90_vt=P['CN90_vt'], w_ca=P['w_ca'], ca_sep=0.02)
    fig, axs = plt.subplots(1, 3, figsize=(W2, 2.3))
    for key, lab, col in (('wroot', 'FX S 02-196 (root)', C_V9), ('wout4', 'FX 60-126 (tip)', '#5aae61'), ('ht', 'FX 71-L-150/30 (tail)', '#8c510a')):
        tb = T[key]; j0 = list(tb['delta']).index(0.0); a = tb['alpha']
        axs[0].plot(a, tb['cl'][:, j0], color=col, label=lab); axs[1].plot(a, tb['cd'][:, j0], color=col)
        a_r = np.radians(a); cn = tb['cl'][:, j0] * np.cos(a_r) + tb['cd'][:, j0] * np.sin(a_r); ca = tb['cd'][:, j0] * np.cos(a_r) - tb['cl'][:, j0] * np.sin(a_r)
        axs[2].plot(a, ca, color=col)
    for ax, yl in zip(axs, ('$c_l$', '$c_d$', '$c_a$ (chordwise, + aft)')):
        ax.set_xlim(-180, 180); ax.set_xticks(np.arange(-180, 181, 90)); ax.set_xlabel(r'$\alpha$ (deg)'); ax.set_ylabel(yl)
    axs[2].set_xlim(-30, 60); axs[2].set_xticks(np.arange(-30, 61, 15)); axs[2].set_ylim(-0.4, 0.5)
    axs[0].legend(loc='lower center', frameon=False, fontsize=6.5)
    fig.tight_layout(); save(fig, out, 'sections')


def fig_curves(Rf, Rv, out):
    fig, axs = plt.subplots(1, 3, figsize=(W2, 2.4))
    for R, col, lab in ((Rf, C_FG, LAB_FG), (Rv, C_V9, LAB_V9)):
        c = R['curves'][0][1]; a = np.array(c['alpha'])
        axs[0].plot(a, c['CL'], color=col, label=lab); axs[1].plot(a, c['CD'], color=col)
        axs[2].plot(a, c['Cm'], color=col); axs[2].plot(a, c['Cm_up'], color=col, ls='--', lw=0.7); axs[2].plot(a, c['Cm_dn'], color=col, ls=':', lw=0.7)
    for ax, yl in zip(axs, ('$C_L$', '$C_D$', r'$C_m$ about CG (12.1–12.3 in)')):
        ax.set_xlabel(r'$\alpha$ (deg)'); ax.set_ylabel(yl); ax.set_xlim(-30, 90)
    axs[2].set_ylim(-1.5, 1.0)
    axs[0].legend(frameon=False, loc='lower right', fontsize=6.5)
    axs[2].text(0.98, 0.95, 'solid: $\\delta_e$ neutral\ndashed: full up\ndotted: full down', transform=axs[2].transAxes, ha='right', va='top', fontsize=6)
    fig.tight_layout(); save(fig, out, 'curves')


def fig_lateral(Rf, Rv, out):
    fig, axs = plt.subplots(1, 3, figsize=(W2, 2.2))
    for R, col, lab in ((Rf, C_FG, LAB_FG), (Rv, C_V9, LAB_V9)):
        c = R['curves'][0][1]; b = np.array(c['beta'])
        axs[0].plot(b, c['CY'], color=col, label=lab); axs[1].plot(b, c['Cl'], color=col); axs[2].plot(b, c['Cn'], color=col)
    for ax, yl in zip(axs, ('$C_Y$', '$C_l$', '$C_n$')):
        ax.set_xlabel(r'$\beta$ (deg), $\alpha = 4^\circ$'); ax.set_ylabel(yl)
    axs[0].legend(frameon=False, fontsize=6.5)
    fig.tight_layout(); save(fig, out, 'lateral')


def fig_polar(Rf, Rv, out):
    rows = list(csv.DictReader(open(os.path.join(os.path.dirname(os.path.abspath(__file__)), 'polar_fm.csv'))))
    fig, axs = plt.subplots(1, 2, figsize=(W2, 2.6))
    for ax, m in zip(axs, (470, 570)):
        fm = np.array([(float(r['speed_kmh']), float(r['sink_m_s'])) for r in rows if int(r['mass_kg']) == m])
        ax.plot(fm[:, 0], -fm[:, 1], 'o', color=C_REF, ms=3, mfc='none', label='flight manual polar')
        for R, col, lab in ((Rf, C_FG, LAB_FG), (Rv, C_V9, LAB_V9)):
            p = np.array(get(R, 'polar', mass=m)['pts'])
            ax.plot(p[:, 0], -p[:, 1], color=col, label=lab)
        ax.set_xlim(60, 210); ax.set_ylim(-4.0, 0); ax.set_xlabel('$V$ (km/h, EAS)'); ax.set_ylabel('vertical speed (m/s)')
        ax.set_title('%d kg' % m)
    axs[0].legend(frameon=False, loc='lower left')
    fig.tight_layout(); save(fig, out, 'polar')


def polar_rms(R):
    rows = list(csv.DictReader(open(os.path.join(os.path.dirname(os.path.abspath(__file__)), 'polar_fm.csv'))))
    out = {}
    for m in (470, 570):
        p = np.array(get(R, 'polar', mass=m)['pts'])
        fm = np.array([(float(r['speed_kmh']), float(r['sink_m_s'])) for r in rows if int(r['mass_kg']) == m])
        s = np.interp(fm[:, 0], p[:, 0], p[:, 1])
        ld = fm[:, 0] / 3.6 / fm[:, 1]; ldm = p[:, 0] / 3.6 / p[:, 1]
        out[m] = dict(rms=float(np.sqrt(np.mean((s - fm[:, 1]) ** 2))), ld_fm=float(ld.max()), v_ld_fm=float(fm[np.argmax(ld), 0]),
                      ld=float(ldm.max()), v_ld=float(p[np.argmax(ldm), 0]), ws=float(p[:, 1].min()), v_ws=float(p[np.argmin(p[:, 1]), 0]),
                      ws_fm=float(fm[:, 1].min()), v_ws_fm=float(fm[np.argmin(fm[:, 1]), 0]))
    return out


def fig_clmax(Rf, Rv, out):
    fig, ax = plt.subplots(figsize=(W1, 2.4))
    for R, col, lab, mk in ((Rf, C_FG, LAB_FG, 's'), (Rv, C_V9, LAB_V9, 'o')):
        rs = sorted([r for a, r in R['clmax']], key=lambda r: r['cg'])
        cg = [r['cg'] for r in rs]; cl = [r['CLmax'] for r in rs]
        ax.plot(cg, cl, color=col, lw=0.8)
        for r in rs:
            ax.plot(r['cg'], r['CLmax'], mk, ms=3.5, color=col, mfc=col if r['elev_limited'] else 'white')
        ax.plot([], [], mk, color=col, label=lab)
    ax.axvline(13.0, color=C_REF, ls='--', lw=0.7)
    ax.set_ylim(1.25, 1.62)
    ax.text(13.1, 1.395, '$\\leftarrow$ AFFTC: elevator-limited\n    forward of 13 in', ha='left', va='center', fontsize=6)
    ax.set_xlabel('CG (in aft of datum)'); ax.set_ylabel(r'trimmed $C_{L,\max}$')
    ax.legend(frameon=False, fontsize=6.5, loc='lower right', title='filled = elevator-limited', title_fontsize=6)
    fig.tight_layout(); save(fig, out, 'clmax')


def boundaries(ax):
    I = np.linspace(7.5, 9.5, 10)
    ax.plot(sim.cg_inc(I), I, color=C_REF, ls='--', lw=0.8); ax.plot(sim.cg_sust(I), I, color=C_REF, lw=0.8)


def fig_spinmap(Rf, Rv, out):
    ORDER = {'none': 0, 'incipient': 1, 'sustained': 2, 'error': -1}
    MK = {'none': ('o', 'white'), 'incipient': ('o', '0.6'), 'sustained': ('o', 'k'), 'error': ('x', 'r')}
    fig, axs = plt.subplots(1, 3, figsize=(W2, 2.7), sharey=True)
    # AFFTC
    ax = axs[0]; boundaries(ax)
    for row in sim.A1:
        e = sim.expected(row[0], row[6]); m, fc = MK[e]; ax.plot(row[0], row[6], m, mfc=fc, mec='k', ms=4.5)
    ax.set_title('AFFTC-TR-89-27 (boundaries, Fig. 5)')
    for ax, R, ttl in ((axs[1], Rf, LAB_FG), (axs[2], Rv, LAB_V9)):
        boundaries(ax); by = {}
        for a, r in R['spin']:
            if 'error' in r: continue
            if r['idx'] not in by or ORDER[r['cls']] > ORDER[by[r['idx']]['cls']]: by[r['idx']] = r
        bad = 0
        for k, r in by.items():
            row = sim.A1[k]; m, fc = MK[r['cls']]; e = sim.expected(row[0], row[6])
            ax.plot(row[0], row[6], m, mfc=fc, mec='k', ms=4.5)
            if e != r['cls']: ax.plot(row[0], row[6], 'o', mfc='none', mec='#d7191c', ms=8, mew=0.9); bad += 1
        ax.set_title('%s: %d / 24 differ' % (ttl, bad))
    for ax in axs:
        ax.set_xlim(12.0, 18.7); ax.set_xlabel('CG (in aft of datum)')
    axs[0].set_ylabel(r'$I_{yy}$ ($10^6$ lb in$^2$, AFFTC loading)')
    from matplotlib.lines import Line2D
    h = [Line2D([], [], marker='o', ls='', mfc=fc, mec='k', ms=4.5, label=l) for l, fc in (('no spin', 'white'), ('incipient', '0.6'), ('sustained', 'k'))]
    h.append(Line2D([], [], marker='o', ls='', mfc='none', mec='#d7191c', ms=8, label='differs from AFFTC'))
    h.append(Line2D([], [], color=C_REF, ls='--', lw=0.8, label='incipient boundary')); h.append(Line2D([], [], color=C_REF, lw=0.8, label='sustained boundary'))
    fig.legend(handles=h, loc='lower center', ncol=6, frameon=False, bbox_to_anchor=(0.5, -0.04))
    fig.tight_layout(rect=(0, 0.06, 1, 1)); save(fig, out, 'spinmap')


def th_of(R, idx, ail=0.0, thermal=True):
    for a, r in R['spin']:
        if 'error' not in r and r['idx'] == idx and r['ail'] == ail and r['thermal'] == thermal:
            return np.array(r['th']), r
    return None, None


def fig_spin_th(Rf, Rv, out, idx=17, idx_hi=19):
    # cols: t, kias, alpha, theta, phi, r, psi, dh, beta, nz, p, q, Vkmh, el, rud, ail
    fig, axs = plt.subplots(4, 2, figsize=(W2, 5.6), sharex='col')
    for j, (R, col, lab) in enumerate(((Rv, C_V9, LAB_V9), (Rf, C_FG, LAB_FG))):
        for idx_, ls in ((idx, '-'), (idx_hi, '--')):
            th, r = th_of(R, idx_)
            if th is None: continue
            t = th[:, 0] - r['stall_t']; s = t > -4
            row = sim.A1[idx_]
            lb = 'CG %.2f in, $I_{yy}$ %.2f' % (row[0], row[6])
            axs[0, j].plot(t[s], th[s, 3], color=col, ls=ls, label=lb); axs[1, j].plot(t[s], np.abs(th[s, 5]), color=col, ls=ls)
            axs[2, j].plot(t[s], th[s, 2], color=col, ls=ls); axs[3, j].plot(t[s], th[s, 7], color=col, ls=ls)
        axs[0, j].set_title(lab)
        axs[0, j].axhspan(-65, -35, color='0.85', zorder=0); axs[1, j].axhspan(80, 140, color='0.85', zorder=0)
        axs[0, j].legend(frameon=False, fontsize=6, loc='lower left')
    axs[0, 0].set_ylabel(r'$\theta$ (deg)'); axs[1, 0].set_ylabel(r'$|r|$ (deg/s)'); axs[2, 0].set_ylabel(r'$\alpha$ (deg)'); axs[3, 0].set_ylabel(r'$\Delta h$ (ft)')
    for ax in axs[3]: ax.set_xlabel('time after stall (s)')
    axs[0, 0].text(0.99, 0.03, 'grey: AFFTC pitch range', transform=axs[0, 0].transAxes, ha='right', fontsize=6)
    axs[1, 0].text(0.99, 0.97, 'grey: AFFTC 80–140 deg/s', transform=axs[1, 0].transAxes, ha='right', va='top', fontsize=6)
    fig.tight_layout(); save(fig, out, 'spin_th')


def fig_recovery(Rf, Rv, out):
    cases = ('fm', 'rudder', 'neutral', 'forward', 'ail_against')
    names = {'fm': 'flight-manual\nprocedure', 'rudder': 'opposite rudder,\nstick aft', 'neutral': 'controls\ncentred',
             'forward': 'forward stick,\npro-spin rudder', 'ail_against': 'aileron\nagainst'}
    loads = [21, 11, 19, 16]
    fig, ax = plt.subplots(figsize=(W2, 2.5))
    w = 0.18
    for j, i in enumerate(loads):
        for k, c in enumerate(cases):
            r = get(Rv, 'rec', idx=i, case=c)
            x = k + (j - 1.5) * w
            if r is None or not r['spinning']: continue
            if r['stopped'] is None: ax.bar(x, 3.2, w * 0.9, color='none', ec=C_V9, hatch='///', lw=0.5)
            else: ax.bar(x, r['stopped'][1], w * 0.9, color=C_V9, alpha=0.35 + 0.15 * j)
    ax.axhline(1.0, color=C_REF, ls='--', lw=0.7); ax.axhline(1.5, color=C_REF, ls=':', lw=0.7)
    ax.text(-0.45, 1.02, 'JAR-22: 1 turn', fontsize=6, ha='left', va='bottom'); ax.text(-0.45, 1.52, 'AFFTC max. with FM procedure: 1.5', fontsize=6, ha='left', va='bottom')
    ax.set_xticks(range(len(cases))); ax.set_xticklabels([names[c] for c in cases], fontsize=7)
    ax.set_ylabel('turns until rotation stops'); ax.set_ylim(0, 4.0)
    ax.text(0.30, 0.97, 'v9; bars = CG 17.42 / 14.20 / 16.27 / 15.50 in (light to dark); hatched = no recovery in 20 s.\n'
                        'FlightGear model: no developed spin to recover from at any loading.', transform=ax.transAxes, va='top', fontsize=6.5)
    fig.tight_layout(); save(fig, out, 'recovery')


def fig_stall(Rf, Rv, out):
    fig, axs = plt.subplots(3, 2, figsize=(W2, 4.2), sharex=True)
    for j, idx in enumerate((0, 17)):
        for R, col, lab in ((Rf, C_FG, LAB_FG), (Rv, C_V9, LAB_V9)):
            r = get(R, 'stall1g_th', cg=None) if False else [x for a, x in R['stall1g_th'] if a[0] == idx][0]
            th = np.array(r['th']); t = th[:, 0] - (r['t_full'] or 0) - 1.0
            s = t > -25
            axs[0, j].plot(t[s], th[s, 1], color=col, label=lab); axs[1, j].plot(t[s], th[s, 3], color=col); axs[2, j].plot(t[s], th[s, 2], color=col)
        axs[0, j].set_title('CG %.2f in (%s)' % (sim.A1[idx][0], 'forward' if idx == 0 else 'aft'))
    axs[0, 0].set_ylabel('KIAS'); axs[1, 0].set_ylabel(r'$\theta$ (deg)'); axs[2, 0].set_ylabel(r'$\alpha$ (deg)')
    for ax in axs[2]: ax.set_xlabel('time from full aft stick (s)')
    axs[0, 0].legend(frameon=False, fontsize=6.5)
    fig.tight_layout(); save(fig, out, 'stall')


def fig_roll(Rf, Rv, out):
    fig, ax = plt.subplots(figsize=(W1, 2.3))
    for R, col, lab in ((Rf, C_FG, LAB_FG), (Rv, C_V9, LAB_V9)):
        for coord, ls in ((True, '-'), (False, '--')):
            r = get(R, 'roll45', coord=coord); th = np.array(r['th']); t0 = th[0, 0] + 1.0
            # first row is 1 s before the reversal
            ax.plot(th[:, 0] - t0, th[:, 4], color=col, ls=ls, label=lab if coord else None)
    ax.axhline(45, color='0.5', lw=0.5); ax.axhline(-45, color='0.5', lw=0.5)
    ax.set_xlim(-0.5, 6.5); ax.set_xlabel('time (s)'); ax.set_ylabel(r'bank $\phi$ (deg)')
    ax.legend(frameon=False, fontsize=6.5, loc='lower right'); ax.text(0.02, 0.95, 'solid: with rudder\ndashed: aileron only', transform=ax.transAxes, va='top', fontsize=6)
    fig.tight_layout(); save(fig, out, 'roll')


def fig_slip(Rf, Rv, out):
    fig, axs = plt.subplots(1, 2, figsize=(W2, 2.2))
    for R, col, lab in ((Rf, C_FG, LAB_FG), (Rv, C_V9, LAB_V9)):
        for mode, ls in (('slip', '-'), ('brakes', '--')):
            th = np.array(get(R, 'slip', mode=mode)['th'])
            axs[0].plot(th[:, 0], -th[:, 5], color=col, ls=ls, label='%s, %s' % (lab, 'full-rudder slip' if mode == 'slip' else 'full airbrakes'))
            if mode == 'slip': axs[1].plot(th[:, 0], th[:, 2], color=col)
    axs[0].set_ylabel('sink rate (m/s)'); axs[1].set_ylabel(r'sideslip $\beta$ (deg)')
    for ax in axs: ax.set_xlabel('time (s)')
    axs[1].axhline(50, color=C_REF, ls='--', lw=0.7); axs[1].text(1, 51, 'AFFTC: up to 50 deg', fontsize=6)
    axs[0].legend(frameon=False, fontsize=6, loc='upper right')
    fig.tight_layout(); save(fig, out, 'slip')


def fig_takeoff(Rf, Rv, out):
    # cols: t, V, vtug, theta, alpha, h, F, wow0, wow1, wow2
    fig, axs = plt.subplots(1, 2, figsize=(W2, 2.3))
    for R, col, lab in ((Rf, C_FG, LAB_FG), (Rv, C_V9, LAB_V9)):
        r = get(R, 'aerotow', fr=176, re=176); th = np.array(r['th'])
        axs[0].plot(th[:, 0], th[:, 3], color=col, label=lab); axs[1].plot(th[:, 1], th[:, 5], color=col)
        tlo = r['ev']['tug_lo']
        if tlo: axs[0].axvline(tlo, color=col, lw=0.5, ls=':')
    axs[1].axvline(85, color=C_REF, ls=':', lw=0.7); axs[1].text(84, 0.95 * axs[1].get_ylim()[1], 'tug lifts off', ha='right', va='top', fontsize=6)
    axs[1].axvline(75, color=C_REF, ls='--', lw=0.7); axs[1].text(74, 0.6 * axs[1].get_ylim()[1], 'FM: take-off\nat ~75 km/h', ha='right', fontsize=6)
    axs[0].set_xlabel('time (s)'); axs[0].set_ylabel(r'pitch $\theta$ (deg)'); axs[1].set_xlabel('IAS (km/h)'); axs[1].set_ylabel('wheel height (m)')
    axs[0].legend(frameon=False, fontsize=6.5)
    fig.tight_layout(); save(fig, out, 'takeoff')


def fig_modes(Rf, Rv, out):
    fig, axs = plt.subplots(1, 2, figsize=(W2, 2.3))
    for R, col, lab, mk in ((Rf, C_FG, LAB_FG, 's'), (Rv, C_V9, LAB_V9, 'o')):
        md = R['modes'][0][1]
        for ax, key in zip(axs, ('lon', 'lat')):
            for e in md[key]:
                ax.plot(e['re'], e['im'], mk, color=col, ms=4, mfc='none')
                if e['im']: ax.plot(e['re'], -e['im'], mk, color=col, ms=4, mfc='none')
        axs[0].plot([], [], mk, color=col, mfc='none', label=lab)
    axs[0].set_title('longitudinal'); axs[1].set_title('lateral–directional')
    for ax in axs:
        ax.axvline(0, color='k', lw=0.5); ax.set_xlabel(r'Re $\lambda$ (1/s)'); ax.set_ylabel(r'Im $\lambda$ (rad/s)'); ax.set_xscale('symlog', linthresh=0.1)
    axs[0].legend(frameon=False, fontsize=6.5)
    fig.tight_layout(); save(fig, out, 'modes')


def fig_inertia(Rf, Rv, out):
    fig, ax = plt.subplots(figsize=(W1, 2.3))
    for R, col, lab, mk in ((Rf, C_FG, LAB_FG, 's'), (Rv, C_V9, LAB_V9, 'o')):
        mp = R['massprops'][0][1]
        ax.plot([m['Iyy_afftc'] for m in mp], [m['Iyy'] for m in mp], mk, color=col, ms=3.5, mfc='none', label=lab)
    lim = [1.5, 10]; ax.plot(lim, lim, color='k', lw=0.5); ax.set_xlim(7.5, 9.5)
    ax.set_xlabel(r'AFFTC $I_{yy}$ ($10^6$ lb in$^2$)'); ax.set_ylabel(r'model $I_{yy}$ ($10^6$ lb in$^2$)'); ax.set_ylim(0, 10)
    ax.legend(frameon=False, fontsize=6.5)
    fig.tight_layout(); save(fig, out, 'inertia')


if __name__ == '__main__':
    Rf, Rv, out = load(sys.argv[1]), load(sys.argv[2]), sys.argv[3]
    os.makedirs(out, exist_ok=True)
    for fn in (fig_geometry, fig_sections):
        try: fn(out)
        except Exception as e: print('fig', fn.__name__, 'failed', repr(e))
    for fn in (fig_curves, fig_lateral, fig_polar, fig_clmax, fig_spinmap, fig_spin_th, fig_recovery, fig_stall, fig_roll, fig_slip, fig_takeoff, fig_modes, fig_inertia):
        try: fn(Rf, Rv, out)
        except Exception as e:
            import traceback; traceback.print_exc(); print('fig', fn.__name__, 'failed', repr(e))
    json.dump(dict(fg=polar_rms(Rf), v9=polar_rms(Rv)), open(os.path.join(out, 'polar_stats.json'), 'w'), indent=1)
    print('figures written to', out)
