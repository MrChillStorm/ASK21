import json, sys, sim
from summarize import load
Rf, Rv = load('results_fg.json')[1], load('results_v9.json')[1]
AB = {'none': '--', 'incipient': 'I', 'sustained': 'S'}
def outc(R, idx):
    d = {}
    for a, r in R['spin']:
        if r['idx'] != idx: continue
        k = 'th' if r['thermal'] else 'a%d' % r['ail']; d[k] = AB[r['cls']] + ('' if r['cls'] == 'none' else '\\,{\\scriptsize %.1f}' % r['turns'])
    return d
mpv = {m['idx']: m for m in Rv['massprops'][0][1]}; mpf = {m['idx']: m for m in Rf['massprops'][0][1]}
s = ['\\begin{tabular}{rrrrcc@{\\hspace{1.2em}}ccc@{\\hspace{1.2em}}ccc}', '\\toprule',
     '\\multicolumn{6}{c}{AFFTC Table A1 loading} & \\multicolumn{3}{c}{FlightGear} & \\multicolumn{3}{c}{v9} \\\\',
     '\\cmidrule(lr){1-6}\\cmidrule(lr){7-9}\\cmidrule(lr){10-12}',
     'CG & front & rear & tail/ckpt & $I_{yy}$ & expected & WL & WL+A & TH & WL & WL+A & TH \\\\', '\\midrule']
for i, row in sorted(enumerate(sim.A1), key=lambda t: t[1][0]):
    cg, fr, re, nt, nc, gw, I = row
    e = sim.expected(cg, I); f = outc(Rf, i); v = outc(Rv, i)
    s.append('%.2f & %d & %d & %d/%d & %.2f & %s & %s & %s & %s & %s & %s & %s \\\\' % (cg, fr, re, nt, nc, I, AB[e], f['a0'], f['a1'], f['th'], v['a0'], v['a1'], v['th']))
s += ['\\bottomrule', '\\end{tabular}']
open('tab/tab_a1.tex', 'w').write('\n'.join(s) + '\n'); print('ok')
