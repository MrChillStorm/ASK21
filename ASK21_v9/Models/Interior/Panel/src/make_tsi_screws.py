#!/usr/bin/env python3
"""The turn indicator gets the screws of the other instruments: its own four screws (separate parts of the case, outside
its round bezel) are removed, and the panel shows the shared 57 mm screws (Instruments/GMeter/57mm_screws.ac) in that
hole, for the turn indicator and the transponder alike (panelF.xml, panelF-mi.xml).

Reads src/orig/tsi-5550.ac, writes Instruments/tsi-5550/tsi-5550.ac."""
import os
HERE = os.path.dirname(os.path.abspath(__file__))
SRC = os.path.join(HERE, 'orig', 'tsi-5550.ac')
DST = os.path.join(os.path.dirname(HERE), 'Instruments', 'tsi-5550', 'tsi-5550.ac')

def screw_faces(v, faces):
    """Indices of the faces of the screws: the parts of the case (faces joined by shared vertices) that are smaller
    than 10 mm and lie in its corners (file frame: x towards the pilot, y up, z = -right)."""
    par = list(range(len(faces)))
    def find(a):
        while par[a] != a:
            par[a] = par[par[a]]; a = par[a]
        return a
    owner = {}
    for i, f in enumerate(faces):
        for a in f:
            k = tuple(round(c, 5) for c in v[a])
            if k in owner: par[find(i)] = find(owner[k])
            else: owner[k] = i
    parts = {}
    for i in range(len(faces)):
        parts.setdefault(find(i), []).append(i)
    gone = set()
    for fs in parts.values():
        ys = [v[a][1] for i in fs for a in faces[i]]; zs = [v[a][2] for i in fs for a in faces[i]]
        if (max(ys) - min(ys) < 0.010 and max(zs) - min(zs) < 0.010
                and min(abs(y) for y in ys) > 0.018 and min(abs(z) for z in zs) > 0.018):
            gone |= set(fs)
    return gone

L = open(SRC).read().split('\n')
s = L.index('name "tsi.case"')
nvl = next(k for k in range(s, len(L)) if L[k].startswith('numvert'))
cnt = int(L[nvl].split()[1]); v = [tuple(map(float, L[nvl + 1 + k].split()[:3])) for k in range(cnt)]
b = nvl + 1 + cnt; n = int(L[b].split()[1]); j = b + 1; blocks = []
for _ in range(n):
    nr = int(L[j + 2].split()[1]); blocks.append(L[j:j + 3 + nr]); j += 3 + nr
gone = screw_faces(v, [[int(l.split()[0]) for l in blk[3:]] for blk in blocks])
keep = [blk for i, blk in enumerate(blocks) if i not in gone]
L = L[:b] + ['numsurf %d' % len(keep)] + [l for blk in keep for l in blk] + L[j:]
open(DST, 'w').write('\n'.join(L))
print('tsi-5550 removed', len(gone), 'screw faces, kept', len(keep))
