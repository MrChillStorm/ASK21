#!/usr/bin/env python3
"""Black leather for the glare shields above the instrument panels: their faces move into a new object 'glareshield' of
Models/Interior/interior.ac, which uses the leather texture of the panels.
The faces come from the objects 'interior' and 'canopy.F.frame.int' of interior.ac and from the collar of the fuselage
(objects 'fuselage' and 'fuselage-mi' of ask21.ac). Reads copies of the originals from src/orig/."""
import os, shutil
HERE = os.path.dirname(os.path.abspath(__file__))
MODELS = os.path.normpath(os.path.join(HERE, '..', '..', '..'))
TILE = 0.040

def backup(path):
    orig = os.path.join(HERE, 'orig', os.path.basename(path))
    if not os.path.exists(orig):
        shutil.copy(path, orig)
    return open(orig).read().split('\n')

def take(L, name, test):
    """Remove the faces of object `name` for which test(points) is true. Returns the lines and the removed faces
    as (vertex list of the object, [(surf, [vertex index])])."""
    a = L.index('name "%s"' % name)
    nvl = next(k for k in range(a, len(L)) if L[k].startswith('numvert'))
    cnt = int(L[nvl].split()[1])
    v = [tuple(map(float, L[nvl + 1 + k].split()[:3])) for k in range(cnt)]
    b = nvl + 1 + cnt
    n = int(L[b].split()[1]); j = b + 1; keep, took = [], []
    for _ in range(n):
        nr = int(L[j + 2].split()[1])
        blk = L[j:j + 3 + nr]
        idx = [int(l.split()[0]) for l in blk[3:]]
        if nr >= 3 and test([v[i] for i in idx]):
            took.append((blk[0], idx))
        else:
            keep.append(blk)
        j += 3 + nr
    L = L[:b] + ['numsurf %d' % len(keep)] + [l for blk in keep for l in blk] + L[j:]
    return L, v, took

def inside(p, xr, h, zmax):
    return all(xr[0] < q[0] < xr[1] and q[1] > h and abs(q[2]) < zmax for q in p)

def centre(p, xr, h, zmax):
    c = [sum(q[d] for q in p) / len(p) for d in range(3)]
    return xr[0] < c[0] < xr[1] and c[1] > h and abs(c[2]) < zmax

INTERIOR = os.path.join(MODELS, 'Interior', 'interior.ac')
FUSE = os.path.join(MODELS, 'ask21.ac')
L = backup(INTERIOR); F = backup(FUSE)
parts = []
L, v, t = take(L, 'interior', lambda p: inside(p, (-3.58, -3.02), 0.55, 0.29) or inside(p, (-2.58, -2.12), 0.68, 0.29))
parts.append((v, t))
L, v, t = take(L, 'canopy.F.frame.int', lambda p: inside(p, (-3.60, -3.05), 0.5, 0.4))
parts.append((v, t))

# materials: the leather one is appended
mi = max(k for k, l in enumerate(L) if l.startswith('MATERIAL'))
mat = sum(1 for l in L if l.startswith('MATERIAL'))
L.insert(mi + 1, 'MATERIAL "leather" rgb 1.000 1.000 1.000  amb 0.900 0.900 0.900  emis 0.000 0.000 0.000  spec 0.100 0.100 0.100  shi 10 trans 0.000')

verts, surfs = [], []
for v, t in parts:
    rm = {}
    for surf, idx in t:
        pts = [v[i] for i in idx]
        e1 = [pts[1][d] - pts[0][d] for d in range(3)]; e2 = [pts[2][d] - pts[0][d] for d in range(3)]
        n = [abs(e1[1] * e2[2] - e1[2] * e2[1]), abs(e1[2] * e2[0] - e1[0] * e2[2]), abs(e1[0] * e2[1] - e1[1] * e2[0])]
        m = n.index(max(n)); refs = []
        for i in idx:
            if i not in rm:
                rm[i] = len(verts); verts.append(v[i])
            x, y, z = v[i]
            u, w = ((y, z), (x, z), (x, y))[m]       # planar mapping along the dominant axis of the face
            refs.append('%d %.5f %.5f' % (rm[i], u / TILE, w / TILE))
        surfs.append((surf, refs))
obj = ['OBJECT poly', 'name "glareshield"', 'texture "Panel/leather.png"', 'numvert %d' % len(verts)]
obj += ['%.5f %.5f %.5f' % p for p in verts] + ['numsurf %d' % len(surfs)]
for surf, refs in surfs:
    obj += [surf, 'mat %d' % mat, 'refs %d' % len(refs)] + refs
obj.append('kids 0')

kid = next(k for k, l in enumerate(L) if l.startswith('kids'))
L[kid] = 'kids %d' % (int(L[kid].split()[1]) + 1)
si = L.index('name "interior"')
nxt = next(k for k in range(si, len(L)) if L[k].startswith('OBJECT poly'))
L[nxt:nxt] = obj
open(INTERIOR, 'w').write('\n'.join(L))
shutil.copy(os.path.join(HERE, 'orig', 'ask21.ac'), FUSE)      # the fuselage skin stays as it was
print('glare shield faces:', len(surfs), [len(t) for _, t in parts])
