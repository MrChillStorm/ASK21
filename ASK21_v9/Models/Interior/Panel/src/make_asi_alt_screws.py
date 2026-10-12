#!/usr/bin/env python3
"""The ASI and the altimeter get the screws of the variometers as well: their own screw faces (smoothed together with the
rest of the case, which makes them look stretched) are removed and replaced by the separate 'screws' object of
make_vario_screws.py, at the corners of the 80 mm instrument."""
import math, os
HERE = os.path.dirname(os.path.abspath(__file__))
src = open(os.path.join(HERE, 'make_vario_screws.py')).read().split('def build(')[0]
ns = {'__file__': os.path.join(HERE, 'make_vario_screws.py')}
exec(src, ns)
parse, av, screw, CY, CZ, SCALE = ns['parse'], ns['av'], ns['screw'], ns['CY'], ns['CZ'], ns['SCALE']
INSTR = ns['INSTR']

def near_corner(p, C, rmax=0.0075, xmax=1):
    c = [sum(q[d] for q in p) / len(p) for d in range(3)]
    return any(math.hypot(c[1] - sy * C, c[2] - sz * C) < rmax and 0.0003 < max(q[0] for q in p) < xmax
               for sy in (-1, 1) for sz in (-1, 1))

def convert(name, obj, C, rmax=0.0075, xmax=1):
    L = open(os.path.join(HERE, 'orig', name + '.ac')).read().split('\n')
    s = L.index('name "%s"' % obj)
    nvl = next(k for k in range(s, len(L)) if L[k].startswith('numvert'))
    cnt = int(L[nvl].split()[1]); v = [tuple(map(float, L[nvl + 1 + k].split()[:3])) for k in range(cnt)]
    b = nvl + 1 + cnt; n = int(L[b].split()[1]); j = b + 1; keep = []; gone = 0
    for _ in range(n):
        nr = int(L[j + 2].split()[1]); blk = L[j:j + 3 + nr]
        if nr >= 3 and near_corner([v[int(l.split()[0])] for l in blk[3:]], C, rmax, xmax): gone += 1
        else: keep.append(blk)
        j += 3 + nr
    L = L[:b] + ['numsurf %d' % len(keep)] + [l for blk in keep for l in blk] + L[j:]
    # the separate screws object, as on the variometers
    mi = max(k for k, l in enumerate(L) if l.startswith('MATERIAL'))
    mat = sum(1 for l in L if l.startswith('MATERIAL'))
    L.insert(mi + 1, 'MATERIAL "asi-screw" rgb 1.000 1.000 1.000  amb 0.800 0.800 0.800  emis 0.000 0.000 0.000  spec 0.990 0.990 0.990  shi 64 trans 0.000')
    verts, faces = [], []
    for ty in (-1, 1):
        for tz in (-1, 1):
            rm = {}
            for m, fc in screw:
                nf = []
                for a, u, w in fc:
                    if a not in rm:
                        x, y, z = av[a]; rm[a] = len(verts); verts.append((x * SCALE, ty * C + (y - CY) * SCALE, tz * C + (z - CZ) * SCALE))
                    nf.append((rm[a], u, w))
                faces.append(nf)
    o = ['OBJECT poly', 'name "screws"', 'texture "../winter-asi/winter-asi-ask21.png"', 'numvert %d' % len(verts)]
    o += ['%.6f %.6f %.6f' % p for p in verts] + ['numsurf %d' % len(faces)]
    for nf in faces:
        o += ['SURF 0X10', 'mat %d' % mat, 'refs %d' % len(nf)] + ['%d %.6f %.6f' % t for t in nf]
    o.append('kids 0')
    kid = next(k for k, l in enumerate(L) if l.startswith('kids')); L[kid] = 'kids %d' % (int(L[kid].split()[1]) + 1)
    at = next(k for k in range(s, len(L)) if L[k].startswith('OBJECT poly') and k > s)
    L[at:at] = o
    dst = os.path.join(INSTR, name, name + '.ac')
    open(dst, 'w').write('\n'.join(L))
    print(name, 'removed', gone, 'faces, added', len(faces))

convert('winter-asi', 'asi', CY)
convert('winter-alt', 'alt.case', 0.0315, 0.0045, 0.0035)
