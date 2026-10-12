#!/usr/bin/env python3
"""Gives the Winter variometers the screws of the Winter ASI: the faces of one ASI screw (and its texture) are copied to the
four corners of the variometer, replacing its own 'screws' object. Reads src/orig/*.ac, writes the two vario meshes."""
import math, os, shutil
HERE = os.path.dirname(os.path.abspath(__file__))
INSTR = os.path.join(os.path.dirname(HERE), 'Instruments')

def parse(path):
    L = open(path, errors='ignore').read().split('\n')
    objs = []                                             # (name, header lines, verts, faces[(mat, [(idx,u,v)])])
    i = 0
    while i < len(L):
        if L[i].startswith('OBJECT poly'):
            hdr = []
            while not L[i].startswith('numvert'):
                hdr.append(L[i]); i += 1
            n = int(L[i].split()[1]); v = [tuple(map(float, L[i + 1 + k].split()[:3])) for k in range(n)]; i += 1 + n
            ns = int(L[i].split()[1]); i += 1; faces = []
            for _ in range(ns):
                mat = int(L[i + 1].split()[1]); nr = int(L[i + 2].split()[1])
                faces.append((mat, [(int(L[i + 3 + k].split()[0]), float(L[i + 3 + k].split()[1]), float(L[i + 3 + k].split()[2])) for k in range(nr)]))
                i += 3 + nr
            objs.append((hdr[1].split('"')[1], hdr, v, faces))
        else: i += 1
    first_obj = next(k for k, l in enumerate(L) if l.startswith('OBJECT poly'))
    return L[:first_obj], objs

# --- the ASI screw: the faces around the corner screw at (0.0293, 0.0293)
_, asi_objs = parse(os.path.join(HERE, 'orig', 'winter-asi.ac'))
_, v, faces = next((n, vv, ff) for n, h, vv, ff in asi_objs if n == 'asi')[0:3] if False else (None, None, None)
asi = next(o for o in asi_objs if o[0] == 'asi')
av, af = asi[2], asi[3]
CY = CZ = 0.0315
SCALE = 0.55          # the screws are scaled about their own centre
screw = [f for f in af if (lambda c: math.hypot(c[0] - CY, c[1] - CZ) < 0.0075 and max(av[a][0] for a, _, _ in f[1]) > 0.0003)([sum(av[a][d] for a, _, _ in f[1]) / len(f[1]) for d in (1, 2)])]

def build(src, dst, tex_rel):
    head, objs = parse(src)
    # screw positions: the middle of the four corner groups of the variometer's own screws
    own = next(o for o in objs if o[0] == 'screws')
    centres = []
    for sy in (-1, 1):
        for sz in (-1, 1):
            pts = [p for p in own[2] if p[1] * sy > 0.03 and p[2] * sz > 0.03]
            ys = [p[1] for p in pts]; zs = [p[2] for p in pts]
            centres.append(((min(ys) + max(ys)) / 2, (min(zs) + max(zs)) / 2))
    verts, faces_out, mat = [], [], len([l for l in head if l.startswith('MATERIAL')])
    for (ty, tz) in centres:
        remap = {}
        for m, fc in screw:
            nf = []
            for a, u, w in fc:
                if a not in remap:
                    x, y, z = av[a]
                    remap[a] = len(verts); verts.append((x * SCALE, ty + (y - CY) * SCALE, tz + (z - CZ) * SCALE))
                nf.append((remap[a], u, w))
            faces_out.append((mat, nf))
    # header: one more material, like the one of the ASI
    head = list(head)
    last = max(k for k, l in enumerate(head) if l.startswith('MATERIAL'))
    head.insert(last + 1, 'MATERIAL "asi-screw" rgb 1.000 1.000 1.000  amb 0.800 0.800 0.800  emis 0.000 0.000 0.000  spec 0.990 0.990 0.990  shi 64 trans 0.000')
    out = list(head)
    for name, hdr, vv, ff in objs:
        if name == 'screws':
            out += ['OBJECT poly', 'name "screws"', 'texture "%s"' % tex_rel, 'numvert %d' % len(verts)]
            out += ['%.6f %.6f %.6f' % p for p in verts]
            out.append('numsurf %d' % len(faces_out))
            for m, fc in faces_out:
                out += ['SURF 0X10', 'mat %d' % m, 'refs %d' % len(fc)] + ['%d %.6f %.6f' % t for t in fc]
            out.append('kids 0')
            continue
        out += hdr + ['numvert %d' % len(vv)] + ['%.7f %.7f %.7f' % p for p in vv] + ['numsurf %d' % len(ff)]
        for m, fc in ff:
            out += ['SURF 0X10', 'mat %d' % m, 'refs %d' % len(fc)] + ['%d %.6f %.6f' % t for t in fc]
        out.append('kids 0')
    open(dst, 'w').write('\n'.join(out) + '\n')
    print(os.path.basename(dst), len(centres), 'screws,', len(faces_out), 'faces')

for name, rel in (('vario-5stv5/winter-vario-5stv5.ac', None), ('vario-knots/winter-vario-5stv5-knots.ac', None)):
    orig = os.path.join(HERE, 'orig', os.path.basename(name))
    if not os.path.exists(orig):
        shutil.copy(os.path.join(INSTR, name), orig)
    build(orig, os.path.join(INSTR, name), '../winter-asi/winter-asi-ask21.png')
