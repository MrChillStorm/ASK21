#!/usr/bin/env python3
"""Instruments/GMeter/57mm_screws.ac: four screws of the 80 mm instruments (the faces and the texture of one screw of the
Winter ASI) at the corners of a 57 mm instrument. The file is shared by the G-meter, the radio, the compass and the
transponder."""
import os, sys, math
HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import importlib.util
spec = importlib.util.spec_from_file_location('vs', os.path.join(HERE, 'make_vario_screws.py'))
src = open(os.path.join(HERE, 'make_vario_screws.py')).read().split('def build(')[0]   # parse() and the ASI screw faces
ns = {'__file__': os.path.join(HERE, 'make_vario_screws.py')}
exec(src, ns)
av, screw, CY, CZ, SCALE = ns['av'], ns['screw'], ns['CY'], ns['CZ'], ns['SCALE']

OUT = os.path.join(os.path.dirname(HERE), 'Instruments', 'GMeter', '57mm_screws.ac')
P = 0.0230            # screw positions on a 57 mm instrument (file frame: up, -right)

lines = ['AC3Db',
         'MATERIAL "asi-screw" rgb 1.000 1.000 1.000  amb 0.800 0.800 0.800  emis 0.000 0.000 0.000  spec 0.990 0.990 0.990  shi 64 trans 0.000',
         'OBJECT world', 'name "57mm_screws"', 'kids 4']
for k, (sy, sz) in enumerate(((1, -1), (-1, -1), (-1, 1), (1, 1)), 1):
    verts, faces, remap = [], [], {}
    for m, fc in screw:
        nf = []
        for a, u, w in fc:
            if a not in remap:
                x, y, z = av[a]
                remap[a] = len(verts); verts.append((x * SCALE, sy * P + (y - CY) * SCALE, sz * P + (z - CZ) * SCALE))
            nf.append((remap[a], u, w))
        faces.append(nf)
    lines += ['OBJECT poly', 'name "57mm-s%d"' % k, 'texture "../winter-asi/winter-asi-ask21.png"', 'numvert %d' % len(verts)]
    lines += ['%.6f %.6f %.6f' % v for v in verts] + ['numsurf %d' % len(faces)]
    for nf in faces:
        lines += ['SURF 0x10', 'mat 0', 'refs %d' % len(nf)] + ['%d %.6f %.6f' % t for t in nf]
    lines.append('kids 0')
open(OUT, 'w').write('\n'.join(lines) + '\n')
print(OUT, sum(len(f) for f in [faces]) * 4, 'faces')
