#!/usr/bin/env python3
"""Black leather for the instrument panels, and patches for the holes that no longer have an instrument that fills them.

Reads the original meshes from src/orig/ and writes panelF.ac, panelF-mi.ac, panelR.ac and panelR-mi.ac.

* The panels use a 64 x 64 swatch sheet (panels.png): the UV coordinates point at flat colours. The faces that use the
  dark swatches (the big panel surface) are moved into a second object with the leather texture (leather.png) and a
  planar, tiling UV mapping; the faces with the other swatches (grey hole walls, orange screws) stay as they were.
* Patches: a disc at the front plane over the round hole of the V8 (80 mm hole, left-middle) and over the
  57 mm hole of the transponder / turn indicator, so that the instruments do not have to cover the rim.

File frame: x towards the pilot, y up, z = -right (like all AC files of the aircraft).
"""
import math, os, random, shutil
from PIL import Image, ImageFilter

HERE = os.path.dirname(os.path.abspath(__file__))
PANEL = os.path.dirname(HERE)
TILE = 0.040          # a leather tile is 40 mm wide

# --------------------------------------------------------------------------- the leather texture
def wrap_blur(img, radius):
    """Gaussian blur that wraps around the edges, so that the texture tiles without a seam."""
    w, h = img.size
    big = Image.new(img.mode, (3 * w, 3 * h))
    for ix in range(3):
        for iy in range(3):
            big.paste(img, (ix * w, iy * h))
    return big.filter(ImageFilter.GaussianBlur(radius)).crop((w, h, 2 * w, 2 * h))

def make_leather(path, size=512, cells=40, seed=11):
    """Tileable pebble-grain leather: rounded bumps (wrapped cellular pattern), soft creases, fine grain."""
    random.seed(seed)
    pts = [(random.random() * size, random.random() * size) for _ in range(cells * cells)]
    g = 20
    buckets = {}
    for p in pts:
        buckets.setdefault((int(p[0] * g / size), int(p[1] * g / size)), []).append(p)
    cell = size / cells
    img = Image.new('L', (size, size))
    px = img.load()
    for y in range(size):
        for x in range(size):
            bx, by = int(x * g / size), int(y * g / size)
            d1 = d2 = 1e9
            for dy in (-1, 0, 1):
                for dx in (-1, 0, 1):
                    for (qx, qy) in buckets.get(((bx + dx) % g, (by + dy) % g), ()):
                        ddx = abs(x - qx); ddx = min(ddx, size - ddx)
                        ddy = abs(y - qy); ddy = min(ddy, size - ddy)
                        d = math.hypot(ddx, ddy)
                        if d < d1: d1, d2 = d, d1
                        elif d < d2: d2 = d
            crease = max(0.0, min(1.0, (d2 - d1) / (cell * 0.55)))     # 0 in the creases, 1 inside a pebble
            crease = crease * crease * (3 - 2 * crease)                # smooth
            bump = max(0.0, 1.0 - d1 / (cell * 0.9))                   # rounded top of the pebble
            px[x, y] = int(14 + 16 * crease + 8 * bump)
    img = wrap_blur(img, 1.1)
    rnd = wrap_blur(Image.effect_noise((size, size), 22), 30)
    fine = wrap_blur(Image.effect_noise((size, size), 40), 0.6)
    out = Image.new('RGB', (size, size))
    po, pi, pr, pf = out.load(), img.load(), rnd.load(), fine.load()
    for y in range(size):
        for x in range(size):
            v = pi[x, y] + (pr[x, y] - 128) * 0.06 + (pf[x, y] - 128) * 0.05
            v = max(6, min(52, v))
            po[x, y] = (int(v), int(v), int(v * 1.04))
    out.save(path, optimize=True)

# --------------------------------------------------------------------------- AC file helpers
def parse(path):
    L = open(path).read().split('\n')
    i = next(k for k, l in enumerate(L) if l.startswith('numvert'))
    n = int(L[i].split()[1])
    verts = [tuple(map(float, L[i + 1 + k].split()[:3])) for k in range(n)]
    j = i + 1 + n
    ns = int(L[j].split()[1]); j += 1
    faces = []
    for _ in range(ns):
        nr = int(L[j + 2].split()[1])
        faces.append([(int(L[j + 3 + k].split()[0]), float(L[j + 3 + k].split()[1]), float(L[j + 3 + k].split()[2])) for k in range(nr)])
        j += 3 + nr
    head = L[:next(k for k, l in enumerate(L) if l.startswith('OBJECT poly'))]
    return head, verts, faces

def normal(pts):
    a, b, c = pts[0], pts[1], pts[2]
    u = [b[k] - a[k] for k in range(3)]; v = [c[k] - a[k] for k in range(3)]
    n = (u[1]*v[2] - u[2]*v[1], u[2]*v[0] - u[0]*v[2], u[0]*v[1] - u[1]*v[0])
    m = math.sqrt(sum(x * x for x in n)) or 1.0
    return (n[0] / m, n[1] / m, n[2] / m)

def planar_uv(p, n):
    x, y, z = p
    ax = [abs(c) for c in n]
    if ax[0] >= ax[1] and ax[0] >= ax[2]:
        return (z / TILE, y / TILE)           # facing the pilot: panel face and the walls of the holes seen from the front
    if ax[1] >= ax[2]:
        return (z / TILE, x / TILE)           # top / bottom
    return (y / TILE, x / TILE)               # left / right

def swatch(face):
    uc = sum(u for _, u, _ in face) / len(face); vc = sum(w for _, _, w in face) / len(face)
    return ('L' if uc < 0.5 else 'R') + ('T' if vc >= 0.5 else 'B')


_PANELS = None
def amber(face):
    """True where the face lies on the amber (brass) part of panels.png: the nuts that hold the panel"""
    global _PANELS
    if _PANELS is None:
        _PANELS = Image.open(os.path.join(PANEL, 'panels.png')).convert('RGB')
    w, h = _PANELS.size
    u = sum(t[1] for t in face) / len(face); v = sum(t[2] for t in face) / len(face)
    r, g, b = _PANELS.getpixel((min(w - 1, int(u * w)), min(h - 1, int((1 - v) * h))))
    return r > 120 and r > g > b and r - b > 60

def object_text(name, verts, faces, mat, texture, rep=False):
    o = ['OBJECT poly', 'name "%s"' % name, 'data 6', 'Circle', 'crease 45.0', 'texture "%s"' % texture]
    o.append('numvert %d' % len(verts))
    o += ['%.5f %.5f %.5f' % v for v in verts]
    o.append('numsurf %d' % len(faces))
    for f in faces:
        o += ['SURF 0X10', 'mat %d' % mat, 'refs %d' % len(f)]
        o += ['%d %.5f %.5f' % t for t in f]
    o.append('kids 0')
    return '\n'.join(o)

def disc_faces(cy, cz, r, base, n=48):
    """Triangles of a disc at the front plane x = 0 (file frame: y up, z = -right), facing the pilot."""
    pts = [(0.0, cy, cz)] + [(0.0, cy + r * math.cos(2 * math.pi * k / n), cz + r * math.sin(2 * math.pi * k / n)) for k in range(n)]
    verts = [p for p in pts]
    faces = []
    for k in range(n):
        tri = [0, 1 + k, 1 + (k + 1) % n]
        if normal([pts[t] for t in tri])[0] < 0:
            tri = [tri[0], tri[2], tri[1]]
        faces.append([(base + t, ) + planar_uv(pts[t], (1, 0, 0)) for t in tri])
    return verts, faces

# --------------------------------------------------------------------------- build
LEATHER_SWATCHES = {'panelF.ac': ('LT',), 'panelF-mi.ac': ('LT',), 'panelR.ac': ('LT', 'RT'), 'panelR-mi.ac': ('LT', 'RT')}
# (centre up, centre -right, radius of the front rim of the hole), measured in the meshes
PATCHES = {
    'panelF.ac':    [(0.035, -0.085, 0.0396), (-0.025, 0.0, 0.0282)],
    'panelF-mi.ac': [(0.035, -0.085, 0.0396), (0.060, 0.0, 0.0282)],
    'panelR.ac': [], 'panelR-mi.ac': [],
}

def build(name):
    head, verts, faces = parse(os.path.join(HERE, 'orig', name))
    keep, leather = [], []
    for f in faces:
        # the bevel around the panel, facing the pilot (but not the amber nuts, which stand as far out)
        rim = sum(verts[a][0] for a, _, _ in f) / len(f) > 0.005 and not amber(f)
        (leather if swatch(f) in LEATHER_SWATCHES[name] or rim else keep).append(f)
    # leather object: own vertex list, planar UVs
    lv, lf, remap = [], [], {}
    for f in leather:
        n = normal([verts[a] for a, _, _ in f])
        nf = []
        for a, _, _ in f:
            if a not in remap:
                remap[a] = len(lv); lv.append(verts[a])
            nf.append((remap[a],) + planar_uv(verts[a], n))
        lf.append(nf)
    for (cy, cz, r) in PATCHES[name]:
        pv, pf = disc_faces(cy, cz, r, len(lv))
        lv += pv; lf += pf
    head = [l for l in head]
    head.insert(2, 'MATERIAL "leather" rgb 1.000 1.000 1.000  amb 0.900 0.900 0.900  emis 0.000 0.000 0.000  spec 0.100 0.100 0.100  shi 10 trans 0.000')
    head = [('kids 1' if l == 'kids 1' else l) for l in head]
    # kids 2
    head = ['kids 2' if l.startswith('kids') else l for l in head]
    # the original object keeps only its remaining faces, with vertices re-indexed
    kv, kf, kremap = [], [], {}
    for f in keep:
        nf = []
        for a, u, w in f:
            if a not in kremap:
                kremap[a] = len(kv); kv.append(verts[a])
            nf.append((kremap[a], u, w))
        kf.append(nf)
    text = '\n'.join(head) + '\n' + object_text('panel_hole_walls', kv, kf, 0, 'panels.png') + '\n' + object_text('panel_leather', lv, lf, 1, 'leather.png') + '\n'
    open(os.path.join(PANEL, name), 'w').write(text)
    print(name, 'leather faces', len(leather), 'other faces', len(keep), 'patch discs', len(PATCHES[name]))

if __name__ == '__main__':
    make_leather(os.path.join(PANEL, 'leather.png'))
    for n in LEATHER_SWATCHES:
        build(n)
