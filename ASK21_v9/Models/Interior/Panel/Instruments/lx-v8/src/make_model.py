#!/usr/bin/env python3
"""Generates lxv8.ac (LXNAV V8 vario indicator, front part only) and the screen placeholder texture.

Dimensions from the LXNAV V8 drawing in the installation manual: body 61 x 61 mm, screws on a 47.4 x 47.4 mm
square, 57 mm panel cut-out, right-hand corners chamfered, round window, three push buttons and the SD
card slot on the right. (The 80 mm V80 has the same layout and the same screen content.)

Design coordinates: x towards the pilot, "right" and "up" on the panel. FlightGear's AC loader maps the
file's y to up and z to the left, so the vertices are written as (x, up, -right).
Front face at x = 0.010. Everything is in metres.
"""
import math, os, sys
from PIL import Image
sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', '..', '..', 'src'))
import screwlib            # the screw style shared by all instruments (Models/Interior/Panel/src/screwlib.py)

HERE = os.path.dirname(os.path.abspath(__file__))
OUT = os.path.dirname(HERE)

K = 61.0 / 80.2     # the layout of the V80 drawing scaled to the 61 mm V8 body

MATS = [  # name, rgb, ambient, spec, shi
    ("body",   (0.55, 0.55, 0.58), 0.35, 0.18, 12),   # textured black metal (lxv8-metal.png)
    ("bezel",  (0.45, 0.45, 0.48), 0.30, 0.35, 24),
    ("button", (0.10, 0.10, 0.11), 0.30, 0.15, 10),
    ("screw",  (0.150, 0.160, 0.190), 0.35, 0.70, 40),
    ("black",  (0.01, 0.01, 0.01), 0.10, 0.00, 1),
        ("screen", (0.8, 0.8, 0.8), 0.80, 0.30, 12),
]
METAL = "lxv8-metal.png"

def sub(a, b): return tuple(x - y for x, y in zip(a, b))
def cross(a, b): return (a[1]*b[2]-a[2]*b[1], a[2]*b[0]-a[0]*b[2], a[0]*b[1]-a[1]*b[0])
def dot(a, b): return sum(x*y for x, y in zip(a, b))

class Mesh:
    def __init__(self, name, mat, texture=None):
        self.name, self.mat, self.texture = name, mat, texture
        self.verts, self.faces = [], []
    def poly(self, pts, hint, uvs=None):
        """Add a convex polygon (design coordinates); the winding is chosen so that the normal points along `hint`."""
        pts = list(pts)
        if uvs is None and self.texture == METAL:
            # planar mapping, 61 mm per tile; depth shifts the side walls so the streaks continue
            uvs = [((p[1] + 0.0305) / 0.061 + p[0] * 2.0, (p[2] + 0.0305) / 0.061) for p in pts]
        uvs = list(uvs) if uvs else [(0, 0)] * len(pts)
        n = (0, 0, 0)
        for i in range(1, len(pts) - 1):
            n = tuple(a + b for a, b in zip(n, cross(sub(pts[i], pts[0]), sub(pts[i+1], pts[0]))))
        if dot(n, hint) < 0:
            pts.reverse(); uvs.reverse()
        base = len(self.verts)
        self.verts += pts
        self.faces.append([(base + i, uvs[i]) for i in range(len(pts))])
    def ac(self):
        o = ['OBJECT poly', 'name "%s"' % self.name]
        if self.texture:
            o += ['texture "%s"' % self.texture, 'texrep 1 1']
        o.append('numvert %d' % len(self.verts))
        # design (x, right, up) -> file (x, up, -right)
        o += ['%.6f %.6f %.6f' % (v[0], v[2], -v[1]) for v in self.verts]
        o.append('numsurf %d' % len(self.faces))
        for f in self.faces:
            o += ['SURF 0x10', 'mat %d' % self.mat, 'refs %d' % len(f)]
            o += ['%d %.5f %.5f' % (i, uv[0], uv[1]) for i, uv in f]
        o.append('kids 0')
        return '\n'.join(o)

def P(x, y, z): return (x, y, z)
XF = 0.0100          # front face of the housing
XB = -0.0120         # back of the modelled part (hidden behind the panel)

meshes = []

# ---- housing, right corners chamfered
hw = hh = 0.0305
c = 0.0062
outline = [(-hw, -hh), (hw - c, -hh), (hw, -hh + c), (hw, hh - c), (hw - c, hh), (-hw, hh)]   # (right, up), counter-clockwise
body = Mesh("V8.body", 0, METAL); meshes.append(body)
body.poly([P(XF, y, z) for y, z in outline], (1, 0, 0))
body.poly([P(XB, y, z) for y, z in outline], (-1, 0, 0))
for i in range(len(outline)):
    (y0, z0), (y1, z1) = outline[i], outline[(i + 1) % len(outline)]
    out = (0, z1 - z0, -(y1 - y0))   # outward normal of a counter-clockwise outline
    body.poly([P(XF, y0, z0), P(XB, y0, z0), P(XB, y1, z1), P(XF, y1, z1)], out)

# ---- bezel ring around the round window
N = 56
r_in, r_out, xb = 0.0270, 0.0298, 0.0114
bezel = Mesh("V8.bezel", 1, METAL); meshes.append(bezel)
for i in range(N):
    a0, a1 = 2 * math.pi * i / N, 2 * math.pi * (i + 1) / N
    c0, s0, c1, s1 = math.cos(a0), math.sin(a0), math.cos(a1), math.sin(a1)
    bezel.poly([P(xb, r_in*c0, r_in*s0), P(xb, r_out*c0, r_out*s0), P(xb, r_out*c1, r_out*s1), P(xb, r_in*c1, r_in*s1)], (1, 0, 0))
    bezel.poly([P(XF, r_out*c0, r_out*s0), P(xb, r_out*c0, r_out*s0), P(xb, r_out*c1, r_out*s1), P(XF, r_out*c1, r_out*s1)], (c0 + c1, s0 + s1, 0))
    bezel.poly([P(xb, r_in*c0, r_in*s0), P(0.0112, r_in*c0, r_in*s0), P(0.0112, r_in*c1, r_in*s1), P(xb, r_in*c1, r_in*s1)], (-(c0 + c1), -(s0 + s1), 0))

# ---- screen: a disc cut off on the right (the buttons take that space); the canvas covers +-r_in
screen = Mesh("V8.screen", 5, "lxv8-screen.png"); meshes.append(screen)
cut = 0.0160
t0 = math.acos(cut / r_in)
M = 64
ring = [t0 + (2 * math.pi - 2 * t0) * i / M for i in range(M + 1)]
span = 2 * r_in
def uv(y, z): return ((y + r_in) / span, (z + r_in) / span)
pts = [(r_in * math.cos(a), r_in * math.sin(a)) for a in ring]
for pa, pb in zip(pts[:-1], pts[1:]):
    screen.poly([P(0.0112, 0, 0), P(0.0112, *pa), P(0.0112, *pb)], (1, 0, 0), [uv(0, 0), uv(*pa), uv(*pb)])
# the fan only covers the arc; close the straight cut on the right with one more triangle
screen.poly([P(0.0112, 0, 0), P(0.0112, *pts[-1]), P(0.0112, *pts[0])], (1, 0, 0), [uv(0, 0), uv(*pts[-1]), uv(*pts[0])])

# the screws are the shared model 57mm_screws (screws of the Winter ASI), added in lxv8.xml
slots = Mesh("V8.slots", 4); meshes.append(slots)
# SD card slot, right edge
xs = XF + 0.00005
slots.poly([P(xs, 0.0278, -0.0045), P(xs, 0.0292, -0.0045), P(xs, 0.0292, 0.0076), P(xs, 0.0278, 0.0076)], (1, 0, 0))

# ---- three push buttons right of the window (layout measured from the manual's drawing of the V80)
def button(name, poly, height=0.0022):
    area = sum(poly[i][0] * poly[(i + 1) % len(poly)][1] - poly[(i + 1) % len(poly)][0] * poly[i][1] for i in range(len(poly)))
    if area < 0:
        poly = poly[::-1]                          # counter-clockwise outline
    m = Mesh(name, 2); meshes.append(m)
    xt = XF + height
    m.poly([P(xt, y, z) for y, z in poly], (1, 0, 0))
    for i in range(len(poly)):
        (y0, z0), (y1, z1) = poly[i], poly[(i + 1) % len(poly)]
        m.poly([P(XF, y0, z0), P(xt, y0, z0), P(xt, y1, z1), P(XF, y1, z1)], (0, z1 - z0, -(y1 - y0)))
def scaled(pts): return [(y * K, z * K) for y, z in pts]
top = scaled([(0.0225, 0.0100), (0.0335, 0.0100), (0.0335, 0.0215), (0.0300, 0.0255), (0.0225, 0.0250)])
mid = scaled([(0.0225, -0.0072), (0.0332, -0.0072), (0.0338, -0.0066), (0.0338, 0.0066), (0.0332, 0.0072), (0.0225, 0.0072)])
bot = [(y, -z) for y, z in top]
button("V8.btn.up", top)
button("V8.btn.mid", mid)
button("V8.btn.dn", bot)

# ---- write
lines = ['AC3Db']
for name, rgb, amb, spec, shi in MATS:
    lines.append('MATERIAL "%s" rgb %.3f %.3f %.3f  amb %.3f %.3f %.3f  emis 0 0 0  spec %.3f %.3f %.3f  shi %d  trans 0' %
                 ((name,) + rgb + (amb,) * 3 + (spec,) * 3 + (shi,)))
lines += ['OBJECT world', 'name "LXNAV V8"', 'kids %d' % len(meshes)]
lines += [m.ac() for m in meshes]
open(os.path.join(OUT, 'lxv8.ac'), 'w').write('\n'.join(lines) + '\n')
Image.new('RGB', (8, 8), (0, 0, 0)).save(os.path.join(OUT, 'lxv8-screen.png'))

# black anodised / brushed metal: fine grain plus faint horizontal brushing, no alpha channel
import random
random.seed(8)
W = 256
rows = [random.gauss(0, 3.0) for _ in range(W)]
img = Image.new('RGB', (W, W))
px = img.load()
for y in range(W):
    for x in range(W):
        v = 34 + rows[y] + random.gauss(0, 4.5)
        if random.random() < 0.004:
            v += random.uniform(12, 30)              # tiny bright specks of the anodising
        v = max(8, min(110, v))
        px[x, y] = (int(v), int(v), int(v * 1.04))
img.save(os.path.join(OUT, METAL))
print('objects:', [(m.name, len(m.verts), len(m.faces)) for m in meshes])
