"""One screw style for all instruments: a small dark pan-head Phillips screw like the ones of the Winter ASI.

screw(cy, cz, ...) returns polygons in design coordinates (x towards the pilot, 'right', 'up') as
(material, [points], outward normal) tuples; material is 'screw' (dark steel) or 'slot' (the cross).
"""
import math

R = 0.0021          # radius of the head at its foot
H = 0.0011          # height of the head over the surface

def screw(cy, cz, x0=0.0, r=R, h=H, n=14):
    out = []
    rings = [(0.0, r), (h * 0.55, r * 0.97), (h * 0.85, r * 0.78), (h, r * 0.5)]       # (height, radius): a low dome
    for (ha, ra), (hb, rb) in zip(rings[:-1], rings[1:]):
        for i in range(n):
            a0, a1 = 2 * math.pi * i / n, 2 * math.pi * (i + 1) / n
            p = lambda hh, rr, a: (x0 + hh, cy + rr * math.cos(a), cz + rr * math.sin(a))
            nrm = ((r - rb) * 0.6 + 0.2, math.cos((a0 + a1) / 2), math.sin((a0 + a1) / 2))
            out.append(('screw', [p(ha, ra, a0), p(ha, ra, a1), p(hb, rb, a1), p(hb, rb, a0)], nrm))
    top = x0 + h
    for i in range(n):
        a0, a1 = 2 * math.pi * i / n, 2 * math.pi * (i + 1) / n
        out.append(('screw', [(top, cy, cz), (top, cy + rings[-1][1] * math.cos(a0), cz + rings[-1][1] * math.sin(a0)),
                              (top, cy + rings[-1][1] * math.cos(a1), cz + rings[-1][1] * math.sin(a1))], (1, 0, 0)))
    # the Phillips cross, a little above the top face
    w, l, xs = r * 0.12, rings[-1][1] * 0.95, top + 0.00004
    out.append(('slot', [(xs, cy - l, cz - w), (xs, cy + l, cz - w), (xs, cy + l, cz + w), (xs, cy - l, cz + w)], (1, 0, 0)))
    out.append(('slot', [(xs, cy - w, cz - l), (xs, cy + w, cz - l), (xs, cy + w, cz + l), (xs, cy - w, cz + l)], (1, 0, 0)))
    return out

# the two materials of a screw, as AC3D MATERIAL lines
MATERIALS = {
    'screw': 'rgb 0.150 0.160 0.190  amb 0.350 0.350 0.350  emis 0 0 0  spec 0.700 0.700 0.750  shi 40  trans 0',
    'slot':  'rgb 0.010 0.010 0.010  amb 0.100 0.100 0.100  emis 0 0 0  spec 0.000 0.000 0.000  shi 1  trans 0',
}
