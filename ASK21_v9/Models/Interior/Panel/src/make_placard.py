#!/usr/bin/env python3
"""Instruments/placard/: a take-off check placard (white paper label, English) for the empty hole at the bottom right of
the front panel of the glider (panelF.xml), like the ones stuck on the panels of real ASK 21s.
Writes placard.png (opaque RGB), placard.ac and placard.xml."""
import os
from PIL import Image, ImageDraw, ImageFont

OUT = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), 'Instruments', 'placard')
W_M, H_M = 0.076, 0.082                 # size of the label in metres
PX = 640
W, H = PX, round(PX * H_M / W_M)

FONT = '/System/Library/Fonts/Helvetica.ttc'
bold = lambda s: ImageFont.truetype(FONT, s, index=1)
reg = lambda s: ImageFont.truetype(FONT, s, index=0)

# ---- the texture
img = Image.new('RGB', (W, H), (244, 242, 235))
d = ImageDraw.Draw(img)
ink = (28, 28, 30)
d.rectangle([6, 6, W - 7, H - 7], outline=(150, 150, 150), width=2)
d.text((W // 2, 66), 'TAKE-OFF CHECK', font=bold(54), fill=ink, anchor='mm')
d.line([40, 112, W - 40, 112], fill=ink, width=3)
items = ['LOADING', 'CANOPY', 'STRAPS', 'CONTROLS', 'TRIM', 'AIRBRAKES', 'INSTRUMENTS', 'RADIO', 'ROPE', 'WIND']
y = 170
step = (H - 170 - 46) // (len(items) - 1)
for it in items:
    d.text((70, y), it, font=bold(40), fill=ink, anchor='lm')
    d.rectangle([W - 108, y - 15, W - 78, y + 15], outline=ink, width=3)       # tick box
    y += step
img.save(os.path.join(OUT, 'placard.png'))

# ---- the mesh: one quad just in front of the panel (file frame: x toward the pilot, y up, z = -right)
hw, hh, x = W_M / 2, H_M / 2, 0.0008
verts = [(x, -hh, hw), (x, -hh, -hw), (x, hh, -hw), (x, hh, hw)]       # bottom left, bottom right, top right, top left
uvs = [(0, 0), (1, 0), (1, 1), (0, 1)]
ac = ['AC3Db',
      'MATERIAL "paper" rgb 1.000 1.000 1.000  amb 0.800 0.800 0.800  emis 0.000 0.000 0.000  spec 0.050 0.050 0.050  shi 4  trans 0',
      'OBJECT world', 'name "placard"', 'kids 1', 'OBJECT poly', 'name "placard.paper"', 'texture "placard.png"',
      'numvert 4'] + ['%.5f %.5f %.5f' % v for v in verts] + \
     ['numsurf 1', 'SURF 0x10', 'mat 0', 'refs 4'] + ['%d %.3f %.3f' % (i, u, v) for i, (u, v) in enumerate(uvs)] + ['kids 0']
open(os.path.join(OUT, 'placard.ac'), 'w').write('\n'.join(ac) + '\n')
open(os.path.join(OUT, 'placard.xml'), 'w').write('''<?xml version="1.0"?>
<PropertyList>
	<path>placard.ac</path>
</PropertyList>
''')
print('placard', W, H)
