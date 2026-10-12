"""ASK 21 geometry for the v9 component model.

Frames
  D (datum frame, metres): X aft of the wing-root leading edge (the W&B datum), Y right, Z up from the
    root-LE height.
  S (JSBSim structural frame, metres): x_s = X - 1.637, y_s = Y, z_s = Z + 0.701.
    (The FlightGear 3-D model and the existing gear/CG/hook positions use this frame; the report
     datum sits at x_s = -1.637 m, which reproduces the report empty CG 29.02 in for the model's -0.9 m.)
Angles are relative to the JSBSim body x axis.

Sources
  [TCDS/FM]  span 17.0 m, area 17.95 m2, AR 16.1, length 8.35 m, FX S 02-196 / FX 60-126
  [Mi-WHB]   ASK 21 Mi flight manual (Flughandbuch) technical data: HT 3.1 m / 1.92 m2 / chords 0.8-0.4 m,
             FX 71-L-150/30, elevator 30 % 0.576 m2; fin 1.357 m2, chords 1.17/0.8 m, 1.37 m above
             fuselage centre line, FX 71-L-150/30, rudder 31 % 0.42 m2; airbrakes 0.326 m2 (both), 2.9-4.3 m;
             fuselage width 0.70, height 1.04, surface 12.33 m2
  [FM-rig]   ASK 21 flight manual p.54 "Three view - rigging data": aileron 110/45 mm at R 240,
             elevator 90/65 mm at R 230, rudder 180 mm at R 375; incidence checks 45 / 36 / 13 mm at root,
             aileron inner end, aileron outer end; tailplane centre line 0 deg (fuselage levelled 1000:52)
  [3-view]   K21 three-view vector drawing (2020, CC BY 4.0), 1500 units/m: planform, dihedral,
             fuselage contours, wheels, root section at 2.64 deg to the drawing horizontal, ground line
  [AFFTC]    AFFTC-TR-89-27 App. C: MAC 1121 mm, LE 8 mm aft of datum; moment arms (front -47.91 in,
             rear -3.15 in, tail ballast 209.8 in, cockpit ballast -63.39 in), inertias
"""
import numpy as np

XS0, ZS0 = -1.637, 0.701          # datum (root LE) in the structural frame
IN = 0.0254


def to_s(X, Y, Z):
    return X + XS0, Y, Z + ZS0


# ---------------------------------------------------------------- wing
B = 17.0
S_REF = 17.95
CBAR = 1.121                      # AFFTC MAC (3-view gives 1.142)
DIHEDRAL = 4.0                    # deg, front view (upper 3.2 / lower 4.8 deg)
Y_FUS = 0.355                     # fuselage half width at the wing
Y_KINK = 5.20
# planform from the 3-view (LE straight, TE tapered in two panels), chords in m
_cy = np.array([0.0, 0.36, 1.16, 1.97, 2.77, 3.58, 4.38, 5.18, 5.99, 6.79, 7.60, 8.40, 8.50])
_cc = np.array([1.52, 1.50, 1.401, 1.327, 1.247, 1.164, 1.081, 1.001, 0.880, 0.758, 0.636, 0.512, 0.40])
_cle = np.array([0.0, 0.0, -0.013, -0.017, -0.014, -0.008, -0.003, 0.0, 0.009, 0.017, 0.025, 0.036, 0.05])  # LE x (aft +)


def chord(y):
    return np.interp(abs(y), _cy, _cc)


def x_le(y):
    return np.interp(abs(y), _cy, _cle)


# incidence: ground geometry of the 3-view puts the root chord at 4.30 deg to the runway on nose+main
# wheel; the JSBSim gear sits at 1.82 deg there -> root 2.48 deg to body x.  Spanwise variation and the
# tailplane setting follow the Schleicher rigging data (relative to the root).
I_ROOT = 2.48
_iy = np.array([0.0, 0.505, 5.20, 7.95, 8.5])
_ii = np.array([0.0, 0.0, 0.31, -0.49, -0.6])      # 45/1470, 36/1000, 13/590 mm -> 1.75, 2.06, 1.26 deg


def incidence(y):
    return I_ROOT + np.interp(abs(y), _iy, _ii)


# strips (per side): edges in |y|
WING_EDGES = np.array([0.0, Y_FUS, 1.40, 2.40, 2.90, 3.60, 4.30, 5.20, 6.10, 7.00, 7.95, 8.50])
AIL_Y = (5.20, 7.95)
BRAKE_Y = (2.90, 4.30)            # Mi-WHB 2.9-4.3 m
BRAKE_H = 0.326 / 2 / 1.40        # plate height per side (m): 0.326 m2 both wings, upper surface only
BRAKE_XC = 0.40                   # chordwise position


def wing_strips():
    """list of dicts for the right wing (left = mirror)."""
    out = []
    for i in range(len(WING_EDGES) - 1):
        y0, y1 = WING_EDGES[i], WING_EDGES[i + 1]
        ys = np.linspace(y0, y1, 21)
        cs = chord(ys)
        area = np.trapezoid(cs, ys)
        ym = np.trapezoid(cs * ys, ys) / area
        c = area / (y1 - y0)
        xle = x_le(ym)
        # outer-panel airfoil blend fraction (FX S 02-196 at the kink -> FX 60-126 at the tip)
        frac = 0.0 if ym < Y_KINK else (ym - Y_KINK) / (8.5 - Y_KINK)
        ail = (ym > AIL_Y[0]) and (ym < AIL_Y[1])
        brk = max(0.0, min(y1, BRAKE_Y[1]) - max(y0, BRAKE_Y[0])) / (y1 - y0)
        z = (ym - Y_FUS) * np.tan(np.radians(DIHEDRAL)) if ym > Y_FUS else 0.0
        out.append(dict(i=i, y0=y0, y1=y1, y=ym, dy=y1 - y0, c=c, area=area, x_c4=xle + 0.25 * c,
                        x_c34=xle + 0.75 * c, z=z, inc=float(incidence(ym)), frac=frac, ail=ail, brake=brk,
                        fus=(y1 <= Y_FUS + 1e-6)))
    return out


# ---------------------------------------------------------------- horizontal tail  [Mi-WHB]
HT_B, HT_S, HT_CR, HT_CT = 3.10, 1.92, 0.826, 0.413
HT_X_LE_ROOT = 4.92               # 3-view: LE 4.92 (root) .. 5.18 (tip), TE ~ straight 5.67..5.58
HT_X_LE_TIP = 5.18
HT_Z = 1.27                       # chord plane above the root-LE height (3-view side view)
HT_INC = I_ROOT - 1.75            # [FM-rig] tailplane 0 deg to the levelled fuselage, root wing 1.75 deg
HT_EDGES = np.array([0.0, 0.55, 1.10, 1.55])
ELEV_UP, ELEV_DN = 22.6, 16.2     # deg  [FM-rig] 90/65 mm at R 230


def ht_strips():
    out = []
    for i in range(len(HT_EDGES) - 1):
        y0, y1 = HT_EDGES[i], HT_EDGES[i + 1]
        ym = 0.5 * (y0 + y1); f = ym / (HT_B / 2)
        c = HT_CR + (HT_CT - HT_CR) * f
        xle = HT_X_LE_ROOT + (HT_X_LE_TIP - HT_X_LE_ROOT) * f
        out.append(dict(i=i, y=ym, dy=y1 - y0, c=c, area=c * (y1 - y0), x_c4=xle + 0.25 * c, x_c34=xle + 0.75 * c, z=HT_Z))
    s = sum(o['area'] for o in out)
    for o in out: o['area'] *= HT_S / 2 / s; o['c'] = o['area'] / o['dy']
    return out


# ---------------------------------------------------------------- fin + rudder  [Mi-WHB, 3-view]
VT_S = 1.357
VT_ZB, VT_ZT = -0.05, 1.26         # from the fuselage centre line region up to the tailplane
VT_TE = 5.80
RUD_MAX = 27.8                     # [FM-rig] 180 mm at R 375
VT_EDGES = np.array([-0.05, 0.40, 0.85, 1.26])


def vt_strips():
    out = []
    for i in range(len(VT_EDGES) - 1):
        z0, z1 = VT_EDGES[i], VT_EDGES[i + 1]
        zm = 0.5 * (z0 + z1)
        xle = 4.645 + (4.996 - 4.645) * (zm - 0.15) / (1.25 - 0.15)
        c = VT_TE - xle
        out.append(dict(i=i, z=zm, dz=z1 - z0, c=c, area=c * (z1 - z0), x_c4=xle + 0.25 * c, x_c34=xle + 0.75 * c))
    s = sum(o['area'] for o in out)
    for o in out: o['area'] *= VT_S / s; o['c'] = o['area'] / o['dz']
    return out


# ---------------------------------------------------------------- fuselage (3-view side + top views)
# X (aft of datum), top Z, bottom Z, width
FUS = np.array([
    [-2.575, -0.31, -0.31, 0.00], [-2.50, -0.234, -0.363, 0.126], [-2.40, -0.160, -0.398, 0.214],
    [-2.20, -0.043, -0.452, 0.341], [-2.00, 0.049, -0.491, 0.436], [-1.80, 0.131, -0.520, 0.511],
    [-1.60, 0.203, -0.543, 0.572], [-1.40, 0.263, -0.560, 0.622], [-1.20, 0.316, -0.571, 0.661],
    [-1.00, 0.362, -0.577, 0.685], [-0.80, 0.402, -0.577, 0.699], [-0.60, 0.433, -0.571, 0.702],
    [-0.40, 0.454, -0.558, 0.698], [-0.20, 0.465, -0.544, 0.690], [0.00, 0.464, -0.529, 0.700],
    [0.40, 0.440, -0.593, 0.705], [0.80, 0.396, -0.557, 0.700], [1.00, 0.368, -0.448, 0.650],
    [1.20, 0.336, -0.351, 0.600], [1.50, 0.295, -0.316, 0.540], [2.00, 0.248, -0.279, 0.494],
    [2.50, 0.213, -0.246, 0.446], [3.00, 0.178, -0.215, 0.398], [3.50, 0.154, -0.190, 0.350],
    [4.00, 0.131, -0.167, 0.302], [4.40, 0.111, -0.148, 0.264], [4.80, 0.100, -0.125, 0.226],
    [5.20, 0.090, -0.120, 0.150], [5.80, 0.05, -0.00, 0.03]])
FUS_SHAPE_K = 0.82                # cross-section area / (width*height): front view of the 3-view (super-ellipse)
FUS_STATIONS = np.array([-2.575, -1.9, -1.3, -0.7, -0.1, 0.5, 1.2, 2.0, 3.0, 4.0, 4.8, 5.8])


def fuselage():
    x = FUS[:, 0]; h = FUS[:, 1] - FUS[:, 2]; w = FUS[:, 3]; zc = 0.5 * (FUS[:, 1] + FUS[:, 2])
    xs = np.linspace(x[0], x[-1], 400)
    hs = np.interp(xs, x, h); ws = np.interp(xs, x, w); zs = np.interp(xs, x, zc)
    A = FUS_SHAPE_K * hs * ws
    a, b = ws / 2, hs / 2
    per = np.pi * (3 * (a + b) - np.sqrt((3 * a + b) * (a + 3 * b)))     # Ramanujan
    vol = np.trapezoid(A, xs); wet = np.trapezoid(per, xs)
    side = np.trapezoid(hs, xs); plan = np.trapezoid(ws, xs)
    st = []
    for i in range(len(FUS_STATIONS) - 1):
        x0, x1 = FUS_STATIONS[i], FUS_STATIONS[i + 1]
        sel = (xs >= x0) & (xs <= x1)
        st.append(dict(i=i, x=0.5 * (x0 + x1), dx=x1 - x0, h=float(np.mean(hs[sel])), w=float(np.mean(ws[sel])),
                       z=float(np.mean(zs[sel])), side=float(np.trapezoid(hs[sel], xs[sel])),
                       plan=float(np.trapezoid(ws[sel], xs[sel]))))
    return dict(xs=xs, hs=hs, ws=ws, zs=zs, A=A, vol=vol, wet=wet, side=side, plan=plan, stations=st,
                length=x[-1] - x[0], dmax=np.sqrt(4 * A.max() / np.pi))


# ---------------------------------------------------------------- masses (structural frame, m)
EMPTY_X = -0.900                  # 29.02 in aft of datum (AFFTC, S/N 21235)
ARM_FRONT, ARM_REAR, ARM_TAIL, ARM_CKPT = -47.91 * IN, -3.15 * IN, 209.8 * IN, -63.39 * IN
PILOT_FRONT = (ARM_FRONT + XS0, 0.0, 0.50)
PILOT_REAR = (ARM_REAR + XS0, 0.0, 0.58)
TAIL_BALLAST = (ARM_TAIL + XS0, 0.0, 0.65)
CKPT_BALLAST = (ARM_CKPT + XS0, 0.0, 0.25)

if __name__ == '__main__':
    W = wing_strips(); s = 2 * sum(o['area'] for o in W)
    print('wing area %.3f (TCDS 17.95)' % s)
    for o in W: print('  strip %2d y %.2f dy %.2f c %.3f inc %.2f frac %.2f ail %d brake %.2f z %.3f' % (o['i'], o['y'], o['dy'], o['c'], o['inc'], o['frac'], o['ail'], o['brake'], o['z']))
    for o in ht_strips(): print('  HT', {k: round(v, 3) for k, v in o.items()})
    for o in vt_strips(): print('  VT', {k: round(v, 3) for k, v in o.items()})
    F = fuselage()
    print('fuselage: length %.2f  volume %.3f m3  wetted %.2f m2 (Mi-WHB 12.33)  side %.2f m2  plan %.2f m2  dmax %.3f' % (F['length'], F['vol'], F['wet'], F['side'], F['plan'], F['dmax']))
    for st in F['stations']: print('   st', {k: round(v, 3) for k, v in st.items()})
