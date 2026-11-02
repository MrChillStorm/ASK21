#!/usr/bin/env python3
"""The ASK 21's sound at physical levels.

The airflow is PyWopWop's (https://github.com/MrChillStorm/PyWopWop), a code that computes aircraft noise from the
physics: its glider bank of the ASK 21 (`pywopwop glider-bank ask21`), seamless loops of the wing and tail's
self-noise, the canopy's wind, what the airbrakes and the ailerons add and the stall, in the cockpit and outside, at sea
level from 40 to 150 kt, the stall at 1 to 5 g, inverted flight from 60 to 150 kt and its stall, and the canopy's wind
in a full-rudder slip, and its FlightGear section, which cross-fades them by the indicated airspeed, the g, the angle
of attack and the sideslip. At altitude the same indicated
airspeed is a higher true airspeed in thinner air. PyWopWop's own model (pywopwop.glider.spectra) at the same
equivalent airspeed at 1 to 5 km gives each loop's change: the frequency shift that best matches its spectrum and the
level that then matches its A-weighted energy follow sigma^b and sigma^a (sigma: the air's density over sea level's)
to within 0.5 dB (0.9 dB for the canopy in a full slip at 150 kt), and each loop's pitch and volume carry its law. What a shift and a level leave of the spectrum's
change is small but for the wing and tail's self-noise (about 1.5 dB per third octave at 3 km), which in the cockpit
lies some 60 dB under the canopy's wind. SimGear clips a volume above 1, so the quieter loops' samples are raised by
what their law needs up to TOP_M, and the whole aircraft turned down by what a loop's peak can't give. The wing's
separated sound fades in on the angle of attack over PyWopWop's window (its root's separation to its stall), upright
and inverted, each moved to where the flight model stalls (the campaign's results).

Every other sound plays at the level it makes, on the bank's scale: a full-scale sample is 93.98 - gain dB re 20 uPa
(the bank's manifest; about 110 dB in the cockpit, 111.5 dB 30 m outside) plus the turn-down, so a file whose
A-weighted rms (a click's: its loudest 125 ms) is L dB re its full scale plays at volume
10^((SPL - full scale - L)/20). LEVELS has each sound's
level in the cockpit (the front seat) and 30 m outside, the law it follows and where the level comes from: measured
data where there are any, otherwise an estimate, said so. Outside, the 30 m level holds nearer (reference-dist, as the
bank's) and falls by 6 dB per doubling beyond. The made-up airflow sounds of the original (a howl at low speed, a whoosh
with the pitch rate) are taken out: the airflow is computed now.

    FG_ROOT=<FlightGear's data> python3 airsound.py <PyWopWop checkout>/out/ask21
        copies the loops the bank's section plays to ASK21_v9/Sounds/pywopwop/, writes the section with the altitude
        laws into Sounds/ask21-sound.xml (between BEGIN and END) and the other sounds' volumes (after LEVEL_TAG)

PyWopWop must be importable (it computes the altitude laws, about half a minute).
"""
import json, os, re, shutil, sys

import numpy as np
import scipy.io.wavfile as wav

HERE = os.path.dirname(os.path.abspath(__file__))
AIRCRAFT = os.path.join(HERE, '..', 'ASK21_v9')
SOUNDS = os.path.join(AIRCRAFT, 'Sounds')
SOUND_FILE = os.path.join(SOUNDS, 'ask21-sound.xml')
SUBDIR = 'pywopwop'
BEGIN = '\t\t<!-- the airflow: PyWopWop\'s glider bank with its altitude laws, written by fdm/airsound.py -->'
END = '\t\t<!-- end of fdm/airsound.py\'s airflow -->'
LEVEL_TAG = '<!-- level: fdm/airsound.py, '
REMOVE = ('low-speed-howl', 'whoosh-positive', 'whoosh-negative')
# The wing's separated sound fades in on the angle of attack between where its root starts to separate and its stall,
# both PyWopWop's (its sections' boundary layers). The flight model's wing stalls at its own angle (the trimmed maximum
# lift's, the campaign's 'clmax' in RESULTS; inverted the trimmed minimum's, its 'inverted'): the window moves by the
# difference, so the sound stalls with the model
RESULTS = os.path.join(HERE, 'results_v9.json')
H_FIT = (1000.0, 2000.0, 3000.0, 4000.0, 5000.0)
# SimGear clips a sound's volume at 1, and at altitude a loop's sigma^a is more than 1: each loop has the room for it up
# to TOP_M, its samples raised as far as its peak allows (to PEAK_DB re full scale), the whole aircraft turned down by
# what is still missing
TOP_M = 10000.0
SIGMA_TOP = (1 - 2.25577e-5 * TOP_M) ** 4.25588            # the ISA's, in the troposphere
PEAK_DB = -0.5
SIGMA = '<div><property>/environment/density-slugft3</property><value>0.00237689</value></div>'
INTERNAL = '<property>sim/current-view/internal</property>'
KMH = 1 / 1.852                                            # kt per km/h

# The tyres: tyre/road noise at the tyre (CPX, ISO 11819-2: 0.2 m from the tyre) on dense asphalt at 80 km/h is about
# 98 dB(A) (the Dutch TKP 7 reference; SMA surfaces 96.8-100.7 dB), growing about as 30 log10 of the speed (Sandberg and
# Ejsmont, Tyre/Road Noise Reference Book); the glider's 5.00-5 tyre, narrower than a car's, some 2 dB less. Outside at
# 30 m: 98 - 2 - 20 log10(30 / 0.2) = 52.5 dB(A). In the cockpit no data: the airborne part through the fuselage's floor
# alone would be about 50 dB(A), but the wheel is bolted to the shell under the seat with no suspension or lining, so the
# rumble comes through the structure. Estimate: a car's tyre noise inside at 80 km/h, about 66 dB(A), plus 10 dB.
TYRE_REF_KT = 80 * KMH
VARIO_DBA = 92.0      # a vario at full volume at the pilot: LXNAV's 2.6 W class-D amplifier into 4 ohm (SxHAWK manual),
                      # a small speaker of about 82 dB/W at 1 m, 0.5 m from the ear
LEVELS = [
    # names, cockpit dB(A), outside dB(A) at 30 m (None: not heard outside), the law of the amplitude (1 at the
    # reference state), a click (its loudest 125 ms is its level), where the level comes from
    (('wheel-rumble-main',), 76.0, 52.5, ('pow', 'velocities/groundspeed-kt', TYRE_REF_KT, 1.5), False,
     'main wheel rolling at 80 km/h: outside the tyre/road data, in the cockpit an estimate (see the tyres)'),
    (('wheel-rumble-front', 'wheel-rumble-rear'), 70.0, 46.5, ('pow', 'velocities/groundspeed-kt', TYRE_REF_KT, 1.5), False,
     'nose and tail wheels rolling at 80 km/h: estimate, 6 dB under the main wheel (little of the weight)'),
    (('wheel-spin-main',), 66.0, 42.5, ('pow', 'gear/gear[1]/rollspeed-ms', 80 / 3.6, 1.5), False,
     'the main wheel\'s whine as it turns, at 80 km/h: estimate, 10 dB under its rolling'),
    (('wheel-spin-front', 'wheel-spin-tail'), 60.0, 36.5, (('pow', 'gear/gear[0]/rollspeed-ms', 80 / 3.6, 1.5),
                                                           ('pow', 'gear/gear[2]/rollspeed-ms', 80 / 3.6, 1.5)), False,
     'nose and tail wheels turning, at 80 km/h: estimate, 6 dB under the main wheel\'s'),
    (('squeal-main',), 75.0, 68.0, ('pow', 'velocities/groundspeed-kt', 70 * KMH, 1.5), True,
     'the tyre scrubbing as it spins up at a touchdown at 70 km/h: estimate'),
    (('squeal-front', 'squeal-rear'), 69.0, 62.0, ('pow', 'velocities/groundspeed-kt', 70 * KMH, 1.5), True,
     'nose and tail wheels touching down at 70 km/h: estimate, 6 dB under the main wheel'),
    (('wheelbrake',), 72.0, 50.0, ('pow', 'velocities/groundspeed-kt', 40 * KMH, 1.0), False,
     'the drum brake rubbing at 40 km/h: estimate'),
    (('airbrake-open', 'airbrake-close'), 85.0, 55.0, None, True,
     'the airbrakes\' over-centre lock and paddles: estimate'),
    (('variometer-up', 'variometer-down'), VARIO_DBA, None, ('prop', 'instrumentation/ilec-sc7/volume'), False,
     'the ILEC vario\'s tones at full volume (see VARIO_DBA), times its volume'),
    (('lxv8-tone',), VARIO_DBA, None, ('prop', 'instrumentation/lxv8/audio/volume'), False,
     'the V8\'s tones at full volume (see VARIO_DBA), times its volume'),
    (('winch_tow',), 70.0, 50.0, ('pow', 'velocities/airspeed-kt', 50.0, 3.0), False,
     'the winch cable singing (an Aeolian tone: amplitude as the cube of the speed) at 50 kt: estimate'),
    (('hook_open', 'nose_hook_cable_release', 'belly_hook_cable_release'), 85.0, 60.0, None, True,
     'a release snapping open: estimate'),
    (('window_front_open', 'window_front_close', 'window_rear_open', 'window_rear_close'), 75.0, None, None, True,
     'a sliding window\'s catch: estimate'),
    (('window_front_transit', 'window_rear_transit'), 60.0, None, None, False,
     'a sliding window sliding: estimate'),
]
WINDOW_DB = 10.0      # the wind through a fully open sliding window over the canopy's own (estimate)
WINDOWS = (('wind front', 'sim/model/sound/wind-volume-front'), ('wind rear', 'sim/model/sound/wind-volume-rear'))


# ------------------------------------------------------------------------------------------------- levels of files
def a_weight_db(f):
    """IEC 61672 A-weighting, dB"""
    f2 = np.asarray(f, float) ** 2
    ra = 12194.0 ** 2 * f2 ** 2 / ((f2 + 20.6 ** 2) * np.sqrt((f2 + 107.7 ** 2) * (f2 + 737.9 ** 2)) * (f2 + 12194.0 ** 2))
    return 20 * np.log10(np.maximum(ra, 1e-30)) + 2.0


def file_level(path, click=False):
    """a WAV's A-weighted level, dB re its full scale: its rms, or a click's loudest 125 ms"""
    fs, y = wav.read(path)
    if y.ndim > 1:
        y = y.mean(axis=1)
    y = (y.astype(np.float64) - (128.0 if y.dtype == np.uint8 else 0.0)) / (128.0 if y.dtype == np.uint8 else 32768.0)
    Y = np.fft.rfft(y - y.mean())
    w = 10 ** (a_weight_db(np.maximum(np.fft.rfftfreq(len(y), 1 / fs), 1e-3)) / 20)
    ya = np.fft.irfft(Y * w, len(y))
    if not click:
        return 10 * np.log10(np.mean(ya ** 2))
    n = max(1, int(0.125 * fs))
    ms = np.convolve(ya ** 2, np.ones(n) / n, 'valid') if len(ya) > n else [np.mean(ya ** 2)]
    return 10 * np.log10(np.max(ms))


def find(path):
    """a sound's file as FlightGear finds it: next to the sound file, in the aircraft, in FG_ROOT"""
    fg = os.environ.get('FG_ROOT', '')
    for r in (SOUNDS, AIRCRAFT, fg, os.path.join(fg, 'Sounds') if fg else ''):
        p = os.path.join(r, path)
        if r and os.path.exists(p):
            return p
    sys.exit('no file %s (set FG_ROOT to FlightGear\'s data)' % path)


# ------------------------------------------------------------------------------------------------- the bank's altitude laws
def altitude_laws(bank):
    """each loop's law, sigma^a in amplitude and sigma^b in frequency, from PyWopWop's model at the loop's equivalent
    airspeed at 1-5 km against sea level: {'40kt-cockpit-wind': (a, b, worst level misfit dB), ...}"""
    try:
        import pywopwop
    except ImportError:                                    # a bank is made in the checkout's out/
        sys.path.insert(0, os.path.abspath(os.path.join(bank, '..', '..')))
        try:
            import pywopwop
        except ImportError:
            sys.exit('PyWopWop is not importable: install it, or run with its Python')
    from pywopwop import glider as G
    from pywopwop.atmosphere import Atmosphere
    from pywopwop.model import Observer, STANDARD_OBSERVERS
    m = json.load(open(os.path.join(bank, 'manifest.json')))
    g = G.load_glider('ask21')
    sig_kw = set(G.GliderCondition.__dataclass_fields__)
    B = np.asarray(G.BANDS)
    lB, A = np.log(B), a_weight_db(B)
    so = STANDARD_OBSERVERS['side']
    pos = {'cockpit': (G.to_body(*g.cockpit['front']), True),
           'side': (Observer('side', so.azimuth_deg, so.elevation_deg, m['exterior_distance_m']).offset(), False)}
    rho0 = Atmosphere(0.0).rho
    # each condition as the bank made it: its airspeed and load factor (negative: inverted); the brakes fully out and full
    # aileron in a steady roll where it has their layers (each layer is only its own source's); full rudder in a slip
    conds = {}
    for f in m['files']:
        c = conds.setdefault(f['condition'], dict(eas=f['airspeed_kt'], n=f.get('load_factor', 1.0), layers=set()))
        c['layers'].add(f['source'])
    slips = {'slip_%gkt' % v for v in m.get('slip_speeds_kt', [])}

    def spectra(name, c, h, L, ins):
        kw = dict(airspeed_kt=c['eas'] / np.sqrt(Atmosphere(h).rho / rho0), altitude_m=h, load_factor=c['n'],
                  brakes=1.0 if 'brakes' in c['layers'] else 0.0, aileron=1.0 if 'ailerons' in c['layers'] else 0.0,
                  rudder=1.0 if name in slips else 0.0)
        return G.spectra(g, G.GliderCondition(**{k: v for k, v in kw.items() if k in sig_kw}), L, ins)[0]

    laws = {}
    for name, c in conds.items():
        s0 = {v: spectra(name, c, 0.0, L, ins) for v, (L, ins) in pos.items()}
        rows = {}
        for h in H_FIT:
            sig = Atmosphere(h).rho / rho0
            for v, (L, ins) in pos.items():
                s1 = spectra(name, c, h, L, ins)
                for layer in s1:
                    d0 = 10 * np.log10(np.maximum(np.asarray(s0[v][layer]), 1e-30) / 4e-10)
                    d1 = 10 * np.log10(np.maximum(np.asarray(s1[layer]), 1e-30) / 4e-10)
                    if d0.max() < -50:
                        continue
                    sel = (d1 + A) > (d1 + A).max() - 20
                    best = None
                    for P in np.exp(np.linspace(np.log(0.8), np.log(1.6), 241)):
                        sh = np.interp(lB, lB + np.log(P), d0 - 10 * np.log10(P), left=-200, right=-200)
                        r = (d1 - sh)[sel]
                        err = np.sqrt(np.mean((r - r.mean()) ** 2))
                        if best is None or err < best[0]:
                            best = (err, P, 10 * np.log10(np.sum(10 ** ((d1 + A) / 10)) / np.sum(10 ** ((sh + A) / 10))))
                    rows.setdefault((v, layer), []).append((sig, best[1], best[2]))
        for (v, layer), r in rows.items():
            r = np.array(r)
            ls = np.log(r[:, 0])
            b = float(np.sum(ls * np.log(r[:, 1])) / np.sum(ls ** 2))
            a = float(np.sum(ls * r[:, 2] / 20 * np.log(10)) / np.sum(ls ** 2))
            laws['%s-%s-%s' % (name, v, layer)] = (a, b, float(np.max(np.abs(r[:, 2] - 20 * a * ls / np.log(10)))))
    return laws


def stall_shift(bank):
    """the flight model's stall angles of attack (body; upright: the trimmed maximum lift's, inverted: the trimmed
    minimum's) less the bank's, deg: {'upright': (shift, model's, bank's), 'inverted': (...)}"""
    m = json.load(open(os.path.join(bank, 'manifest.json')))
    R = json.load(open(RESULTS))['results']
    up = float(np.mean([r['res']['alpha'] for r in R if r['kind'] == 'clmax' and r['res'].get('alpha')]))
    out = {'upright': (up - m['stall_alpha_body_deg'], up, m['stall_alpha_body_deg'])}
    if 'inverted_stall_alpha_body_deg' in m:
        inv = float(next(r['res']['alpha'] for r in R if r['kind'] == 'inverted'))
        out['inverted'] = (inv - m['inverted_stall_alpha_body_deg'], inv, m['inverted_stall_alpha_body_deg'])
    return out


def headroom(bank, laws, names):
    """the gain on each loop's samples (dB) and the aircraft's level turned down (dB), so that no loop's volume passes
    1 below TOP_M"""
    fm = {'%s-%s-%s' % (f['condition'], f['observer'], f['source']): f['file']
          for f in json.load(open(os.path.join(bank, 'manifest.json')))['files']}
    need, room = {}, {}
    for n in names:
        need[n] = max(0.0, 20 * laws[n][0] * np.log10(SIGMA_TOP))
        y = wav.read(os.path.join(bank, fm[n]))[1]
        room[n] = PEAK_DB - 20 * np.log10(np.abs(y.astype(float)).max() / 32768)
    down = max(0.0, max(need[n] - room[n] for n in names))
    return {n: max(0.0, need[n] - down) for n in names}, down


def bank_section(bank, laws, shifts, gain, down):
    """the bank's FlightGear entries, each loop's volume times sigma^a and its pitch sigma^b, the angle of attack's
    fades moved by the shifts (deg): the upright stall's (rising with alpha), the inverted's (falling); the volume less
    by the loop's gain and the aircraft's turn-down (dB). Returns the section and its (name, file)s"""
    x = open(os.path.join(bank, 'pywopwop-glider.xml'), encoding='utf-8').read()
    body = x[x.index('<fx>') + len('<fx>'):x.index('</fx>')].strip('\n')

    def move(m):                                           # clip(k alpha + c) for alpha - shift: c - k shift
        k, c = float(m.group(2)), float(m.group(3))
        shift = shifts['upright' if k > 0 else 'inverted'][0]
        return '%s%.7g%s' % (m.group(1), c - k * shift, m.group(4))
    body = re.sub(r'(<property>/orientation/alpha-deg</property><value>([-0-9.e]+)</value></product><value>)([-0-9.e]+)(</value>)',
                  move, body)
    out, files = [], []
    for m in re.finditer(r'(\t\t<([\w-]+)>\n.*?\n\t\t</\2>)', body, re.S):
        e = m.group(1)
        name = re.search(r'<name>pww-(.+?)</name>', e).group(1)
        a, b, _ = laws[name]
        db = gain[name] + down
        k = '' if db == 0 else '<value>%.6g</value>' % 10 ** (-db / 20)
        e = re.sub(r'<volume><expression>(.*?)</expression></volume>',
                   lambda v: '<volume><expression><product>%s<pow>%s<value>%.4f</value></pow>%s</product></expression></volume>'
                   % (v.group(1), SIGMA, a, k), e, count=1)
        e = e.replace('</volume>\n', '</volume>\n\t\t\t<pitch><expression><pow>%s<value>%.4f</value></pow></expression></pitch>\n'
                      % (SIGMA, b), 1)
        out.append(e)
        files.append((name, re.search(r'<path>Sounds/%s/([^<]+)</path>' % SUBDIR, e).group(1)))
    return '\n'.join(out), files


# ------------------------------------------------------------------------------------------------- the other sounds
def law_xml(law):
    """the amplitude's law as an expression (None: a constant)"""
    if law is None:
        return None
    if isinstance(law[0], tuple):
        return law
    if law[0] == 'pow':
        _, prop, ref, n = law
        return ('<pow><div><max><property>%s</property><value>0</value></max><value>%.6g</value></div><value>%g</value></pow>'
                % (prop, ref, n))
    if law[0] == 'prop':
        return '<property>%s</property>' % law[1]
    raise ValueError(law)


def volume_xml(k_in, k_out, law):
    """the volume: the cockpit's level inside, the outside's outside, times the law"""
    view = ('<product>%s<value>%.6g</value></product>' % (INTERNAL, k_in) if k_out is None else
            '<sum><value>%.6g</value><product>%s<value>%.6g</value></product></sum>' % (k_out, INTERNAL, k_in - k_out))
    return '<volume><expression>%s</expression></volume>' % ('<product>%s%s</product>' % (view, law) if law else view)


def set_entry(t, name, inner, note):
    """the sound <name>'s volumes and distances replaced by inner (its other settings kept)"""
    i = t.find('<name>%s</name>' % name)
    if i < 0:
        sys.exit('no sound %s in %s' % (name, SOUND_FILE))
    tag = re.findall(r'\n\t\t<([\w-]+)>\n', t[:i])[-1]
    s = t.rindex('\n\t\t<%s>\n' % tag, 0, i) + 1
    j = t.index('\n\t\t</%s>' % tag, i) + 1
    e = t[s:j]
    e = re.sub(r'\t*<!-- level: fdm/airsound\.py.*?-->\n', '', e)
    e = re.sub(r'\t*<volume>.*?</volume>\n', '', e, flags=re.S)
    e = re.sub(r'\t*<(reference-dist|max-dist)>.*?</\1>\n', '', e)
    e = e.replace('<name>%s</name>\n' % name, '<name>%s</name>\n\t\t\t%s%s -->\n%s' % (name, LEVEL_TAG, note, inner), 1)
    return t[:s] + e + t[j:]


def remove_entry(t, name):
    i = t.find('<name>%s</name>' % name)
    if i < 0:
        return t
    tag = re.findall(r'\n\t\t<([\w-]+)>\n', t[:i])[-1]
    s = t.rindex('\n\t\t<%s>\n' % tag, 0, i) + 1
    j = t.index('\n\t\t</%s>\n' % tag, i) + len('\n\t\t</%s>\n' % tag)
    if t[s - 2:s] == '\n\n':
        s -= 1
    return t[:s] + t[j:]


def other_sounds(t, full, bank):
    """LEVELS and the sliding windows' wind at their levels; returns the text and a report"""
    rep = []
    for names, l_in, l_out, law, click, note in LEVELS:
        for n_i, name in enumerate(names):
            path = re.search(r'<name>%s</name>.*?<path>([^<]+)</path>' % re.escape(name), t, re.S).group(1)
            lf = file_level(find(path), click)
            k_in = 10 ** ((l_in - full['cockpit'] - lf) / 20)
            k_out = None if l_out is None else 10 ** ((l_out - full['exterior'] - lf) / 20)
            for k in (k_in, k_out):
                if k is not None and k > 1:
                    sys.exit('%s would need a volume of %.2f: a louder file needed' % (name, k))
            lw = law_xml(law)
            if isinstance(lw, tuple):
                lw = law_xml(lw[n_i])
            dist = ('\t\t\t<reference-dist>30</reference-dist>\n\t\t\t<max-dist>1500</max-dist>\n' if l_out is not None else
                    '\t\t\t<reference-dist>3</reference-dist>\n\t\t\t<max-dist>10</max-dist>\n')
            t = set_entry(t, name, '\t\t\t%s\n%s' % (volume_xml(k_in, k_out, lw), dist),
                          '%.0f dB(A) in the cockpit%s: %s' % (l_in, '' if l_out is None else ', %.1f at 30 m' % l_out, note))
            rep.append('%-26s %5.1f dB re FS  cockpit %5.1f dB(A) k %.4f%s' % (
                name, lf, l_in, k_in, '' if k_out is None else '   outside %5.1f k %.4f' % (l_out, k_out)))
    # the wind through an open sliding window: the canopy's own level (the bank's cockpit wind) plus WINDOW_DB, against
    # the airspeed, times how far the window is open (Systems/sound.xml)
    m = json.load(open(os.path.join(bank, 'manifest.json')))
    wind = sorted((f['airspeed_kt'], f['dba']) for f in m['files']
                  if f['observer'] == 'cockpit' and f['source'] == 'wind' and f['condition'] != 'stall')
    v = np.array([w[0] for w in wind]); L = np.array([w[1] for w in wind])
    slope = np.polyfit(np.log10(v), L, 1)[0]
    for name, prop in WINDOWS:
        path = re.search(r'<name>%s</name>.*?<path>([^<]+)</path>' % re.escape(name), t, re.S).group(1)
        lf = file_level(find(path))
        kv = [(0.0, 0.0), (20.0, 10 ** ((L[0] + slope * np.log10(20.0 / v[0]) + WINDOW_DB - full['cockpit'] - lf) / 20))]
        kv += [(vi, 10 ** ((li + WINDOW_DB - full['cockpit'] - lf) / 20)) for vi, li in zip(v, L)]
        # the file can't be louder than its full scale: fully open the level holds up to where the volume reaches 1
        # (a window is opened at low speed: on the ground, thermalling)
        cap = 10 ** ((full['cockpit'] + lf - WINDOW_DB - L[0]) / slope) * v[0]
        kv = [(vi, min(k, 1.0)) for vi, k in kv]
        tab =('<table><property>/velocities/airspeed-kt</property>%s</table>'
               % ''.join('<entry><ind>%g</ind><dep>%.6g</dep></entry>' % e for e in kv))
        inner = ('\t\t\t<volume><expression><product>%s%s<property>%s</property></product></expression></volume>\n'
                 '\t\t\t<reference-dist>3</reference-dist>\n\t\t\t<max-dist>10</max-dist>\n' % (INTERNAL, tab, prop))
        t = set_entry(t, name, inner, 'the wind through the open sliding window: the canopy\'s own (PyWopWop) plus %g dB '
                      'fully open (estimate), times the opening (Systems/sound.xml)' % WINDOW_DB)
        rep.append('%-26s %5.1f dB re FS  cockpit, fully open: %.1f dB(A) at 40 kt; the file\'s full scale from %.0f kt'
                   % (name, lf, L[0] + WINDOW_DB, cap))
    for name in REMOVE:
        t = remove_entry(t, name)
    return t, rep


def install(bank):
    m = json.load(open(os.path.join(bank, 'manifest.json')))
    laws = altitude_laws(bank)
    names = re.findall(r'<name>pww-(.+?)</name>', open(os.path.join(bank, 'pywopwop-glider.xml'), encoding='utf-8').read())
    gain, down = headroom(bank, laws, names)
    full = {k: 20 * np.log10(1.0 / 10 ** (g / 20) / 2e-5) + down for k, g in m['gain_db'].items()}
    print('full scale: cockpit %.2f dB, outside (30 m) %.2f dB re 20 uPa (the bank\'s turned down %.2f dB; loops raised '
          'up to %.1f dB: the room for their altitude laws to %.0f km)'
          % (full['cockpit'], full['exterior'], down, max(gain.values()), TOP_M / 1000))
    shifts = stall_shift(bank)
    for k, (s, a_fdm, a_bank) in shifts.items():
        print('%s stall: the flight model\'s at %.2f deg, the bank\'s at %.2f: its angle of attack fades moved %+.2f deg'
              % (k, a_fdm, a_bank, s))
    body, loops = bank_section(bank, laws, shifts, gain, down)
    out = os.path.join(SOUNDS, SUBDIR)
    if os.path.isdir(out):
        shutil.rmtree(out)
    os.makedirs(out)
    for name, fn in loops:
        if gain[name] == 0:
            shutil.copy(os.path.join(bank, fn), os.path.join(out, fn))
        else:
            fs, y = wav.read(os.path.join(bank, fn))
            wav.write(os.path.join(out, fn), fs, np.round(y * 10 ** (gain[name] / 20)).astype(np.int16))
    files = [fn for _, fn in loops]
    shutil.copy(os.path.join(bank, 'manifest.json'), os.path.join(out, 'manifest.json'))
    t = open(SOUND_FILE, encoding='utf-8').read()
    t = re.sub(r'<!-- the airflow sounds are PyWopWop\'s.*?-->\n', '', t, flags=re.S)
    t = t.replace('<PropertyList include="pywopwop/pywopwop-glider.xml">', '<PropertyList>')
    block = BEGIN + '\n' + body + '\n' + END + '\n'
    if BEGIN in t:
        t = t[:t.index(BEGIN)] + block + t[t.index(END) + len(END) + 1:]
    else:
        at = t.index('<fx>\n') + len('<fx>\n')
        t = t[:at] + block + '\n' + t[at:]
    t, rep = other_sounds(t, full, bank)
    open(SOUND_FILE, 'w', encoding='utf-8').write(t)
    mb = sum(os.path.getsize(os.path.join(out, fn)) for fn in files) / 2 ** 20
    print('%d loops (%.0f MB) in %s; %d airflow sounds and %d others in %s' % (len(files), mb, out, len(files), len(rep), SOUND_FILE))
    print('altitude laws (amplitude sigma^a, pitch sigma^b; worst level misfit 0-5 km):')
    for k in sorted(laws, key=lambda k: (k.split('-')[1], k.split('-')[2], float(k.split('-')[0][:-2]) if k[0].isdigit() else 0)):
        a, b, e = laws[k]
        print('  %-24s a %+.3f  b %+.3f  (%.2f dB)' % (k, a, b, e))
    print('the other sounds:')
    for r in rep:
        print('  ' + r)


if __name__ == '__main__':
    if len(sys.argv) != 2:
        sys.exit(__doc__)
    install(os.path.expanduser(sys.argv[1]))
