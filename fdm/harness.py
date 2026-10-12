"""JSBSim harness for the ASK 21 tests: private root per process, loadings by AFFTC arms."""
import jsbsim, os, shutil, atexit, numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
SRC = os.path.join(HERE, 'root')
LB = 0.45359237
DATUM_X_IN = -1.637 / 0.0254          # report datum in the structural frame (in)


def make_root(aero=None, jsb=None, tag=None):
    R = SRC + '_p%d%s' % (os.getpid(), ('_' + tag) if tag else '')
    if not os.path.isdir(R):
        shutil.copytree(SRC, R)
        atexit.register(lambda: shutil.rmtree(R, ignore_errors=True))
    if aero: shutil.copy(aero, R + '/aircraft/ask21-jsb/Systems/ask21-aero.xml')
    if jsb: shutil.copy(jsb, R + '/aircraft/ask21-jsb/ask21-jsb.xml')
    return R


def is_official(jsb):
    if jsb is None: return False
    s = open(jsb).read()
    return 'Spin Ballast' in s and 'Cockpit Ballast' not in s


def mk(aero=None, jsb=None, front_lb=176.0, rear_lb=0.0, tail_lb=0.0, ckpt_lb=0.0, dt=1 / 120.0, tag=None):
    if is_official(jsb):          # official FG model: no cockpit-ballast mass -> ballast goes with the front pilot
        front_lb, ckpt_lb = front_lb + ckpt_lb, 0.0
    R = make_root(aero, jsb, tag)
    f = jsbsim.FGFDMExec(R); f.set_debug_level(0)
    f.set_dt(dt)
    f['test/adot'] = 0.0          # free alpha-dot input of the *_noadot.xml test variants
    for p in ['/controls/engines/engine/throttle', '/systems/groundhandling/rudder', '/systems/groundhandling/throttle',
              '/systems/groundhandling/wingholder']:
        f[p] = 0.0
    f['/controls/engines/engine/throttle'] = 1.0          # throttle 1 = airbrakes closed (ask21-fcs)
    f.load_model('ask21-jsb')
    f['/controls/engines/engine/throttle'] = 1.0
    set_load(f, front_lb, rear_lb, tail_lb, ckpt_lb)
    f['ic/terrain-elevation-ft'] = -4000; f['ic/h-sl-ft'] = 3000
    return f


def set_load(f, front_lb, rear_lb, tail_lb=0.0, ckpt_lb=0.0):
    f['inertia/pointmass-weight-lbs[0]'] = front_lb
    f['inertia/pointmass-weight-lbs[1]'] = rear_lb
    f['inertia/pointmass-weight-lbs[2]'] = tail_lb
    if ckpt_lb:
        f['inertia/pointmass-weight-lbs[3]'] = ckpt_lb


def cg_report_in(f):
    return f['inertia/cg-x-in'] - DATUM_X_IN
