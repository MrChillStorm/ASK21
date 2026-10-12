# ####################################################################################
# Yaw strings on the front and rear canopy.
#
# Each string shows the airflow where it is taped, not at the centre of gravity. The
# yaw and roll rates move the canopy sideways through the air, so in a turn the front
# string, about 2 m ahead of the centre of gravity, reads a few degrees different
# from the sideslip (r * l / V). With JSBSim the rates are the air-relative ones,
# which include the rotation of the turbulence: a gust that swings the air around the
# nose reaches the string, not only one that pushes the whole glider sideways.
#
# On top of that the wool quivers, which is too small and too fast for the wind field
# of the simulation: a little in smooth air, more the stronger the turbulence that
# JSBSim applies (FlightGear's turbulence setting with YASim), and a lot when the
# airflow is weak (take-off run).
#
# Writes instrumentation/yawstring[0] (front) and [1] (rear) for
# Models/Interior/yawstring/yawstring.xml, in degrees: positive = airflow from the
# right, which blows the free end to the left.
# ####################################################################################

var strings = [                                         # where the tape is (model frame, m)
	{x: -3.4328, z: 0.8786, node: props.globals.getNode("instrumentation/yawstring[0]", 1), quiver: 0},
	{x: -2.22,   z: 1.1772, node: props.globals.getNode("instrumentation/yawstring[1]", 1), quiver: 0},
];
var quiver_smooth_deg = 0.7;                            # quiver in smooth air (rms)
var quiver_gust_k     = 0.5;                            # quiver per degree of gust angle
var quiver_weak_deg   = 12;                             # extra quiver when the airflow is weak
var weak_below_mps    = 25 * KT2MPS;                    # the airflow counts as weak below this
var quiver_tau_s      = 0.12;                           # how quickly the quiver changes
var gust_tau_s        = 2;                              # filter time of the measured gust strength
var cg_default        = {x: -1.33, z: 0.70};            # two pilots, when the FDM does not tell

var gust_ms = 0;                                        # mean square of the turbulence velocity (fps^2)

var gauss = func {
	var u = rand();
	if (u < 1e-9) u = 1e-9;
	return math.sqrt(-2 * math.ln(u)) * math.cos(2 * math.pi * rand());
}

var update = func {
	var dt = getprop("/sim/time/delta-sec");
	if (dt == nil or dt <= 0) return;

	var beta = (getprop("/orientation/side-slip-deg") or 0) * D2R;
	var jsb = getprop("/fdm/jsbsim/velocities/vt-fps");
	var v = 0; var p = 0; var r = 0; var cgx = 0; var cgz = 0; var gust_deg = 0;
	if (jsb != nil) {
		v = jsb * FT2M;
		p = getprop("/fdm/jsbsim/velocities/p-aero-rad_sec") or 0;
		r = getprop("/fdm/jsbsim/velocities/r-aero-rad_sec") or 0;
		cgx = (getprop("/fdm/jsbsim/inertia/cg-x-in") or 0) * IN2M;
		cgz = (getprop("/fdm/jsbsim/inertia/cg-z-in") or 0) * IN2M;
		var tn = getprop("/fdm/jsbsim/atmosphere/turb-north-fps") or 0;
		var te = getprop("/fdm/jsbsim/atmosphere/turb-east-fps") or 0;
		var td = getprop("/fdm/jsbsim/atmosphere/turb-down-fps") or 0;
		gust_ms += (tn * tn + te * te + td * td - gust_ms) * math.min(1, dt / gust_tau_s);
		gust_deg = (jsb > 10) ? math.atan2(math.sqrt(gust_ms), jsb) * R2D : 0;
	} else {
		v = (getprop("/velocities/airspeed-kt") or 0) * KT2MPS;
		p = (getprop("/orientation/roll-rate-degps") or 0) * D2R;
		r = (getprop("/orientation/yaw-rate-degps") or 0) * D2R;
		cgx = cg_default.x;
		cgz = cg_default.z;
		gust_deg = 8 * (getprop("/environment/turbulence/magnitude-norm") or 0);
	}

	var weak = math.max(0, 1 - v / weak_below_mps);
	var rms = quiver_smooth_deg + quiver_gust_k * gust_deg + quiver_weak_deg * weak;
	var k = math.min(0.5, dt / quiver_tau_s);
	foreach (var s; strings) {
		var l = cgx - s.x;                              # string ahead of the centre of gravity
		var h = s.z - cgz;                              # and above it
		var local = beta * R2D;
		if (v > 1) {
			# sideways airflow at the string: the glider's own, plus the canopy's motion from yaw and roll
			local = math.atan2(v * math.sin(beta) + r * l + p * h, v * math.cos(beta)) * R2D;
		}
		s.quiver += -s.quiver * k + rms * math.sqrt(2 * k) * gauss();
		s.node.setDoubleValue(local + s.quiver);
	}
}

var timer = maketimer(0, update);
timer.start();
