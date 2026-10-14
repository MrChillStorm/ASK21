# ####################################################################################
# The wake of the tow plane (and of any other aircraft ahead).
#
# FlightGear does not simulate the wake of AI or multiplayer aircraft, so this script
# works out the wake that the glider is flying through and hands it to the flight model
# as airflow (aero/v9/wash/*): every wing, tail, fin and fuselage station of the
# aerodynamics then feels the air velocity of the wake at its own position.
#
# The wake is where the other aircraft was: the script records its path, finds the piece
# of path the glider is passing now, and lets that piece drift with the wind and sink
# with its own downwash for the time since. It has two parts:
#  - the propeller slipstream: a jet on the thrust line, faster than the air around it
#    by what the thrust needs (momentum theory), spreading and slowing down behind the
#    propeller, turning with the propeller (clockwise seen from behind) and turbulent;
#  - the wing wake: the two trailing vortices of the lifting wing, with the circulation
#    of the tug's weight, sinking at their own induced speed. Between and around them the
#    air goes down, outside them up; a glider beside the wake is rolled.
# In normal high tow the glider stays above all this and is calm; on it or below it, it
# is rough. Before the tug flies, its slipstream simply blows straight back (take-off run).
#
# The tug is taken as a typical one (1000 kg, 9 m span, 1.9 m propeller). The strength
# (Aerotow Settings) scales everything. Only the JSBSim models have the wake inputs.
# ####################################################################################

var update_s      = 0;              # every frame
var max_range_m   = 300;            # aircraft further away are ignored
var history_s     = 15;             # how long the path of an aircraft is kept
var tug_mass_kg   = 1000;
var tug_span_m    = 9;
var prop_dia_m    = 1.9;
var thrust_line_m = 0.5;            # slipstream above, wing below the reported position
var wing_below_m  = 0.3;
var vortex_b_m    = math.pi / 4 * tug_span_m;     # spacing of the rolled-up vortices
var candidates    = [];
var paths         = {};             # recorded paths, by property path of the aircraft
var last_scan_s   = -10;
var active        = 0;
var turb          = [0, 0, 0, 0];   # u, v, w, roll (m/s), filtered noise

var wash = nil;                     # fdm/jsbsim/aero/v9/wash, once the FDM is up
var out = func(name, value) { wash.getNode(name, 1).setDoubleValue(value); }
var names = ["y-ft", "z-ft", "r-ft", "u-fps", "swirl-fps", "turb-u-fps", "turb-v-fps", "turb-w-fps",
	"turb-roll-fps", "vr-y-ft", "vr-z-ft", "vl-y-ft", "vl-z-ft", "gamma-ft2_sec", "core-ft"];
var clear = func {
	foreach (var n; names) out(n, 0);
	active = 0;
}

var gauss = func {
	var u = rand();
	if (u < 1e-9) u = 1e-9;
	return math.sqrt(-2 * math.ln(u)) * math.cos(2 * math.pi * rand());
}

# the aircraft that could have a wake, once per second
var scan = func {
	candidates = [];
	foreach (var n; props.globals.getNode("ai/models", 1).getChildren()) {
		var name = n.getName();
		if (name != "aircraft" and name != "multiplayer" and name != "dragger") continue;
		var valid = n.getNode("valid");
		if (valid == nil or !valid.getValue()) continue;
		if (n.getNode("position/latitude-deg") == nil or n.getNode("orientation/true-heading-deg") == nil) continue;
		append(candidates, n);
	}
}

# record where each candidate is
var record = func(t, me_lat, me_lon) {
	var seen = {};
	foreach (var n; candidates) {
		var lat = n.getNode("position/latitude-deg").getValue();
		var lon = n.getNode("position/longitude-deg").getValue();
		var dn = (lat - me_lat) * 111320;
		var de = (lon - me_lon) * 111320 * math.cos(me_lat * D2R);
		if (dn * dn + de * de > max_range_m * max_range_m) continue;
		var key = n.getPath();
		seen[key] = 1;
		if (!contains(paths, key)) paths[key] = {t: [], lat: [], lon: [], alt: [], hdg: []};
		var h = paths[key];
		if (size(h.t) and t - h.t[size(h.t) - 1] < 0.05) continue;
		append(h.t, t); append(h.lat, lat); append(h.lon, lon);
		append(h.alt, (n.getNode("position/altitude-ft").getValue() or 0) * FT2M);
		append(h.hdg, (n.getNode("orientation/nose-heading-deg") or n.getNode("orientation/true-heading-deg")).getValue() or 0);
		var k = 0;
		while (k < size(h.t) - 2 and t - h.t[k] > history_s) k += 1;
		if (k > 0) {
			h.t = subvec(h.t, k); h.lat = subvec(h.lat, k); h.lon = subvec(h.lon, k);
			h.alt = subvec(h.alt, k); h.hdg = subvec(h.hdg, k);
		}
	}
	foreach (var key; keys(paths)) {
		if (!contains(seen, key)) delete(paths, key);
	}
}

# The wake of one aircraft at the glider: centre (local east, north, up from the glider, m),
# direction of flight, age and strengths; nil if the glider is not behind it.
var wake_of = func(n, t, me, rho) {
	var h = paths[n.getPath()];
	if (h == nil or size(h.t) < 2) return nil;
	var v_tug = (n.getNode("velocities/true-airspeed-kt") != nil ? n.getNode("velocities/true-airspeed-kt").getValue() : 0) * KT2MPS;
	var vs_tug = (n.getNode("velocities/vertical-speed-fps") != nil ? n.getNode("velocities/vertical-speed-fps").getValue() : 0) * FT2M;
	var coslat = math.cos(me.lat() * D2R);
	var air_e = -(getprop("/environment/wind-from-east-fps") or 0) * FT2M;     # the air mass moves
	var air_n = -(getprop("/environment/wind-from-north-fps") or 0) * FT2M;
	var e = 0; var nn = 0; var u = 0; var hdg = 0; var age = 0;
	var last = size(h.t) - 1;

	if (v_tug < 15) {
		# not flying yet: the slipstream blows straight back from where the aircraft is now, at full
		# power from the tug of the drag robot, at idle from a taxiing aircraft, not at all from a parked one
		var thrust = 0;
		if (n.getName() == "dragger" and getprop("sim/glider/dragger/flags/run") == 1) thrust = 3000;
		elsif (v_tug > 3) thrust = 400;
		if (thrust == 0) return nil;
		hdg = h.hdg[last];
		var de = (h.lon[last] - me.lon()) * 111320 * coslat;     # the aircraft, seen from the glider
		var dn = (h.lat[last] - me.lat()) * 111320;
		var along = de * math.sin(hdg * D2R) + dn * math.cos(hdg * D2R);     # > 0: the glider is behind it
		if (along < 2) return nil;
		e = de - along * math.sin(hdg * D2R);                    # the point of its axis abeam the glider
		nn = dn - along * math.cos(hdg * D2R);
		u = h.alt[last] - me.alt();
		return {e: e, n: nn, u: u, hdg: hdg, x: along, age: 0, v: v_tug, vs: vs_tug, jet: 1, thrust: thrust};
	}

	# flying: find the piece of path that the glider is passing now (along-track distance changes sign)
	var newer = nil;
	for (var i = last; i >= 0; i -= 1) {
		var a = t - h.t[i];
		var de = (h.lon[i] - me.lon()) * 111320 * coslat + air_e * a;   # this piece of path, drifted with the air
		var dn = (h.lat[i] - me.lat()) * 111320 + air_n * a;
		var s = math.sin(h.hdg[i] * D2R); var c = math.cos(h.hdg[i] * D2R);
		var along = -(de * s + dn * c);                          # > 0: the glider is already past this point
		if (along > 0 and newer != nil) {
			var f = along / (along - newer.along);               # 0 at this point, 1 at the newer one
			e = de + (newer.de - de) * f;
			nn = dn + (newer.dn - dn) * f;
			u = h.alt[i] + (newer.alt - h.alt[i]) * f - me.alt();
			hdg = h.hdg[i];
			age = a + (newer.age - a) * f;
			var x = v_tug * age;                                 # distance behind the propeller in the air
			if (x < 2) return nil;
			return {e: e, n: nn, u: u, hdg: hdg, x: x, age: age, v: v_tug, vs: vs_tug, jet: 0, thrust: nil};
		}
		newer = {along: along, de: de, dn: dn, alt: h.alt[i], age: a};
	}
	return nil;
}

# local east/north/up offset -> structural frame of the glider (m: y right, z up)
var to_body = func(e, n, u) {
	var psi = (getprop("/orientation/heading-deg") or 0) * D2R;
	var the = (getprop("/orientation/pitch-deg") or 0) * D2R;
	var phi = (getprop("/orientation/roll-deg") or 0) * D2R;
	var d = -u;                                                   # NED
	var x1 = n * math.cos(psi) + e * math.sin(psi);
	var y1 = -n * math.sin(psi) + e * math.cos(psi);
	var x2 = x1 * math.cos(the) - d * math.sin(the);
	var z2 = x1 * math.sin(the) + d * math.cos(the);
	var y3 = y1 * math.cos(phi) + z2 * math.sin(phi);
	var z3 = -y1 * math.sin(phi) + z2 * math.cos(phi);
	return [y3, -z3];
}

var update = func {
	var t = getprop("/sim/time/elapsed-sec") or 0;
	var dt = getprop("/sim/time/delta-sec") or 0;
	if (dt <= 0) return;
	var me = geo.aircraft_position();
	if (t - last_scan_s > 1) {
		scan();
		last_scan_s = t;
	}
	record(t, me.lat(), me.lon());

	var strength = getprop("sim/glider/propwash/strength");
	if (strength == nil or strength <= 0) {
		if (active) clear();
		return;
	}
	var rho = (getprop("/environment/density-slugft3") or 0.0023769) * 515.379;   # kg/m3

	# the nearest wake
	var best = nil;
	var best_d = 1e9;
	foreach (var n; candidates) {
		var w = wake_of(n, t, me, rho);
		if (w == nil) continue;
		var d = math.sqrt(w.e * w.e + w.n * w.n + w.u * w.u);
		if (d < best_d) { best = w; best_d = d; }
	}
	if (best == nil) {
		if (active) clear();
		return;
	}
	var w = best;

	# strengths
	var v = math.max(w.v, 1);
	var area = math.pi * prop_dia_m * prop_dia_m / 4;
	var thrust = w.jet ? w.thrust : math.min(3500, math.max(400, 1800 + tug_mass_kg * 9.81 * w.vs / v));
	var dv0 = -v + math.sqrt(v * v + 2 * thrust / (rho * area));                 # slipstream far behind the disc
	var u_jet = dv0 * math.min(1, 4 * prop_dia_m / w.x) * strength;
	var r_jet = 0.45 * prop_dia_m + 0.07 * w.x;
	var lift = w.jet ? 0 : math.min(1, math.max(0, (w.v / KT2MPS - 35) / 15));     # the wing carries the tug
	var gamma0 = tug_mass_kg * 9.81 * lift / (rho * v * vortex_b_m) * math.exp(-w.age / 40);
	var sink = gamma0 / (2 * math.pi * vortex_b_m);                 # the vortex pair sinks by itself
	var gamma = gamma0 * strength;
	var core = 0.3 + 0.25 * math.sqrt(w.age);

	# far from everything: nothing to do
	if (best_d > 5 * r_jet and best_d > 6 * vortex_b_m) {
		if (active) clear();
		return;
	}

	# turbulence in the slipstream: eddies the size of the jet, passed at flying speed
	var sigma = 0.4 * u_jet;
	var vg = math.max(5, (getprop("/fdm/jsbsim/velocities/vt-fps") or 0) * FT2M);
	var k = math.min(0.5, dt / math.max(0.06, r_jet / vg));
	var rms = [0.5 * sigma, sigma, sigma, 0.7 * sigma];
	for (var i = 0; i < 4; i += 1)
		turb[i] += -turb[i] * k + rms[i] * math.sqrt(2 * k) * gauss();

	# positions in the glider's frame: slipstream centre and the two vortices
	var up_s = w.u + thrust_line_m - (w.jet ? 0 : sink * w.age);
	var up_v = w.u - wing_below_m - sink * w.age;
	var c = to_body(w.e, w.n, up_s);
	var re = math.cos(w.hdg * D2R) * vortex_b_m / 2;             # right of the tug's track (east, north)
	var rn = -math.sin(w.hdg * D2R) * vortex_b_m / 2;
	var vr = to_body(w.e + re, w.n + rn, up_v);
	var vl = to_body(w.e - re, w.n - rn, up_v);

	out("y-ft", c[0] * M2FT);       out("z-ft", c[1] * M2FT);       out("r-ft", r_jet * M2FT);
	out("u-fps", u_jet * M2FT);     out("swirl-fps", 0.3 * u_jet * M2FT);
	out("turb-u-fps", turb[0] * M2FT); out("turb-v-fps", turb[1] * M2FT);
	out("turb-w-fps", turb[2] * M2FT); out("turb-roll-fps", turb[3] * M2FT);
	out("vr-y-ft", vr[0] * M2FT);   out("vr-z-ft", vr[1] * M2FT);
	out("vl-y-ft", vl[0] * M2FT);   out("vl-z-ft", vl[1] * M2FT);
	out("gamma-ft2_sec", gamma * M2FT * M2FT);
	out("core-ft", core * M2FT);
	active = 1;
}

var timer = maketimer(update_s, update);

setlistener("sim/signals/fdm-initialized", func {
	if (getprop("sim/glider/propwash/strength") == nil) {
		setprop("sim/glider/propwash/strength", 1);
	}
	# only the JSBSim models have the wake inputs
	wash = props.globals.getNode("fdm/jsbsim/aero/v9/wash");
	if (wash != nil and wash.getNode("u-fps") != nil) {
		timer.start();
	}
});
