# ####################################################################################
# LXNAV V8 vario indicator: flight data, vario audio and voice
#
# The maths and the audio behaviour are ported from the FG9070 trainer
# (js/flight/te.js, js/flight/polar.js, js/flight/state.js, js/audio/vario-audio.js),
# which follows the LX90xx/LX80xx user manual. Defaults are the manual's defaults.
#
# Provides the values shown by the display (lxv8.nas) in the hash `f`, and drives the
# vario tone through /instrumentation/lxv8/audio/{pitch,volume}, which Sounds/ask21-sound.xml
# plays with a looped triangle wave (lxv8-tone.wav, 500 Hz). Voice messages are pre-rendered
# clips (Sounds/lxv8-voice/, made with tools/make_voice.sh) played one after the other.
#
# Approximations (like in FG9070): the polar of the ASK 21 is taken from three published
# points and not from the flight model; the audio cadence is not given in the manual.
# ####################################################################################

var G = 9.80665;
var KT2MS = 0.514444;

var root = props.globals.getNode("/instrumentation/lxv8", 1);

var clamp = func(x, lo, hi) { return x < lo ? lo : (x > hi ? hi : x); };
var wrap360 = func(a) { while (a < 0) a += 360; while (a >= 360) a -= 360; return a; };
var wrap180 = func(a) { while (a < -180) a += 360; while (a > 180) a -= 360; return a; };

# ---- settings (manual defaults, see FG9070 js/core/lx.js)
var S = {
	# Indicator (manual 7.1.12.4.1 / 7.1.12.4.2.1)
	nbCount: 3,              # number of navboxes on the numeric page, 1 ... 4 (manual default 4; fewer = bigger digits)
	nb1: "avgv", nb2: "netto", nb3: "alt", nb4: "tas",
	needleClimb: "vario",    # vario | netto | relative | sc, in climb mode ...
	needleCruise: "vario",   # ... and in cruise mode
	needleStyle: "medium",   # thin | medium | thick
	scaleType: "non-linear", # linear | non-linear
	theme: "black",          # white | grey | black (colour of the dial ring)
	yellowBar: "minmax",     # minmax | none
	redDiamond: "avg",       # avg | netto | none
	mcDiamond: 1,            # blue MacCready triangle
	sideBar: "stf",          # stf | none
	flarmIcon: 1,
	thermalMode: "auto",     # thermal assistant colours: auto span | MacCready | average vario
	autoThermal: 1,          # switch to the thermal assistant while circling
	# FLARM
	flarmAlarms: 1,          # show the radar by itself on a warning
	flarmDismiss: 15,        # s, a dismissed warning stays quiet
	# Display
	brightness: 90,          # %
	nightMode: 0,
	nightDark: 40,           # % of the brightness
	# Sounds
	volume: 80,              # % (also the S / s keys)
	speechThermal: 1,        # "Thermal average ..." when a thermal ends
	speechFlarm: 1,          # spoken FLARM warnings
	varioRange: 5,           # m/s, full scale of the needle
	needleTau: 1.5,          # s
	soundTau: 1.5,           # s
	scBand: 1.0,             # m/s, audio dead band in speed-to-fly mode
	integrator: 20,          # s, average vario
	nettoFilter: 1.5,
	scTau: 1.5,
	relTau: 1.5,
	nettoTime: 20,
	teComp: 1.0,             # digital TE compensation
	audioMode: "Linear positive",
	scAudioMode: "SC mixed",
	freq0: 500,
	freqPlus: 1500,
	freqMinus: 200,
	mc: 1.0,                 # MacCready, m/s
	uVario: "m/s",           # m/s | kt | ft/min
	uAlt: "m",               # m | ft
	uSpeed: "km/h",          # km/h | kt | mph
	speech: 1,               # voice messages
};

# settings are kept in /instrumentation/lxv8/settings, which the aircraft saves (ask21-main.xml, aircraft-data)
var settings_node = root.getNode("settings", 1);
var is_number = func(x) { return typeof(x) == "scalar" and num(x) != nil; };
var load_settings = func {
	foreach (var k; keys(S)) {
		var n = settings_node.getNode(k);
		if (n != nil and n.getValue() != nil) {
			S[k] = is_number(S[k]) ? num(n.getValue()) : sprintf("%s", n.getValue());
		} else {
			settings_node.getNode(k, 1).setValue(S[k]);
		}
	}
	setprop("/instrumentation/ilec-sc7/volume", S.volume / 100);
};

var setting_set = func(key, val) {
	S[key] = val;
	settings_node.getNode(key, 1).setValue(val);
	if (key == "volume") setprop("/instrumentation/ilec-sc7/volume", val / 100);
};

# ---- polar: w(v) = a v^2 + b v + c (v in m/s, sink positive), through three points
var fit_polar = func(pts) {
	var v = [pts[0][0] / 3.6, pts[1][0] / 3.6, pts[2][0] / 3.6];
	var w = [pts[0][1], pts[1][1], pts[2][1]];
	var det = func(m) {
		return m[0][0] * (m[1][1] * m[2][2] - m[1][2] * m[2][1])
		     - m[0][1] * (m[1][0] * m[2][2] - m[1][2] * m[2][0])
		     + m[0][2] * (m[1][0] * m[2][1] - m[1][1] * m[2][0]);
	};
	var M = [[v[0]*v[0], v[0], 1], [v[1]*v[1], v[1], 1], [v[2]*v[2], v[2], 1]];
	var D = det(M);
	var col = func(i) {
		var r = [];
		for (var k = 0; k < 3; k += 1) {
			var row = [M[k][0], M[k][1], M[k][2]];
			row[i] = w[k];
			append(r, row);
		}
		return r;
	};
	return [det(col(0)) / D, det(col(1)) / D, det(col(2)) / D];
};

# ASK 21 (approximate, as in FG9070)
var polar_abc = fit_polar([[75, 0.65], [98, 0.80], [150, 1.75]]);
var polar_min_speed = 60 / 3.6;
var polar_max_speed = 250 / 3.6;
var sink_at = func(v) {
	return math.max(0.05, polar_abc[0] * v * v + polar_abc[1] * v + polar_abc[2]);
};
var min_sink = 9;
for (var v = polar_min_speed; v <= polar_max_speed; v += 0.1) {
	var w = sink_at(v);
	if (w < min_sink) min_sink = w;
}

# MacCready speed to fly, m/s. air: vertical air movement (+ lift), headwind: m/s (+ head)
var speed_to_fly = func(mc, air, headwind) {
	var best = polar_min_speed;
	var best_score = -1e9;
	for (var v = polar_min_speed; v <= polar_max_speed; v += 0.5) {
		var gs = v - headwind;
		var denom = sink_at(v) - air + math.max(0, mc);
		if (denom <= 0.01) continue;
		var score = gs / denom;
		if (score > best_score) { best_score = score; best = v; }
	}
	return best;
};

# ---- moving average over a window (s)
var Averager = {
	new: func(window_s) { return { parents: [Averager], w: window_s, t: [], v: [], sum: 0 }; },
	push: func(t, x) {
		append(me.t, t); append(me.v, x); me.sum += x;
		while (size(me.t) > 1 and t - me.t[0] > me.w) {
			me.sum -= me.v[0];
			me.t = subvec(me.t, 1);
			me.v = subvec(me.v, 1);
		}
	},
	get: func { return size(me.t) ? me.sum / size(me.t) : 0; },
	reset: func { me.t = []; me.v = []; me.sum = 0; },
};

# ---- state shared with the display
var f = {
	valid: 0, t: 0,
	ias: 0, tas: 0, vs: 0, alt: 0, hdg: 0, pitch: 0, roll: 0, gload: 1, maxG: 1, minG: 1,
	windDir: 0, windSpd: 0, oat: 15, volts: 0, power: 0,
	te: 0, teRaw: 0, teSound: 0, netto: 0, relative: 0, avgV: 0, avgN: 0, nettoSC: 0,
	stf: 0, stfDelta: 0, mode: "sc", circling: 0, turnRate: 0, lastThermal: nil,
	minV: 0, maxV: 0, flightTime: 0,
};
var flight_t = nil;
# thermal assistant: smoothed TE per 15 degree sector of the heading while circling
var bins = [];
for (var i = 0; i < 24; i += 1) append(bins, nil);
# FLARM-like list of other aircraft, refreshed twice a second: {id, dist, brg, dh, clock, vs}
var targets = [];
var warn_level = 0;
var flarm_quiet_until = 0;

var te_last_t = nil;
var te_last_v = 0;
var te_dvdt = 0;
var te_vs = nil;
var te_primed = 0;
var te_out = {raw: 0, te: 0, fast: 0, netto: 0, nettoInst: 0};
var lps = {};
var avg_vario = Averager.new(S.integrator);
var avg_netto = Averager.new(S.nettoTime);
var range_t = [];
var range_v = [];
var last_hdg = nil;
var last_hdg_t = 0;
var turn_rate = 0;
var circling_since = nil;
var straight_since = nil;
var thermal = nil;

var lp = func(key, x, t, tau) {
	if (!contains(lps, key)) { lps[key] = {v: x, t: t}; return x; }
	var p = lps[key];
	var dt = math.max(0, t - p.t);
	p.t = t;
	p.v += (x - p.v) * (1 - math.exp(-dt / math.max(0.05, tau)));
	return p.v;
};

var te_update = func(t, vs, v, sink_now, tau, comp, netto_tau) {
	var o = te_out;
	if (te_last_t != nil and t > te_last_t) {
		var dt = t - te_last_t;
		if (dt < 1) {
			var inst = (v - te_last_v) / dt;
			var a = 1 - math.exp(-dt / 0.25);
			te_dvdt += (inst - te_dvdt) * a;
			te_vs = te_vs == nil ? vs : te_vs + (vs - te_vs) * a;
			var raw = te_vs + comp * (v / G) * te_dvdt;
			o.raw = raw;
			o.nettoInst = raw + sink_now;
			if (!te_primed) {
				o.te = o.fast = raw;
				o.netto = raw + sink_now;
				te_primed = 1;
			} else {
				var k1 = 1 - math.exp(-dt / math.max(0.05, tau));
				var k2 = 1 - math.exp(-dt / 0.35);
				var k3 = 1 - math.exp(-dt / math.max(0.05, netto_tau));
				o.te += (raw - o.te) * k1;
				o.fast += (raw - o.fast) * k2;
				o.netto += (raw + sink_now - o.netto) * k3;
			}
		}
	}
	te_last_t = t;
	te_last_v = v;
};

var read_inputs = func {
	f.ias = (getprop("/velocities/airspeed-kt") or 0) * KT2MS;
	var tas_kt = getprop("/instrumentation/airspeed-indicator/true-speed-kt");
	f.tas = (tas_kt != nil ? tas_kt : (getprop("/velocities/airspeed-kt") or 0)) * KT2MS;
	f.vs = (getprop("/velocities/vertical-speed-fps") or 0) * 0.3048;
	f.alt = (getprop("/instrumentation/altimeter/indicated-altitude-ft") or getprop("/position/altitude-ft") or 0) * 0.3048;
	f.hdg = getprop("/orientation/heading-deg") or 0;
	f.pitch = getprop("/orientation/pitch-deg") or 0;
	f.roll = getprop("/orientation/roll-deg") or 0;
	f.windDir = getprop("/environment/wind-from-heading-deg") or 0;
	f.windSpd = (getprop("/environment/wind-speed-kt") or 0) * KT2MS;
	f.oat = getprop("/environment/temperature-degc") or 15;
	var g = getprop("/accelerations/pilot-gdamped");
	f.gload = g != nil ? g : 1;
	if (f.gload > f.maxG) f.maxG = f.gload;
	if (f.gload < f.minG) f.minG = f.gload;
	f.volts = getprop("/systems/electrical/outputs/ilec-sc7") or 0;
	f.power = f.volts >= 9;
};

var update_circling = func(t) {
	# turn rate from the heading, filtered
	if (last_hdg != nil and t > last_hdg_t) {
		var dt = t - last_hdg_t;
		if (dt < 1) {
			var inst = wrap180(f.hdg - last_hdg) / dt;
			turn_rate += (inst - turn_rate) * (1 - math.exp(-dt / 1.5));
		}
	}
	last_hdg = f.hdg;
	last_hdg_t = t;
	f.turnRate = turn_rate;

	if (math.abs(turn_rate) > 6) {
		straight_since = nil;
		if (circling_since == nil) circling_since = t;
	} else {
		circling_since = nil;
		if (straight_since == nil) straight_since = t;
	}
	var was = f.circling;
	f.circling = (circling_since != nil and t - circling_since > 8) ? 1 : 0;
	if (f.circling and !was) {
		for (var i = 0; i < 24; i += 1) bins[i] = nil;
		thermal = {t0: t, alt0: f.alt};
	}
	if (!f.circling and was) {
		if (straight_since != nil and t - straight_since > 6) {
			if (thermal != nil) {
				var dur = t - thermal.t0;
				if (dur > 25) {
					f.lastThermal = (f.alt - thermal.alt0) / dur;
					if (S.speechThermal) thermal_done(f.lastThermal);
				}
			}
			thermal = nil;
			for (var i = 0; i < 24; i += 1) bins[i] = nil;
		} else {
			f.circling = 1;       # hysteresis: still turning
		}
	}
	f.mode = f.circling ? "vario" : "sc";
	if (f.circling) {
		var b = int(wrap360(f.hdg) / 15) - 24 * int(wrap360(f.hdg) / 360);
		bins[b] = bins[b] == nil ? te_out.raw : bins[b] + (te_out.raw - bins[b]) * 0.5;
	}
};

var update = func {
	var t = getprop("/sim/time/elapsed-sec") or 0;
	read_inputs();
	var sink_now = sink_at(f.ias);
	te_update(t, f.vs, f.ias, sink_now, S.needleTau, S.teComp, S.nettoFilter);
	var o = te_out;
	f.teRaw = o.raw;
	f.te = o.te;
	f.netto = o.netto;
	f.relative = lp("rel", o.nettoInst - min_sink, t, S.relTau);
	f.nettoSC = lp("sc", o.nettoInst, t, S.scTau);
	if (f.ias < 15) {                          # no air mass information on the ground (and the polar is meaningless there)
		f.netto = 0;
		f.relative = 0;
		f.nettoSC = 0;
	}
	f.teSound = lp("snd", o.raw, t, S.soundTau);
	avg_vario.w = S.integrator;
	avg_netto.w = S.nettoTime;
	avg_vario.push(t, o.raw);
	avg_netto.push(t, f.netto);
	f.avgV = avg_vario.get();
	f.avgN = avg_netto.get();

	# min / max of the vario over the integrator time: the yellow bar
	append(range_t, t); append(range_v, o.te);
	while (size(range_t) > 1 and t - range_t[0] > S.integrator) {
		range_t = subvec(range_t, 1);
		range_v = subvec(range_v, 1);
	}
	var lo = range_v[0]; var hi = range_v[0];
	foreach (var x; range_v) {
		if (x < lo) lo = x;
		if (x > hi) hi = x;
	}
	f.minV = lo;
	f.maxV = hi;

	update_circling(t);

	var hw = f.windSpd * math.cos((f.windDir - f.hdg) * math.pi / 180);
	f.stf = speed_to_fly(S.mc, clamp(f.nettoSC, -3, 3), hw);
	f.stfDelta = f.ias - f.stf;
	if (flight_t != nil and f.tas > 15) f.flightTime += math.max(0, math.min(1, t - flight_t));
	flight_t = t;
	f.valid = 1;
	f.t = t;
};

var needle_value = func {
	var src = f.circling ? S.needleClimb : S.needleCruise;
	if (src == "netto") return f.netto;
	if (src == "relative") return f.relative;
	if (src == "sc") return clamp((f.stf - f.ias) * 3.6 / 10, -20, 20);
	return f.te;
};

# ####################################################################################
# FLARM-like target list (the stock flarm.nas keeps its list private)
# ####################################################################################
var clock_of = func(rel) {
	var c = int((rel + 15) / 30);
	return c == 0 ? 12 : (c > 12 ? c - 12 : c);
};

# Only gliders carry FLARM here (the same list of model names as in FG9070, js/data/ai-traffic.js)
var GLIDER_WORDS = [
	"glider", "sailplane", "ask21", "ask-21", "ask13", "ask-13", "ask23", "asw",
	"asw28", "discus", "ventus", "nimbus", "janus", "ls1", "duo-discus", "duodiscus",
	"club-astir", "std-cirrus", "ls3", "ls4", "ls6", "ls8", "ls10", "lak",
	"libelle", "cirrus", "mdm", "pik", "dg-", "dg1", "dg2", "dg5",
	"dg8", "dg3", "grob", "g102", "g103", "astir", "jantar", "zuni",
	"ka6", "ka7", "ka8", "ka13", "schleicher", "schempp", "rolladen", "blanik",
	"l-13", "l13", "slingsby", "kestrel", "skylark", "olympia", "k-21", "k21",
	"arcus", "quintus", "antares", "stemme", "diana", "jonker", "jsm", "sparrowhawk",
	"puchacz", "bocian", "zefir", "sb1", "sb10", "schweizer", "2-33", "1-26",
	"1-34", "1-35",
];
var is_glider = func(node) {
	var hay = "";
	foreach (var path; ["sim/model/path", "model-short", "type", "callsign"]) {
		var n = node.getNode(path);
		if (n != nil and n.getValue() != nil) hay ~= " " ~ n.getValue();
	}
	hay = string.lc(hay);
	foreach (var w; GLIDER_WORDS) if (find(w, hay) >= 0) return 1;
	return 0;
};

var update_targets = func {
	var out = [];
	var me_pos = geo.aircraft_position();
	var my_hdg = getprop("/orientation/heading-deg") or 0;
	foreach (var type; ["aircraft", "multiplayer"]) {
		var n = props.globals.getNode("/ai/models");
		if (n == nil) continue;
		foreach (var m; n.getChildren(type)) {
			var valid = m.getNode("valid");
			if (valid != nil and !valid.getValue()) continue;
			var la = m.getNode("position/latitude-deg");
			var lo = m.getNode("position/longitude-deg");
			var al = m.getNode("position/altitude-ft");
			if (la == nil or lo == nil or al == nil) continue;
			if (!is_glider(m)) continue;
			# parked and taxiing aircraft do not show up on FLARM
			var spd = m.getNode("velocities/true-airspeed-kt");
			if (spd != nil and spd.getValue() < 15) continue;
			var p = geo.Coord.new().set_latlon(la.getValue(), lo.getValue(), al.getValue() * 0.3048);
			var dist = me_pos.distance_to(p);
			if (dist > 4000 or dist < 5) continue;
			var rel = wrap360(me_pos.course_to(p) - my_hdg);
			var cs = m.getNode("callsign");
			append(out, {
				id: cs != nil ? cs.getValue() : "?",
				dist: dist, rel: rel,
				dh: p.alt() - me_pos.alt(),
				clock: clock_of(rel),
				vs: (m.getNode("velocities/vertical-speed-fps") != nil ? m.getNode("velocities/vertical-speed-fps").getValue() * 0.3048 : 0),
			});
		}
	}
	targets = sort(out, func(a, b) { return a.dist < b.dist ? -1 : (a.dist > b.dist ? 1 : 0); });
	var w = getprop("/instrumentation/flarm/warn") or 0;
	if (w > warn_level and size(targets)) {
		var now = getprop("/sim/time/elapsed-sec") or 0;
		if (S.flarmAlarms and S.speechFlarm and now > flarm_quiet_until) announce_traffic(targets[0]);
	}
	warn_level = w;
};

# ####################################################################################
# Vario audio (js/audio/vario-audio.js)
# ####################################################################################
var VARIO_SHAPES = {
	"Linear positive":       {kind: "positive"},
	"Linear negative":       {kind: "negative"},
	"Linear":                {kind: "linear"},
	"Digital positive":      {kind: "positive", digital: 1},
	"Digital negative":      {kind: "negative", digital: 1},
	"Linear positive only":  {kind: "positive-only"},
	"Digital positive only": {kind: "positive-only", digital: 1},
};
var SC_SHAPES = {
	"SC positive": {kind: "positive"},
	"SC negative": {kind: "negative"},
	"SC":          {kind: "linear"},
};
var DIGITAL_STEPS = 10;

# x: value / full scale (-1 ... +1)
var tone = func(shape, x, p) {
	x = clamp(x, -1, 1);
	var DEAD = 0.02;
	var xf = x;
	if (contains(shape, "digital") and shape.digital) xf = math.round(x * DIGITAL_STEPS) / DIGITAL_STEPS;
	var freq = xf >= 0 ? p.f0 + xf * (p.fPlus - p.f0) : p.f0 + (-xf) * (p.fMinus - p.f0);
	var positive = x > DEAD;
	var negative = x < -DEAD;
	if (!positive and !negative) return {silent: 1, freq: freq, beeping: 0, period: 1, duty: 0};

	var side = "continuous";
	if (shape.kind == "positive") side = positive ? "beep" : "continuous";
	elsif (shape.kind == "negative") side = negative ? "beep" : "continuous";
	elsif (shape.kind == "positive-only") side = positive ? "beep" : "silent";
	if (side == "silent") return {silent: 1, freq: freq, beeping: 0, period: 1, duty: 0};
	if (side == "continuous") return {silent: 0, freq: freq, beeping: 0, period: 1, duty: 1};
	# beep cadence: 1.0 s per beep just above zero, 0.16 s at full scale
	var m = math.abs(x);
	return {silent: 0, freq: freq, beeping: 1, period: 1.0 - 0.84 * math.pow(m, 0.8), duty: 0.5};
};

var resolve_audio = func {
	var range = S.varioRange;
	var p = {f0: S.freq0, fPlus: S.freqPlus, fMinus: S.freqMinus};
	var vario_shape = VARIO_SHAPES[S.audioMode];
	if (vario_shape == nil) vario_shape = VARIO_SHAPES["Linear positive"];
	var sc = (f.stf - f.ias) * 3.6 / 10;       # + = slower than speed to fly
	if (f.mode == "vario") return tone(vario_shape, f.teSound / range, p);

	# Speed-to-fly audio makes no sense on the ground or behind the tug: with the airspeed at zero it would
	# beep "fly faster" all the time. (My choice; FG9070 does not do this.)
	var hitch_open = getprop("/sim/hitches/aerotow/open");
	var towed = (hitch_open != nil and !hitch_open);
	if (f.ias < 20 or towed) return {silent: 1, freq: S.freq0, beeping: 0, period: 1, duty: 0};

	var value = 0;
	var shape = VARIO_SHAPES["Linear positive"];
	var mode = S.scAudioMode;
	var use_sc = 0;
	if (mode == "Vario") value = f.teSound;
	elsif (mode == "Netto") value = f.netto;
	elsif (mode == "Relative") value = f.relative;
	elsif (mode == "SC mixed") {
		if (f.relative > 0.1) value = f.relative;
		else { value = sc; shape = SC_SHAPES["SC positive"]; use_sc = 1; }
	} else {
		value = sc; use_sc = 1;
		shape = SC_SHAPES[mode];
		if (shape == nil) shape = SC_SHAPES["SC positive"];
	}
	if (use_sc and math.abs(sc) < S.scBand) return {silent: 1, freq: S.freq0, beeping: 0, period: 1, duty: 0};
	return tone(shape, value / range, p);
};

# envelope: gate for the beeps, smoothed like the 4 ms / 6 ms time constants of FG9070
var audio_gain = 0;
var audio_next = 0;
var audio_on_at = 0;
var audio_off_at = 0;
var audio_base_pitch = 1.0;
var audio_node_pitch = root.getNode("audio/pitch", 1);
var audio_node_volume = root.getNode("audio/volume", 1);
var audio_node_voice = root.getNode("audio/voice-volume", 1);
audio_node_pitch.setDoubleValue(1.0);
audio_node_volume.setDoubleValue(0);

var audio_tick = func {
	var t = getprop("/sim/time/elapsed-sec") or 0;
	var master = getprop("/instrumentation/ilec-sc7/volume");
	if (master == nil) master = 0.8;
	var vol = (f.power and f.valid) ? math.pow(clamp(master, 0, 1), 2) : 0;
	var target = 0;
	if (vol > 0) {
		var r = resolve_audio();
		audio_node_pitch.setDoubleValue(clamp(r.freq / 500, 0.2, 3.5));
		if (r.silent) { target = 0; audio_next = t; }
		elsif (!r.beeping) { target = 1; audio_next = t; }
		else {
			if (audio_next < t) audio_next = t;
			if (t >= audio_next) {
				audio_on_at = audio_next;
				audio_off_at = audio_on_at + r.period * r.duty;
				audio_next = audio_on_at + r.period;
			}
			target = (t >= audio_on_at and t < audio_off_at) ? 1 : 0;
		}
	}
	audio_gain += (target - audio_gain) * 0.6;
	audio_node_volume.setDoubleValue(vol * audio_gain);
	audio_node_voice.setDoubleValue(f.power ? math.pow(clamp(master, 0.1, 1), 1.0) : 0);
};

# ####################################################################################
# Voice: pre-rendered clips of the system speech voice, played one after the other.
# Sounds/lxv8-voice/index.nas (generated) defines voice_clips = {name: duration_s}.
# ####################################################################################
var voice_dir = nil;
var voice_busy_until = 0;
var voice_queue = [];
var voice_clips = {};
var voice_numbers = {};
var voice_text_node = root.getNode("voice/text", 1);
var voice_seq_node = root.getNode("voice/seq", 1);
var voice_helper_node = root.getNode("voice/helper-time", 1);
var voice_helper_time = nil;

# tools/lxv8-speak.py (macOS) speaks the text with `say`; it writes the wall clock time to voice/helper-time while it runs
var helper_alive = func {
	var h = voice_helper_node.getValue();
	return h != nil and math.abs(systime() - num(h)) < 3;
};

var voice_clip_node = root.getNode("voice/clip", 1);
voice_clip_node.setIntValue(0);

# Every clip is a one-shot sound in Sounds/ask21-sound.xml that plays when /instrumentation/lxv8/voice/clip is its number.
var voice_play_next = func {
	if (!size(voice_queue)) { voice_clip_node.setIntValue(0); return; }
	var name = voice_queue[0];
	voice_queue = subvec(voice_queue, 1);
	var dur = contains(voice_clips, name) ? voice_clips[name] : 0.6;
	voice_clip_node.setIntValue(contains(voice_numbers, name) ? voice_numbers[name] : 0);
	voice_busy_until = (getprop("/sim/time/elapsed-sec") or 0) + dur;
	settimer(func {
		voice_clip_node.setIntValue(0);              # back to 0 so that the next clip (or the same one again) starts
		settimer(voice_play_next, 0.03);
	}, math.max(0.05, dur - 0.04));                  # the next clip starts while the end of this one is still playing
};

# text: the sentence for the speech helper; names: list of clip names without ".wav" for the prerecorded fallback.
# Ignored while another message is playing.
var speak = func(text, names) {
	if (!S.speech or !f.power) return;
	if (helper_alive()) {
		voice_text_node.setValue(text);
		voice_seq_node.setIntValue((voice_seq_node.getValue() or 0) + 1);
		return;
	}
	if (voice_dir == nil) return;
	if (size(voice_queue) or (getprop("/sim/time/elapsed-sec") or 0) < voice_busy_until) return;
	voice_queue = names;
	voice_play_next();
};

var digit_names = ["zero", "one", "two", "three", "four", "five", "six", "seven", "eight", "nine"];

var thermal_done = func(avg) {
	var tenths = int(math.round(math.abs(avg) * 10));
	var names = ["thermal-average"];
	if (avg < 0) append(names, "minus");
	append(names, digit_names[int(tenths / 10) < 10 ? int(tenths / 10) : 9]);
	append(names, "point");
	append(names, digit_names[tenths - 10 * int(tenths / 10)]);
	speak(sprintf("Thermal average %s%d.%d", avg < 0 ? "minus " : "", int(tenths / 10), tenths - 10 * int(tenths / 10)), names);
};

var announce_traffic = func(tg) {
	var names = ["traffic", "clock-" ~ tg.clock];
	append(names, math.abs(tg.dh) < 30 ? "level" : (tg.dh > 0 ? "high" : "low"));
	var d = tg.dist;
	var hundreds = d < 1000 ? int(math.round(d / 100)) * 100 : int(math.round(d / 500)) * 500;
	if (hundreds < 100) hundreds = 100;
	if (hundreds > 4000) hundreds = 4000;
	append(names, "meters-" ~ hundreds);
	var vert = math.abs(tg.dh) < 30 ? "level" : (tg.dh > 0 ? "high" : "low");
	speak(sprintf("Traffic %d o'clock, %s, %d meters", tg.clock, vert, hundreds), names);
};

# ####################################################################################
# Timers and start
# ####################################################################################
var start_core = func {
	load_settings();
	# timers are kept in globals so that a reload of this file can stop the old ones
	if (contains(globals, "lxv8_timers")) foreach (var tm; globals.lxv8_timers) tm.stop();
	globals.lxv8_timers = [];
	voice_dir = getprop("/sim/aircraft-dir") ~ "/Sounds/lxv8-voice";
	if (io.stat(voice_dir ~ "/index.nas") != nil) io.load_nasal(voice_dir ~ "/index.nas", "lxv8");
	var t1 = maketimer(0.05, update);
	var t2 = maketimer(0.02, audio_tick);
	var t3 = maketimer(0.5, update_targets);
	foreach (var tm; [t1, t2, t3]) { tm.start(); append(globals.lxv8_timers, tm); }
};
