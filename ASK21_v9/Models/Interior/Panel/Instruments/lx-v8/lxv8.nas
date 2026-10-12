# ####################################################################################
# LXNAV V8 vario indicator: the screen (Canvas), the three buttons and the setup menu
#
# Pages (middle button) and settings follow the LX90xx/LX80xx user manual, chapter 7.1.12.4:
# numeric (navboxes), AHRS, wind, FLARM radar, thermal assistant. Long press of the middle
# button opens the setup menu; every setting is kept in /instrumentation/lxv8/settings and
# saved with the aircraft data.
# Data and audio: lxv8-core.nas.
#
# Screen layout: the canvas covers a square of 54 mm on the instrument (512 px), the round
# dial has its centre in the middle of it. The mesh only shows the part left of the push buttons.
# ####################################################################################

var FONT = "LiberationFonts/LiberationSans-Bold.ttf";
var CX = 256;
var CY = 256;
var R_OUT = 250;
var R_IN = 175;
var D2R = math.pi / 180;

# colours
var WHITE = [1, 1, 1];
var GREY = [0.62, 0.62, 0.65];
var DARK = [0.12, 0.12, 0.14];
var RED = [0.95, 0.2, 0.12];
var ORANGE = [0.96, 0.62, 0.08];
var BLUE = [0.2, 0.4, 1.0];
var GREEN = [0.1, 0.85, 0.2];
var YELLOW = [1.0, 0.85, 0.1];
var THEMES = {
	white: {ring: [0.97, 0.97, 0.97], text: [0.12, 0.12, 0.14]},
	grey:  {ring: [0.66, 0.68, 0.72], text: [0.05, 0.05, 0.08]},
	black: {ring: [0.10, 0.10, 0.11], text: [1, 1, 1]},
};

var PAGES = ["vario", "ahrs", "wind", "flarm", "thermal", "gmeter"];
var page = 0;
var saved_page = nil;
var prev_circling = 0;
var in_setup = 0;
var radar_zoom = 1;                 # index into RADAR_RANGES
var RADAR_RANGES = [500, 1000, 2000, 4000];
var toast_until = 0;
var press_t = {up: 0, mid: 0, dn: 0};
var boot_until = 0;
var was_powered = 0;
var last_view = "";

# ---- value -> position on the dial
var SCALE_A = [0, 34, 64, 90, 112, 132];      # degrees for 0 ... 5 on the non-linear scale (picture in the manual)
var dial_angle = func(v) {
	# math angle, counter-clockwise from the right: 180 = left (zero), up for lift, down for sink
	var x = clamp(math.abs(v) * 5 / S.varioRange, 0, 5);
	var a = 0;
	if (S.scaleType == "linear") a = x * 26.4;
	else {
		var i = int(x);
		if (i >= 5) i = 4;
		a = SCALE_A[i] + (SCALE_A[i + 1] - SCALE_A[i]) * (x - i);
	}
	return v >= 0 ? 180 - a : 180 + a;
};

# ---- units
var fvario = func(x) {
	if (S.uVario == "kt") return x * 1.94384;
	if (S.uVario == "ft/min") return x * 196.85;
	return x;
};
var vfmt = func(x) {
	var v = fvario(x);
	if (S.uVario == "ft/min") return sprintf("%+d", math.round(v));
	if (math.abs(v) < 0.05) v = 0;                  # no "-0.0"
	return sprintf("%+.1f", v);
};
var falt = func(x) { return S.uAlt == "ft" ? x * 3.28084 : x; };
var fspeed = func(x) {
	if (S.uSpeed == "kt") return x * 1.94384;
	if (S.uSpeed == "mph") return x * 2.23694;
	return x * 3.6;
};
var dial_label = func(v) {
	if (S.uVario == "kt") return sprintf("%d", math.round(v * 1.94384));
	if (S.uVario == "ft/min") return sprintf("%d", math.round(v * 196.85 / 100) * 100);
	return sprintf("%g", v);
};

# ---- canvas helpers
var text = func(parent, str, x, y, size, color, align) {
	var t = parent.createChild("text");
	t.setText(str);
	t.setFont(FONT);
	t.setFontSize(size, 1.0);
	t.setAlignment(align);
	t.setTranslation(x, y);
	t.setColor(color[0], color[1], color[2]);
	return t;
};

var poly = func(parent, pts, fill, stroke, width) {
	var p = parent.createChild("path");
	p.moveTo(pts[0][0], pts[0][1]);
	for (var i = 1; i < size(pts); i += 1) p.lineTo(pts[i][0], pts[i][1]);
	p.close();
	if (fill != nil) p.setColorFill(fill[0], fill[1], fill[2]);
	if (stroke != nil) { p.setColor(stroke[0], stroke[1], stroke[2]); p.setStrokeLineWidth(width); }
	return p;
};

var line = func(parent, x0, y0, x1, y1, color, width) {
	var p = parent.createChild("path");
	p.moveTo(x0, y0).lineTo(x1, y1);
	p.setColor(color[0], color[1], color[2]);
	p.setStrokeLineWidth(width);
	return p;
};

# polyline along a circle (angles are math angles in degrees, counter-clockwise, y up)
var arc = func(parent, cx, cy, r, a0, a1, color, width) {
	var p = parent.createChild("path");
	var n = math.max(2, int(math.abs(a1 - a0) / 4));
	for (var i = 0; i <= n; i += 1) {
		var a = (a0 + (a1 - a0) * i / n) * D2R;
		var x = cx + r * math.cos(a);
		var y = cy - r * math.sin(a);
		if (i == 0) p.moveTo(x, y); else p.lineTo(x, y);
	}
	p.setColor(color[0], color[1], color[2]);
	p.setStrokeLineWidth(width);
	return p;
};

var disc = func(parent, cx, cy, r, fill, stroke, width) {
	var p = parent.createChild("path");
	for (var i = 0; i < 24; i += 1) {
		var a = i * 15 * D2R;
		if (i == 0) p.moveTo(cx + r, cy); else p.lineTo(cx + r * math.cos(a), cy - r * math.sin(a));
	}
	p.close();
	if (fill != nil) p.setColorFill(fill[0], fill[1], fill[2]);
	if (stroke != nil) { p.setColor(stroke[0], stroke[1], stroke[2]); p.setStrokeLineWidth(width); }
	return p;
};

# a group that is moved to (x, y); content is rotated in the group below it
var placed = func(parent, x, y) {
	var g = parent.createChild("group");
	g.setTranslation(x, y);
	return g;
};

# ####################################################################################
# build the screen
# ####################################################################################
var cv = nil;
var el = {};

var build = func {
	close_window();                     # the window shows this canvas: deleting the canvas under it hangs the simulator
	if (contains(globals, "lxv8_canvas") and globals.lxv8_canvas != nil) {
		globals.lxv8_canvas.del();
		globals.lxv8_canvas = nil;
	}
	cv = canvas.new({name: "LXV8", size: [512, 512], view: [512, 512], mipmapping: 1});
	cv.addPlacement({node: "V8.screen"});
	cv.setColorBackground(0, 0, 0, 1);
	globals.lxv8_canvas = cv;
	var top = cv.createGroup();

	el.pages = {};
	foreach (var nm; ["vario", "ahrs", "wind", "flarm", "thermal", "gmeter", "setup", "boot"]) {
		el.pages[nm] = top.createChild("group");
		el.pages[nm].hide();
	}
	build_vario(el.pages.vario);
	build_ahrs(el.pages.ahrs);
	build_wind(el.pages.wind);
	build_flarm(el.pages.flarm);
	build_thermal(el.pages.thermal);
	build_gmeter(el.pages.gmeter);
	build_setup(el.pages.setup);
	build_boot(el.pages.boot);

	# everything outside the dial ring is black
	el.mask = arc(top, CX, CY, 340, 0, 360, [0, 0, 0], 180);

	el.ring = top.createChild("group");
	build_ring(el.ring);
	el.dyn = top.createChild("group");
	build_dyn(el.dyn);
	el.toast_bg = poly(top, [[CX - 158, 400], [CX + 142, 400], [CX + 142, 446], [CX - 158, 446]], [0, 0, 0], WHITE, 2);
	el.toast = text(top, "", CX - 8, 423, 34, WHITE, "center-center");
	el.toast_bg.hide();
	el.toast.hide();
};

var build_ring = func(g) {
	var th = THEMES[S.theme];
	if (th == nil) th = THEMES.white;
	arc(g, CX, CY, (R_OUT + R_IN) / 2, 0, 360, th.ring, R_OUT - R_IN);
	for (var sg = -1; sg <= 1; sg += 2) {
		for (var h = 0; h <= 10; h += 1) {
			if (h == 0 and sg == 1) continue;
			var v = h / 2 * S.varioRange / 5;
			var a = dial_angle(sg * v) * D2R;
			var major = (math.mod(h, 2) == 0);
			var rin = major ? 222 : 234;
			line(g, CX + rin * math.cos(a), CY - rin * math.sin(a), CX + 247 * math.cos(a), CY - 247 * math.sin(a), th.text, major ? 4 : 2);
			if (major and h > 0) {
				text(g, dial_label(v), CX + 201 * math.cos(a), CY - 201 * math.sin(a), 38, th.text, "center-center");
			}
		}
	}
	var a0 = dial_angle(0) * D2R;
	text(g, "0", CX + 201 * math.cos(a0), CY - 201 * math.sin(a0), 38, th.text, "center-center");
	el.unit = text(g, S.uVario, CX + 118, CY + 205, 28, th.text, "center-center");
	arc(g, CX, CY, R_IN - 2, 0, 360, [0.85, 0.65, 0.1], 3);
};

var build_dyn = func(g) {
	el.bar_g = g.createChild("group");
	# MacCready (blue triangle), last thermal (green T), average (red diamond)
	el.mc = placed(g, CX, CY).createChild("group");
	poly(el.mc, [[R_IN + 2, 0], [R_IN + 36, -14], [R_IN + 36, 14]], BLUE, [1, 1, 1], 1);
	el.th = placed(g, CX, CY).createChild("group");
	line(el.th, R_IN + 4, 0, R_IN + 36, 0, GREEN, 6);
	line(el.th, R_IN + 4, -15, R_IN + 4, 15, GREEN, 6);
	el.th.hide();
	el.avg = placed(g, CX, CY).createChild("group");
	poly(el.avg, [[R_IN + 4, 0], [R_IN + 21, -12], [R_IN + 38, 0], [R_IN + 21, 12]], RED, [1, 1, 1], 1);
	# needle
	var hw = {thin: 6, medium: 11, thick: 17}[S.needleStyle];
	if (hw == nil) hw = 11;
	el.needle = placed(g, CX, CY).createChild("group");
	poly(el.needle, [[R_IN - 2, -hw / 2], [R_OUT - 4, -hw], [R_OUT - 4, hw], [R_IN - 2, hw / 2]], RED, [0.5, 0.1, 0.05], 2);
	el.title = text(g, "", CX + 138, CY - 196, 28, THEMES[S.theme].text, "center-center");
};

# ---- numeric page: the navboxes and the status line
var NB_OPTS = ["avgv", "netto", "alt", "tas", "ias", "oat", "volts", "thermal", "avgnetto", "gload", "fl", "time", "ftime", "stf"];
var NB_NAMES = {
	avgv: "Average vario", netto: "Netto", alt: "Altitude", tas: "True airspeed", ias: "Airspeed",
	oat: "Temperature", volts: "Voltage", thermal: "Thermal average", avgnetto: "Average netto",
	gload: "G-load", fl: "Flight level", time: "Time", ftime: "Flight time", stf: "Speed to fly",
};

# [label, value text, unit] of a navbox
var nb_info = func(key) {
	if (key == "avgv") return [NB_NAMES[key], vfmt(f.avgV), S.uVario];
	if (key == "netto") return [NB_NAMES[key], vfmt(f.netto), S.uVario];
	if (key == "avgnetto") return [NB_NAMES[key], vfmt(f.avgN), S.uVario];
	if (key == "thermal") return [NB_NAMES[key], f.lastThermal == nil ? "---" : vfmt(f.lastThermal), S.uVario];
	if (key == "alt") return [NB_NAMES[key], sprintf("%d", math.round(falt(f.alt))), S.uAlt];
	if (key == "tas") return [NB_NAMES[key], sprintf("%d", math.round(fspeed(f.tas))), S.uSpeed];
	if (key == "ias") return [NB_NAMES[key], sprintf("%d", math.round(fspeed(f.ias))), S.uSpeed];
	if (key == "stf") return [NB_NAMES[key], sprintf("%d", math.round(fspeed(f.stf))), S.uSpeed];
	if (key == "oat") return [NB_NAMES[key], sprintf("%d", math.round(f.oat)), "C"];
	if (key == "volts") return [NB_NAMES[key], sprintf("%.1f", f.volts), "V"];
	if (key == "gload") return [NB_NAMES[key], sprintf("%.2f", f.gload), "g"];
	if (key == "fl") return [NB_NAMES[key], sprintf("FL%03d", math.round(f.alt * 3.28084 / 100)), ""];
	if (key == "time") {
		var s = int(getprop("/sim/time/local-day-seconds") or 0);
		return [NB_NAMES[key], sprintf("%02d:%02d", int(s / 3600), int(math.mod(s, 3600) / 60)), ""];
	}
	if (key == "ftime") {
		var m = int(f.flightTime / 60);
		return [NB_NAMES[key], sprintf("%d:%02d", int(m / 60), math.mod(m, 60)), "h"];
	}
	return ["", "", ""];
};

var build_vario = func(g) {
	var n = clamp(S.nbCount, 1, 4);
	var upper = int((n + 1) / 2);               # boxes above the status line
	var lower = n - upper;
	var nbkeys = [S.nb1, S.nb2, S.nb3, S.nb4];
	el.nb = [];
	var place = func(count, top, bottom, first) {
		var slot = (bottom - top) / count;
		for (var i = 0; i < count; i += 1) {
			var t0 = top + slot * i;
			var big = slot >= 100;
			var vsize = big ? 78 : 50;
			var lsize = big ? 26 : 22;
			var ly = t0 + (big ? 22 : 14);
			var vy = t0 + (big ? 84 : 48);
			var box = {key: nbkeys[first + i], vy: vy};
			box.label = text(g, "", CX - 8, ly, lsize, GREY, "center-center");
			box.value = text(g, "", CX - 8, vy, vsize, WHITE, "center-center");
			box.unit = text(g, "", CX, vy + 4, big ? 30 : 22, WHITE, "left-center");
			box.vsize = vsize;
			append(el.nb, box);
		}
	};
	place(upper, 92, 236, 0);
	if (lower > 0) place(lower, 292, 436, upper);
	# status line: GPS, FLARM, climb/cruise, battery
	var row = placed(g, CX - 8, 264);
	el.gps = disc(row, -92, 0, 11, GREEN, nil, 0);
	el.flarm = row.createChild("group");
	el.flarm_arcs = [arc(el.flarm, -48, 10, 12, 40, 140, GREY, 4),
	                 arc(el.flarm, -48, 10, 20, 40, 140, GREY, 4),
	                 arc(el.flarm, -48, 10, 28, 40, 140, GREY, 4)];
	if (!S.flarmIcon) el.flarm.hide();
	el.mode_climb = row.createChild("group");
	arc(el.mode_climb, 6, 0, 13, 20, 330, WHITE, 4);
	el.mode_cruise = row.createChild("group");
	line(el.mode_cruise, -10, 0, 14, 0, WHITE, 5);
	poly(el.mode_cruise, [[14, -10], [27, 0], [14, 10]], WHITE, nil, 0);
	var bat = row.createChild("group");
	poly(bat, [[52, -11], [84, -11], [84, 11], [52, 11]], nil, WHITE, 3);
	el.bat_fill = poly(bat, [[56, -7], [80, -7], [80, 7], [56, 7]], GREEN, nil, 0);
	# speed to fly chevrons beside the display
	el.stf_up = [];
	el.stf_dn = [];
	for (var i = 0; i < 4; i += 1) {
		var c = placed(g, 392, 264 - 26 - 22 * i);
		poly(c, [[-11, 6], [0, -5], [11, 6], [11, 11], [0, 0], [-11, 11]], RED, nil, 0);
		c.hide();
		append(el.stf_up, c);
		var d = placed(g, 392, 264 + 26 + 22 * i);
		poly(d, [[-11, -6], [0, 5], [11, -6], [11, -11], [0, 0], [-11, -11]], [0.3, 0.5, 1], nil, 0);
		d.hide();
		append(el.stf_dn, d);
	}
};

var rebuild_vario = func {
	el.pages.vario.removeAllChildren();
	build_vario(el.pages.vario);
	last_view = "";
};

# ---- AHRS page
var PXDEG = 5.5;
var build_ahrs = func(g) {
	el.hor = placed(g, CX, CY).createChild("group");
	el.pitch_g = el.hor.createChild("group");
	poly(el.pitch_g, [[-500, -900], [500, -900], [500, 0], [-500, 0]], [0.28, 0.45, 0.95], nil, 0);
	poly(el.pitch_g, [[-500, 0], [500, 0], [500, 900], [-500, 900]], [0.5, 0.32, 0.1], nil, 0);
	line(el.pitch_g, -500, 0, 500, 0, WHITE, 4);
	for (var p = -30; p <= 30; p += 5) {
		if (p == 0) continue;
		var w = (math.mod(p, 10) == 0) ? 80 : 40;
		line(el.pitch_g, -w, -p * PXDEG, w, -p * PXDEG, WHITE, 3);
		if (math.mod(p, 10) == 0) {
			text(el.pitch_g, sprintf("%d", math.abs(p)), -w - 26, -p * PXDEG, 26, WHITE, "center-center");
			text(el.pitch_g, sprintf("%d", math.abs(p)), w + 26, -p * PXDEG, 26, WHITE, "center-center");
		}
	}
	var ac = placed(g, CX, CY);
	poly(ac, [[-120, 0], [-38, 0], [0, 14], [38, 0], [120, 0], [38, 12], [0, 32], [-38, 12]], YELLOW, [0.2, 0.2, 0.2], 2);
};

var update_ahrs = func {
	el.hor.setRotation(-f.roll * D2R);
	el.pitch_g.setTranslation(0, f.pitch * PXDEG);
};

# ---- wind page
var wind_x = 0;
var wind_y = 0;
var wind_init = 0;
var build_wind = func(g) {
	el.w_avg = placed(g, CX, CY).createChild("group");
	line(el.w_avg, 0, 50, 0, -70, [0.55, 0.55, 0.58], 18);
	poly(el.w_avg, [[-34, -62], [0, -122], [34, -62]], [0.55, 0.55, 0.58], nil, 0);
	el.w_live = placed(g, CX, CY).createChild("group");
	line(el.w_live, 0, 50, 0, -70, [0.45, 0.45, 1.0], 18);
	poly(el.w_live, [[-34, -62], [0, -122], [34, -62]], [0.45, 0.45, 1.0], nil, 0);
	el.w_txt = text(g, "", CX - 8, 392, 40, WHITE, "center-center");
};

var update_wind = func {
	# arrows point where the air goes, relative to the nose
	el.w_live.setRotation((f.windDir + 180 - f.hdg) * D2R);
	var wx = f.windSpd * math.sin((f.windDir + 180) * D2R);
	var wy = f.windSpd * math.cos((f.windDir + 180) * D2R);
	if (!wind_init) { wind_x = wx; wind_y = wy; wind_init = 1; }
	wind_x += (wx - wind_x) * 0.05 / 180;           # 3 minute average
	wind_y += (wy - wind_y) * 0.05 / 180;
	el.w_avg.setRotation((math.atan2(wind_x, wind_y) / D2R - f.hdg) * D2R);
	el.w_txt.setText(sprintf("%03d/%d", int(wrap360(f.windDir)), math.round(fspeed(f.windSpd))));
};

# ---- FLARM radar page
var build_flarm = func(g) {
	arc(g, CX, CY, 70, 0, 360, [0.35, 0.35, 0.35], 3);
	arc(g, CX, CY, 140, 0, 360, [0.35, 0.35, 0.35], 3);
	text(g, "N", CX - 8, CY - 160, 28, WHITE, "center-center");
	var own = placed(g, CX, CY);
	line(own, 0, -16, 0, 16, WHITE, 4);
	line(own, -16, 0, 16, 0, WHITE, 4);
	el.fl_targets = g.createChild("group");
	el.fl_id = text(g, "", CX - 130, CY - 120, 26, [0.4, 0.7, 1.0], "left-center");
	el.fl_n = text(g, "", CX + 70, CY - 120, 28, YELLOW, "center-center");
	el.fl_dist = text(g, "", CX + 130, CY + 150, 26, GREY, "right-center");
	el.fl_range = text(g, "", CX - 130, CY + 150, 26, GREY, "left-center");
	el.fl_vario = text(g, "", CX - 130, CY + 112, 26, WHITE, "left-center");
	el.fl_alt = text(g, "", CX + 130, CY + 112, 26, WHITE, "right-center");
};

var update_flarm = func {
	var range = RADAR_RANGES[radar_zoom];
	el.fl_targets.removeAllChildren();
	el.fl_n.setText(sprintf("TX/%d", size(targets)));
	el.fl_range.setText(sprintf("%.1fkm", range / 1000));
	var first = 1;
	foreach (var tg; targets) {
		var rr = clamp(tg.dist / range, 0, 1) * 150;
		var a = tg.rel * D2R;
		var col = math.abs(tg.dh) < 50 ? GREEN : (tg.dh > 0 ? RED : BLUE);
		var p = placed(el.fl_targets, CX + rr * math.sin(a), CY - rr * math.cos(a));
		poly(p, [[0, -12], [9, 9], [0, 4], [-9, 9]], col, nil, 0);
		if (first) {
			first = 0;
			el.fl_id.setText(tg.id);
			el.fl_dist.setText(sprintf("%.1fkm", tg.dist / 1000));
			el.fl_vario.setText(vfmt(tg.vs));
			el.fl_alt.setText(sprintf("%+d", int(falt(tg.dh))));
		}
	}
	if (first) {
		el.fl_id.setText("");
		el.fl_dist.setText("");
		el.fl_vario.setText("");
		el.fl_alt.setText("");
	}
};

# ---- G-meter page (replaces the separate G-meter of the panel): the load, its maximum and minimum since the
#      last reset; the top button resets them
var G_MIN = -4;
var G_MAX = 8;
var G_BAR_W = 270;
var gx = func(g) { return CX - G_BAR_W / 2 + (clamp(g, G_MIN, G_MAX) - G_MIN) / (G_MAX - G_MIN) * G_BAR_W; };
var build_gmeter = func(g) {
	var y = CY + 70;
	line(g, CX - G_BAR_W / 2, y, CX + G_BAR_W / 2, y, [0.6, 0.6, 0.62], 6);
	for (var v = G_MIN; v <= G_MAX; v += 2) {
		line(g, gx(v), y - 14, gx(v), y + 14, WHITE, v == 0 ? 5 : 3);
		text(g, sprintf("%d", v), gx(v), y + 38, 26, WHITE, "center-center");
	}
	el.g_range = g.createChild("group");
	el.g_now = g.createChild("group");
	poly(el.g_now, [[0, y - 4], [-13, y - 30], [13, y - 30]], YELLOW, [0.2, 0.2, 0.2], 2);
	el.g_val = text(g, "", CX, CY - 40, 78, WHITE, "center-center");
	el.g_max = text(g, "", CX - 70, CY + 6, 32, GREEN, "center-center");
	el.g_min = text(g, "", CX + 70, CY + 6, 32, [0.45, 0.75, 1.0], "center-center");
};

var update_gmeter = func {
	el.g_val.setText(sprintf("%+.1f", f.gload));
	el.g_max.setText(sprintf("%+.1f", f.maxG));
	el.g_min.setText(sprintf("%+.1f", f.minG));
	el.g_now.setTranslation(gx(f.gload), 0);
	el.g_range.removeAllChildren();
	var y = CY + 70;
	line(el.g_range, gx(f.minG), y, gx(f.maxG), y, ORANGE_G, 12);
};
var ORANGE_G = [0.95, 0.55, 0.1];

# ---- thermal assistant page
var build_thermal = func(g) {
	el.th_dots = g.createChild("group");
	el.th_glider = placed(g, CX, CY);
	poly(el.th_glider, [[-18, 0], [0, -26], [18, 0], [0, -5]], WHITE, nil, 0);
	el.th_avg = text(g, "", CX - 8, CY - 52, 44, WHITE, "center-center");
	el.th_netto = text(g, "", CX - 8, CY + 52, 44, WHITE, "center-center");
};

var update_thermal = func {
	el.th_dots.removeAllChildren();
	var sum = 0;
	var n = 0;
	for (var i = 0; i < 24; i += 1) if (bins[i] != nil) { sum += bins[i]; n += 1; }
	var ref = n ? sum / n : 0;                      # auto span: the average of the circle
	if (S.thermalMode == "mc") ref = S.mc;
	elsif (S.thermalMode == "avg") ref = f.avgV;
	var best = -99;
	var best_i = -1;
	for (var i = 0; i < 24; i += 1) if (bins[i] != nil and bins[i] > best) { best = bins[i]; best_i = i; }
	for (var i = 0; i < 24; i += 1) {
		var a = (i * 15 + 7.5) * D2R;
		var x = CX + 120 * math.sin(a);
		var y = CY - 120 * math.cos(a);
		var col = [0.3, 0.3, 0.6];
		var r = 4;
		if (bins[i] != nil) {
			var d = bins[i] - ref;
			if (d < -0.3) { col = [0.3, 0.45, 1.0]; r = 6; }
			elsif (d > 0.3) { col = RED; r = 12; }
			else { col = YELLOW; r = 8; }
			if (i == best_i and n > 3) { col = WHITE; r = 14; }
		}
		disc(el.th_dots, x, y, r, col, nil, 0);
	}
	el.th_glider.setRotation(f.hdg * D2R);
	el.th_avg.setText(vfmt(f.avgV));
	el.th_netto.setText(vfmt(f.netto));
};

# ---- boot screen
var build_boot = func(g) {
	text(g, "LXNAV", CX - 8, CY - 36, 84, WHITE, "center-center");
	text(g, "V8", CX - 8, CY + 44, 76, ORANGE, "center-center");
};

# ####################################################################################
# setup menu: every setting of the instrument
# ####################################################################################
var VARIO_MODES = ["Linear positive", "Linear negative", "Linear", "Digital positive", "Digital negative", "Linear positive only", "Digital positive only"];
var SC_MODES = ["SC positive", "SC negative", "SC", "SC mixed", "Relative", "Netto", "Vario"];
var NEEDLES = ["vario", "netto", "relative", "sc"];
var NEEDLE_NAMES = {vario: "Vario", netto: "Netto", relative: "Relative", sc: "Speed to fly"};

var num_item = func(key, label, lo, hi, step, fmt) { return {t: "num", key: key, label: label, min: lo, max: hi, step: step, fmt: fmt}; };
var choice_item = func(key, label, opts, names) { return {t: "choice", key: key, label: label, opts: opts, names: names}; };
var bool_item = func(key, label) { return {t: "bool", key: key, label: label}; };
var sub_item = func(label, to) { return {t: "sub", label: label, to: to}; };
var act_item = func(label, act) { return {t: "act", label: label, act: act}; };

var MENUS = {
	main: {title: "Setup", items: [
		sub_item("Indicator", "indicator"),
		sub_item("Vario parameters", "vario"),
		sub_item("Audio", "audio"),
		sub_item("Voice", "voice"),
		sub_item("FLARM", "flarm"),
		sub_item("Thermal assistant", "thermal"),
		sub_item("Display", "display"),
		sub_item("Units", "units"),
		act_item("Reset G", func { f.maxG = 1; f.minG = 1; toast("G reset"); }),
		act_item("About", func { toast("LXNAV V8 sim"); }),
		act_item("Exit", func { leave_setup(); }),
	]},
	indicator: {title: "Indicator", items: [
		num_item("nbCount", "Navboxes", 1, 4, 1, "%d"),
		choice_item("nb1", "Navbox 1", NB_OPTS, NB_NAMES),
		choice_item("nb2", "Navbox 2", NB_OPTS, NB_NAMES),
		choice_item("nb3", "Navbox 3", NB_OPTS, NB_NAMES),
		choice_item("nb4", "Navbox 4", NB_OPTS, NB_NAMES),
		choice_item("needleClimb", "Needle in climb", NEEDLES, NEEDLE_NAMES),
		choice_item("needleCruise", "Needle in cruise", NEEDLES, NEEDLE_NAMES),
		choice_item("needleStyle", "Needle style", ["thin", "medium", "thick"], nil),
		choice_item("scaleType", "Vario scale", ["non-linear", "linear"], nil),
		choice_item("theme", "Colour style", ["white", "grey", "black"], nil),
		choice_item("yellowBar", "Yellow bar", ["minmax", "none"], {minmax: "Min / Max vario", none: "None"}),
		choice_item("redDiamond", "Red diamond", ["avg", "netto", "none"], {avg: "Average vario", netto: "Netto", none: "None"}),
		bool_item("mcDiamond", "MacCready arrow"),
		choice_item("sideBar", "Side bar", ["stf", "none"], {stf: "Speed to fly", none: "None"}),
		bool_item("flarmIcon", "FLARM symbol"),
	]},
	vario: {title: "Vario parameters", items: [
		choice_item("varioRange", "Vario range", [2.5, 5, 10], nil),
		num_item("needleTau", "Needle filter", 0.2, 5, 0.1, "%.1f s"),
		num_item("integrator", "Integrator time", 1, 60, 1, "%d s"),
		num_item("nettoFilter", "Netto filter", 0.2, 5, 0.1, "%.1f s"),
		num_item("soundTau", "Sound filter", 0.2, 5, 0.1, "%.1f s"),
		num_item("mc", "MacCready", 0, 5, 0.1, "%.1f"),
	]},
	audio: {title: "Audio", items: [
		choice_item("audioMode", "Vario audio mode", VARIO_MODES, nil),
		choice_item("scAudioMode", "SC audio mode", SC_MODES, nil),
		num_item("freq0", "Freq at 0%", 100, 2000, 50, "%d Hz"),
		num_item("freqPlus", "Freq at +100%", 500, 3000, 50, "%d Hz"),
		num_item("freqMinus", "Freq at -100%", 80, 1000, 20, "%d Hz"),
		num_item("scBand", "SC tab", 0, 3, 0.1, "%.1f m/s"),
		num_item("volume", "Volume", 0, 100, 10, "%d %%"),
	]},
	voice: {title: "Voice", items: [
		bool_item("speech", "Voice messages"),
		bool_item("speechThermal", "Thermal average"),
		bool_item("speechFlarm", "FLARM warnings"),
		act_item("Test voice", func { thermal_done(2.1); }),
	]},
	flarm: {title: "FLARM", items: [
		bool_item("flarmAlarms", "Radar on warning"),
		num_item("flarmDismiss", "Dismiss time", 0, 120, 5, "%d s"),
	]},
	thermal: {title: "Thermal assistant", items: [
		choice_item("thermalMode", "Colours from", ["auto", "mc", "avg"], {auto: "Auto span", mc: "MacCready", avg: "Average vario"}),
		bool_item("autoThermal", "Show while circling"),
	]},
	display: {title: "Display", items: [
		num_item("brightness", "Brightness", 10, 100, 10, "%d %%"),
		bool_item("nightMode", "Night mode"),
		num_item("nightDark", "Night darkness", 10, 90, 10, "%d %%"),
	]},
	units: {title: "Units", items: [
		choice_item("uVario", "Vertical speed", ["m/s", "kt", "ft/min"], nil),
		choice_item("uAlt", "Altitude", ["m", "ft"], nil),
		choice_item("uSpeed", "Speed", ["km/h", "kt", "mph"], nil),
	]},
};

var menu_stack = [];          # [{name, sel, top}]
var menu_edit = 0;
var ROWS = 6;

var menu_cur = func { return menu_stack[size(menu_stack) - 1]; };

var item_value = func(it) {
	if (it.t == "num") return sprintf(it.fmt, S[it.key]);
	if (it.t == "bool") return S[it.key] ? "on" : "off";
	if (it.t == "choice") {
		var v = S[it.key];
		if (it.names != nil and contains(it.names, v)) return it.names[v];
		return sprintf("%s", v);
	}
	return "";
};

var apply_setting = func(key) {
	if (key == "theme" or key == "scaleType" or key == "varioRange" or key == "uVario" or key == "needleStyle") {
		el.ring.removeAllChildren(); build_ring(el.ring);
		el.dyn.removeAllChildren(); build_dyn(el.dyn);
	}
	if (key == "nbCount" or key == "nb1" or key == "nb2" or key == "nb3" or key == "nb4" or key == "flarmIcon") rebuild_vario();
	if (key == "integrator") { avg_vario.w = S.integrator; }
};

# change the value of an item: dir = +1 / -1
var item_change = func(it, dir) {
	var key = it.key;
	if (it.t == "bool") setting_set(key, S[key] ? 0 : 1);
	elsif (it.t == "num") {
		var v = S[key] + dir * it.step;
		v = math.round(clamp(v, it.min, it.max) / it.step) * it.step;
		setting_set(key, clamp(v, it.min, it.max));
	} elsif (it.t == "choice") {
		var idx = 0;
		for (var i = 0; i < size(it.opts); i += 1) if (it.opts[i] == S[key]) idx = i;
		idx = math.mod(idx + dir + size(it.opts), size(it.opts));
		setting_set(key, it.opts[idx]);
	}
	apply_setting(key);
};

var build_setup = func(g) {
	el.set_title = text(g, "Setup", CX - 8, 66, 40, ORANGE, "center-center");
	el.set_sel = poly(g, [[70, 98], [446, 98], [446, 142], [70, 142]], [0.2, 0.3, 0.55], WHITE, 2);
	el.set_lines = [];
	for (var i = 0; i < ROWS; i += 1) {
		var l = text(g, "", 82, 120 + 52 * i, 30, WHITE, "left-center");
		var v = text(g, "", 436, 120 + 52 * i, 30, ORANGE, "right-center");
		append(el.set_lines, [l, v]);
	}
	el.set_more = text(g, "", CX - 8, 440, 24, GREY, "center-center");
};

var update_setup = func {
	var m = menu_cur();
	var menu = MENUS[m.name];
	el.set_title.setText(menu.title);
	var n = size(menu.items);
	if (m.sel < m.top) m.top = m.sel;
	if (m.sel >= m.top + ROWS) m.top = m.sel - ROWS + 1;
	el.set_sel.setTranslation(0, 52 * (m.sel - m.top));
	var sel_color = menu_edit ? ORANGE : [0.2, 0.3, 0.55];
	el.set_sel.setColorFill(sel_color[0], sel_color[1], sel_color[2]);
	for (var i = 0; i < ROWS; i += 1) {
		var idx = m.top + i;
		if (idx >= n) { el.set_lines[i][0].setText(""); el.set_lines[i][1].setText(""); continue; }
		var it = menu.items[idx];
		el.set_lines[i][0].setText(it.label);
		var vt = item_value(it);
		if (it.t == "sub") vt = ">";
		el.set_lines[i][1].setText(vt);
		# label and value share one line (354 px): when both are long, the text gets smaller instead of overlapping
		var est = 0.6 * 30 * (size(it.label) + size(vt)) + 24;
		var fs = est > 354 ? math.max(18, math.floor(30 * 354 / est)) : 30;
		el.set_lines[i][0].setFontSize(fs, 1.0);
		el.set_lines[i][1].setFontSize(fs, 1.0);
		var hot = (idx == m.sel and menu_edit);
		el.set_lines[i][1].setColor(hot ? 0 : ORANGE[0], hot ? 0 : ORANGE[1], hot ? 0 : ORANGE[2]);
	}
	el.set_more.setText((m.top > 0 ? "^  " : "") ~ (m.top + ROWS < n ? "v" : ""));
};

var enter_setup = func { in_setup = 1; menu_stack = [{name: "main", sel: 0, top: 0}]; menu_edit = 0; last_view = ""; };
var leave_setup = func { in_setup = 0; menu_edit = 0; last_view = ""; };
var menu_back = func {
	if (size(menu_stack) <= 1) leave_setup();
	else { menu_stack = subvec(menu_stack, 0, size(menu_stack) - 1); menu_edit = 0; }
};

# ####################################################################################
# show / update
# ####################################################################################
var toast = func(msg) {
	el.toast.setText(msg);
	el.toast.setFontSize(clamp(math.floor(272 / (0.62 * size(msg))), 20, 34), 1.0);       # long messages stay inside the box
	el.toast_bg.show();
	el.toast.show();
	toast_until = (getprop("/sim/time/elapsed-sec") or 0) + 1.5;
};

var flarm_dismissed_until = 0;
var view_now = func(t) {
	if (!f.power) return "off";
	if (t < boot_until) return "boot";
	if (in_setup) return "setup";
	if (S.flarmAlarms and warn_level > 0 and t > flarm_dismissed_until) return "flarm";
	return PAGES[page];
};

var show_page = func(view) {
	foreach (var nm; keys(el.pages)) el.pages[nm].hide();
	if (view == "off") { el.mask.hide(); el.ring.hide(); el.dyn.hide(); return; }
	if (view == "boot" or view == "setup") {
		el.pages[view].show();
		el.mask.hide(); el.ring.hide(); el.dyn.hide();
		return;
	}
	el.pages[view].show();
	el.mask.show(); el.ring.show(); el.dyn.show();
	var titles = {vario: "", ahrs: "Hor", wind: "Wnd", flarm: warn_level > 0 ? "WARN" : "Flm", thermal: "Thm", gmeter: "G"};
	el.title.setText(titles[view]);
};

var redraw = func {
	var t = getprop("/sim/time/elapsed-sec") or 0;
	if (f.power and !was_powered) boot_until = t + 3;
	was_powered = f.power;
	var br = S.brightness / 100 * (S.nightMode ? S.nightDark / 100 : 1);
	root.getNode("brightness", 1).setDoubleValue(f.power ? br : 0);

	# the thermal assistant shows by itself while circling (7.1.12.4.2.3)
	if (S.autoThermal and !in_setup) {
		if (f.circling and !prev_circling) { saved_page = page; page = 4; }
		if (!f.circling and prev_circling and saved_page != nil) { page = saved_page; saved_page = nil; }
	}
	prev_circling = f.circling;

	var view = view_now(t);
	if (view != last_view) { show_page(view); last_view = view; }
	if (toast_until and t > toast_until) { el.toast.hide(); el.toast_bg.hide(); toast_until = 0; }
	if (view == "off" or view == "boot") return;
	if (view == "setup") { update_setup(); return; }

	# dial: needle, markers, bar
	el.needle.setRotation(-dial_angle(needle_value()) * D2R);
	if (S.mcDiamond) { el.mc.show(); el.mc.setRotation(-dial_angle(S.mc) * D2R); } else el.mc.hide();
	if (S.redDiamond == "none") el.avg.hide();
	else {
		el.avg.show();
		el.avg.setRotation(-dial_angle(S.redDiamond == "netto" ? f.avgN : f.avgV) * D2R);
	}
	if (f.lastThermal != nil) { el.th.show(); el.th.setRotation(-dial_angle(f.lastThermal) * D2R); }
	el.bar_g.removeAllChildren();
	if (S.yellowBar == "minmax") {
		var a0 = dial_angle(f.minV);
		var a1 = dial_angle(f.maxV);
		if (math.abs(a1 - a0) > 1) arc(el.bar_g, CX, CY, R_IN - 5, a0, a1, YELLOW, 9);
	}

	if (view == "vario") update_vario_page();
	elsif (view == "ahrs") update_ahrs();
	elsif (view == "wind") update_wind();
	elsif (view == "flarm") update_flarm();
	elsif (view == "thermal") update_thermal();
	elsif (view == "gmeter") update_gmeter();
};

# numeric page values and status line
var update_vario_page = func {
	foreach (var b; el.nb) {
		var info = nb_info(b.key);
		b.label.setText(info[0]);
		b.value.setText(info[1]);
		b.unit.setText(info[2]);
		# keep value and unit together, centred
		var w = size(info[1]) * b.vsize * 0.55;
		var ux = size(info[2]) * (b.vsize * 0.5) * 0.55;
		b.value.setTranslation(CX - 8 - ux * 0.5, b.vy);
		b.unit.setTranslation(CX - 8 - ux * 0.5 + w * 0.5 + 6, b.vy + 6);
	}
	if (f.circling) { el.mode_climb.show(); el.mode_cruise.hide(); }
	else { el.mode_climb.hide(); el.mode_cruise.show(); }
	var fc = size(targets) ? RED : GREY;
	foreach (var a; el.flarm_arcs) a.setColor(fc[0], fc[1], fc[2]);
	var n = (S.sideBar == "stf" and f.ias >= 20) ? int(clamp(math.round(f.stfDelta * 3.6 / 10), -4, 4)) : 0;
	for (var i = 0; i < 4; i += 1) {
		if (n > i) el.stf_up[i].show(); else el.stf_up[i].hide();
		if (-n > i) el.stf_dn[i].show(); else el.stf_dn[i].hide();
	}
};

# ####################################################################################
# buttons
# ####################################################################################
var button = func(which, down) {
	var t = getprop("/sim/time/elapsed-sec") or 0;
	if (!f.power) return;
	if (down and S.flarmAlarms and warn_level > 0 and t > flarm_dismissed_until and !in_setup) {
		flarm_dismissed_until = t + S.flarmDismiss;      # any button dismisses a FLARM warning
		flarm_quiet_until = flarm_dismissed_until;
		return;
	}
	if (which == "mid") {
		if (down) { press_t.mid = t; return; }
		var held = t - press_t.mid;
		if (in_setup) {
			if (held >= 1.0) { if (menu_edit) menu_edit = 0; else menu_back(); return; }
			var m = menu_cur();
			var it = MENUS[m.name].items[m.sel];
			if (it.t == "sub") { append(menu_stack, {name: it.to, sel: 0, top: 0}); }
			elsif (it.t == "act") it.act();
			elsif (it.t == "bool") item_change(it, 1);
			else menu_edit = !menu_edit;
			return;
		}
		if (held >= 1.0) enter_setup();
		else { page = math.mod(page + 1, size(PAGES)); saved_page = nil; last_view = ""; }
		return;
	}
	if (!down) return;
	var dir = which == "up" ? -1 : 1;
	if (in_setup) {
		var m = menu_cur();
		if (menu_edit) item_change(MENUS[m.name].items[m.sel], -dir);     # top button = more
		else m.sel = math.mod(m.sel + dir + size(MENUS[m.name].items), size(MENUS[m.name].items));
	} elsif (PAGES[page] == "gmeter") {
		if (which == "up") { f.maxG = f.gload; f.minG = f.gload; toast("G reset"); }       # the reset of a G-meter
	} elsif (PAGES[page] == "flarm") {
		radar_zoom = clamp(radar_zoom - dir, 0, size(RADAR_RANGES) - 1);
	} elsif (PAGES[page] == "vario") {
		setting_set("mc", clamp(math.round((S.mc - dir * 0.1) * 10) / 10, 0, 5));
		toast(sprintf("MC %.1f", fvario(S.mc)));
	}
};

# ####################################################################################
# a window with the screen, for a closer look (click the screen, or the ASK21 menu)
# The window has a canvas of its own that shows the instrument's canvas as an image. Its buttons are in the
# dialog gui/dialogs/lxv8.xml (mouse events on canvas elements are not reliable in this window).
# ####################################################################################
# the controls dialog (gui/dialogs/lxv8.xml); FlightGear finds dialog files when it starts
var show_controls = func {
	var err = [];
	call(func { gui.showDialog("lxv8_dialog"); }, nil, nil, nil, err);
};

var close_window = func {
	var err0 = [];
	call(func { fgcommand("dialog-close", props.Node.new({"dialog-name": "lxv8_dialog"})); }, nil, nil, nil, err0);
	if (contains(globals, "lxv8_window") and globals.lxv8_window != nil) {
		var err = [];
		call(func { globals.lxv8_window.del(); }, nil, nil, nil, err);
		globals.lxv8_window = nil;
	}
};

var toggle_window = func {
	# opening again replaces the window; it is closed with its own close button
	close_window();
	var w = canvas.Window.new([460, 460], "dialog");
	w.setTitle("LXNAV V8");
	var wc = w.createCanvas();
	wc.setColorBackground(0.07, 0.07, 0.08, 1);
	var g = wc.createGroup();
	var img = g.createChild("image");
	img.set("src", "canvas://by-index/texture[" ~ cv._node.getIndex() ~ "]");
	img.setSize(460, 460);
	globals.lxv8_window = w;
	move_window();
	show_controls();
};

# the window cannot be dragged reliably (its title bar does not take clicks everywhere), so the controls dialog has
# sliders for its position
var move_window = func {
	if (!contains(globals, "lxv8_window") or globals.lxv8_window == nil) return;
	var x = getprop("/instrumentation/lxv8/window/x"); var y = getprop("/instrumentation/lxv8/window/y");
	var err = [];
	call(func { globals.lxv8_window.setPosition(x, y); }, nil, nil, nil, err);
};

# A press of one of the push buttons from the controls dialog (gui/dialogs/lxv8.xml); long = 1 for the long press
# of the middle button. The 3D buttons and the mouse presses on them go through the property listeners below.
var click = func(which, long) {
	var t = getprop("/sim/time/elapsed-sec") or 0;
	button(which, 1);
	if (long and which == "mid") press_t.mid = t - 1.2;
	button(which, 0);
};

# the window can also be opened by property (a key binding, the menu, telnet)
var window_listener = func(n) {
	if (n.getValue()) { n.setValue(0); toggle_window(); }
};

if (getprop("/instrumentation/lxv8/window/x") == nil) setprop("/instrumentation/lxv8/window/x", 330);
if (getprop("/instrumentation/lxv8/window/y") == nil) setprop("/instrumentation/lxv8/window/y", 170);

# listeners are kept in globals so that reloading this file does not add them twice
if (contains(globals, "lxv8_listeners")) foreach (var id; globals.lxv8_listeners) removelistener(id);
globals.lxv8_listeners = [];
append(globals.lxv8_listeners, setlistener("/instrumentation/lxv8/inputs/window", window_listener, 0, 0));
append(globals.lxv8_listeners, setlistener("/instrumentation/lxv8/window/x", move_window, 0, 0));
append(globals.lxv8_listeners, setlistener("/instrumentation/lxv8/window/y", move_window, 0, 0));
foreach (var b; ["up", "mid", "dn"]) {
	(func(which) {
		append(globals.lxv8_listeners, setlistener("/instrumentation/lxv8/inputs/btn-" ~ which, func(n) { button(which, n.getValue()); }, 0, 0));
	})(b);
}

# ####################################################################################
# start
# ####################################################################################
var start_ui = func {
	if (contains(globals, "lxv8_ui_timer") and globals.lxv8_ui_timer != nil) globals.lxv8_ui_timer.stop();
	build();
	last_view = "";
	var t = maketimer(0.05, redraw);
	globals.lxv8_ui_timer = t;
	t.start();
};

setlistener("/sim/signals/fdm-initialized", func {
	# the instrument has its own sound: the ILEC tone of the stock script is switched off
	setprop("/instrumentation/ilec-sc7/audio", 0);
	start_core();
	start_ui();
}, 0, 0);

# ---- DEVELOPMENT ONLY: reload both files and restart the instrument
#      set /instrumentation/lxv8/dev/reload to 1, e.g. over the telnet interface;
#      /instrumentation/lxv8/dev/path = folder to load from (default: the aircraft folder)
append(globals.lxv8_listeners, setlistener("/instrumentation/lxv8/dev/reload", func(n) {
	if (!n.getValue()) return;
	n.setValue(0);
	var dir = getprop("/instrumentation/lxv8/dev/path");
	if (dir == nil or dir == "") dir = getprop("/sim/aircraft-dir") ~ "/Models/Interior/Panel/Instruments/lx-v8/";
	var result = "ok";
	foreach (var fn; ["lxv8-core.nas", "lxv8.nas"]) {
		if (!io.load_nasal(dir ~ fn, "lxv8")) result = "failed to load " ~ fn;     # parse errors end up here
	}
	setprop("/instrumentation/lxv8/dev/load-result", result);
	lxv8.start_core();
	lxv8.start_ui();
}, 0, 0));
