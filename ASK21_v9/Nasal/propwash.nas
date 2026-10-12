# ####################################################################################
# Prop wash of other aircraft, mainly of the tow plane.
#
# FlightGear does not simulate the wake of AI or multiplayer aircraft, so this script
# calculates where the glider is in relation to the slipstream of every nearby aircraft
# that is moving fast enough, and shakes the glider with random forces accordingly.
#
# The wake lies on the thrust line of the other aircraft, sinks slowly towards the rear and
# gets wider the further back it goes. Being on or below it is rough, flying well above
# it (high tow) is calm. It is still clearly felt at the end of a 60 m rope.
#
# The forces are applied through the external reactions wash_lw, wash_rw, wash_tz and
# wash_ty, which are defined in the JSBSim files. Without them (YASim) nothing happens.
# ####################################################################################

var update_s       = 0.05;          # update interval
var max_range_m    = 150;           # nothing is calculated beyond this distance
var axis_height_m  = 1.0;           # thrust line above the position of the other aircraft
var candidates     = [];
var last_scan_s    = -10;
var active         = 0;

var forces = ["wash_lw", "wash_rw", "wash_tz", "wash_ty"];

var setForces = func(lw, rw, tz, ty) {
	var v = [lw, rw, tz, ty];
	for (var i = 0; i < 4; i += 1) {
		setprop("fdm/jsbsim/external_reactions/" ~ forces[i] ~ "/magnitude", v[i]);
	}
}

# smooth random signal between -1 and 1, different for every channel
var noise = func(t, channel) {
	var k = 1 + 0.13 * channel;
	var w = 2 * math.pi * t * k;
	return (math.sin(1.1 * w + 1.7 * channel)
	      + math.sin(2.3 * w + 4.1 * channel)
	      + math.sin(3.7 * w + 0.3 * channel)
	      + 0.5 * math.sin(5.3 * w + 2.2 * channel)) / 3.2;
}

# remember the aircraft that could have a wake, once per second
var scan = func {
	candidates = [];
	foreach (var n; props.globals.getNode("ai/models", 1).getChildren()) {
		var name = n.getName();
		if ( name != "aircraft" and name != "multiplayer" and name != "dragger" ) {
			continue;
		}
		var valid = n.getNode("valid");
		if ( valid == nil or !valid.getValue() ) {
			continue;
		}
		if ( n.getNode("position/latitude-deg") == nil or n.getNode("orientation/true-heading-deg") == nil ) {
			continue;
		}
		append(candidates, n);
	}
}

# strength of the wake (0 ... 1) at the position of the glider
var wakeStrength = func(node, me) {
	var speed_kt = node.getNode("velocities/true-airspeed-kt") != nil
		? node.getNode("velocities/true-airspeed-kt").getValue() : 0;
	var speed_factor = (speed_kt - 30) / 25;                 # no wake while taxiing
	if ( speed_factor <= 0 ) {
		return 0;
	}
	if ( speed_factor > 1 ) {
		speed_factor = 1;
	}

	var other = geo.Coord.new().set_latlon(
		node.getNode("position/latitude-deg").getValue(),
		node.getNode("position/longitude-deg").getValue(),
		node.getNode("position/altitude-ft").getValue() * FT2M);
	var dist_m = other.distance_to(me);
	if ( dist_m > max_range_m or dist_m < 1 ) {
		return 0;
	}

	# position of the glider in the frame of the other aircraft
	var rel = (other.course_to(me) - node.getNode("orientation/true-heading-deg").getValue()) * math.pi / 180;
	var behind_m = -dist_m * math.cos(rel);
	var side_m   = dist_m * math.sin(rel);
	var height_m = me.alt() - (other.alt() + axis_height_m);  # above the thrust line
	if ( behind_m < 3 ) {
		return 0;
	}

	var radius_m = 1.2 + 0.1 * behind_m;                     # the slipstream spreads
	var axis_m   = -(0.4 + 0.05 * behind_m);                 # and sinks
	var vert_m   = height_m - axis_m;
	var scale_m  = vert_m > 0 ? 0.7 * radius_m : 1.8 * radius_m;   # much wider below
	var u = vert_m / scale_m;
	var v = side_m / radius_m;

	var fade = math.exp(-behind_m / 150);
	if ( behind_m < 8 ) {
		fade *= behind_m / 8;
	}
	return math.exp(-(u * u + v * v)) * fade * speed_factor;
}

var update = func {
	var t = getprop("sim/time/elapsed-sec");
	if ( t - last_scan_s > 1 ) {
		scan();
		last_scan_s = t;
	}

	var strength = getprop("sim/glider/propwash/strength");
	var best = 0;
	if ( strength != nil and strength > 0 ) {
		var me = geo.aircraft_position();
		foreach (var n; candidates) {
			var s = wakeStrength(n, me);
			if ( s > best ) {
				best = s;
			}
		}
		best *= strength;
	}

	if ( best > 0.003 ) {
		var weight_lbs = getprop("fdm/jsbsim/inertia/weight-lbs") or 1100;
		var bump  = best * weight_lbs * 0.18 * noise(t, 0);     # vertical gusts
		var roll  = best * weight_lbs * 0.07 * noise(t, 1);     # difference between the wings
		var pitch = best * weight_lbs * 0.03 * noise(t, 2);     # force at the tail
		var yaw   = best * weight_lbs * 0.03 * noise(t, 3);     # sideways force at the tail
		setForces(bump / 2 + roll, bump / 2 - roll, pitch, yaw);
		active = 1;
	} elsif ( active ) {
		setForces(0, 0, 0, 0);
		active = 0;
	}
}

var timer = maketimer(update_s, update);

setlistener("sim/signals/fdm-initialized", func {
	if ( getprop("sim/glider/propwash/strength") == nil ) {
		setprop("sim/glider/propwash/strength", 1);
	}
	# only the JSBSim models have the external reactions
	if ( props.globals.getNode("fdm/jsbsim/external_reactions/wash_lw") != nil ) {
		timer.start();
	}
});
