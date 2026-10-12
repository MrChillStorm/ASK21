# Anemoi Wind Indicator by Bea Wolf based on

# A3XX Lower ECAM Canvas
# Joshua Davidson (Octal450)
#######################################

var Anemoi_start = nil;
var Anemoi_wind = nil;
var Anemoi_display = nil;

var state = 0; # 0 = off, 1 = start, 2 = on

var volts = props.globals.getNode("/systems/electrical/outputs/wind-compass", 1);

var instrument_path = "Aircraft/ASK21_v9/Models/Interior/Panel/Instruments/Anemoi/";

var wind_kts = props.globals.getNode("/environment/wind-speed-kt");
var wind_dir = props.globals.getNode("/environment/wind-from-heading-deg");

var wind_avg_kts = props.globals.getNode("/instrumentation/anemoi/avg-wind-speed-kt");
var wind_avg_dir = props.globals.getNode("/instrumentation/anemoi/avg-wind-from-heading-deg");

var ac_hdg = props.globals.getNode("/orientation/heading-deg");

var canvas_Anemoi_base = {
	init: func(canvas_group, file) {
		var font_mapper = func(family, weight) {
			if( weight == "bold" ){
				return "LiberationFonts/LiberationSans-Bold.ttf";
			} else {
				return "LiberationFonts/LiberationSans-Regular.ttf";
			}
		};

		canvas.parsesvg(canvas_group, file, {'font-mapper': font_mapper});

		var svg_keys = me.getKeys();
		foreach(var key; svg_keys) {
			me[key] = canvas_group.getElementById(key);
		}

		me.page = canvas_group;

		return me;
	},
	getKeys: func() {
		return [];
	},
	update: func() {
		if( state == 2 ){
			Anemoi_wind.page.show();
			Anemoi_wind.update();
			Anemoi_start.page.hide();
		} elsif( state == 1 ){
			Anemoi_wind.page.hide();
			Anemoi_start.page.show();
		} else {
			Anemoi_wind.page.hide();
			Anemoi_start.page.hide();
		}
	},
};

var canvas_Anemoi_start = {
	new: func(canvas_group, file) {
		var m = { parents: [canvas_Anemoi_start , canvas_Anemoi_base] };
		m.init(canvas_group, file);

		return m;
	},
	getKeys: func() {
		return [];
	},
	update: func() {
	},
};
var canvas_Anemoi_wind = {
	new: func(canvas_group, file) {
		var m = { parents: [canvas_Anemoi_wind , canvas_Anemoi_base] };
		m.init(canvas_group, file);

		return m;
	},
	getKeys: func() {
		return [ "live.deg", "live.kph", "live.needle", "avg.deg", "avg.kph", "avg.needle"];
	},
	update: func() {
		var hdg = ac_hdg.getDoubleValue();

		# Live Wind
		var live_wind_deg = wind_dir.getDoubleValue();
		me["live.deg"].setText( sprintf("%03d", math.round( live_wind_deg ) ) );
		me["live.kph"].setText( sprintf("%2d", math.round( wind_kts.getDoubleValue() * KT2MPS * 3.6 ) ) );
		me["live.needle"].setRotation( ( hdg - live_wind_deg ) * -D2R );

		# Average Wind
		var avg_wind_deg = wind_avg_dir.getDoubleValue();
		me["avg.deg"].setText( sprintf("%03d", math.round( avg_wind_deg ) ) );
		me["avg.kph"].setText( sprintf("%2d", math.round( wind_avg_kts.getDoubleValue() * KT2MPS * 3.6 ) ) );
		me["avg.needle"].setRotation( ( hdg - avg_wind_deg ) * -D2R );

	},
};

var btn = func() {
	# TODO
}

var canvas_Anemoi_base_timer = maketimer( 0.1, canvas_Anemoi_base.update );
canvas_Anemoi_base_timer.simulatedTime = 1;

setlistener("sim/signals/fdm-initialized", func {
	Anemoi_display = canvas.new({
		"name": "Anemoi_display",
		"size": [240, 240],
		"view": [240, 240],
		"mipmapping": 1
	});
	Anemoi_display.addPlacement({"node": "anemoi.screen"});

	Anemoi_start = canvas_Anemoi_start.new( Anemoi_display.createGroup(), instrument_path~"anemoi-start.svg");
	Anemoi_wind = canvas_Anemoi_wind.new( Anemoi_display.createGroup(), instrument_path~"anemoi-wind.svg");

	canvas_Anemoi_base_timer.start();
}, 0, 0);

setlistener(volts, func{
	if(volts.getDoubleValue() >= 9 and state == 0 ){
		state = 1;
		settimer( func{
			state = 2;
		}, 0.7 + rand() * 0.5 );
	}else if( volts.getDoubleValue() < 9 ){
		state = 0;
	}
});
