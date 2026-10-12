# ILEC Engine Instrument by Bea Wolf based on

# A3XX Lower ECAM Canvas
# Joshua Davidson (Octal450)
#######################################

var ILEC_only = nil;
var ILEC_display = nil;
var rpm_v = nil;

var disp_page = 0;	# 0 = standard, 1 = Liquid Coolant Temp, 2 = Cooling Air Temp, 3 = Fuel Consumption, 4 = Engine Battery Voltage

var volts_p = props.globals.getNode("/systems/electrical/outputs/ilec", 1);
var rpm_p = props.globals.getNode("/engines/engine[0]/engine-rpm", 1);
var fuel_level_p = props.globals.getNode("/consumables/fuel/tank/level-m3", 1);

var coolant_temp = props.globals.initNode("/engines/engine[0]/coolant-temp-degc", 0.0, "DOUBLE");
var air_temp = props.globals.initNode("/engines/engine[0]/air-temp-degc", 0.0, "DOUBLE");
var fuel_flow = props.globals.getNode("/engines/engine[0]/fuel-flow-gph", 1);
var eng_batt_volts = props.globals.getNode("/systems/electrical/eng-volts", 1);

var canvas_ILEC_base = {
	init: func(canvas_group, file) {
		var font_mapper = func(family, weight) {
			return "LiberationFonts/LiberationSans-Bold.ttf";
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
		if ( volts_p.getDoubleValue() >= 10.0 ) {
			ILEC_only.page.show();
			ILEC_only.update();
		} else {
			ILEC_only.page.hide();
		}
	},
};
	
var canvas_ILEC_only = {
	new: func(canvas_group, file) {
		var m = { parents: [canvas_ILEC_only , canvas_ILEC_base] };
		m.init(canvas_group, file);

		return m;
	},
	getKeys: func() {
		return ["LeftInd", "RightInd"];
	},
	update: func() {
		if( disp_page == 0 ){

			rpm_v = rpm_p.getDoubleValue();
			if( rpm_v >= 1000 ){
				me["LeftInd"].setText( sprintf("%4d", math.round( rpm_v,100 ) ) );
			}else{
				me["LeftInd"].setText( sprintf("%4d", 0) );
			}

			me["RightInd"].setText( sprintf("%2d", math.round( fuel_level_p.getDoubleValue() * 1000 ) ) );
		} elsif( disp_page == 1 ){
			me["LeftInd"].setText( "H2O" );
			me["RightInd"].setText( sprintf("%3d", math.round( coolant_temp.getDoubleValue() ) ) );
		} elsif( disp_page == 2 ){
			me["LeftInd"].setText( "Air" );
			me["RightInd"].setText( sprintf("%3d", math.round( air_temp.getDoubleValue() ) ) );
		} elsif( disp_page == 3 ){
			me["LeftInd"].setText( "Fuel" );
			me["RightInd"].setText( sprintf("%3.1f", fuel_flow.getDoubleValue() * GAL2L ) );
		} elsif( disp_page == 4 ){
			me["LeftInd"].setText( "U" );
			me["RightInd"].setText( sprintf("%3.1f", eng_batt_volts.getDoubleValue() ) );
		} else {
			me["LeftInd"].setText( "Invalid Page Number" );
			me["RightInd"].setText( "--" );
		}
	},
};

var ILEC_updater = maketimer( 0.1, canvas_ILEC_base.update );
ILEC_updater.simulatedTime = 1;

setlistener("sim/signals/fdm-initialized", func {
	ILEC_display = canvas.new({
		"name": "ILEC",
		"size": [320, 128],
		"view": [320, 128],
		"mipmapping": 1
	});
	ILEC_display.addPlacement({"node": "ILEC.screen"});
	var groupOnly = ILEC_display.createGroup();

	ILEC_only = canvas_ILEC_only.new(groupOnly, "Aircraft/ASK21_v9/Models/Interior/Panel/Instruments/ILEC/ILEC-display.svg");

	ILEC_updater.start();
}, 0, 0);


var reset_page_timer = maketimer( 5, func{ disp_page = 0 } );
reset_page_timer.simulatedTime = 1;
reset_page_timer.singleShot = 1;

var cycle_display = func{

	disp_page += 1;

	if( disp_page > 4 ){
		disp_page = 0;
	} else {
		if( reset_page_timer.isRunning ){
			reset_page_timer.restart( 5.0 );
		} else {
			reset_page_timer.start();
		}
	}
}
