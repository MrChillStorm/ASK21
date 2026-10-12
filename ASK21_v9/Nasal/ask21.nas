#	ASK21 Nasal Code
aircraft.livery.init("Aircraft/ASK21_v9/Models/Liveries");


#		G-Force Meter
# Script for recording the minimum / maximum G forces seen by the

var GCurrent = props.globals.initNode("/accelerations/pilot-g", 1.0, "DOUBLE");
var GMin = props.globals.initNode("/accelerations/pilot-gmin", 1.0, "DOUBLE");
var GMax = props.globals.initNode("/accelerations/pilot-gmax", 1.0, "DOUBLE");

gmeterUpdate = func {
	var current_val = GCurrent.getDoubleValue();
	if( current_val < GMin.getDoubleValue() ){
		GMin.setDoubleValue( math.max( current_val, -6 ) );
	} elsif( current_val > GMax.getDoubleValue() ){
		GMax.setDoubleValue( math.min( current_val, 10 ) );
	}
}

var gmeterTimer = maketimer( 0.0, gmeterUpdate );
gmeterTimer.simulatedTime = 1;
gmeterTimer.start();

#		Listeners
setlistener("/controls/electric/battery-switch", func {
    if(getprop("/controls/electric/battery-switch")){
        interpolate("/controls/electric/battery-switch-pos", 1.00, 0.25);
    }else{
        interpolate("/controls/electric/battery-switch-pos", 0.00, 0.25);
    }
});
setlistener("/controls/electric/turn-slip-switch", func {
    if(getprop("/controls/electric/turn-slip-switch")){
        interpolate("/controls/electric/turn-slip-switch-pos", 1.00, 0.25);
    }else{
        interpolate("/controls/electric/turn-slip-switch-pos", 0.00, 0.25);
    }
});
setlistener("/controls/flight/elevator", func{
	if(getprop("/controls/flight/elevator")<-0.9){
		setprop("/controls/flight/elevator-trim-jump", -1);
	}else if(getprop("/controls/flight/elevator")>0.9){
		setprop("/controls/flight/elevator-trim-jump", 1);
	}else{
		setprop("/controls/flight/elevator-trim-jump", 0);
	}
});


var windcheck = func{
	wd=getprop("/environment/wind-from-heading-deg");
	ch=getprop("/orientation/heading-deg");
	ws=getprop("/environment/wind-speed-kt");
	var wdi=wd-ch;
	if(wdi<0){
		gui.popupTip(sprintf("Wind from %d deg left at %.1f knots", -wdi, ws));
	}else{
		gui.popupTip(sprintf("Wind from %d deg right at %.1f knots", wdi, ws));
	}
}

#Helper functions to print messages
var msg = func ( destination, string ) {
	setprop("/sim/messages/"~destination, string);
}

#Rain system from f16, adapted for ASK21
var rtimer = maketimer(0.1, func {
   var airspeed = getprop("/velocities/airspeed-kt");
   var airspeed_max = 120;
   if (airspeed > airspeed_max) {
           airspeed = airspeed_max;
   }
   airspeed = math.sqrt(airspeed/airspeed_max);

   var splash_x = -0.1 - 2.0 * airspeed;
   var splash_y = 0.0;
   var splash_z = 1.0 - 1.35 * airspeed;;

   setprop("/environment/aircraft-effects/splash-vector-x", splash_x);
   setprop("/environment/aircraft-effects/splash-vector-y", splash_y);
   setprop("/environment/aircraft-effects/splash-vector-z", splash_z);
   }
);
rtimer.start();

# Vibrating yaw string
io.include("Aircraft/Generic/soaring-instrumentation-sdk.nas");

var yawstring = YawString.new(
	on_update: update_prop("instrumentation/yawstring"));

var string_instrument = Instrument.new(
	components: [yawstring],
	enable: 1);

#	Properties available to other scripts e.g. electrical-*.nas
var com_ptt		= props.globals.getNode("/instrumentation/comm[0]/ptt", 1);
var com_start	= props.globals.getNode("/instrumentation/comm[0]/start", 1);
var vario_vol	= props.globals.getNode("/instrumentation/ilec-sc7/volume", 1);
var vario_aud	= props.globals.getNode("/instrumentation/ilec-sc7/audio", 1);
var vario_read	= props.globals.getNode("/instrumentation/ilec-sc7/te-reading-mps", 1);
var turnbank_spin	= props.globals.initNode("/instrumentation/turn-indicator/spin", 0.0, "DOUBLE");

var electrical	=	props.globals.initNode("/systems/electrical");
var batt_prop	=	electrical.initNode("battery");

var electric_serviceable = electrical.initNode("serviceable", 1, "BOOL");

var electrical_switches = {
	battery:	props.globals.initNode("/controls/electric/battery-switch", 0, "BOOL"),
	turnslip:	props.globals.initNode("/controls/electric/turn-slip-switch", 0, "BOOL"),
};

var electrical_consumers = {
	ilec_sc7:	electrical.initNode("outputs/ilec-sc7", 0.0, "DOUBLE"),
	comm:		electrical.initNode("outputs/comm", 0.0, "DOUBLE"),
	turn:		electrical.initNode("outputs/turn-indicator", 0.0, "DOUBLE"),
	flarm:	electrical.initNode("outputs/flarm", 0.0, "DOUBLE"),
	wind_comp:	electrical.initNode("outputs/wind-compass", 0.0, "DOUBLE"),
	transponder:	electrical.initNode("outputs/transponder", 0.0, "DOUBLE"),
};

var windcompass_installed = props.globals.getNode("/sim/model/windcompass", 1);
var transponder_installed = props.globals.getNode("/sim/model/transponder", 1);	# replaces the turn indicator

var amps_p  = electrical.initNode("amps",  0.0, "DOUBLE");
var volts_p = electrical.initNode("volts", 0.0, "DOUBLE");
var load_p  = electrical.initNode("load",  0.0, "DOUBLE");

var freeze_replay	=	props.globals.getNode("/sim/freeze/replay-state");
