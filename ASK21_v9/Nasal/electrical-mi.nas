##
# Procedural model of an ASK21mi electrical system.  Includes a
# preliminary battery charge/discharge model and realistic ammeter
# gauge modeling.
#
# Based on C172P electrical system.

#Reference: 	https://www.alexander-schleicher.de/wp-content/uploads/2015/02/219_TM03_D_HB.pdf
#			http://hakenesch.userweb.mwn.de/vtp_flugeigenschaften/ASK21Mi_Flughandbuch.pdf


# Initialize properties

var eng_battswitch 	=	props.globals.getNode("/controls/electric/engine-battery-switch", 1);
var eng_starter		=	props.globals.getNode("/controls/engines/engine[0]/starter-btn", 1);
var eng_starter_int	=	props.globals.getNode("/controls/engines/engine[0]/starter", 1);
var eng_magneto		=	props.globals.getNode("/controls/engines/engine[0]/magnetos", 1);
var eng_mixture		=	props.globals.getNode("/controls/engines/engine[0]/mixture-int", 1);
var eng_prop_pos		=	props.globals.getNode("/engines/engine/prop-pos-norm", 1);
var eng_prop_cmd		=	props.globals.getNode("/controls/engines/engine/extend-propeller", 1);


var eng_amps_p  = electrical.initNode("eng-amps",  0.0, "DOUBLE");
var eng_volts_p = electrical.initNode("eng-volts", 0.0, "DOUBLE");
var eng_load_p  = electrical.initNode("eng-load",  0.0, "DOUBLE");

var eng_rpm		= props.globals.getNode("/engines/engine[0]/rpm", 1);
var eng_rpm_chkd	= props.globals.initNode("/engines/engine[0]/rpm-checked", 0.0, "DOUBLE");

var genfail = props.globals.initNode("/controls/electric/generator-fail", 0, "BOOL");

var eng_elec_consumers = {
	spindle:	electrical.initNode("outputs/spindle-motor", 0.0, "DOUBLE"),
	ignition:	electrical.initNode("outputs/ignition", 0.0, "DOUBLE"),
	starter:	electrical.initNode("outputs/starter", 0.0, "DOUBLE"),
	ilec:		electrical.initNode("outputs/ilec", 0.0, "DOUBLE"),
};

##
# Initialize internal values
#

var avbus_volts = 0.0;
var engbus_volts = 0.0;

var ammeter_ave = 0.0;


##
# Battery model class.
#

var BatteryClass = {
	# Constructor
	new: func( ideal_volts, ideal_amps, amp_hours, charge_amps, n ){
		var charge_prop	= batt_prop.getNode( "charge["~n~"]" );
		var charge	= nil;
		if( getprop("/systems/electrical/battery/charge["~n~"]") != nil ){	# If the battery charge has been set from a previous FG instance
			charge = charge_prop.getDoubleValue();
		} else {
			charge = 1.0;
			charge_prop = batt_prop.initNode("charge["~n~"]", 1.0, "DOUBLE");
		}
		var obj = {
			parents: [BatteryClass],
			ideal_volts: ideal_volts,
			ideal_amps: ideal_amps,
			amp_hours: amp_hours,
			charge_amps: charge_amps,
			charge: charge,
			charge_prop: charge_prop,
			n: n
		};
		return obj;
	},
	# Passing in positive amps means the battery will be discharged.
	# Negative amps indicates a battery charge.
	apply_load: func( amps, dt ){
		var old_charge = me.charge_prop.getDoubleValue();
		if( freeze_replay.getBoolValue() ){
			return me.amp_hours * old_charge;
		}
		var amphrs_used = amps * dt / 3600.0;
		var fraction_used = amphrs_used / me.amp_hours;

		var new_charge = std.max(0.0, std.min(old_charge - fraction_used, 1.0));

		if (new_charge < 0.1 and old_charge >= 0.1)
			gui.popupTip("Warning: Low battery! Recharge your battery!", 10);
		me.charge = new_charge;
		me.charge_prop.setDoubleValue( new_charge );
		return me.amp_hours * new_charge;
	},
	# Return output volts based on percent charged.  Currently based on a simple
	# polynomial percent charge vs. volts function.
	get_output_volts: func() {
		var x = 1.0 - me.charge;
		var tmp = -(3.0 * x - 1.0);
		var factor = ( math.pow( tmp, 5) + 32 ) / 32;
		return me.ideal_volts * factor;
	},
	# Return output amps available.  This function is totally wrong and should be
	# fixed at some point with a more sensible function based on charge percent.
	# There is probably some physical limits to the number of instantaneous amps
	# a battery can produce (cold cranking amps?)
	get_output_amps: func() {
		var x = 1.0 - me.charge;
		var tmp = -(3.0 * x - 1.0);
		var factor = ( math.pow( tmp, 5) + 32) / 32;
		return me.ideal_amps * factor;
	},
	# Set the current charge instantly to 100 %.
	reset_to_full_charge: func() {
		me.apply_load(-(1.0 - me.charge) * me.amp_hours, 3600);
	},
	# Get current charge
	get_charge: func() {
		return me.charge;
	}
};

var battery_main = BatteryClass.new(12.0, 0.325, 7.2, 25, 0);
var battery_eng = BatteryClass.new(12.0, 0.325, 24, 25, 1); #2*12Ah

##
# Alternator model class.
#
var AlternatorClass = {
	new: func ( rpm_source, rpm_threshold, ideal_volts, ideal_amps, n = 0, switch = nil ) {
		var obj = {
			parents : [AlternatorClass],
			rpm_source : props.globals.getNode( rpm_source, 1 ),
			rpm_threshold : rpm_threshold,
			ideal_volts : ideal_volts,
			ideal_amps : ideal_amps,
			amps_prop: electrical.initNode("generators["~ n ~"]/amps", 0.0, "DOUBLE"),
			switch: switch,
		};
		obj.rpm_source.setDoubleValue( 0.0 );
		return obj;
	},
	apply_load: func( amps ){
		if( me.switch != nil and !me.switch.getBoolValue() ) return 0.0;

		var dt = delta_sec.getDoubleValue();

		me.amps_prop.setDoubleValue( amps );

		# Computes available amps and returns remaining amps after load is applied
		# Scale alternator output for rpms < 800.  For rpms >= 800
		# give full output.  This is just a WAG, and probably not how
		# it really works but I'm keeping things "simple" to start.
		var factor = me.rpm_source.getDoubleValue() / me.rpm_threshold;
		if ( factor > 1.0 ) {
			factor = 1.0;
		}

		# print( "alternator amps = ", me.ideal_amps * factor );
		var available_amps = me.ideal_amps * factor;
		return available_amps - amps;
	},
	get_output_volts: func {
		if( me.switch != nil and !me.switch.getBoolValue() ) return 0.0;

		# Return output volts based on rpm

		# scale alternator output for rpms < 800.  For rpms >= 800
		# give full output.  This is just a WAG, and probably not how
		# it really works but I'm keeping things "simple" to start.
		var rpm = me.rpm_source.getDoubleValue();
		if( rpm < 100 ){
			return 0.0;
		}
		var factor = rpm / me.rpm_threshold;
		if ( factor > 1.0 ) {
			factor = 1.0;
		}
		# print( "alternator volts = ", me.ideal_volts * factor );
		return me.ideal_volts * factor;
	},
	get_output_amps: func {
		if( me.switch != nil and !me.switch.getBoolValue() ) return 0.0;

		# Return output amps available based on rpm.

		# scale alternator output for rpms < 800.  For rpms >= 800
		# give full output.  This is just a WAG, and probably not how
		# it really works but I'm keeping things "simple" to start.
		var factor = me.rpm_source.getDoubleValue() / me.rpm_threshold;
		if ( factor > 1.0 ) {
			factor = 1.0;
		}

		# print( "alternator amps = ", ideal_amps * factor );
		return me.ideal_amps * factor;
	},

};

var alternator = AlternatorClass.new( "/engines/engine[0]/rpm-checked", 2200, 28.0, 60.0 );

var recharge_battery = func {
	# Charge battery to 100 %
	battery_main.reset_to_full_charge();
	battery_eng.reset_to_full_charge();
}
##
# This is the main electrical system update function.
#

var ElectricalSystemUpdater = {
	new : func {
		var m = {
			parents: [ElectricalSystemUpdater]
		};
		# Request that the update function be called each frame
		m.loop = updateloop.UpdateLoop.new(components: [m], update_period: 0.0, enable: 0);
		return m;
	},

	enable: func {
		me.loop.reset();
		me.loop.enable();
	},

	disable: func {
		me.loop.disable();
	},

	reset: func {
		# Do nothing
	},

	update: func (dt) {
		update_avionic_bus(dt);
		update_engine_bus(dt);
	}
};

##
# Model the system of relays and connections that join the battery,master/alt switches.
#

var update_avionic_bus = func (dt) {
	var load = 0.0;
	var battery_volts = 0.0;
	if ( electric_serviceable.getBoolValue() ) {
		battery_volts = battery_main.get_output_volts();
	}

	# determine power source
	var bus_volts=0.0;
	var power_source = nil;
	if ( electrical_switches.battery.getBoolValue() ){
		var bus_volts = battery_volts;
		var power_source = "battery";
	}
	#print( "virtual bus volts = ", bus_volts );

	# bus network (1. these must be called in the right order, 2. the
	# bus routine itself determins where it draws power from.)
	load += electrical_bus_1();

	# swtich the master breaker off if load is out of limits
	if ( load > 55 ) {
		bus_volts = 0;
	}

	# system loads and ammeter gauge
	var ammeter = 0.0;
	if ( bus_volts > 1.0 ) {
		# ammeter gauge
		if ( power_source == "battery" ) {
			ammeter = -load;
		} else {
			ammeter = battery.charge_amps;
		}
	}
	# print( "ammeter = ", ammeter );

	# charge/discharge the battery
	if ( power_source == "battery" ) {
		battery_main.apply_load( load, dt );
	} elsif ( bus_volts > battery_volts ) {
		battery_main.apply_load( -battery_main.charge_amps, dt );
	}

	# filter ammeter needle pos
	ammeter_ave = 0.8 * ammeter_ave + 0.2 * ammeter;

	# outputs
	amps_p.setDoubleValue( ammeter_ave );
	volts_p.setDoubleValue( bus_volts );
	load_p.setDoubleValue( load );

	if (bus_volts > 9)
		avbus_volts = bus_volts;
	else
		avbus_volts = 0.0;

	return load;
}


#Load sources:
#	com:			https://www.skyfox.com/becker-ar6201-022-vhf-am-sprechfunkgeraet-8-33.html
#	vario:		http://www.ilec-gmbh.com/ilec/manuals/SC7pd.pdf
#	turn:			https://www.airteam.eu/de/p/falcon-tb02e-2-1 (not the same but similar)
#	transponder:	no data sheet used, the 6W are an estimate
#	flarm:		http://flarm.com/wp-content/uploads/man/FLARM_InstallationManual_D.pdf
#	flarm display:	https://www.air-store.eu/Display-V3-FLARM
#	wind compass:	https://cdn.shopify.com/s/files/1/0635/7129/6477/files/IOM_anemoi_v1.05.pdf?v=1657134707
var electrical_bus_1 = func() {
	var bus_volts = 0.0;
	var load = 0.0;
	bus_volts = avbus_volts;
	#print("Bus volts: ", bus_volts);

	if(bus_volts > 9){

		# Vario
		electrical_consumers.ilec_sc7.setDoubleValue( bus_volts );
		#LXNAV V8: 160 mA at 12V (LX90xx/LX80xx manual, technical data); the vario sound is fed by the same unit
		load += 1.9 / bus_volts;
		#Energy consumption of the audio (guess, from the ILEC): 25mA (medium volume) 60mA (max volume)
		if ((getprop("/instrumentation/lxv8/audio/volume") or 0) > 0.001) {
			load += (vario_vol.getValue()*0.66) / bus_volts;
		}


		# Radio
		electrical_consumers.comm.setDoubleValue( bus_volts );
		if(com_ptt.getBoolValue() and com_start.getValue()==1){
			load += 19.2 / bus_volts;
		} else {
			load += 1.02*com_start.getValue() / bus_volts;
		}

		#Turn Coordinator
		#Energy Consumption:
		#	starting ~9.9W (approx)
		#	running ~7.8W (approx)
		if ( electrical_switches.turnslip.getBoolValue() and !transponder_installed.getBoolValue() ) {
			electrical_consumers.turn.setDoubleValue( bus_volts );
			if( turnbank_spin.getValue() > 0.99 ){
				load += 7.8 / bus_volts;
			}else{
				load += 9.9 / bus_volts;
			}
		} else {
			electrical_consumers.turn.setDoubleValue( 0.0 );
		}

		# Transponder (guess: about 6W, the BXP 6401 is specified with less in standby)
		if ( transponder_installed.getBoolValue() ) {
			electrical_consumers.transponder.setDoubleValue( bus_volts );
			load += 6.0 / bus_volts;
		} else {
			electrical_consumers.transponder.setDoubleValue( 0.0 );
		}

		electrical_consumers.flarm.setDoubleValue( bus_volts );
		load += 0.66 / bus_volts; #FLARM
		load += 0.12 / bus_volts; #FLARM display

		if( windcompass_installed.getBoolValue() ){
			electrical_consumers.wind_comp.setDoubleValue( bus_volts );
			load += 1.2 / bus_volts; # 100mA at 12VDC
		} else {
			electrical_consumers.wind_comp.setDoubleValue( 0.0 );
		}
	} else {
		foreach( var el; keys(electrical_consumers) ){
			electrical_consumers[el].setDoubleValue( 0.0 );
		}
	}

	# return cumulative load
	return load;
}

var update_engine_bus = func (dt) {
	var load = 0.0;
	var battery_volts = 0.0;
	if ( electric_serviceable.getBoolValue() ) {
		battery_volts = battery_eng.get_output_volts();
		alternator_volts = alternator.get_output_volts();
	}

	# switch state
	var master_bat = electrical_switches.battery.getBoolValue();

	if(eng_prop_pos.getValue()==1){
		eng_rpm_chkd.setDoubleValue( eng_rpm.getDoubleValue() );
	} else {
		eng_rpm_chkd.setDoubleValue( 0.0 );
	}

	# determine power source
	var bus_volts=0.0;
	var power_source = nil;
	if (master_bat){
		var bus_volts = battery_volts;
		var power_source = "battery";
	}
	if ( alternator_volts > bus_volts and !genfail.getBoolValue() ) {
		bus_volts = alternator_volts;
		power_source = "alternator";
	}
	#print( "virtual bus volts = ", bus_volts );

	# bus network (1. these must be called in the right order, 2. the
	# bus routine itself determins where it draws power from.)
	load += engine_bus_1();

	# swtich the master breaker off if load is out of limits
	if ( load > 55 ) {
		bus_volts = 0;
	}

	# system loads and ammeter gauge
	var ammeter = 0.0;
	if ( bus_volts > 1.0 ) {
		# ammeter gauge
		if ( power_source == "battery" ) {
			ammeter = -load;
		} else {
			ammeter = battery_eng.charge_amps;
		}
	}
	# print( "ammeter = ", ammeter );

	# charge/discharge the battery
	if ( power_source == "battery" ) {
		battery_eng.apply_load( load, dt );
	} elsif ( bus_volts > battery_volts ) {
		battery_eng.apply_load( -battery_eng.charge_amps, dt );
	}

	# filter ammeter needle pos
	ammeter_ave = 0.8 * ammeter_ave + 0.2 * ammeter;

	# outputs
	eng_amps_p.setDoubleValue( ammeter_ave );
	eng_volts_p.setDoubleValue( bus_volts );
	eng_load_p.setDoubleValue( load );

	if (bus_volts > 9)
		engbus_volts = bus_volts;
	else
		engbus_volts = 0.0;

	return load;
}

var engine_bus_1 = func() {
	var bus_volts = 0.0;
	var load = 0.0;
	bus_volts = engbus_volts;

	if( bus_volts > 5 ){

		#Prop in/out
		if ( ( eng_prop_cmd.getValue() == 1 and eng_prop_pos.getValue() != 1 ) or ( eng_prop_cmd.getValue() == 0 and eng_prop_pos.getValue() != 0 ) )  {
			eng_elec_consumers.spindle.setDoubleValue( bus_volts );
			load += 50 / bus_volts; # guess: 50W
		} else {
			eng_elec_consumers.spindle.setDoubleValue( 0.0 );
		}

		#Ignition
		if ( eng_battswitch.getBoolValue() and eng_magneto.getValue()==3 ) {
			eng_elec_consumers.ignition.setDoubleValue( bus_volts );
			if(bus_volts > 12.5){
				eng_mixture.setValue(1);
			}else{
				eng_mixture.setValue(0);
			}
			load += 10 / bus_volts; # guess: 10W
		} else {
			eng_elec_consumers.ignition.setDoubleValue( 0.0 );
		}

		#Starter
		if ( eng_battswitch.getBoolValue() and eng_starter.getBoolValue() and eng_prop_pos.getDoubleValue() > 0.99 ) {
			eng_elec_consumers.starter.setDoubleValue( bus_volts );
			eng_starter_int.setBoolValue( 1 );
			load += 600 / bus_volts; # 0.6 kW at 12V (ref. IAE-50R-AA manual, p.19-24)
		} else {
			eng_elec_consumers.starter.setDoubleValue(0.0 );
			eng_starter_int.setBoolValue(0 );
		}

		eng_elec_consumers.ilec.setDoubleValue( bus_volts );
		load += 1.0 / bus_volts; # guess: 1W
	} else {
		foreach( var el; keys(eng_elec_consumers) ){
			eng_elec_consumers[el].setDoubleValue( 0.0 );
		}
	}

	# return cumulative load
	return load;
}
##
# Initialize the electrical system
#

var system_updater = ElectricalSystemUpdater.new();

setlistener("/sim/signals/fdm-initialized", func {
	# checking if battery should be automatically recharged
	if (!getprop("/systems/electrical/save-battery-charge")) {
		battery_main.reset_to_full_charge();
		battery_eng.reset_to_full_charge();
	};

	system_updater.enable();
	print("Electrical system initialized");
});
