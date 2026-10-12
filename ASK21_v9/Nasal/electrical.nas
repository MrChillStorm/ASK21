##
# Procedural model of an ASK21 electrical system.  Includes a
# preliminary battery charge/discharge model and realistic ammeter
# gauge modeling.
#
# Based on C172P electrical system.

# Initialize properties

##
# Initialize internal values
#

var vbus_volts = 0.0;

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

		if (new_charge < 0.1 and old_charge_percent >= 0.1)
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

##	example glider battery: https://shop.segelflugbedarf24.de/Flugzeugausstattung/Akkus-Energieversorgung/Sonnenschein-Dryfit-A512-12V/6-5-Ah::731.html
##				http://www.sonnenschein.org/A500.htm	(A512-6.5S)
##				ideal volts: 12.0
##				ideal amps: 0.325 (max. 80 / 300 for 5 sec))
##				amp hours: 6.5
##				charge amps: 25

var	battery = BatteryClass.new( 12.0, 0.325, 6.5, 25, 0);

var recharge_battery = func {
	# Charge battery to 100 %
	battery.reset_to_full_charge();
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
		update_virtual_bus(dt);
	}
};

##
# Model the system of relays and connections that join the battery,master/alt switches.
#

var update_virtual_bus = func (dt) {
	var load = 0.0;
	var battery_volts = 0.0;
	if ( electric_serviceable.getBoolValue() ) {
		battery_volts = battery.get_output_volts();
	}

	# determine power source
	if ( electrical_switches.battery.getBoolValue() ){
		var bus_volts = battery_volts;
		var power_source = battery;
	}else{
		var bus_volts = 0.0;
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
	battery.apply_load( load, dt );

	# filter ammeter needle pos
	ammeter_ave = 0.8 * ammeter_ave + 0.2 * ammeter;

	# outputs
	amps_p.setDoubleValue( ammeter_ave );
	volts_p.setDoubleValue( bus_volts );
	load_p.setDoubleValue( load );

	if (bus_volts > 9)
		vbus_volts = bus_volts;
	else
		vbus_volts = 0.0;

	return load;
}


#Load sources:
#	com:			https://www.skyfox.com/becker-ar6201-022-vhf-am-sprechfunkgeraet-8-33.html
#	vario:		http://www.ilec-gmbh.com/ilec/manuals/SC7pd.pdf
#	turn:			https://www.airteam.eu/de/p/falcon-tb02e-2-1 (not the same but similar)
#				https://www.mcico.com/Assets/products/TechnicalDrawings/5550-8340N3LrevF_td.pdf
#	transponder:	no data sheet used, the 6W are an estimate
#	flarm:		http://flarm.com/wp-content/uploads/man/FLARM_InstallationManual_D.pdf
#	flarm display:	https://www.air-store.eu/Display-V3-FLARM
#	wind compass:	https://cdn.shopify.com/s/files/1/0635/7129/6477/files/IOM_anemoi_v1.05.pdf?v=1657134707

var electrical_bus_1 = func() {
	var bus_volts = 0.0;
	var load = 0.0;

	bus_volts = vbus_volts;       
	
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
		#	starting ~9.9 / volts (approx)
		#	running ~7.8 / volts (approx)
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

##
# Initialize the electrical system
#

var system_updater = ElectricalSystemUpdater.new();

setlistener("/sim/signals/fdm-initialized", func {
	# checking if battery should be automatically recharged
	if (!getprop("/systems/electrical/save-battery-charge")) {
		battery.reset_to_full_charge();
	};

	system_updater.enable();
	print("Electrical system initialized");
});
