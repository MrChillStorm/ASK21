# ####################################################################################
# ####################################################################################
# Nasal script to handle drag robot, in case there is no other dragger
#
# ####################################################################################
# Author:  Klaus Kerner
# Version: 2012-03-08
# improved by Bea Wolf (D-ECHO) in 2019
#
# ####################################################################################
# Concepts:
# 1. search for allready existing dragger in the property tree                  done
# 2. if an dragger does not exist, create a new one                             done
# 3. place dragger in front of glider                                           done
# 4. run the dragger up into the sky                                       mostly done
# 5. after releasing tow reset the dragger                                 mostly done
# 6. allow redefining presets                                                   done

# ####################################################################################

# possible models for the Dragger
var models = { 	DR400 : 	'AI/Aircraft/DR400/Models/dr400-ai.xml', 
		PA28 : 		'AI/Aircraft/pa-28/pa-28-d-emlh.xml', 
		Ercoupe : 	'AI/Aircraft/ercoupe/Models/ercoupe-ai.xml', 
		Staggerwing : 	'AI/Aircraft/Beechcraft-Staggerwing/Models/model17-ai.xml'
		};

# order in which installed models are tried when the selected one is missing
var model_order = ["DR400", "PA28", "Ercoupe", "Staggerwing"];

var dragger_model_node = nil;              # the node of the dragger's 3d model, removed with it

var sent = 0;
var last_leg = 0;
var saved_roll_deg = nil;                  # settings overridden for the final descent,
var saved_turnrate_degs = nil;             # restored when the dragger is removed
var saved_lift_mps = nil;
    
    
# drag-robot dialog: helper function to run the roboter, avoiding race conditions
var guiRunDragRobot = func {
	# the tow hitch can only find the dragger if it exists, so create it when needed
	if ( getprop("sim/glider/dragger/flags/run") != 1 and checkDragger() < 0 ) {
		if ( !setupDragRobot() ) {
			return;
		}
	}
	towing.findBestAIObject();
	#hookDragger();  
	setprop("controls/gear/assist-1",1);                    # level the plane
	ask21.msg("atc","Glider levelled."); 
	startDragRobot();
}

var print_straight= func (leg_distance_m) {	
    ask21.msg("ai-plane", "straight ahead for " ~ math.round(leg_distance_m) ~ "m");
}
var print_turn = func (direction, degrees) {
    ask21.msg("ai-plane", "turn " ~ direction ~ " " ~ math.round(degrees) ~ " deg");
}

# ####################################################################################
# drag-robot dialog: helper function to cancel the roboter, avoiding race conditions
var guiCancelDragRobot = func {
    # do not leave the glider hooked to a dragger that is about to vanish
    towing.releaseHitch("aerotow");
    removeDragRobot();
    resetRobotAttributes();
    #removeTowingRope();
}


# ####################################################################################
# findDragger
# used by key 
# used by gui, when dealing with drag roboter
# the first found plane, that is close enough and has callsign "dragger" will be used
var findDragger = func {
  
	# local variables
	var aiobjects = [];                     # keeps the ai-planes from the property tree
	var dragid = 0;                         # id of dragger
	var callsign = 0;                       # callsign of dragger
	var cur = geo.Coord.new();              # current processed ai-plane
	var lat_deg = 0;                        # latitude of current processed aiobject
	var lon_deg = 0;                        # longitude of current processed aiobject
	var alt_m = 0;                          # altitude of current processed aiobject
	var glider = geo.Coord.new();           # coordinates of glider
	var distance_m = 0;                     # distance to ai-plane
	
	
	var towlength_m = getprop("sim/glider/towing/conf/rope_length_m");
	
	
	aiobjects = props.globals.getNode("ai/models").getChildren(); 
	glider = geo.aircraft_position(); 
	
	foreach (var aimember; aiobjects) { 
		if ( (var c = aimember.getNode("callsign") ) != nil ) { 
			callsign = c.getValue();
			dragid = aimember.getNode("id").getValue();
			if ( callsign == "dragger" ) {
				lat_deg = aimember.getNode("position/latitude-deg").getValue(); 
				lon_deg = aimember.getNode("position/longitude-deg").getValue(); 
				alt_m = aimember.getNode("position/altitude-ft").getValue() * FT2M; 
				
				cur = geo.Coord.set_latlon( lat_deg, lon_deg, alt_m ); 
				distance_m = (glider.distance_to(cur)); 
				
				if ( distance_m < towlength_m ) { 
					ask21.msg("atc", sprintf("dragger with id %s nearby in %.0f m", dragid, distance_m));
					setprop("sim/glider/towing/dragid", dragid); 
					break; 
				}else {
					ask21.msg("atc", sprintf("dragger with id %s too far at %.0f m", dragid, distance_m));
				}
			}
		} else {
			ask21.msg("atc","no dragger found");
		}
	}
  
} # End Function findDragger


# ####################################################################################
# hookDragger
# used by key
# used by gui
var hookDragger = func {
  
	# if dragid > 0
	#  set property /fdm/jsbsim/fcs/dragger-cmd-norm
	#  level plane
	
	if ( getprop("sim/glider/towing/dragid") != nil ) { 
		createTowingRope();                                           # create rope model
		setprop("fdm/jsbsim/fcs/dragger-cmd-norm", 1);                # closes the hook
		setprop("sim/glider/towing/hooked", 1); 
		ask21.msg("atc","hook closed"); 
		setprop("orientation/roll-deg", 0); 
		ask21.msg("atc","glider leveled"); 
	} else { 
		ask21.msg("atc","no dragger nearby"); 
	}
  
} # End Function hookDragger

# ####################################################################################
# ## new properties in the property tree
# ai/models/dragger
# ai/models/dragger/id
# ai/models/dragger/callsign
# ai/models/dragger/valid
# ai/models/dragger/position/latitude-deg
# ai/models/dragger/position/longitude-deg
# ai/models/dragger/position/altitude-ft
# ai/models/dragger/orientation/true-heading-deg
# ai/models/dragger/orientation/pitch-deg
# ai/models/dragger/orientation/roll-deg
# ai/models/dragger/velocities/true-airspeed-kt
# ai/models/dragger/velocities/vertical-speed-fps
# models/model[id_model]/path
# models/model[id_model]/longitude-deg-prop
# models/model[id_model]/latitude-deg-prop
# models/model[id_model]/elevation-ft-prop
# models/model[id_model]/heading-deg-prop
# sim/glider/dragger/flags/exist                flag for existence of robot
# sim/glider/dragger/flags/run                  flag for triggering operation
# sim/glider/dragger/robot/id_AI
# sim/glider/dragger/robot/id_model
# sim/glider/dragger/robot/wp0lat_deg           wp0 reference point for different legs
# sim/glider/dragger/robot/wp0lon_deg
# sim/glider/dragger/robot/wp0alt_m
# sim/glider/dragger/robot/wp0head_deg
# sim/glider/dragger/robot/exit_height_m        exit height for stopping robot
# sim/glider/dragger/robot/anchorlat_deg        anchor for checking leg position
# sim/glider/dragger/robot/anchorlon_deg
# sim/glider/dragger/robot/anchoralt_m
# sim/glider/dragger/robot/leg_type             storing type of leg, 0 start, 
#                                                                    1 turn, 
#                                                                    2 straight
#                                                                    3 end 
# sim/glider/dragger/robot/leg_distance_m       target distance in straight leg
# sim/glider/dragger/robot/leg_angle_deg        target turn angle in turn leg
# sim/glider/dragger/robot/turnside             1: right turn, 0: left turn
# sim/glider/dragger/robot/leg_segment          storing segment of leg0, 
#                                                                    0 tauten, 
#                                                                    1 acceleration
#                                               storing segment of leg1
#                                                                    2 roll in 
#                                                                    3 keep roll angle
#                                                                    4 roll out
# sim/glider/dragger/conf/glob_min_speed_takeoff_mps
#                .../glob/glob_min_speed_takeoff_mps
# sim/glider/dragger/conf/glob_max_speed_mps
#                .../glob/glob_max_speed_mps
# sim/glider/dragger/conf/glob_max_speed_lift_mps
#                .../glob/glob_max_speed_lift_mps
# sim/glider/dragger/conf/glob_max_speed_tauten_mps
#                .../glob/glob_max_speed_tauten_mps
# sim/glider/dragger/conf/glob_min_acceleration_mpss
#                .../glob/glob_min_acceleration_mpss
# sim/glider/dragger/conf/glob_max_acceleration_mpss
#                .../glob/glob_max_acceleration_mpss
# sim/glider/dragger/conf/glob_max_roll_deg
#                .../glob/glob_max_roll_deg
# sim/glider/dragger/conf/glob_max_rollrate_degs
#                .../glob/glob_max_rollrate_degs
# sim/glider/dragger/conf/glob_max_turnrate_degs
#                .../glob/glob_max_turnrate_degs
# sim/glider/dragger/conf/glob_max_lift_height_m
#                .../glob/glob_max_lift_height_m
# sim/glider/dragger/conf/glob_max_tautendist_m
#                .../glob/glob_max_tautendist_m

# ## used properties from the property tree
# environment/wind-from-north-fps
# environment/wind-from-east-fps
# environment/wind-from-down-fps
# orientation/heading-deg
# sim/glider/dragger/hooked



# ####################################################################################
# global variables:
var dragrobot_timeincrement_s = 0;                     # timer increment



# ####################################################################################
# set drag roboter parameters to global values, if not properly defined by plane 
# setup-file
# store global values or plane-specific values to prepare for reset option
var initRobotAttributes = func {
	# the dialog list only sets the model once the user clicks an entry
	if ( getprop("sim/glider/dragger/model") == nil ) {
		setprop("sim/glider/dragger/model", "DR400");
	}
	# SI units for the advanced dialog
	# constants for describing dragger attributes, if not defined by plane setup-file
	var glob_min_speed_takeoff_mps  = 27.8;     # speed at which the dragger leaves the ground (100 km/h)
	var glob_max_speed_mps          = 36;       # max. speed of drag-robot (130 km/h)
	var glob_max_speed_lift_mps     = 2.5;      # max. climb rate of drag-robot
	var glob_max_speed_tauten_mps   = 3;        # max. speed to tauten the rope
	var glob_min_acceleration_mpss  = 0.5;      # min. acceleration
	var glob_max_acceleration_mpss  = 1.3;      # max. acceleration
	var glob_max_roll_deg           = 20;       # max. roll angle
	var glob_max_rollrate_degs      = 5;        # max. roll rate per second
	var glob_max_turnrate_degs      = 3;        # max. turn rate per second 
							# at max roll angle
	var glob_max_lift_height_m      = 500;      # max. lift height over start point (default: 500 m tow)
	var glob_max_tautendist_m       = 50;       # max. distance for tauten the rope
	var glob_climb_start_speed_mps  = 31.9;     # speed up to which the dragger stays low (115 km/h)
	var glob_low_height_m           = 1.5;      # height kept above ground while staying low
	var glob_climb_tau_s            = 6;        # time constant for building up the climb rate
	if ( getprop("sim/glider/dragger/conf/terrain_guard") == nil ) {
		setprop("sim/glider/dragger/conf/terrain_guard", 1);       # turn away from terrain ahead
	}
	if ( getprop("sim/glider/dragger/conf/tow_type") == nil ) {
		setprop("sim/glider/dragger/conf/tow_type", "500 m tow");
	}
	
	if ( getprop("sim/glider/dragger/conf/glob_min_speed_takeoff_mps") == nil ) {
		setprop("sim/glider/dragger/conf/glob_min_speed_takeoff_mps", glob_min_speed_takeoff_mps);
		setprop("sim/glider/dragger/glob/glob_min_speed_takeoff_mps", glob_min_speed_takeoff_mps);
	} else {
		setprop("sim/glider/dragger/glob/glob_min_speed_takeoff_mps", getprop("sim/glider/dragger/conf/glob_min_speed_takeoff_mps"));
	}
	
	if ( getprop("sim/glider/dragger/conf/glob_max_speed_mps") == nil ) {
		setprop("sim/glider/dragger/conf/glob_max_speed_mps", glob_max_speed_mps);
		setprop("sim/glider/dragger/glob/glob_max_speed_mps", glob_max_speed_mps);
	} else {
		setprop("sim/glider/dragger/glob/glob_max_speed_mps", getprop("sim/glider/dragger/conf/glob_max_speed_mps"));
	}
	
	if ( getprop("sim/glider/dragger/conf/glob_max_speed_lift_mps") == nil ) {
		setprop("sim/glider/dragger/conf/glob_max_speed_lift_mps", glob_max_speed_lift_mps);
		setprop("sim/glider/dragger/glob/glob_max_speed_lift_mps", glob_max_speed_lift_mps);
	} else {
		setprop("sim/glider/dragger/glob/glob_max_speed_lift_mps", getprop("sim/glider/dragger/conf/glob_max_speed_lift_mps"));
	}
	
	if ( getprop("sim/glider/dragger/conf/glob_max_speed_tauten_mps") == nil ) {
		setprop("sim/glider/dragger/conf/glob_max_speed_tauten_mps", glob_max_speed_tauten_mps);
		setprop("sim/glider/dragger/glob/glob_max_speed_tauten_mps", glob_max_speed_tauten_mps);
	} else { 
		setprop("sim/glider/dragger/glob/glob_max_speed_tauten_mps", getprop("sim/glider/dragger/conf/glob_max_speed_tauten_mps"));
	}
	
	if ( getprop("sim/glider/dragger/conf/glob_min_acceleration_mpss") == nil ) {
		setprop("sim/glider/dragger/conf/glob_min_acceleration_mpss", glob_min_acceleration_mpss);
		setprop("sim/glider/dragger/glob/glob_min_acceleration_mpss", glob_min_acceleration_mpss);
	} else {
		setprop("sim/glider/dragger/glob/glob_min_acceleration_mpss", getprop("sim/glider/dragger/conf/glob_min_acceleration_mpss"));
	}
	
	if ( getprop("sim/glider/dragger/conf/glob_max_acceleration_mpss") == nil ) {
	setprop("sim/glider/dragger/conf/glob_max_acceleration_mpss", 
		glob_max_acceleration_mpss);
	setprop("sim/glider/dragger/glob/glob_max_acceleration_mpss", 
		glob_max_acceleration_mpss);
	}
	else {
	setprop("sim/glider/dragger/glob/glob_max_acceleration_mpss", 
		getprop("sim/glider/dragger/conf/glob_max_acceleration_mpss"));
	}
	
	if ( getprop("sim/glider/dragger/conf/glob_max_roll_deg") == nil ) {
	setprop("sim/glider/dragger/conf/glob_max_roll_deg", 
		glob_max_roll_deg);
	setprop("sim/glider/dragger/glob/glob_max_roll_deg", 
		glob_max_roll_deg);
	}
	else {
	setprop("sim/glider/dragger/glob/glob_max_roll_deg", 
		getprop("sim/glider/dragger/conf/glob_max_roll_deg"));
	}
	
	if ( getprop("sim/glider/dragger/conf/glob_max_rollrate_degs") == nil ) {
	setprop("sim/glider/dragger/conf/glob_max_rollrate_degs", 
		glob_max_rollrate_degs);
	setprop("sim/glider/dragger/glob/glob_max_rollrate_degs", 
		glob_max_rollrate_degs);
	}
	else {
	setprop("sim/glider/dragger/glob/glob_max_rollrate_degs", 
		getprop("sim/glider/dragger/conf/glob_max_rollrate_degs"));
	}
	
	if ( getprop("sim/glider/dragger/conf/glob_max_turnrate_degs") == nil ) {
	setprop("sim/glider/dragger/conf/glob_max_turnrate_degs", 
		glob_max_turnrate_degs);
	setprop("sim/glider/dragger/glob/glob_max_turnrate_degs", 
		glob_max_turnrate_degs);
	}
	else {
	setprop("sim/glider/dragger/glob/glob_max_turnrate_degs", 
		getprop("sim/glider/dragger/conf/glob_max_turnrate_degs"));
	}
	
	if ( getprop("sim/glider/dragger/conf/glob_max_lift_height_m") == nil ) {
	setprop("sim/glider/dragger/conf/glob_max_lift_height_m", 
		glob_max_lift_height_m);
	setprop("sim/glider/dragger/glob/glob_max_lift_height_m", 
		glob_max_lift_height_m);
	}
	else {
	setprop("sim/glider/dragger/glob/glob_max_lift_height_m", 
		getprop("sim/glider/dragger/conf/glob_max_lift_height_m"));
	}
	
	if ( getprop("sim/glider/dragger/conf/glob_max_tautendist_m") == nil ) {
	setprop("sim/glider/dragger/conf/glob_max_tautendist_m", 
		glob_max_tautendist_m);
	setprop("sim/glider/dragger/glob/glob_max_tautendist_m", 
		glob_max_tautendist_m);
	}
	else {
	setprop("sim/glider/dragger/glob/glob_max_tautendist_m", 
		getprop("sim/glider/dragger/conf/glob_max_tautendist_m"));
	}
	
	if ( getprop("sim/glider/dragger/conf/glob_climb_start_speed_mps") == nil ) {
		setprop("sim/glider/dragger/conf/glob_climb_start_speed_mps", glob_climb_start_speed_mps);
		setprop("sim/glider/dragger/glob/glob_climb_start_speed_mps", glob_climb_start_speed_mps);
	} else {
		setprop("sim/glider/dragger/glob/glob_climb_start_speed_mps", getprop("sim/glider/dragger/conf/glob_climb_start_speed_mps"));
	}
	
	if ( getprop("sim/glider/dragger/conf/glob_low_height_m") == nil ) {
		setprop("sim/glider/dragger/conf/glob_low_height_m", glob_low_height_m);
		setprop("sim/glider/dragger/glob/glob_low_height_m", glob_low_height_m);
	} else {
		setprop("sim/glider/dragger/glob/glob_low_height_m", getprop("sim/glider/dragger/conf/glob_low_height_m"));
	}
	
	if ( getprop("sim/glider/dragger/conf/glob_climb_tau_s") == nil ) {
		setprop("sim/glider/dragger/conf/glob_climb_tau_s", glob_climb_tau_s);
		setprop("sim/glider/dragger/glob/glob_climb_tau_s", glob_climb_tau_s);
	} else {
		setprop("sim/glider/dragger/glob/glob_climb_tau_s", getprop("sim/glider/dragger/conf/glob_climb_tau_s"));
	}
	
	# km/h mirrors for the advanced dialog, which shows speeds in km/h
	var mirror = func(src, dst) {
		var update = func { setprop("sim/glider/gui/dragrobot/" ~ dst, (getprop("sim/glider/dragger/conf/" ~ src) or 0) * 3.6); }
		setlistener("sim/glider/dragger/conf/" ~ src, update, 1, 0);
	}
	mirror("glob_min_speed_takeoff_mps", "min_speed_takeoff");
	mirror("glob_climb_start_speed_mps", "climb_start_speed");
	mirror("glob_max_speed_mps", "max_speed");
	mirror("glob_max_speed_tauten_mps", "max_speed_tauten");
}



# ####################################################################################
# re-initialize presets
var resetRobotAttributes = func {
	# reading all global variables in case they has been changed in the property tree
	setprop("sim/glider/dragger/conf/glob_min_speed_takeoff_mps", 
		getprop("sim/glider/dragger/glob/glob_min_speed_takeoff_mps"));
	setprop("sim/glider/dragger/conf/glob_max_speed_mps",  
		getprop("sim/glider/dragger/glob/glob_max_speed_mps"));
	setprop("sim/glider/dragger/conf/glob_max_speed_lift_mps", 
		getprop("sim/glider/dragger/glob/glob_max_speed_lift_mps"));
	setprop("sim/glider/dragger/conf/glob_max_speed_tauten_mps", 
		getprop("sim/glider/dragger/glob/glob_max_speed_tauten_mps"));
	setprop("sim/glider/dragger/conf/glob_min_acceleration_mpss", 
		getprop("sim/glider/dragger/glob/glob_min_acceleration_mpss"));
	setprop("sim/glider/dragger/conf/glob_max_acceleration_mpss", 
		getprop("sim/glider/dragger/glob/glob_max_acceleration_mpss"));
	setprop("sim/glider/dragger/conf/glob_max_roll_deg", 
		getprop("sim/glider/dragger/glob/glob_max_roll_deg"));
	setprop("sim/glider/dragger/conf/glob_max_rollrate_degs", 
		getprop("sim/glider/dragger/glob/glob_max_rollrate_degs"));
	setprop("sim/glider/dragger/conf/glob_max_turnrate_degs", 
		getprop("sim/glider/dragger/glob/glob_max_turnrate_degs"));
	setprop("sim/glider/dragger/conf/glob_max_lift_height_m", 
		getprop("sim/glider/dragger/glob/glob_max_lift_height_m"));
	setprop("sim/glider/dragger/conf/glob_max_tautendist_m", 
		getprop("sim/glider/dragger/glob/glob_max_tautendist_m"));
	setprop("sim/glider/dragger/conf/glob_climb_start_speed_mps", 
		getprop("sim/glider/dragger/glob/glob_climb_start_speed_mps"));
	setprop("sim/glider/dragger/conf/glob_low_height_m", 
		getprop("sim/glider/dragger/glob/glob_low_height_m"));
	setprop("sim/glider/dragger/conf/glob_climb_tau_s", 
		getprop("sim/glider/dragger/glob/glob_climb_tau_s"));
}



# ####################################################################################
# check for allready available dragger
var checkDragger = func {
  
	#local variables
	var dragid = -1;                              # the allready used id
	var aiobjects = {};                           # vector to keep all ai objects
	
	aiobjects = props.globals.getNode("ai/models", 1).getChildren();  # store AI objects
	foreach ( var aimember; aiobjects ) { 
		# get data from aimember
		if ( (var c = aimember.getNode("callsign")) != nil) {
			var callsign = c.getValue();
			if ( callsign == "dragger" ) {
				dragid = aimember.getNode("id").getValue();
			}
		}
	}
	return(dragid);
}



# ####################################################################################
# get the next free id of ai/models members
var getFreeAIID = func {
  
	#local variables
	var aiid = 0;                                 # for the next unsused id
	var aiobjects = {};                           # vector to keep all ai objects
	
	aiobjects = props.globals.getNode("ai/models", 1).getChildren();  # store AI objects
	foreach ( var aimember; aiobjects ) { 
		# get data from aimember
		if ( (var c = aimember.getNode("id")) != nil) {
			var id = c.getValue();
			if ( aiid <= id ) {
				aiid = id +1;
			} 
		}
	}
	
	# dirty bug-fix for double-used IDs, assign a most probably never reached ID (9999)
	# hopefully this will change in the future with correct ID assignment as the 
	# AI-system does, needs access to flightgear core functions
	#  return(aiid);
	return(9999);
}



# ####################################################################################
# get the next free id of models/model members
var getFreeModelID = func {
  
	# model nodes have no "id" child, so use the node index like the tow rope code does.
	# Starting at 0 every time made the dragger share a node with other models.
	var modelid = 0;                                 # for the next unsused id
	foreach ( var member; props.globals.getNode("models", 1).getChildren() ) { 
		if ( member.getIndex() >= modelid ) {
			modelid = member.getIndex() + 1;
		}
	}
	return(modelid);
}



# ####################################################################################
# find an installed dragger model, preferring the selected one
# returns the model path, or nil if none of the models is installed
var pickDraggerModel = func {
	var selected = getprop("sim/glider/dragger/model");
	var candidates = [selected];
	foreach (var name; model_order) {
		append(candidates, name);
	}
	foreach (var name; candidates) {
		if ( name == nil or models[name] == nil ) {
			continue;
		}
		var path = resolvepath(models[name]);
		if ( path != "" and path != nil ) {
			if ( selected != nil and name != selected ) {
				ask21.msg("atc", "Model " ~ (selected or "(none)") ~ " is not installed, using " ~ name);
			}
			return models[name];
		}
	}
	return nil;
}



# ####################################################################################
# create the drag robot in the ai property tree
# returns 1 on success, 0 if no dragger model is installed
# heading of the runway that the position is on, in the direction closest to `hdg`; nil when it is not on a runway
var runwayHeadingAt = func(pos, hdg) {
	var info = airportinfo(pos.lat(), pos.lon());
	if ( info == nil ) {
		return nil;
	}
	var best = nil;
	var best_across = 1e9;
	var best_diff = 1e9;
	foreach ( var id; keys(info.runways) ) {
		var r = info.runways[id];
		var thr = geo.Coord.new().set_latlon(r.lat, r.lon);
		var d = thr.distance_to(pos);
		var a = (thr.course_to(pos) - r.heading) * D2R;
		var along = d * math.cos(a);
		var across = math.abs(d * math.sin(a));
		var width = (r.width != nil) ? r.width : 30;
		if ( along < -30 or along > r.length + 30 or across > width / 2 + 25 ) {
			continue;
		}
		var diff = math.abs(math.mod(r.heading - hdg + 540, 360) - 180);
		# the closest runway wins; the two directions of a runway are about equally close, then the direction counts
		if ( across < best_across - 5 or (across < best_across + 5 and diff < best_diff) ) {
			best = r.heading;
			best_across = across;
			best_diff = diff;
		}
	}
	return best;
}

var createDragRobot = func {
	var model_path = pickDraggerModel();
	if ( model_path == nil ) {
		ask21.msg("atc", "No dragger model installed. Download the AI aircraft in the FlightGear launcher.");
		return 0;
	}
	
	# place drag roboter with a distance, that the tow is nearly tautened
	var rope_length_m = getprop("/sim/hitches/aerotow/tow/length");
	var tauten_relative = getprop("/sim/glider/towing/conf/rope_x1") or 0.7;
	var install_distance_m = rope_length_m * (tauten_relative - 0.02);
	
	# local variables
	var ac_pos = geo.aircraft_position();                   	# get position of aircraft
	var ac_hd  = getprop("orientation/heading-deg");        	# get heading of aircraft
	# on a runway the dragger takes the heading of the runway, anywhere else the heading of the glider
	var heading = runwayHeadingAt(ac_pos, ac_hd);
	if ( heading == nil ) {
		heading = ac_hd;
	}
	var dip    = ac_pos.apply_course_distance( heading , install_distance_m );  # set initial dragger position
	var dipalt_m = geo.elevation(dip.lat(), dip.lon());     # height at dragger position
	var glob_max_lift_height_m     = 
	getprop("sim/glider/dragger/conf/glob_max_lift_height_m");
	
	# get the next free ai id and model id
	var freeAIid = getFreeAIID();
	var freeModelid = getFreeModelID();
	
	
	var dragger_ai  = props.globals.getNode("ai/models/dragger", 1);
	var dragger_mod = props.globals.getNode("models", 1);
	var dragger_sim = props.globals.getNode("sim/glider/dragger/robot", 1);
	var dragger_flg = props.globals.getNode("sim/glider/dragger/flags", 1);
	
	dragger_sim.getNode("id_AI", 1).setIntValue(freeAIid);
	dragger_sim.getNode("id_model", 1).setIntValue(freeModelid);
	dragger_sim.getNode("leg_type", 1).setIntValue(0);
	dragger_sim.getNode("leg_distance_m", 1).setValue(2000);
	dragger_sim.getNode("leg_angle_deg", 1).setValue(heading);
	dragger_sim.getNode("leg_segment", 1).setIntValue(0);
	
	dragger_flg.getNode("exist", 1).setIntValue(1);
	
	
	dragger_ai.getNode("id", 1).setIntValue(freeAIid);
	dragger_ai.getNode("callsign", 1).setValue("dragger");
	dragger_ai.getNode("valid", 1).setBoolValue(1);
	dragger_ai.getNode("position/latitude-deg", 1).setValue(dip.lat());
	dragger_ai.getNode("position/longitude-deg", 1).setValue(dip.lon());
	dragger_ai.getNode("position/altitude-ft", 1).setValue(dipalt_m * M2FT);
	dragger_ai.getNode("orientation/true-heading-deg", 1).setValue(heading);
	dragger_ai.getNode("orientation/pitch-deg", 1).setValue(0);
	dragger_ai.getNode("orientation/roll-deg", 1).setValue(0);
	dragger_ai.getNode("velocities/true-airspeed-kt", 1).setValue(0);
	dragger_ai.getNode("velocities/vertical-speed-fps", 1).setValue(0);
	
	dragger_mod.model = dragger_mod.getChild("model", freeModelid, 1);
	dragger_model_node = dragger_mod.model;
	dragger_mod.model.getNode("path", 1).setValue(model_path);
	dragger_mod.model.getNode("longitude-deg-prop", 1).setValue(
		"ai/models/dragger/position/longitude-deg");
	dragger_mod.model.getNode("latitude-deg-prop", 1).setValue(
		"ai/models/dragger/position/latitude-deg");
	dragger_mod.model.getNode("elevation-ft-prop", 1).setValue(
		"ai/models/dragger/position/altitude-ft");
	dragger_mod.model.getNode("heading-deg-prop", 1).setValue(
		"ai/models/dragger/orientation/true-heading-deg");
	dragger_mod.model.getNode("roll-deg-prop", 1).setValue(
		"ai/models/dragger/orientation/roll-deg");
	dragger_mod.model.getNode("load", 1).remove();
	
	
	#storing initial position for reseting after reaching escape height
	setprop("sim/glider/dragger/robot/wp0lat_deg", dip.lat() );
	setprop("sim/glider/dragger/robot/wp0lon_deg", dip.lon() );
	setprop("sim/glider/dragger/robot/wp0alt_m", dipalt_m );
	setprop("sim/glider/dragger/robot/wp0head_deg", heading );
	setprop("sim/glider/dragger/robot/exit_height_m", dip.alt() + glob_max_lift_height_m ); 
	return 1;
}



# ####################################################################################
# main function to initialize the drag roboter
# used by key "D" or gui
var setupDragRobot = func {
  
	# look for allready existing ai object with callsign "dragger"
	var existingdragid = checkDragger();
	if ( existingdragid > -1 ) {              				 # dragger already exists, we can exit
		ask21.msg("atc"," existing dragger id: " ~ existingdragid);
		return 1;
	} else {                                    				 # dragger does not exist, we have to work
		# create a new ai object with callsign "dragger"
		# set initial position 
		if ( !createDragRobot() ) {
			return 0;
		}
		ask21.msg("ai-plane"," I will lift you up into the sky.");
		return 1;
	}
}



# ####################################################################################
# dummy function to delete the drag roboter
# used by gui and dragrobot.nas functions
var removeDragRobot = func {
	hold_alt_m = nil;
	hold_climb_mps = 0;
	guard_mode = 0;
	guard_next_s = 0;
	guard_cache = {};
  
	# look for allready existing ai object with callsign "dragger"
	# will be filled in the next future
	
	# in any case, first stop the dragger
	setprop("sim/glider/dragger/flags/run", 0);
	
	# reset the state of the launch sequence and the settings overridden for the descent
	sent = 0;
	last_leg = 0;
	accel_time_s = 0;
	load_steady_n = 0;
	load_dev_mps = 0;
	if ( saved_roll_deg != nil ) {
		setprop("sim/glider/dragger/conf/glob_max_roll_deg", saved_roll_deg);
		setprop("sim/glider/dragger/conf/glob_max_turnrate_degs", saved_turnrate_degs);
		setprop("sim/glider/dragger/conf/glob_max_speed_lift_mps", saved_lift_mps);
		saved_roll_deg = nil;
	}
	
	# next check for the dragger is still existent
	# if yes, 
	#   remove the dragger from the property tree ai/models
	#   remove the dragger from the property tree models/
	#   remove the dragger working properties
	# if no, 
	#   do nothing
	
	# local variables
	var modelsNode = {};
	
	if ( getprop("/sim/glider/dragger/flags/exist") == 1 ) {   		# does the dragger exist?
		# remove 3d model from scenery
		# identification is /models/model[x] with x=id_model
		# remove the exact node that was created, not whatever sits at the stored index
		if ( dragger_model_node != nil ) {
			dragger_model_node.remove();
			dragger_model_node = nil;
		} else {
			var id_model = getprop("sim/glider/dragger/robot/id_model");
			modelsNode = props.globals.getNode("models/model[" ~ id_model ~ "]");
			if ( modelsNode != nil ) { modelsNode.remove(); }
		}
		props.globals.getNode("ai/models/dragger").remove();
		props.globals.getNode("sim/glider/dragger/robot").remove();
		ask21.msg("atc","dragger removed");
		setprop("/sim/glider/dragger/flags/exist", 0);
	} else {                                                     		# do nothing
		ask21.msg("atc","dragger does not exist");
	}
  
}



# ####################################################################################
# acceleration of the dragger
# The tow rope is a spring without damping, so every abrupt change of the tug's
# acceleration makes the glider surge for a minute or more. The acceleration is therefore
# faded in, and faded out when the max. speed is approached.
var accel_time_s = 0;                                  # time since the acceleration started
var accel_ramp_s = 6;                                  # fade in / fade out time

var tugAccel = func(speed_mps, deltatime_s) {
	var max_accel_mpss = getprop("sim/glider/dragger/conf/glob_max_acceleration_mpss");
	var max_speed_mps  = getprop("sim/glider/dragger/conf/glob_max_speed_mps");
	
	accel_time_s += deltatime_s;
	var fade_in = 0.5 - 0.5 * math.cos(math.pi * math.min(1, accel_time_s / accel_ramp_s));
	var accel_mpss = max_accel_mpss * fade_in;
	var fade_out = (max_speed_mps - speed_mps) / accel_ramp_s;
	if ( accel_mpss > fade_out ) {
		accel_mpss = fade_out;
	}
	if ( accel_mpss < 0 ) {
		accel_mpss = 0;
	}
	return accel_mpss;
}



# ####################################################################################
# reaction of the dragger to the load of the tow rope
# A real tug is slowed by a sudden pull and the pilot brings it back to speed. This
# damps the surging of the glider on the rope. The steady pull is filtered out, so the
# dragger holds its speed in steady flight.
var load_mass_kg = 850;                                # mass of the dragger
var load_recover_s = 6;                                # time to get back to speed
var load_steady_s = 15;                                # filter time for the steady pull
var load_steady_n = 0;
var load_dev_mps = 0;                                  # speed deviation caused by the load

var loadResponse = func(deltatime_s) {
	var hitch = getprop("sim/hitches/aerotow/force_name_jsbsim") or "aerotow_hitch";
	var force_n = (getprop("fdm/jsbsim/external_reactions/" ~ hitch ~ "/magnitude") or 0) * 4.44822;
	if ( deltatime_s > 0.1 ) {
		deltatime_s = 0.1;
	}
	load_steady_n += (force_n - load_steady_n) * deltatime_s / load_steady_s;
	load_dev_mps += (-(force_n - load_steady_n) / load_mass_kg - load_dev_mps / load_recover_s) * deltatime_s;
	return load_dev_mps;
}



# ####################################################################################
# vertical speed of the dragger after lift-off
# Stays low above the ground (ground effect) until climb start speed is reached, then
# the climb rate builds up exponentially to the max. climb rate.
var hold_alt_m = nil;                                  # height to hold after reaching the target height (pattern tows)
var hold_climb_mps = 0;                                # climb rate asked for by the terrain guard while holding
var liftModel = func(speed_mps, oldlift_mps, dragpos_geo, deltatime_s) {
	var climb_start_mps = getprop("sim/glider/dragger/conf/glob_climb_start_speed_mps");
	var max_speed_mps   = getprop("sim/glider/dragger/conf/glob_max_speed_mps");
	var max_lift_mps    = getprop("sim/glider/dragger/conf/glob_max_speed_lift_mps");
	var low_height_m    = getprop("sim/glider/dragger/conf/glob_low_height_m");
	var tau_s           = getprop("sim/glider/dragger/conf/glob_climb_tau_s");
	
	# never let a low max. speed keep the dragger on the ground forever
	if ( climb_start_mps > max_speed_mps - 1 ) {
		climb_start_mps = max_speed_mps - 1;
	}
	
	var target_mps = max_lift_mps;
	if ( speed_mps < climb_start_mps ) {
		tau_s = 1.5;                                 # quick but smooth rise to staying low
		target_mps = 0;
		var ground_m = geo.elevation(dragpos_geo.lat(), dragpos_geo.lon());
		if ( ground_m != nil ) {
			target_mps = (low_height_m - (dragpos_geo.alt() - ground_m)) / 2;
			if ( target_mps > 1 ) { target_mps = 1; }
			if ( target_mps < -1 ) { target_mps = -1; }
		}
	}
	
	# at the target height the dragger levels off and holds it, unless the terrain ahead needs more
	if ( hold_alt_m != nil and speed_mps >= climb_start_mps ) {
		var alt_now_m = dragpos_geo.alt();
		if ( hold_climb_mps > 0 and alt_now_m > hold_alt_m ) {
			hold_alt_m = alt_now_m;                      # climbing for terrain: the new height is the one to hold
		}
		target_mps = (hold_alt_m - alt_now_m) * 0.25;
		if ( target_mps > max_lift_mps ) { target_mps = max_lift_mps; }
		if ( target_mps < -1.5 ) { target_mps = -1.5; }
		if ( hold_climb_mps > target_mps ) {
			target_mps = math.min(hold_climb_mps, max_lift_mps);
		}
	}
	
	if ( tau_s < 0.1 ) { tau_s = 0.1; }
	return oldlift_mps + (target_mps - oldlift_mps) * (1 - math.exp(-deltatime_s / tau_s));
}



# ####################################################################################
# run the drag robot for start leg
var leg0DragRobot = func {
  
	# ##################################################################################
	# Strategy:
	# set flag for start
	# tauten the rope
	# accelerate up to minimum lift speed
	# switch to next leg
	
	
	
	
	var initpos_geo = geo.Coord.new();
	var dragpos_geo = geo.Coord.new();
	var temppos_geo = geo.Coord.new();
	
	var oldspeed_mps     = 0;
	var oldlift_mps      = 0;
	var oldheading_deg   = 0;
	var newspeed_mps     = 0;
	var newlift_mps      = 0;
	var newliftdist_m    = 0;
	var newelevation_m   = 0;
	var distance_m       = 0;
	var leg_distance_m   = 0;
	var deltatime_s      = 0;
	var leg_angle_deg    = 0;
	var headwind_mps     = 0;
	
	
	var segment = getprop("sim/glider/dragger/robot/leg_segment");
	var glob_min_speed_takeoff_mps = getprop("sim/glider/dragger/conf/glob_min_speed_takeoff_mps");
	var glob_max_speed_tauten_mps  = getprop("sim/glider/dragger/conf/glob_max_speed_tauten_mps");
	var glob_min_acceleration_mpss = getprop("sim/glider/dragger/conf/glob_min_acceleration_mpss");
	var glob_max_acceleration_mpss = getprop("sim/glider/dragger/conf/glob_max_acceleration_mpss");
	var glob_max_tautendist_m      = getprop("sim/glider/dragger/conf/glob_max_tautendist_m");
	
	
	if ( dragrobot_timeincrement_s == 0 ) {
		deltatime_s = getprop("sim/time/delta-sec");
	} else {
		deltatime_s = dragrobot_timeincrement_s;
	}
	
	
	oldspeed_mps     = getprop("ai/models/dragger/velocities/true-airspeed-kt") * KT2MPS;
	oldheading_deg   = getprop("ai/models/dragger/orientation/true-heading-deg");
	
	initpos_geo.set_latlon( getprop("sim/glider/dragger/robot/wp0lat_deg"), 
				getprop("sim/glider/dragger/robot/wp0lon_deg"), 
				getprop("sim/glider/dragger/robot/wp0alt_m") );
	dragpos_geo.set_latlon( getprop("ai/models/dragger/position/latitude-deg"), 
				getprop("ai/models/dragger/position/longitude-deg"), 
				getprop("ai/models/dragger/position/altitude-ft") * FT2M);
	
	headwind_mps = aircraft.wind_speed_from(oldheading_deg) * KT2MPS;
	
	
	# update properties like speed and position
	if ( segment == 1 ) {                         							# accelerate to min take-off speed
		var accel_mpss = tugAccel(oldspeed_mps, deltatime_s);
		newspeed_mps = oldspeed_mps + accel_mpss * deltatime_s;
		distance_m = (oldspeed_mps - headwind_mps) * deltatime_s + 0.5 * accel_mpss * deltatime_s * deltatime_s ;
		distance_m += loadResponse(deltatime_s) * deltatime_s;
	} else {                                        						# segment 0, tauten rope
		if ( ( oldspeed_mps - headwind_mps ) < glob_max_speed_tauten_mps ) {
			if ( oldspeed_mps < headwind_mps ) {
				oldspeed_mps = headwind_mps;
			}
			newspeed_mps = glob_min_acceleration_mpss * deltatime_s + oldspeed_mps; 
			distance_m = (oldspeed_mps - headwind_mps) * deltatime_s 
					+ 0.5 * glob_min_acceleration_mpss * deltatime_s * deltatime_s ;
			if ( distance_m < 0.01 ) {  # keep robot locked until speed is high enough
				distance_m = 0;
			}
		} else {
			newspeed_mps = oldspeed_mps;
			distance_m = (oldspeed_mps - headwind_mps) * deltatime_s ;
		}
		if ( dragpos_geo.direct_distance_to(initpos_geo) > glob_max_tautendist_m ) { 
			setprop("sim/glider/dragger/robot/leg_segment", 1);
		}
	}
	
	dragpos_geo.apply_course_distance( oldheading_deg , distance_m );
	newelevation_m = geo.elevation( dragpos_geo.lat(), dragpos_geo.lon() );
	if ( newelevation_m != nil ) {
		dragpos_geo.set_alt(newelevation_m);
	}
	
	setprop("ai/models/dragger/position/latitude-deg", dragpos_geo.lat());
	setprop("ai/models/dragger/position/longitude-deg", dragpos_geo.lon());
	setprop("ai/models/dragger/position/altitude-ft", dragpos_geo.alt() * M2FT);
	setprop("ai/models/dragger/velocities/true-airspeed-kt", newspeed_mps * MPS2KT);
	
	
	# check for exit criteria
	if ( oldspeed_mps > glob_min_speed_takeoff_mps ) { 
		# set anchor point
		setprop("sim/glider/dragger/robot/anchorlat_deg", dragpos_geo.lat());
		setprop("sim/glider/dragger/robot/anchorlon_deg", dragpos_geo.lon());
		setprop("sim/glider/dragger/robot/anchoralt_m", dragpos_geo.alt());
		# set flags for next leg
		if ( (getprop("sim/glider/dragger/conf/pattern") or 0) != 0 ) {
			patternSetup();
			setprop("sim/glider/dragger/robot/leg_type", 4);    # landing pattern tow
		} else {
			setprop("sim/glider/dragger/robot/leg_type", 2);    # next one is straight forward
			# set next exit criteria for straight leg, 200m ... 400m 
			leg_distance_m = 200 + rand() * 200;
			setprop("sim/glider/dragger/robot/leg_distance_m", leg_distance_m ); 
			print_straight(leg_distance_m);
		}
	}
}



# ####################################################################################
# run the drag robot for turns
var leg1DragRobot = func {
	# turns are described by the turn angle, so the delta angle from heading at initial 
	# position to heading from current position is the criteria for exit
	
	
	var initpos_geo = geo.Coord.new();
	var dragpos_geo = geo.Coord.new();
	
	var oldspeed_mps     = 0;
	var oldlift_mps      = 0;
	var oldheading_deg   = 0;
	var oldroll_deg      = 0;
	var deltatime_s      = 0;
	var distance_m       = 0;
	var newspeed_mps     = 0;
	var newlift_mps      = 0;
	var newelevation_m   = 0;
	var newroll_deg      = 0;
	var newturn_deg      = 0;
	var newheading_deg   = 0;
	var wind_from_east_mps = 0;
	var wind_from_nord_mps = 0;
	var wind_from_down_mps = 0;
	
	
	var segment = getprop("sim/glider/dragger/robot/leg_segment");
	var side    = getprop("sim/glider/dragger/robot/turnside");
	var targetheading_deg = getprop("sim/glider/dragger/robot/leg_angle_deg");
	var glob_min_speed_takeoff_mps = getprop("sim/glider/dragger/conf/glob_min_speed_takeoff_mps");
	var glob_max_speed_mps         = getprop("sim/glider/dragger/conf/glob_max_speed_mps");
	var glob_max_speed_lift_mps    = getprop("sim/glider/dragger/conf/glob_max_speed_lift_mps");
	var glob_max_acceleration_mpss = getprop("sim/glider/dragger/conf/glob_max_acceleration_mpss");
	var glob_max_roll_deg          = getprop("sim/glider/dragger/conf/glob_max_roll_deg");
	var glob_max_rollrate_degs     = getprop("sim/glider/dragger/conf/glob_max_rollrate_degs");
	var glob_max_turnrate_degs     = getprop("sim/glider/dragger/conf/glob_max_turnrate_degs");
	
	
	if ( dragrobot_timeincrement_s == 0 ) {
		deltatime_s = getprop("sim/time/delta-sec");
	} else {
		deltatime_s = dragrobot_timeincrement_s;
	}
	
	oldspeed_mps     = getprop("ai/models/dragger/velocities/true-airspeed-kt") * KT2MPS;
	oldlift_mps      = getprop("ai/models/dragger/velocities/vertical-speed-fps") * FT2M;
	oldheading_deg   = getprop("ai/models/dragger/orientation/true-heading-deg");
	oldroll_deg      = getprop("ai/models/dragger/orientation/roll-deg");
	wind_from_east_mps = getprop("environment/wind-from-east-fps") * FT2M;
	wind_from_nord_mps = getprop("environment/wind-from-north-fps") * FT2M;
	wind_from_down_mps = getprop("environment/wind-from-down-fps") * FT2M;
	
	dragpos_geo.set_latlon( getprop("ai/models/dragger/position/latitude-deg"), 
				getprop("ai/models/dragger/position/longitude-deg"), 
				getprop("ai/models/dragger/position/altitude-ft") * FT2M);
	
	# calculate current roll angle for turns
	if ( side == 1 ) {         									# right turns
		if ( segment == 2 ) {
			if ( oldroll_deg < glob_max_roll_deg) {
				# calculate new roll angle
				newroll_deg = oldroll_deg + deltatime_s * glob_max_rollrate_degs;
			}
			else {
				newroll_deg = oldroll_deg;
				setprop("sim/glider/dragger/robot/leg_segment", 3);
			}
		}
	
		if (segment == 3 ) {
			newroll_deg = oldroll_deg;
			# check for target turn 
			if ( (oldheading_deg > targetheading_deg) and (oldheading_deg <= (targetheading_deg+5)) ) { 
				# turn finished
				setprop("sim/glider/dragger/robot/leg_segment", 4);
			}
			# if yes, change segment type
		}
		
		if ( segment == 4 ) {
			if ( oldroll_deg > 0) {
				# calculate new roll angle
				newroll_deg = oldroll_deg - deltatime_s * glob_max_rollrate_degs;
			} else {                                                      			# also exit criteria
				newroll_deg = 0;
				# set anchor point
				setprop("sim/glider/dragger/robot/anchorlat_deg", dragpos_geo.lat());
				setprop("sim/glider/dragger/robot/anchorlon_deg", dragpos_geo.lon());
				setprop("sim/glider/dragger/robot/anchoralt_m", dragpos_geo.alt());
				# set next leg
				setprop("sim/glider/dragger/robot/leg_segment", 2);
				setprop("sim/glider/dragger/robot/leg_type", 2);
				var length_m = 100;                                    # first turn after 100m
				setprop("sim/glider/dragger/robot/leg_distance_m", length_m); 
				print_straight(length_m);
			}
		}
	} else {                 									# left turns
		if ( segment == 2 ) {
			if ( oldroll_deg > -glob_max_roll_deg) {
				# calculate new roll angle
				newroll_deg = oldroll_deg - deltatime_s * glob_max_rollrate_degs;
			} else {
				newroll_deg = oldroll_deg;
				setprop("sim/glider/dragger/robot/leg_segment", 3);
			}
		}
			
		if (segment == 3 ) {
			newroll_deg = oldroll_deg;
			# check for target turn 
			if ( (oldheading_deg < targetheading_deg) 
				and 
				(oldheading_deg >= (targetheading_deg-5)) ) { 
				# turn finished
				setprop("sim/glider/dragger/robot/leg_segment", 4);
			}
			# if yes, change segment type
		}
			
		if ( segment == 4 ) {
			if ( oldroll_deg < 0) {
				# calculate new roll angle
				newroll_deg = oldroll_deg + deltatime_s * glob_max_rollrate_degs;
			} else {                                                      # also exit criteria
				newroll_deg = 0;
				# set anchor point
				setprop("sim/glider/dragger/robot/anchorlat_deg", dragpos_geo.lat());
				setprop("sim/glider/dragger/robot/anchorlon_deg", dragpos_geo.lon());
				setprop("sim/glider/dragger/robot/anchoralt_m", dragpos_geo.alt());
				if(!last_leg){
					# set next leg
					setprop("sim/glider/dragger/robot/leg_segment", 2);
					setprop("sim/glider/dragger/robot/leg_type", 2);
					var length_m = 100;                                    # first turn after 100m
					setprop("sim/glider/dragger/robot/leg_distance_m", length_m); 
					print_straight(length_m);
				} else {
					setprop("sim/glider/dragger/robot/leg_segment", 2);
					setprop("sim/glider/dragger/robot/leg_type", 2);
					setprop("sim/glider/dragger/robot/leg_distance_m", 1500); 
					setprop("sim/glider/dragger/conf/glob_max_speed_lift_mps", 0);
				}
			}
		}
	}
	# calculate current speed
	var accel_mpss = tugAccel(oldspeed_mps, deltatime_s);
	newspeed_mps = oldspeed_mps + accel_mpss * deltatime_s;
	distance_m = oldspeed_mps * deltatime_s + 0.5 * accel_mpss * deltatime_s * deltatime_s;
	distance_m += loadResponse(deltatime_s) * deltatime_s;
	
	# calculate current lift
	newlift_mps = liftModel(oldspeed_mps, oldlift_mps, dragpos_geo, deltatime_s);
	newliftdist_m = (newlift_mps + wind_from_down_mps) * deltatime_s;
	
	# calculate current turn rate based on roll angle
	newturn_deg = glob_max_turnrate_degs * newroll_deg / glob_max_roll_deg * deltatime_s;
	
	# calculate new heading based on turn rate
	if ( (oldheading_deg + newturn_deg) > 360 ) { # if a rightturn exceeds 360 heading
		newheading_deg = oldheading_deg + newturn_deg - 360;
	} else {
		if ( (oldheading_deg + newturn_deg) < 0 ) { # if a leftturn exceeds 0 heading
			newheading_deg = oldheading_deg + newturn_deg +360;
		} else { # for all other headings
			newheading_deg = oldheading_deg + newturn_deg;
		}
	}
	
	# calculate new position based on new heading and distance increment
	dragpos_geo.apply_course_distance( newheading_deg , distance_m );
	dragpos_geo.apply_course_distance( 270.0 , wind_from_east_mps * deltatime_s );
	dragpos_geo.apply_course_distance( 180.0 , wind_from_nord_mps * deltatime_s );
	newelevation_m = dragpos_geo.alt() + newliftdist_m;
	dragpos_geo.set_alt(newelevation_m);
	
	
	setprop("ai/models/dragger/position/latitude-deg",         dragpos_geo.lat());
	setprop("ai/models/dragger/position/longitude-deg",        dragpos_geo.lon());
	setprop("ai/models/dragger/position/altitude-ft",          dragpos_geo.alt() * M2FT);
	setprop("ai/models/dragger/orientation/true-heading-deg",  newheading_deg);
	setprop("ai/models/dragger/orientation/roll-deg",          newroll_deg);
	setprop("ai/models/dragger/velocities/true-airspeed-kt",   newspeed_mps * MPS2KT);
	setprop("ai/models/dragger/velocities/vertical-speed-fps", newlift_mps * M2FT);
}



# ####################################################################################
# run the drag robot for straight legs
var leg2DragRobot = func {
	# straight legs are described by the length, so the delta distance from initial 
	# position to current position is the criteria for exit
	
	
	
	var initpos_geo = geo.Coord.new();
	var dragpos_geo = geo.Coord.new();
	
	var oldspeed_mps     = 0;
	var oldheading_deg   = 0;
	var newspeed_mps     = 0;
	var newlift_mps      = 0;
	var newliftdist_m    = 0;
	var newelevation_m   = 0;
	var distance_m       = 0;
	var leg_distance_m   = 0;
	var deltatime_s      = 0;
	var leg_angle_deg    = 0;
	var wind_from_east_mps = 0;
	var wind_from_nord_mps = 0;
	var wind_from_down_mps = 0;
	
	var glob_min_speed_takeoff_mps = getprop("sim/glider/dragger/conf/glob_min_speed_takeoff_mps");
	var glob_max_speed_mps         = getprop("sim/glider/dragger/conf/glob_max_speed_mps");
	var glob_max_speed_lift_mps    = getprop("sim/glider/dragger/conf/glob_max_speed_lift_mps");
	var glob_max_acceleration_mpss = getprop("sim/glider/dragger/conf/glob_max_acceleration_mpss");
	
	
	if ( dragrobot_timeincrement_s == 0 ) {
		deltatime_s = getprop("sim/time/delta-sec");
	} else {
		deltatime_s = dragrobot_timeincrement_s;
	}
	
	oldspeed_mps     = getprop("ai/models/dragger/velocities/true-airspeed-kt") * KT2MPS;
	oldheading_deg   = getprop("ai/models/dragger/orientation/true-heading-deg");
	leg_distance_m   = getprop("sim/glider/dragger/robot/leg_distance_m");
	wind_from_east_mps = getprop("environment/wind-from-east-fps") * FT2M;
	wind_from_nord_mps = getprop("environment/wind-from-north-fps") * FT2M;
	wind_from_down_mps = getprop("environment/wind-from-down-fps") * FT2M;
	
	initpos_geo.set_latlon( getprop("sim/glider/dragger/robot/anchorlat_deg"), 
				getprop("sim/glider/dragger/robot/anchorlon_deg"), 
				getprop("sim/glider/dragger/robot/anchoralt_m") );
	dragpos_geo.set_latlon( getprop("ai/models/dragger/position/latitude-deg"), 
				getprop("ai/models/dragger/position/longitude-deg"), 
				getprop("ai/models/dragger/position/altitude-ft") * FT2M);
	
	var accel_mpss = tugAccel(oldspeed_mps, deltatime_s);
	newspeed_mps = oldspeed_mps + accel_mpss * deltatime_s;
	distance_m = oldspeed_mps * deltatime_s + 0.5 * accel_mpss * deltatime_s * deltatime_s;
	distance_m += loadResponse(deltatime_s) * deltatime_s;
	
	newlift_mps = liftModel(oldspeed_mps, getprop("ai/models/dragger/velocities/vertical-speed-fps") * FT2M, dragpos_geo, deltatime_s);
	newliftdist_m = (newlift_mps + wind_from_down_mps) * deltatime_s;
	
	dragpos_geo.apply_course_distance( oldheading_deg , distance_m );
	dragpos_geo.apply_course_distance( 270.0 , wind_from_east_mps * deltatime_s );
	dragpos_geo.apply_course_distance( 180.0 , wind_from_nord_mps * deltatime_s );
	newelevation_m = dragpos_geo.alt() + newliftdist_m;
	dragpos_geo.set_alt(newelevation_m);
	
	setprop("ai/models/dragger/position/latitude-deg", dragpos_geo.lat());
	setprop("ai/models/dragger/position/longitude-deg", dragpos_geo.lon());
	setprop("ai/models/dragger/position/altitude-ft", dragpos_geo.alt() * M2FT);
	setprop("ai/models/dragger/velocities/true-airspeed-kt", newspeed_mps * MPS2KT);
	setprop("ai/models/dragger/velocities/vertical-speed-fps", newlift_mps * M2FT);
  
	# exit criteria to final drop-down: max height reached
	if(!last_leg){
		if ( dragpos_geo.alt() > getprop("sim/glider/dragger/robot/exit_height_m") and sent == 0) {
			ask21.msg("ai-plane"," we have reached max height, bye bye"); 
			sent = 1;
			settimer(func(){
					if ( getprop("sim/glider/dragger/flags/exist") != 1 ) { return; }   # cancelled meanwhile
					setprop("sim/glider/dragger/robot/leg_type", 3);
					setprop("sim/glider/dragger/robot/leg_segment", 2);
					ask21.msg("ai-plane","turn right, I turn left" );
					# unhook from dragger
					towing.releaseHitch("aerotow");
				}, 2);
		}else{
			# exit criteria to next turn
			# no turns until a safe height above the start point is reached
			if ( dragpos_geo.direct_distance_to(initpos_geo) > leg_distance_m 
				and sent == 0
				and (dragpos_geo.alt() - getprop("sim/glider/dragger/robot/wp0alt_m")) > 50 ) { 
				var turn_deg = 30 + rand() * 240;                    # turn range from 30 to 270 deg
				var side = rand();
				if (side > 0.5) {                                    # right turn: heading increases
					leg_angle_deg = oldheading_deg + turn_deg;
					setprop("sim/glider/dragger/robot/turnside", 1);
					print_turn("right", turn_deg);
				}else {                                              # left turn: heading decreases
					leg_angle_deg = oldheading_deg - turn_deg;
					setprop("sim/glider/dragger/robot/turnside", 0);
					print_turn("left", turn_deg);
				}
				if ( leg_angle_deg >= 360 ) { leg_angle_deg -= 360; }
				if ( leg_angle_deg < 0 ) { leg_angle_deg += 360; }
				setprop("sim/glider/dragger/robot/leg_angle_deg", leg_angle_deg);
				setprop("sim/glider/dragger/robot/leg_type", 1);
				setprop("sim/glider/dragger/robot/leg_segment", 2);
			}
		}
	}else if ( dragpos_geo.direct_distance_to(initpos_geo) > leg_distance_m ){
		stopRobot();
	}
}



# ####################################################################################
# run the drag robot for final leg
var leg3DragRobot = func {

	var initpos_geo = geo.Coord.new();
	var dragpos_geo = geo.Coord.new();
	
	var oldspeed_mps     = 0;
	var oldheading_deg   = 0;
	var newspeed_mps     = 0;
	var newlift_mps      = 0;
	var newliftdist_m    = 0;
	var newelevation_m   = 0;
	var distance_m       = 0;
	var leg_distance_m   = 0;
	var deltatime_s      = 0;
	var leg_angle_deg    = 0;
	var wind_from_east_mps = 0;
	var wind_from_nord_mps = 0;
	var wind_from_down_mps = 0;
	
	var glob_min_speed_takeoff_mps = getprop("sim/glider/dragger/conf/glob_min_speed_takeoff_mps");
	var glob_max_speed_mps         = getprop("sim/glider/dragger/conf/glob_max_speed_mps");
	var glob_max_speed_lift_mps    = getprop("sim/glider/dragger/conf/glob_max_speed_lift_mps");
	var glob_max_acceleration_mpss = getprop("sim/glider/dragger/conf/glob_max_acceleration_mpss");
	
	
	if ( dragrobot_timeincrement_s == 0 ) {
		deltatime_s = getprop("sim/time/delta-sec");
	} else {
		deltatime_s = dragrobot_timeincrement_s;
	}
	
	var speed_mps     	= getprop("ai/models/dragger/velocities/true-airspeed-kt") * KT2MPS;
	var oldheading_deg	= getprop("ai/models/dragger/orientation/true-heading-deg");
	leg_distance_m	= 50;
	wind_from_east_mps = getprop("environment/wind-from-east-fps") * FT2M;
	wind_from_nord_mps = getprop("environment/wind-from-north-fps") * FT2M;
	wind_from_down_mps = getprop("environment/wind-from-down-fps") * FT2M;
	
	initpos_geo.set_latlon( getprop("sim/glider/dragger/robot/anchorlat_deg"), 
				getprop("sim/glider/dragger/robot/anchorlon_deg"), 
				getprop("sim/glider/dragger/robot/anchoralt_m") );
	dragpos_geo.set_latlon( getprop("ai/models/dragger/position/latitude-deg"), 
				getprop("ai/models/dragger/position/longitude-deg"), 
				getprop("ai/models/dragger/position/altitude-ft") * FT2M);
	
	distance_m = speed_mps * deltatime_s;
	
	newlift_mps = 0.0;
	newliftdist_m = wind_from_down_mps * deltatime_s;
	
	dragpos_geo.apply_course_distance( oldheading_deg , distance_m );
	dragpos_geo.apply_course_distance( 270.0 , wind_from_east_mps * deltatime_s );
	dragpos_geo.apply_course_distance( 180.0 , wind_from_nord_mps * deltatime_s );
	newelevation_m = dragpos_geo.alt() + newliftdist_m;
	dragpos_geo.set_alt(newelevation_m);
	
	setprop("ai/models/dragger/position/latitude-deg", dragpos_geo.lat());
	setprop("ai/models/dragger/position/longitude-deg", dragpos_geo.lon());
	setprop("ai/models/dragger/position/altitude-ft", dragpos_geo.alt() * M2FT);
	setprop("ai/models/dragger/velocities/true-airspeed-kt", speed_mps * MPS2KT);
	setprop("ai/models/dragger/velocities/vertical-speed-fps", newlift_mps * M2FT);
	
	
	# left turn
	if ( dragpos_geo.direct_distance_to(initpos_geo) > leg_distance_m ) { 
		setprop("sim/glider/dragger/robot/leg_type", 1);
		setprop("sim/glider/dragger/robot/leg_segment", 2);
		setprop("sim/glider/dragger/robot/turnside", 0);
		saved_roll_deg = getprop("sim/glider/dragger/conf/glob_max_roll_deg");
		saved_turnrate_degs = getprop("sim/glider/dragger/conf/glob_max_turnrate_degs");
		saved_lift_mps = getprop("sim/glider/dragger/conf/glob_max_speed_lift_mps");
		setprop("sim/glider/dragger/conf/glob_max_speed_lift_mps", -1);
		setprop("sim/glider/dragger/conf/glob_max_roll_deg", 45);
		setprop("sim/glider/dragger/conf/glob_max_turnrate_degs", 8);
		if ( (oldheading_deg - 90) <= 0) {
			leg_angle_deg = 360 + ( oldheading_deg - 90 );
		}
		else {
			leg_angle_deg = oldheading_deg - 90;
		}
		setprop("sim/glider/dragger/robot/leg_angle_deg", leg_angle_deg);
		last_leg = 1;
		
	}
  
  
}


# ####################################################################################
# terrain awareness
# Every second the dragger looks ahead along the path it is going to fly (the planned turn or the straight leg)
# and compares it with the terrain (geo.elevation). The path has to clear the ground by 3 % of the distance
# ahead, at least 10 m and at most 60 m. It only turns away when this fails on two checks in a row. The dragger already climbs as fast as the combination of dragger
# and glider can (glob_max_speed_lift_mps), so the only way out is to turn away: with the same bank angle,
# roll rate and turn rate as in its normal turns. The shortest turn that clears the terrain, to the left or
# to the right, is chosen. When the straight path is clear again it rolls out and goes on with straight legs.
var guard_horizon_s = 100;                             # how far ahead the dragger looks
var guard_step_s = 6;                                  # sample interval along the path
var guard_side_m = 100;                                # terrain is also sampled this far to both sides
var guard_mode = 0;                                    # 0 = follows its plan, 1 = turning away, 2 = rolling out
var guard_side = 0;                                    # 1 = turn right, -1 = turn left
var guard_next_s = 0;
var guard_clear_n = 0;
var guard_viol_n = 0;                                  # consecutive checks that found the plan in terrain
var guard_from_leg = 0;                                # the leg that the avoidance interrupted
var guard_cache = {};
var guard_cache_s = 0;

var guardTerrain = func(lat, lon) {
	var key = sprintf("%.3f,%.3f", lat, lon);          # cells of about 100 m
	if ( contains(guard_cache, key) ) {
		return guard_cache[key];
	}
	var e = geo.elevation(lat, lon);
	guard_cache[key] = e;
	return e;
}

# climb rate (m/s) that the dragger would need to clear the terrain along a path, from its current position
#   mode "straight": straight ahead
#   mode "target":   turns in direction `side` until heading `arg` is reached, then straight
#   mode "time":     turns in direction `side` for `arg` seconds, then rolls out and flies straight
# nil elevations (no scenery loaded there) are ignored
var guardPath = func(lat, lon, alt_m, hdg_deg, speed_mps, roll_deg, mode, arg, side) {
	var maxroll  = getprop("sim/glider/dragger/conf/glob_max_roll_deg");
	var rollrate = getprop("sim/glider/dragger/conf/glob_max_rollrate_degs");
	var turnrate = getprop("sim/glider/dragger/conf/glob_max_turnrate_degs");
	var p = geo.Coord.new();
	p.set_latlon(lat, lon);
	var h = hdg_deg;
	var r = roll_deg;
	var t = 0;
	var dist = 0;
	var need = -99;
	var ph = 0;                                         # pattern mode: phase of the pattern and distance flown in it
	var ph_dist = 0;
	if ( mode == "pattern" ) {
		ph = arg.phase;
		ph_dist = (ph == 0) ? arg.xrem : 0;
		maxroll = pattern_bank_deg;                     # the pattern turns are flown with their own bank
		turnrate = 57.2958 * 9.81 * math.tan(pattern_bank_deg * D2R) / math.max(speed_mps, 25);
	}
	var horizon_s = guard_horizon_s;
	if ( mode == "pattern" and arg.tmax < horizon_s ) {
		horizon_s = arg.tmax;                            # nothing beyond the release matters
	}
	while ( t < horizon_s ) {
		t += guard_step_s;
		var want = 0;
		if ( mode == "pattern" ) {
			var leadd = 0.5 * turnrate * math.abs(r) / rollrate;
			if ( ph == 0 ) {
				ph_dist -= speed_mps * guard_step_s;
				if ( ph_dist <= 0 ) { ph = 1; }
			} elsif ( ph == 1 or ph == 3 ) {
				var tgt = arg.H + arg.side * (ph == 1 ? 90 : 180);
				if ( arg.side * wrap180(tgt - h) <= leadd + 1 ) {
					ph += 1;
					ph_dist = arg.c;
				} else {
					want = arg.side * pattern_bank_deg;
				}
			} elsif ( ph == 2 ) {
				ph_dist -= speed_mps * guard_step_s;
				if ( ph_dist <= 0 ) { ph = 3; }
			}
		} elsif ( mode == "time" ) {
			want = (t <= arg) ? side * maxroll : 0;
		} elsif ( mode == "target" ) {
			var rem = (side == 1) ? math.mod(arg - h + 360, 360) : math.mod(h - arg + 360, 360);   # degrees still to turn
			want = (rem > 6 and rem < 340) ? side * maxroll : 0;
		}
		var dr = want - r;
		var lim = rollrate * guard_step_s;
		if ( dr > lim ) { dr = lim; }
		if ( dr < -lim ) { dr = -lim; }
		r += dr;
		h = math.mod(h + turnrate * r / maxroll * guard_step_s + 360, 360);
		p.apply_course_distance(h, speed_mps * guard_step_s);
		dist += speed_mps * guard_step_s;
		if ( mode == "pattern" and ph == 4 and arg.S.distance_to(p) * math.cos((arg.S.course_to(p) - arg.H) * D2R) <= arg.L + 30 ) {
			break;                                      # released here
		}
		
		var e = guardTerrain(p.lat(), p.lon());
		foreach ( var d; [90, -90] ) {
			var q = geo.Coord.new(p);
			q.apply_course_distance(h + d, guard_side_m);
			var e2 = guardTerrain(q.lat(), q.lon());
			if ( e2 != nil and (e == nil or e2 > e) ) {
				e = e2;
			}
		}
		if ( e != nil ) {
			var clearance_m = 0.03 * dist;
			if ( clearance_m < 10 ) { clearance_m = 10; }
			if ( clearance_m > 60 ) { clearance_m = 60; }
			var n = (e + clearance_m - alt_m) / t;
			if ( n > need ) { need = n; }
		}
	}
	return need;
}

# true while the dragger is turning away from terrain
var terrainGuard = func(leg) {
	var enabled = getprop("sim/glider/dragger/conf/terrain_guard");
	if ( enabled != nil and !enabled ) {
		guard_mode = 0;
		return 0;
	}
	var t = getprop("sim/time/elapsed-sec");
	if ( guard_mode != 0 and t < guard_next_s ) {
		return 1;
	}
	if ( guard_mode == 0 and t < guard_next_s ) {
		return 0;
	}
	guard_next_s = t + 1;
	if ( guard_mode == 2 ) {
		return 1;
	}
	if ( t - guard_cache_s > 30 ) {                    # scenery may have been loaded meanwhile
		guard_cache = {};
		guard_cache_s = t;
	}
	
	var lat     = getprop("ai/models/dragger/position/latitude-deg");
	var lon     = getprop("ai/models/dragger/position/longitude-deg");
	var alt_m   = getprop("ai/models/dragger/position/altitude-ft") * FT2M;
	var hdg     = getprop("ai/models/dragger/orientation/true-heading-deg");
	var roll    = getprop("ai/models/dragger/orientation/roll-deg");
	var speed   = getprop("ai/models/dragger/velocities/true-airspeed-kt") * KT2MPS;
	var maxc    = getprop("sim/glider/dragger/conf/glob_max_speed_lift_mps");
	var ground  = geo.elevation(lat, lon);
	if ( ground == nil or alt_m - ground < 15 or speed < 25 or maxc <= 0 ) {
		# still taking off, or nothing known about the terrain: no new avoidance; a turn under way is finished
		if ( guard_mode == 0 ) {
			return 0;
		}
		guard_mode = 2;
		return 1;
	}
	
	if ( guard_mode == 0 ) {
		var need = 0;
		if ( leg == 1 ) {
			var side = getprop("sim/glider/dragger/robot/turnside") == 1 ? 1 : -1;
			need = guardPath(lat, lon, alt_m, hdg, speed, roll, "target", getprop("sim/glider/dragger/robot/leg_angle_deg"), side);
		} elsif ( leg == 4 and pat.phase < 0 ) {
			return 0;                                   # realigning with the runway after an avoidance
		} elsif ( leg == 4 and pat.planned ) {
			# the rest of the landing pattern: out to the turn, crosswind, downwind
			var here = geo.Coord.new();
			here.set_latlon(lat, lon);
			var xnow = pat.S.distance_to(here) * math.cos((pat.S.course_to(here) - pat.H) * D2R);
			need = guardPath(lat, lon, alt_m, hdg, speed, roll, "pattern",
				{phase: pat.phase, xrem: pat.xT - xnow, c: pat.c, H: pat.H, side: pat.side, S: pat.S, L: pat.L,
				 tmax: (hold_alt_m != nil) ? guard_horizon_s
				       : math.max(guard_step_s, (getprop("sim/glider/dragger/robot/exit_height_m") - alt_m) / maxc + 6)}, 0);
		} else {
			need = guardPath(lat, lon, alt_m, hdg, speed, roll, "straight", 0, 0);
		}
		if ( need <= 0.95 * maxc ) {
			guard_viol_n = 0;
			# at the target height the dragger climbs on only as much as the terrain needs
			hold_climb_mps = (hold_alt_m != nil and need > 0.1) ? math.min(maxc, need + 0.3) : 0;
			return 0;
		}
		if ( hold_alt_m != nil ) {
			hold_climb_mps = maxc;                      # first everything the climb can give, then turn
		}
		guard_viol_n += 1;
		if ( guard_viol_n < 2 ) {
			return 0;                                   # once is not enough: look again in a second
		}
		guard_viol_n = 0;
		# the plan leads into terrain: find the shortest turn to the left or right that clears it
		var best_side = 0;
		var best_time = 1e9;
		var best_need = 1e9;
		foreach ( var s; [1, -1] ) {
			foreach ( var tt; [8, 16, 24, 32, 40, 50, 60] ) {
				var n = guardPath(lat, lon, alt_m, hdg, speed, roll, "time", tt, s);
				if ( n <= 0.9 * maxc ) {
					if ( tt < best_time ) { best_time = tt; best_side = s; best_need = n; }
					break;
				}
				if ( tt == 60 and best_time == 1e9 and n < best_need ) { best_need = n; best_side = s; }
			}
		}
		if ( best_side == 0 ) { best_side = 1; }
		guard_side = best_side;
		guard_mode = 1;
		guard_clear_n = 0;
		guard_from_leg = leg;
		if ( leg == 4 ) {
			pat.avoids += 1;
		}
		ask21.msg("ai-plane", "terrain ahead, turning " ~ (guard_side == 1 ? "right" : "left"));
		return 1;
	}
	
	# turning away: roll out as soon as the straight path is clear again
	var need = guardPath(lat, lon, alt_m, hdg, speed, roll, "time", 0, 0);
	if ( need <= 0.8 * maxc ) {
		guard_clear_n += 1;
	} else {
		guard_clear_n = 0;
	}
	if ( guard_clear_n >= 2 ) {
		guard_mode = 2;
	}
	return 1;
}

# the flight of the dragger while it turns away from terrain (same model as the turns of leg 1)
var avoidDragRobot = func {
	var dragpos_geo = geo.Coord.new();
	var deltatime_s = (dragrobot_timeincrement_s == 0) ? getprop("sim/time/delta-sec") : dragrobot_timeincrement_s;
	var maxroll  = getprop("sim/glider/dragger/conf/glob_max_roll_deg");
	var rollrate = getprop("sim/glider/dragger/conf/glob_max_rollrate_degs");
	var turnrate = getprop("sim/glider/dragger/conf/glob_max_turnrate_degs");
	
	var oldspeed_mps   = getprop("ai/models/dragger/velocities/true-airspeed-kt") * KT2MPS;
	var oldheading_deg = getprop("ai/models/dragger/orientation/true-heading-deg");
	var oldroll_deg    = getprop("ai/models/dragger/orientation/roll-deg");
	var wind_from_east_mps = getprop("environment/wind-from-east-fps") * FT2M;
	var wind_from_nord_mps = getprop("environment/wind-from-north-fps") * FT2M;
	var wind_from_down_mps = getprop("environment/wind-from-down-fps") * FT2M;
	dragpos_geo.set_latlon( getprop("ai/models/dragger/position/latitude-deg"),
				getprop("ai/models/dragger/position/longitude-deg"),
				getprop("ai/models/dragger/position/altitude-ft") * FT2M);
	
	var want = (guard_mode == 1) ? guard_side * maxroll : 0;
	var dr = want - oldroll_deg;
	var lim = rollrate * deltatime_s;
	if ( dr > lim ) { dr = lim; }
	if ( dr < -lim ) { dr = -lim; }
	var newroll_deg = oldroll_deg + dr;
	
	var accel_mpss = tugAccel(oldspeed_mps, deltatime_s);
	var newspeed_mps = oldspeed_mps + accel_mpss * deltatime_s;
	var distance_m = oldspeed_mps * deltatime_s + 0.5 * accel_mpss * deltatime_s * deltatime_s;
	distance_m += loadResponse(deltatime_s) * deltatime_s;
	var newlift_mps = liftModel(oldspeed_mps, getprop("ai/models/dragger/velocities/vertical-speed-fps") * FT2M, dragpos_geo, deltatime_s);
	var newliftdist_m = (newlift_mps + wind_from_down_mps) * deltatime_s;
	
	var newheading_deg = math.mod(oldheading_deg + turnrate * newroll_deg / maxroll * deltatime_s + 360, 360);
	dragpos_geo.apply_course_distance( newheading_deg , distance_m );
	dragpos_geo.apply_course_distance( 270.0 , wind_from_east_mps * deltatime_s );
	dragpos_geo.apply_course_distance( 180.0 , wind_from_nord_mps * deltatime_s );
	dragpos_geo.set_alt(dragpos_geo.alt() + newliftdist_m);
	
	setprop("ai/models/dragger/position/latitude-deg",         dragpos_geo.lat());
	setprop("ai/models/dragger/position/longitude-deg",        dragpos_geo.lon());
	setprop("ai/models/dragger/position/altitude-ft",          dragpos_geo.alt() * M2FT);
	setprop("ai/models/dragger/orientation/true-heading-deg",  newheading_deg);
	setprop("ai/models/dragger/orientation/roll-deg",          newroll_deg);
	setprop("ai/models/dragger/velocities/true-airspeed-kt",   newspeed_mps * MPS2KT);
	setprop("ai/models/dragger/velocities/vertical-speed-fps", newlift_mps * M2FT);
	
	# wings level again: carry on with straight legs from here
	if ( guard_mode == 2 and math.abs(newroll_deg) < 0.5 ) {
		guard_mode = 0;
		guard_next_s = 0;
		setprop("ai/models/dragger/orientation/roll-deg", 0);
		setprop("sim/glider/dragger/robot/anchorlat_deg", dragpos_geo.lat());
		setprop("sim/glider/dragger/robot/anchorlon_deg", dragpos_geo.lon());
		setprop("sim/glider/dragger/robot/anchoralt_m", dragpos_geo.alt());
		if ( guard_from_leg == 4 and pat.avoids < 3 ) {
			# a landing pattern tow keeps its goal: back to the runway heading, then plan the pattern again
			pat.phase = -1;
			pat.planned = 0;
			setprop("sim/glider/dragger/robot/leg_type", 4);
			ask21.msg("ai-plane", "terrain clear, back to the pattern");
		} else {
			setprop("sim/glider/dragger/robot/leg_segment", 2);
			setprop("sim/glider/dragger/robot/leg_type", 2);
			setprop("sim/glider/dragger/robot/leg_distance_m", 400);
			if ( guard_from_leg == 4 ) {
				ask21.msg("ai-plane", "no room for the pattern here, a normal tow then");
			} else {
				ask21.msg("ai-plane", "terrain clear, straight ahead");
			}
		}
	}
}


# ####################################################################################
# tow types (pull-down menu of the drag robot dialog)
# 500 m and 1000 m tows climb to that height over the start and then release, with turns in between.
# The landing pattern tows take the glider to 250 m at the start of the downwind leg: 500 m to the left (or
# right) of the runway, abeam the far end of the runway, flying back towards the start. The tug climbs
# straight ahead first, as far as it needs to reach 250 m there with the climb rate of the combination,
# turns 90 degrees away from the runway (30 degrees of bank), flies a short crosswind leg if the 500 m
# are not already used up by the turns, turns 90 degrees again onto the downwind heading and releases when
# the height is reached.
var tow_types = {
	"500 m tow":                       {height: 500,  pattern: 0},
	"1000 m tow":                      {height: 1000, pattern: 0},
	"landing pattern, left (250 m)":   {height: 250,  pattern: -1},
	"landing pattern, right (250 m)":  {height: 250,  pattern: 1},
	"custom height (slider)":          {height: nil,  pattern: 0},
};
var pattern_bank_deg = 30;                              # bank of the turns of a pattern tow
var pattern_offset_m = 500;                             # distance of the downwind leg from the runway
var pat = {phase: 0, planned: 0, side: -1, S: nil, H: 0, L: 1000, xT: 0, c: 0, mark: nil, avoids: 0};

var towTypeChanged = func(n) {
	var v = n.getValue();
	var t = (v == nil) ? nil : tow_types[v];
	if ( t == nil ) {
		t = tow_types["500 m tow"];
	}
	if ( t.height != nil ) {
		setprop("sim/glider/dragger/conf/glob_max_lift_height_m", t.height);
		setprop("sim/glider/dragger/glob/glob_max_lift_height_m", t.height);
	}
	setprop("sim/glider/dragger/conf/pattern", t.pattern);
}

var wrap180 = func(deg) {
	return math.mod(deg + 540, 360) - 180;
}

# the runway frame of the pattern: start point, runway heading and the distance from there to the far end
var patternSetup = func {
	var side = getprop("sim/glider/dragger/conf/pattern");
	var lat  = getprop("sim/glider/dragger/robot/wp0lat_deg");
	var lon  = getprop("sim/glider/dragger/robot/wp0lon_deg");
	pat.S = geo.Coord.new().set_latlon(lat, lon);
	pat.H = getprop("sim/glider/dragger/robot/wp0head_deg");
	pat.side = side;
	pat.L = 1000;                                       # without runway data
	pat.phase = 0;
	pat.planned = 0;
	pat.mark = nil;
	pat.avoids = 0;
	hold_alt_m = nil;
	hold_climb_mps = 0;
	var info = airportinfo(lat, lon);
	if ( info != nil ) {
		var best = 1e9;
		foreach ( var id; keys(info.runways) ) {
			var r = info.runways[id];
			if ( math.abs(wrap180(r.heading - pat.H)) > 20 ) {
				continue;
			}
			var thr = geo.Coord.new().set_latlon(r.lat, r.lon);
			var d = thr.distance_to(pat.S);
			var a = (thr.course_to(pat.S) - r.heading) * D2R;
			var along = d * math.cos(a);
			var across = math.abs(d * math.sin(a));
			if ( across < 100 and along > -200 and along < r.length and across < best ) {
				best = across;
				pat.L = r.length - along;
			}
		}
	}
	ask21.msg("ai-plane", "landing pattern tow, " ~ (side < 0 ? "left" : "right") ~ ": I take you to the start of the downwind.");
}

var patternDragRobot = func {
	var dragpos_geo = geo.Coord.new();
	var deltatime_s = (dragrobot_timeincrement_s == 0) ? getprop("sim/time/delta-sec") : dragrobot_timeincrement_s;
	var rollrate = getprop("sim/glider/dragger/conf/glob_max_rollrate_degs");
	var max_speed = getprop("sim/glider/dragger/conf/glob_max_speed_mps");
	var max_climb = getprop("sim/glider/dragger/conf/glob_max_speed_lift_mps");
	if ( max_climb < 0.5 ) { max_climb = 0.5; }
	
	var oldspeed_mps   = getprop("ai/models/dragger/velocities/true-airspeed-kt") * KT2MPS;
	var oldheading_deg = getprop("ai/models/dragger/orientation/true-heading-deg");
	var oldroll_deg    = getprop("ai/models/dragger/orientation/roll-deg");
	var wind_from_east_mps = getprop("environment/wind-from-east-fps") * FT2M;
	var wind_from_nord_mps = getprop("environment/wind-from-north-fps") * FT2M;
	var wind_from_down_mps = getprop("environment/wind-from-down-fps") * FT2M;
	dragpos_geo.set_latlon( getprop("ai/models/dragger/position/latitude-deg"),
				getprop("ai/models/dragger/position/longitude-deg"),
				getprop("ai/models/dragger/position/altitude-ft") * FT2M);
	
	# position in the runway frame: x along the runway heading, y to the right of it
	var d = pat.S.distance_to(dragpos_geo);
	var b = (pat.S.course_to(dragpos_geo) - pat.H) * D2R;
	var x = d * math.cos(b);
	var y = d * math.sin(b);
	var exit_m = getprop("sim/glider/dragger/robot/exit_height_m");
	
	var turn_rate_degs = 57.2958 * 9.81 * math.tan(pattern_bank_deg * D2R) / math.max(oldspeed_mps, 25);
	
	if ( !pat.planned and pat.phase >= 0 ) {
		# where the first turn has to start so that the height is reached on the downwind leg
		var rho = max_speed * max_speed / (9.81 * math.tan(pattern_bank_deg * D2R));
		var path_m = 1.1 * max_speed * math.max(0, exit_m - dragpos_geo.alt()) / max_climb;
		# the sideways distance still to gain: from where it is now to 500 m on the pattern side
		var lateral_m = math.max(2 * rho, (pat.side * pattern_offset_m - y) * pat.side);
		pat.c = math.max(0, lateral_m - 2 * rho);
		pat.xT = (path_m + x + pat.L - math.pi * rho - pat.c) / 2;
		if ( pat.xT < x + 50 ) { pat.xT = x + 50; }
		pat.planned = 1;
	}
	
	var want = 0;
	if ( pat.phase == -1 ) {
		# after an avoidance: turn back onto the runway heading
		var back = wrap180(pat.H - oldheading_deg);
		var lead = 0.5 * turn_rate_degs * math.abs(oldroll_deg) / rollrate;
		if ( math.abs(back) <= lead + 2 ) {
			pat.phase = 0;
		} else {
			want = (back > 0 ? 1 : -1) * pattern_bank_deg;
		}
	}
	if ( pat.phase == 0 ) {
		if ( x >= pat.xT ) { pat.phase = 1; }
	}
	if ( pat.phase == 1 or pat.phase == 3 ) {
		var target = pat.H + pat.side * (pat.phase == 1 ? 90 : 180);
		var remaining = pat.side * wrap180(target - oldheading_deg);       # degrees still to turn
		var lead = 0.5 * turn_rate_degs * math.abs(oldroll_deg) / rollrate;
		if ( remaining <= lead + 1 ) {
			pat.phase += 1;
			pat.mark = geo.Coord.new(dragpos_geo);
			if ( pat.phase == 2 ) {
				# the crosswind leg makes up for what the first turn and the wind have not given: the second turn
				# moves the dragger sideways by about one turn radius again
				var rho_now = oldspeed_mps * oldspeed_mps / (9.81 * math.tan(pattern_bank_deg * D2R));
				pat.c = math.max(0, (pat.side * pattern_offset_m - y) * pat.side - 1.35 * rho_now);   # (the dragger is still rolling out of the first turn here, so the second turn gives a little more than a radius)
			}
		} else {
			want = pat.side * pattern_bank_deg;
		}
	}
	if ( pat.phase == 2 ) {
		if ( pat.mark == nil or pat.mark.distance_to(dragpos_geo) >= pat.c ) {
			pat.phase = 3;
		}
	}
	
	var dr = want - oldroll_deg;
	var lim = rollrate * deltatime_s;
	if ( dr > lim ) { dr = lim; }
	if ( dr < -lim ) { dr = -lim; }
	var newroll_deg = oldroll_deg + dr;
	
	var accel_mpss = tugAccel(oldspeed_mps, deltatime_s);
	var newspeed_mps = oldspeed_mps + accel_mpss * deltatime_s;
	var distance_m = oldspeed_mps * deltatime_s + 0.5 * accel_mpss * deltatime_s * deltatime_s;
	distance_m += loadResponse(deltatime_s) * deltatime_s;
	var newlift_mps = liftModel(oldspeed_mps, getprop("ai/models/dragger/velocities/vertical-speed-fps") * FT2M, dragpos_geo, deltatime_s);
	var newliftdist_m = (newlift_mps + wind_from_down_mps) * deltatime_s;
	
	var newheading_deg = math.mod(oldheading_deg + turn_rate_degs * newroll_deg / pattern_bank_deg * deltatime_s + 360, 360);
	dragpos_geo.apply_course_distance( newheading_deg , distance_m );
	dragpos_geo.apply_course_distance( 270.0 , wind_from_east_mps * deltatime_s );
	dragpos_geo.apply_course_distance( 180.0 , wind_from_nord_mps * deltatime_s );
	dragpos_geo.set_alt(dragpos_geo.alt() + newliftdist_m);
	
	setprop("ai/models/dragger/position/latitude-deg",         dragpos_geo.lat());
	setprop("ai/models/dragger/position/longitude-deg",        dragpos_geo.lon());
	setprop("ai/models/dragger/position/altitude-ft",          dragpos_geo.alt() * M2FT);
	setprop("ai/models/dragger/orientation/true-heading-deg",  newheading_deg);
	setprop("ai/models/dragger/orientation/roll-deg",          newroll_deg);
	setprop("ai/models/dragger/velocities/true-airspeed-kt",   newspeed_mps * MPS2KT);
	setprop("ai/models/dragger/velocities/vertical-speed-fps", newlift_mps * M2FT);
	
	# the target height is reached: level off and hold it while flying on to the start of the downwind leg
	if ( dragpos_geo.alt() > exit_m - 1 and hold_alt_m == nil ) {
		hold_alt_m = exit_m;
	}
	# there: release, like at the end of a normal tow
	if ( hold_alt_m != nil and pat.phase == 4 and x <= pat.L + 30 and sent == 0 ) {
		ask21.msg("ai-plane", " we have reached the downwind start, bye bye");
		sent = 1;
		settimer(func(){
				if ( getprop("sim/glider/dragger/flags/exist") != 1 ) { return; }   # cancelled meanwhile
				hold_alt_m = nil;
				hold_climb_mps = 0;
				setprop("sim/glider/dragger/robot/leg_type", 3);
				setprop("sim/glider/dragger/robot/leg_segment", 2);
				ask21.msg("ai-plane", "turn right, I turn left" );
				towing.releaseHitch("aerotow");
			}, 2);
	}
}

var stopRobot = func {
	# stop loop for updating roboter
	if ( getprop("sim/glider/dragger/flags/run") == 1 ) {
		setprop("sim/glider/dragger/flags/run", 0);
	}
	
	
	# reseting all variables and position to initial values
	setprop("ai/models/dragger/position/latitude-deg", getprop("sim/glider/dragger/robot/wp0lat_deg") );
	setprop("ai/models/dragger/position/longitude-deg", getprop("sim/glider/dragger/robot/wp0lon_deg") );
	setprop("ai/models/dragger/position/altitude-ft", getprop("sim/glider/dragger/robot/wp0alt_m")  * M2FT - 33);
	setprop("ai/models/dragger/orientation/true-heading-deg", getprop("sim/glider/dragger/robot/wp0head_deg") );
	setprop("ai/models/dragger/orientation/roll-deg",          0);
	setprop("ai/models/dragger/velocities/true-airspeed-kt",   0);
	setprop("ai/models/dragger/velocities/vertical-speed-fps", 0);
	setprop("sim/glider/dragger/robot/leg_type", 0);
	setprop("sim/glider/dragger/robot/leg_segment", 0);
	removeDragRobot();
}


# ####################################################################################
# function to switch the drag roboter on or off running
# used by key "d" and gui
var startDragRobot = func {
	if ( getprop("sim/glider/dragger/flags/run" ) == 1) {
		setprop("sim/glider/dragger/flags/run", 0);
		print(" stop the drag robot");
	} else { 
		print(" start the drag robot");
		setprop("sim/glider/dragger/flags/run", 1);
	}
}



# ####################################################################################
# triggered function to run the drag roboter
var runDragRobot = func {
	if ( getprop("sim/glider/dragger/flags/run" ) == 1) {
		var leg = -1;
		leg = getprop("sim/glider/dragger/robot/leg_type");
		
		if ( leg == 0 ) { 
			leg0DragRobot();
		}
		
		if ( (leg == 1 or leg == 2 or leg == 4) and !last_leg and terrainGuard(leg) ) {
			avoidDragRobot();
		} else {
			if ( leg == 1 ) { 
				leg1DragRobot();
			}
			
			if ( leg == 2 ) { 
				leg2DragRobot();
			}
			
			if ( leg == 4 ) { 
				patternDragRobot();
			}
		}
		
		if ( leg == 3 ) { 
			leg3DragRobot();
		}
	
		settimer(runDragRobot, dragrobot_timeincrement_s);
	}
}



# ####################################################################################
var pulling = setlistener("sim/glider/dragger/flags/run", runDragRobot);
var initializing_dragrobot = setlistener("sim/signals/fdm-initialized", func {
	initRobotAttributes();
	setlistener("sim/glider/dragger/conf/tow_type", towTypeChanged, 1, 0);
});
