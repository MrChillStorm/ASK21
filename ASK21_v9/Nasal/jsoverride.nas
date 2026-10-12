# Button 1 of the first joystick releases the tow cable (see the help). The button's own
# bindings are replaced, not added to, and the option "Joystick button 1 releases the
# cable" (Simulation Options, kept between sessions) switches this off from the next
# start: FlightGear reads the joystick bindings when it starts.

var enabled = props.globals.initNode("/sim/model/joystick-release", 1, "BOOL");

var set_binding = func(node, prop, value) {
	node.getNode("command", 1).setValue("property-assign");
	node.getNode("property", 1).setValue(prop);
	node.getNode("value", 1).setValue(value);
}

var jsoverride = func {
	if (!enabled.getBoolValue()) return;

	var button = props.globals.getNode("/input/joysticks/js/button");
	if (button == nil) return;

	print('Overriding button 1 on joystick "' ~ (getprop("/input/joysticks/js/name") or "") ~ '": tow release.');

	foreach (var b; button.getChildren("binding")) b.remove();
	var up = button.getNode("mod-up");
	if (up != nil) up.remove();

	set_binding(button.getChild("binding", 0, 1), "/sim/hitches/hook-open", "true");
	set_binding(button.getChild("binding", 1, 1), "/sim/hitches/aerotow/open", "true");
	set_binding(button.getChild("binding", 2, 1), "/sim/hitches/winch/open", "true");
	set_binding(button.getNode("mod-up", 1).getChild("binding", 0, 1), "/sim/hitches/hook-open", "false");
}

jsoverride();
