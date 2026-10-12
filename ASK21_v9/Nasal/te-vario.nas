# Total Energy compensated variometers
io.include("Aircraft/Generic/soaring-instrumentation-sdk.nas");

# Probe from ilec-sc7.nas

# Add second (rear) vario, first one is added by ilec-sc7.nas
var rear_vario = Dampener.new(
			input: probe,
			dampening: 2.3,
			on_update: update_prop("/instrumentation/variometer[1]/te-reading-mps"));
	
var instrument = Instrument.new(
	components: [ rear_vario ],
	enable: 1);
