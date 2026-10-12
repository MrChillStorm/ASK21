# Changelog

## 2026-10-12, evening

Cockpit
- Fixed: the turn indicator was hidden behind a panel patch meant only for the transponder.
- Changed: the turn indicator has the same screws as the other instruments.
- Changed: the transponder option reads "Enable transponder" and fits the Simulation Options window.
- New: yaw strings made of real geometry, so they show on macOS and lie along the canopy.
- New: each yaw string shows the airflow at its own spot, including the swing of the nose in turns.
- New: the yaw strings quiver like wool and react to gusts from the simulated wind.
- New: V8 netto and speed to fly use the flight manual polar, scaled with mass, g-load and altitude.
- Fixed: the V8's total energy compensation used indicated instead of true airspeed.
- New: the airspeed indicator drops to zero in a full slip, as the flight manual says.

Aerotow
- New: the tug's wake is real airflow: its slipstream and wingtip vortices, where the tug flew.
- Removed: the random shake forces that stood in for the prop wash.

Fixes to the original aircraft
- Fixed: the registration letters never followed the callsign or the Immatriculation dialog.
- Fixed: the battery froze at 10 % and the low-battery warning never showed.
- Fixed: the ammeter always showed charging.
- Fixed: a blocked pitot gave zero pressure instead of a slow leak.
- Fixed: the g, VNE and weight warnings stopped working after a replay.
- Fixed: the weight warning never worked with JSBSim.
- New: a warning for the airbrake load limit of +3.5 g / 0 g.
- Fixed: the tutorial took a QFE altimeter setting for a mistake.
- Fixed: joystick button 1 added the tow release to its old bindings instead of replacing them.
- New: an option to stop joystick button 1 from releasing the cable.

## 2026-10-12, first release

- New: the v9 JSBSim flight model, built up from the glider's geometry, airfoils and the AFFTC tests.
- New: the report on the model, and the scripts that rebuild the model and the report.
- Fixed: the nose wheel sits at the rigging drawing's height in all four flight models and the 3-D model.
- New: a reworked cockpit with an LXNAV V8 vario, a take-off check placard and a transponder option.
- New: drag robot tow types: 500 m, 1000 m, and landing pattern left or right.
- New: the drag robot turns away from terrain ahead.
- New: the drag robot reacts to the pull of the rope.
- Changed: the aircraft folder is called `ASK21_v9`, so it installs next to the standard ASK 21.
