# Changelog

## 2026-11-06

Sound
- New: a slip sounds like an airbrake. Up to 50 degrees of sideslip the flow separates off the fuselage and the cockpit's walls and its wake falls on the wing and tail, in the cockpit and outside. In a full slip at 53 kt the cockpit is 8 dB(A) louder, and much more in the lows. These levels are estimates: no recording of a full slip has tested them yet.
- New: the canopy's wind comes from both sides of the cockpit, and in a slip the leeward side grows louder.
- New: the fixed wheels' noise outside, and the low roar measured in the cockpit.
- Changed: below 40 kt the airflow fades as the computed sound does: an 8 kt wind on the ground is 57 dB(A) in the cockpit, no longer 70.
- Changed: everything plays about 9 dB quieter in the cockpit and 6 dB outside, to leave room for a full slip: turn your system volume up to match. The cockpit and the outside now share one scale.
- Changed: only the airflow loops that can be heard run, at most 23 at a time.
- Removed: the radio's static.

## 2026-11-03

Sound
- Changed: the cockpit's airflow is darker and 1 to 3 dB quieter, 77 dB(A) at 40 kt and 88 at 150 kt: the canopy keeps the hiss's highs out as the mass law says, as recordings in an ASK 21's front cockpit show.

## 2026-11-02, evening

Flight model
- New: the wings bend, each panel in its first bending mode, with the stiffness of a ground vibration test of the real glider. The bending acts back on the flight: it adds dihedral and lifts the wing in a pull-up.
- New: the tip skids and the ground-handling wing supports follow the bent wing tips.
- Changed: the JSBSim versions run at 240 Hz instead of 120. Borderline spins depended on the step size at 120 Hz; at 240 Hz they no longer do.
- Changed: the post-stall parameters are fitted again, with the bending and at 240 Hz. The AFFTC spin map matches at 20 of 24 loadings, and at forward centre of gravity the elevator now runs out before the stall, as the AFFTC found.
- Changed: the flight model costs FlightGear about half as much. It reads half as many properties per step, and the tug's wake costs nothing when no tug is near.
- Fixed: the JSBSim motorglider had the original model's mass and inertia, too small for flexible wings, and diverged in roll within a second. It now has the glider's, plus its power plant.

3-D model
- New: the wings bend as the flight model's do, in multiplayer too, and the wing cameras ride on them.

Sound
- New: the airflow is computed by PyWopWop from the glider's shape: the canopy's hiss, the wing and tail's noise, the airbrakes, the ailerons and the stall, in the cockpit and outside, with its level and pitch following the altitude up to 10 km.
- New: the airflow covers aerobatics: the stall when pulling up to 5 g, inverted flight and its stall, and the canopy's wind in a slip.
- Changed: every other sound plays at the level the real thing makes: tyres, touchdown, wheel brake, airbrake lock, releases, windows, varios. The tyres and releases are heard outside too.
- Changed: an open sliding window adds the wind through it.
- Removed: the original wind and airbrake sounds, the howl at low speed and the whoosh when pitching.

Cockpit
- Fixed: the rear seat's vario never moved.
- Changed: the V8 works out the speed to fly about ten times faster, with the same result.

## 2026-11-02

Cockpit
- Changed: the V8 speaks with FlightGear's own speech synthesis, on every system and with no helper program.
- Removed: the V8's recorded voice clips and the macOS speech helper (`tools/lxv8-speak.py`).

## 2026-10-14

Aerotow
- New: the drag robot crabs into the wind to hold its planned track, instead of pointing its nose along it.
- New: in turns the drag robot keeps its crab angle, so it drifts with the wind like the glider, and settles onto the new angle after roll-out.
- Changed: the tug's wake follows the direction the tug's nose points.
- Fixed: the terrain look-ahead ignored the wind. It now flies the same crab and drift as the robot.

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
