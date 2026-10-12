# ASK 21 for FlightGear, with a JSBSim model built from the test reports

The Schleicher ASK 21 two-seat trainer from FlightGear's aircraft
collection, with a new JSBSim flight model. The aerodynamics are built up
piece by piece from the glider's geometry and airfoils, and then checked
against what the real glider is documented to do:

- the US Air Force Flight Test Center's stall and spin evaluation
  (AFFTC-TR-89-27, 1989),
- the manufacturer's flight manuals, including the speed polar and the
  rigging data,
- a few things every ASK 21 pilot knows, like the glider lifting off
  hands-free before the tug does.

How it was built, and a test-by-test comparison of this model and the
standard FlightGear one against those sources, is in the report:
[ASK21-JSBSim-model-report.pdf](ASK21-JSBSim-model-report.pdf).

The cockpit is reworked too, around a new LXNAV V8 vario, and the drag
robot tows like a real tug: it reacts to the pull of the rope, keeps
clear of hills and can take you around a landing pattern.

## Install

Get the files:

```bash
git clone https://github.com/MrChillStorm/ASK21.git
```

Then add the cloned `ASK21` folder in the FlightGear launcher under
Add-ons → Additional aircraft folders (or start FlightGear with
`--fg-aircraft=` pointing at it), and pick **ASK 21 Glider JSB** or
**ASK 21mi Motorglider JSB**.

- The aircraft itself is in the `ASK21_v9` folder, so it sits next to
  the standard ASK 21 from the hangar without replacing it.
- In multiplayer, other pilots need this aircraft installed to see your
  glider as an ASK 21.
- The YASim variants (**ASK 21 Glider (YASim)** and the YASim
  motorglider) keep their original flight model, apart from the nose
  wheel (see below). The cockpit and the tow work the same in them;
  only the prop wash needs JSBSim.

## Flying it

- **Take-off.** As in the real glider, set a little nose-down trim for
  the aerotow: it lifts off by itself before the tug, at about 75–80
  km/h. With the trim neutral it floats up early and you hold it down
  with a slight push.
- **Stalls and spins.** The stall breaks cleanly and can pitch down hard
  if you go looking for it. Whether it spins, and for how long, depends
  on the loading the same way as in the AFFTC tests: at forward centre
  of gravity it won't keep spinning, at aft centre of gravity it does.
  The manual's recovery stops the spin in half to three-quarters of a
  turn.
- **Slips.** Full rudder gives up to about 45° of sideslip and sinks
  faster than full airbrakes at the same speed.
- **Ground.** The glider rests on its nose and main wheels at the
  attitude of the manufacturer's rigging drawing, where both wheels are
  level. The standard model sits about 1.4° more nose-down.

## Cockpit

The front panel of the glider:

- **LXNAV V8 vario**, a new instrument in place of the ILEC SC7 vario
  and the separate G-meter. Pages for climb (vario, netto, relative and
  average), MacCready speed to fly, horizon, wind, FLARM radar, thermal
  assistant and G-meter, the vario and speed-to-fly audio, and the setup
  menu. Click the screen, or use ASK21 → LXNAV V8 close-up, for a larger
  view. It powers from the battery like the real unit.
- **Spoken messages** like "Thermal average 2.1" or "Traffic 2 o'clock,
  high, 400 meters" play as recorded clips. The buttons and menus are
  described in
  [the V8's README](ASK21_v9/Models/Interior/Panel/Instruments/lx-v8/README.md).
- **Take-off check placard** at the bottom right: loading, canopy,
  straps, controls, trim, airbrakes, instruments, radio, rope, wind.
- **Transponder** (BXP 6401) in place of the turn indicator. The turn
  indicator comes back with the option in ASK21 → Simulation Options.
- The altimeter sits under the airspeed indicator and the mechanical
  vario at the top right. The FLARM display at the top of the panel is
  gone; its traffic is on the V8's FLARM page.
- Black leather panels and glare shields, and all instruments' screws in
  one style. The scripts in `ASK21_v9/Models/Interior/Panel/src` make
  these from copies of the original meshes.

The motorglider's front panel gets the same changes, with the V8 at the
bottom right and no placard. The rear panel gets the leather and the
screws; its instruments stay as they were.

## Aerotow

The drag robot (ASK21 → Drag Robot) is the AI tug that tows you up.

- **Tow types.** 500 m and 1000 m tows climb to that height above the
  start with a few turns on the way, and release you there. The landing
  pattern tows, left or right, take you to 250 m at the start of the
  downwind leg: 500 m beside the runway, abeam its far end, heading back
  to the start. A custom height on the slider is still there.
- **Terrain.** Every second the tug looks 100 seconds ahead along the
  path it plans to fly. If that path doesn't clear the ground, it turns
  away the shorter way and rolls out when the way ahead is clear. A
  landing pattern tow then plans its pattern again, or turns into a
  normal tow when there's no room for one. The check can be switched
  off under Advanced Drag Robot Configuration. Either way, the tug makes
  no turns until it is 50 m above the start.
- **No more rubber band.** The tug feels the rope: a sudden pull slows it
  down the way it would slow a real 850 kg tug, and it gets back to
  speed over about 6 seconds. Its acceleration also builds up and fades
  out gently. Together these stop the glider surging back and forth on
  the rope.
- **Take-off run.** The tug lines up with the runway it stands on, lifts
  off at 100 km/h, stays low in ground effect until 115 km/h and then
  builds up its climb to at most 2.5 m/s. All of these can be changed
  under Advanced Drag Robot Configuration.
- **Prop wash.** Flying in the tug's slipstream shakes the glider; high
  tow is calm. The strength is set under Aerotow Settings.
- **Rope.** 50 m long instead of 60 m, because the rope stretches under
  the tug's pull: at 50 m it tows at about 60 m.
- If the chosen tug model isn't installed, the robot uses the next one it
  finds (DR400, PA28, Ercoupe, Staggerwing).

## How it compares

Every number below comes from the same scripted test programme flown
with both models (details and sources in the report).

| | Source | Standard FlightGear model | This model |
|---|---|---|---|
| Polar rms error, 470 / 570 kg | flight manual | 0.09 / 0.17 m/s | 0.025 / 0.028 m/s |
| Best glide ratio | 33.5 | 42 | 32.6–32.8 |
| Stall speed single / dual | 65 / 74 km/h | 60.3 / 69.3 | 63.7 / 72.4 |
| Neutral point | aft of 18.46 in | 16.5 in (unstable at aft CG) | 21.9 in |
| Inverted −1 g stall | 38–40 KIAS | 54 | 40.7 |
| AFFTC spin map, 24 loadings | AFFTC | 2 right (spiral dives) | 22 right |
| Spin rate / height lost per turn | 80–140 °/s / 200 ft | no spin | 94 °/s / 313 ft |
| Recovery with the manual's procedure | under 1 turn | no spin | 0.50–0.73 turn |
| Full-rudder sideslip | up to 50° | 16° | 45° |
| Full-aileron roll, first 180° | about 5 s | 6.2–6.6 s | 4.4–4.7 s |
| Hands-free aerotow lift-off (tug 85 km/h) | before the tug, about 75 | 91–108 km/h | 65–79 km/h |
| 45° airbrake dive at 600 kg | 232 km/h | 194 | 232 |

## Known gaps

- **Phugoid.** Below about 105 km/h the slow pitch-and-speed
  oscillation is very slightly unstable on paper: with the stick held
  perfectly still it grows over a few cycles. In flight it doesn't show
  as porpoising; you damp it without noticing.
- **Spins** lose about 313 ft per turn against the AFFTC's 200 ft, and
  the nose bobs more than reported.
- **Stalls** at the most forward centre of gravity still break, where
  the real glider runs out of elevator first. Dynamic stalls from a
  steep nose-up attitude drop the nose further than reported.
- **Not modelled:** controls that float free when released, and
  airspeed indicator errors.

## Rebuilding the model and the report

The [`fdm`](fdm) folder holds everything that produced the flight model
and the report, and regenerates both exactly. It needs Python 3.12 with
`jsbsim` 1.3.2, `numpy`, `scipy`, `matplotlib`, `aerosandbox`,
`neuralfoil` and `cma`, and [tectonic](https://tectonic-typesetting.github.io)
or another XeLaTeX for the PDF.

```bash
cd fdm
python3 genxml.py ../ASK21_v9/Systems/ask21-aero.xml "$(cat v9_params.json)"
python3 genjsb.py ../ASK21_v9/ask21-jsb.xml
```

- `genxml.py` writes the aerodynamics: 22 wing strips, 6 tailplane and 3
  fin strips, and 11 fuselage stations, with the parameters in
  `v9_params.json`. `genjsb.py` writes the mass, gear and controls file.
- `campaign.py v9` and `campaign.py fg` fly the 186-point test programme
  with each model (under a minute on 8 cores) and store the results in
  `results_v9.json` and `results_fg.json`.
- `figures.py`, `mktables.py`, `paradox.py`, `mknumbers.py`, `mkscore.py`
  and `mktab_a1.py` turn the results into every figure, table and number
  in the report; `tectonic ask21_report.tex` builds the PDF.
- `optimize.py` and `optimize2.py` with `objective.py` identified the
  post-stall parameters against the AFFTC spin results; `opt_final.json`
  is the chosen candidate.
- `fg_*.xml` are the unmodified files of the standard FlightGear model,
  used for the comparison.

The flight manuals and the AFFTC report are not included; the report
lists them as sources.

## Changes to the FlightGear aircraft

Based on the ASK 21 in FlightGear's aircraft repository (FGAddon,
revision 8166). Changed in 2026, all inside `ASK21_v9`:

- **Folder.** Renamed from `ASK21` to `ASK21_v9`; the `Aircraft/ASK21/`
  paths inside its files point to the new name.
- **Flight model.** `Systems/ask21-aero.xml` is replaced by the
  generated aerodynamics (also used by the JSBSim motorglider).
  `ask21-jsb.xml` gets the mass properties and inertias from the AFFTC
  report, a cockpit ballast station, the control travels from the
  rigging data and the pitch control without a stick map. Both JSBSim
  files get the prop wash forces.
- **Nose wheel.** 5.4 cm lower in all four flight models (`ask21-jsb.xml`,
  `ask21mi-jsb.xml`, `ask21.xml`, `ask21mi.xml`) and in the 3-D model
  (`Models/ask21.xml`), so the glider sits as the rigging drawing shows.
- **Cockpit.** New: the V8 (`Models/Interior/Panel/Instruments/lx-v8`,
  with `Sounds/lxv8-tone.wav`, `Sounds/lxv8-voice/` and
  `tools/lxv8-speak.py`), the placard (`Instruments/placard`) and the
  scripts in `Models/Interior/Panel/src`. Changed: the panels
  (`Models/Interior/Panel/panel*.xml` and `panel*.ac`), the glare
  shields in `Models/Interior/interior.ac`, the screws and textures of
  the airspeed indicator, altimeter, varios and compass, the V8's sounds
  in `Sounds/ask21-sound.xml`, the V8 and transponder loads in
  `Nasal/electrical.nas` and `Nasal/electrical-mi.nas`, the transponder
  in `Nasal/ask21.nas` and `Systems/ask21-instrumentation.xml`, its
  option in `gui/dialogs/options.xml`, and the menu entry, settings and
  scripts in `ask21-main.xml`.
- **Aerotow.** `Nasal/dragrobot.nas` (tow types, terrain, the tug's
  reaction to the rope, take-off run) with its dialogs
  `gui/dialogs/dragrobot.xml` and `dragrobot_advanced.xml`, the new
  `Nasal/propwash.nas` with its strength slider in
  `gui/dialogs/aerotow.xml`, and the 50 m rope in `ask21-main.xml`.

Everything else is as its authors made it; they are credited in
[ASK21_v9/README.md](ASK21_v9/README.md).

## Licence

GNU General Public License, version 2 or (at your option) any later
version, the same as the original aircraft. See [COPYING](COPYING).
