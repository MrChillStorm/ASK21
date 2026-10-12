"""Make the v9 ask21-jsb.xml from the v8 file: mass balance on the AFFTC arms, real control travels,
no stick map (the derived pitch model trims near neutral elevator)."""
import re, os, sys, geometry as G, genxml

SRC = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'ask21-jsb-v8-base.xml')


def build(iyy=1446.0, ixx=6518.0, izz=7403.0, src=SRC, emptywt=794.0):
    s = open(src).read()
    # ---- nose-wheel contact: the manufacturer's rigging drawing (flight manual p. 54, levelled attitude) has the nose- and
    # main-wheel bottoms level within 1 cm; in this body frame (tailplane +0.73 deg, root chord +2.48 deg) that puts the nose
    # contact 1.9 cm above the main contact.  The FlightGear 3-D model (and the v8 file) has it 7.3 cm above, which tilts the
    # glider about 1.4 deg nose-down on its wheels.
    a = s.index('name="FRONT_WHEEL"'); b = s.index('</location>', a)
    blk = s[a:b]
    import re as _re
    blk2 = _re.sub(r'<z>\s*0\.07763\s*</z>', '<z> 0.024 </z>', blk)
    assert blk2 != blk, 'nose wheel z not found'
    s = s[:a] + blk2 + s[b:]
    # ---- metrics: MAC (AFFTC 1121 mm)
    s = s.replace('<chord unit="FT">    3.46 </chord>', '<chord unit="FT">    3.68 </chord>')
    # ---- mass balance
    a = s.index('<mass_balance>'); b = s.index('</mass_balance>') + len('</mass_balance>')
    fx, rx, tx, cx = G.PILOT_FRONT[0], G.PILOT_REAR[0], G.TAIL_BALLAST[0], G.CKPT_BALLAST[0]
    mb = f'''<mass_balance>
		<!-- v9: empty-aircraft inertias about the empty CG, AFFTC-TR-89-27 App. C (Ixx 30.2e6, Iyy 6.7e6, Izz 34.3e6 lb-in2).
		     Point masses on the AFFTC / flight-manual moment arms (report datum = wing root leading edge = x -1.637 m):
		     front seat -47.91 in, rear seat -3.15 in, tail ballast 209.8 in, cockpit ballast -63.39 in. -->
		<ixx unit="SLUG*FT2">{ixx:.0f}</ixx>
		<iyy unit="SLUG*FT2">{iyy:.0f}</iyy>
		<izz unit="SLUG*FT2">{izz:.0f}</izz>
		<emptywt unit="LBS">       {emptywt:.0f} </emptywt>
		<location name="CG" unit="M">
			<!-- 29.02 in aft of the datum (AFFTC, S/N 21235) -->
			<x>  -0.9 </x>
			<y>   0.00 </y>
			<z>  0.76674 </z>
		</location>
		<pointmass name="Pilot">
			<description> front seat occupant incl. parachute (arm -47.91 in) </description>
			<weight unit="KG">    80 </weight>
			<location name="POINTMASS" unit="M">
				<x> {fx:.4f} </x>
				<y>   0.00 </y>
				<z>  {G.PILOT_FRONT[2]:.2f} </z>
			</location>
		</pointmass>
		<pointmass name="Copilot">
			<description> rear seat occupant incl. parachute (arm -3.15 in) </description>
			<weight unit="KG">    80 </weight>
			<location name="POINTMASS" unit="M">
				<x> {rx:.4f} </x>
				<y>   0.00 </y>
				<z>  {G.PILOT_REAR[2]:.2f} </z>
			</location>
		</pointmass>
		<pointmass name="Spin Ballast">
			<description> tail (spin) ballast plates at the bottom of the fin, 1 kg each, max 12 (arm 209.8 in) </description>
			<weight unit="KG">   0 </weight>
			<location name="POINTMASS" unit="M">
				<x> {tx:.4f} </x>
				<y>   0.00 </y>
				<z>  {G.TAIL_BALLAST[2]:.2f} </z>
			</location>
		</pointmass>
		<pointmass name="Cockpit Ballast">
			<description> front-seat trim ballast (lead discs, arm -63.39 in) </description>
			<weight unit="KG">   0 </weight>
			<location name="POINTMASS" unit="M">
				<x> {cx:.4f} </x>
				<y>   0.00 </y>
				<z>  {G.CKPT_BALLAST[2]:.2f} </z>
			</location>
		</pointmass>
	</mass_balance>'''
    s = s[:a] + mb + s[b:]
    # ---- pitch channel: no stick map; elevator travel 22.6 up / 16.2 down (rigging data)
    a = s.index('<!-- v8 elevator map.'); b = s.index('</channel>', a) + len('</channel>')
    e0 = genxml.DEFAULTS['elev0']
    pitch = f'''<!-- v9: stick + trim give the elevator position directly (the v8 stick map is gone).  Travel 22.6 deg up /
		     16.2 deg down (flight manual rigging data, 90/65 mm at R 230 mm).  Centred stick with the trim neutral puts the
		     elevator {-e0:.1f} deg up: not documented, chosen so that the stick-free aerotow take-off with the trim neutral
		     (flight manual) lifts off at about 75-80 km/h with two pilots, before the tug.
		     fcs/elevator-pos-rad is the true angle, TE down positive (the aero file reads fcs/elevator-pos-norm). -->
		<channel name="Pitch">
			<summer name="Pitch Trim Sum">
				<input>fcs/elevator-cmd-norm</input>
				<input>fcs/pitch-trim-cmd-norm</input>
				<clipto>
					<min> -1 </min>
					<max>  1 </max>
				</clipto>
			</summer>
			<pure_gain name="fcs/elevator-pos-norm">
				<input>fcs/pitch-trim-sum</input>
				<gain>1</gain>
			</pure_gain>
			<fcs_function name="fcs/elevator-pos-rad">
				<function><table><independentVar lookup="row">fcs/elevator-pos-norm</independentVar><tableData>
					-1 {-G.ELEV_UP * 0.0174533:.4f}
					 0 {e0 * 0.0174533:.4f}
					 1  {G.ELEV_DN * 0.0174533:.4f}
				</tableData></table></function>
			</fcs_function>
		</channel>'''
    s = s[:a] + pitch + s[b:]
    # ---- roll: travel 26.5 up / 10.8 down (110/45 mm at R 240), differential handled in the aero file
    a = s.index('<channel name="Roll">'); b = s.index('</channel>', a) + len('</channel>')
    roll = '''<channel name="Roll">
			<!-- v9: aileron travel 26.5 deg up / 10.8 deg down (rigging data 110/45 mm at R 240 mm).  The aero file
			     reads the normalized positions and applies the differential. pos-rad = TE down positive. -->
			<summer name="Roll Trim Sum">
				<input>fcs/aileron-cmd-norm</input>
				<input>fcs/roll-trim-cmd-norm</input>
				<clipto>
					<min> -1 </min>
					<max>  1 </max>
				</clipto>
			</summer>
			<pure_gain name="fcs/left-aileron-pos-norm">
				<input>fcs/roll-trim-sum</input>
				<gain>1</gain>
			</pure_gain>
			<pure_gain name="fcs/right-aileron-pos-norm">
				<input>fcs/roll-trim-sum</input>
				<gain>1</gain>
			</pure_gain>
			<fcs_function name="fcs/left-aileron-pos-rad">
				<function><table><independentVar lookup="row">fcs/left-aileron-pos-norm</independentVar><tableData>
					-1 -0.4625
					 0  0
					 1  0.1885
				</tableData></table></function>
			</fcs_function>
			<fcs_function name="fcs/right-aileron-pos-rad">
				<function><table><independentVar lookup="row">fcs/right-aileron-pos-norm</independentVar><tableData>
					-1  0.1885
					 0  0
					 1 -0.4625
				</tableData></table></function>
			</fcs_function>
		</channel>'''
    s = s[:a] + roll + s[b:]
    a = s.index('<channel name="Yaw">'); b = s.index('</channel>', a) + len('</channel>')
    yaw = '''<channel name="Yaw">
			<!-- v9: rudder +-27.8 deg (rigging data 180 mm at R 375 mm); JSBSim convention, positive = TE left -->
			<summer name="Rudder Command Sum">
				<input>fcs/rudder-cmd-norm</input>
				<input>fcs/yaw-trim-cmd-norm</input>
				<clipto>
					<min> -1 </min>
					<max>  1 </max>
				</clipto>
			</summer>
			<pure_gain name="fcs/rudder-pos-norm">
				<input>fcs/rudder-command-sum</input>
				<gain>1</gain>
			</pure_gain>
			<pure_gain name="fcs/rudder-pos-rad">
				<input>fcs/rudder-pos-norm</input>
				<gain>0.4852</gain>
			</pure_gain>
		</channel>'''
    s = s[:a] + yaw + s[b:]
    s = s.replace('<description> Models a ASK21. </description>', '<description> Models a ASK21. v9 aerodynamics: component build-up (Systems/ask21-aero.xml) </description>')
    return s


if __name__ == '__main__':
    out = sys.argv[1] if len(sys.argv) > 1 else 'ask21-jsb-v9.xml'
    open(out, 'w').write(build())
    open("jsb_test.xml", "w").write(build(emptywt=851.0, iyy=1507.0))
    print('wrote', out)
