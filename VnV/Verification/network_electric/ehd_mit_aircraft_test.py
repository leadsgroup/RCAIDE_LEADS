# VnV/Verification/network_electric/ehd_mit_aircraft_test.py
#
# Created:  Oct 2026, RCAIDE EHD MVP
#
# Whole-aircraft EHD test modelled on the MIT solid-state aircraft (Xu et al., Nature 563, 2018).
# NOT a validation case: the collector chord, unit spacing, unit count, wing geometry and battery are
# unconfirmed (spec 2.4.1) and are guessed in the PLACEHOLDERS block below. The tests check a thrust
# sweep, steady level flight with power and state-of-charge closure, and an OpenVSP export that draws
# the electrodes. Replace the placeholders with Extended Data Fig. 1 / Table 1 values before comparing
# against the aircraft (spec 3.5 Stage 4).

# ----------------------------------------------------------------------------------------------------------------------
#  IMPORT
# ----------------------------------------------------------------------------------------------------------------------
import RCAIDE
from RCAIDE.Framework.Core                                      import Units, Data
from RCAIDE.Library.Methods.Powertrain                          import setup_operating_conditions
from RCAIDE.Library.Methods.Powertrain.Propulsors.EHD_Thruster  import compute_ehd_thruster_mass

import numpy as np
import pytest
import os

# ----------------------------------------------------------------------------------------------------------------------
#  Aircraft data
# ----------------------------------------------------------------------------------------------------------------------
# Sourced (spec 2.4.1, R17, Part 2.1)
MIT = Data()
MIT.wingspan              = 5.0        # m, Xu et al. 2018
MIT.mass                  = 2.45       # kg, Barrett in The Conversation (secondary)
MIT.gap                   = 0.060      # m, Extended Data Fig. 1
MIT.emitter_diameter      = 0.2e-3     # m, 32 AWG stainless, Extended Data Fig. 1
MIT.electrode_span        = 3.0        # m, Extended Data Fig. 1
MIT.maximum_voltage       = 40.3e3     # V, HVPC regulated output in flight, Extended Data Fig. 2
MIT.hvpc_rated_power      = 600.       # W, He, Woolston & Perreault 2017 (R17)
MIT.flight_distance       = 60.        # m, ~60 m in 10-12 s (secondary)
MIT.flight_speed          = 5.0        # m/s, derived from the line above
MIT.reported_thrust       = 3.2        # N, secondary source, [UNVERIFIED]

# PLACEHOLDERS: guesses, not data. Replace before using this as a validation case.
GUESS = Data()
GUESS.unit_rows           = 4          # "two columns of four rows" (forum post, secondary)
GUESS.unit_columns        = 2
GUESS.unit_spacing        = 0.10       # m, the spec's own assumed S for the MIT-like test vector
GUESS.collector_chord     = 0.05       # m
GUESS.column_spacing      = 0.30       # m, streamwise distance between the two electrode columns
GUESS.electrode_top_z     = -0.15      # m, top electrode row below the wing
GUESS.wing_chord          = 0.40       # m, gives 2.0 m^2 with the 5 m span
GUESS.collector_foam_density       = 30.    # kg/m^3
GUESS.collector_foil_areal_density = 0.05   # kg/m^2 (about 18 um aluminium foil)
GUESS.battery_series_cells         = 50     # puts the bus inside the 160-225 V HVPC input window

# ----------------------------------------------------------------------------------------------------------------------
#  Vehicle
# ----------------------------------------------------------------------------------------------------------------------
def vehicle_setup():
    vehicle                                   = RCAIDE.Vehicle()
    vehicle.tag                               = 'MIT_like_EHD_aircraft'
    vehicle.mass_properties.max_takeoff       = MIT.mass
    vehicle.mass_properties.takeoff           = MIT.mass
    vehicle.mass_properties.operating_empty   = MIT.mass
    vehicle.mass_properties.center_of_gravity = [[0.12, 0., 0.]]
    vehicle.reference_area                    = MIT.wingspan * GUESS.wing_chord
    vehicle.flight_envelope.ultimate_load     = 3.0
    vehicle.flight_envelope.limit_load        = 2.0

    airfoil_file = os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', '..', 'Vehicles', 'Airfoils', 'Clark_y.txt')

    wing                         = RCAIDE.Library.Components.Wings.Main_Wing()
    wing.tag                     = 'main_wing'
    wing.areas.reference         = vehicle.reference_area
    wing.spans.projected         = MIT.wingspan
    wing.chords.root             = GUESS.wing_chord
    wing.chords.tip              = GUESS.wing_chord
    wing.chords.mean_aerodynamic = GUESS.wing_chord
    wing.taper                   = 1.0
    wing.aspect_ratio            = wing.spans.projected**2 / wing.areas.reference
    wing.thickness_to_chord      = 0.12
    wing.sweeps.quarter_chord    = 0.0
    wing.twists.root             = 0.0
    wing.twists.tip              = 0.0
    wing.origin                  = [[0.0, 0., 0.]]
    wing.aerodynamic_center      = [[0.25 * GUESS.wing_chord, 0., 0.]]
    wing.vertical                = False
    wing.xz_plane_symmetric      = True
    wing.dynamic_pressure_ratio  = 1.0
    airfoil                      = RCAIDE.Library.Components.Airfoils.Airfoil()
    airfoil.tag                  = 'Clark_y'
    airfoil.coordinate_file      = airfoil_file
    wing.append_airfoil(airfoil)
    vehicle.append_component(wing)

    for tag, span, chord, x, vertical in (('horizontal_stabilizer', 1.2, 0.25, 1.6, False),
                                          ('vertical_stabilizer',   0.5, 0.25, 1.6, True)):   # placeholders
        tail                         = (RCAIDE.Library.Components.Wings.Vertical_Tail() if vertical
                                        else RCAIDE.Library.Components.Wings.Horizontal_Tail())
        tail.tag                     = tag
        tail.areas.reference         = span * chord
        tail.spans.projected         = span
        tail.chords.root             = chord
        tail.chords.tip              = chord
        tail.chords.mean_aerodynamic = chord
        tail.taper                   = 1.0
        tail.aspect_ratio            = span**2 / tail.areas.reference
        tail.thickness_to_chord      = 0.10
        tail.sweeps.quarter_chord    = 0.0
        tail.twists.root             = 0.0
        tail.twists.tip              = 0.0
        tail.origin                  = [[x, 0., 0.]]
        tail.aerodynamic_center      = [[x + 0.25 * chord, 0., 0.]]
        tail.vertical                = vertical
        tail.xz_plane_symmetric      = not vertical
        tail.dynamic_pressure_ratio  = 0.9
        vehicle.append_component(tail)

    net = RCAIDE.Framework.Networks.Electric()
    bus = RCAIDE.Library.Components.Powertrain.Distributors.Electrical_Bus()
    bat = RCAIDE.Library.Components.Powertrain.Sources.Battery_Modules.Lithium_Ion_NMC()
    bat.electrical_configuration.series        = GUESS.battery_series_cells
    bat.electrical_configuration.parallel      = 1
    bat.geometric_configuration.normal_count   = GUESS.battery_series_cells
    bat.geometric_configuration.parallel_count = 1
    bus.battery_modules.append(bat)
    bus.battery_module_electric_configuration  = 'Series'
    bus.initialize_bus_properties()

    thruster                           = RCAIDE.Library.Components.Powertrain.Propulsors.EHD_Thruster()
    thruster.tag                       = 'ehd_thruster'
    thruster.origin                    = [[0.5 * GUESS.column_spacing + MIT.gap, 0.,
                                           GUESS.electrode_top_z - 0.5 * (GUESS.unit_rows - 1) * GUESS.unit_spacing]]
    array                              = RCAIDE.Library.Components.Powertrain.Converters.EHD_Electrode_Array()
    array.tag                          = 'ehd_electrode_array'
    array.gap                          = MIT.gap
    array.emitter_diameter             = MIT.emitter_diameter
    array.span                         = MIT.electrode_span
    array.number_of_units              = GUESS.unit_rows * GUESS.unit_columns
    array.unit_spacing                 = GUESS.unit_spacing
    array.collector_chord              = GUESS.collector_chord
    array.maximum_voltage              = MIT.maximum_voltage
    array.collector_foam_density       = GUESS.collector_foam_density
    array.collector_foil_areal_density = GUESS.collector_foil_areal_density
    hvpc                               = RCAIDE.Library.Components.Powertrain.Modulators.High_Voltage_Converter()
    hvpc.tag                           = 'hvpc'
    hvpc.rated_power                   = MIT.hvpc_rated_power
    hvpc.bus_voltage                   = bus.voltage
    thruster.electrode_array           = array
    thruster.high_voltage_converter    = hvpc
    compute_ehd_thruster_mass(thruster)
    net.propulsors.append(thruster)

    bus.assigned_propulsors = [[thruster.tag]]
    net.busses.append(bus)
    vehicle.append_energy_network(net)
    return vehicle

def configs_setup(vehicle):
    configs         = RCAIDE.Library.Components.Configs.Config.Container()
    base_config     = RCAIDE.Library.Components.Configs.Config(vehicle)
    base_config.tag = 'base'
    configs.append(base_config)
    return configs

# ----------------------------------------------------------------------------------------------------------------------
#  Analyses and mission
# ----------------------------------------------------------------------------------------------------------------------
def base_analysis(vehicle):
    analyses         = RCAIDE.Framework.Analyses.Vehicle()
    analyses.vehicle = vehicle
    weights = RCAIDE.Framework.Analyses.Weights.Electric_Drone()
    weights.settings.run_weights_analysis = False   # buildups do not know EHD; vehicle mass is set by hand
    analyses.append(weights)
    geometry = RCAIDE.Framework.Analyses.Geometry.Geometry()
    geometry.settings.overwrite_reference = False
    analyses.append(geometry)
    aerodynamics = RCAIDE.Framework.Analyses.Aerodynamics.Vortex_Lattice_Method()
    aerodynamics.settings.number_of_spanwise_vortices  = 10
    aerodynamics.settings.number_of_chordwise_vortices = 4
    analyses.append(aerodynamics)
    analyses.append(RCAIDE.Framework.Analyses.Energy.Energy())
    analyses.append(RCAIDE.Framework.Analyses.Planets.Earth())
    analyses.append(RCAIDE.Framework.Analyses.Atmospheric.US_Standard_1976())
    return analyses

def mission_setup(analyses):
    mission      = RCAIDE.Framework.Mission.Sequential_Segments()
    mission.tag  = 'mission'
    Segments     = RCAIDE.Framework.Mission.Segments
    base_segment = Segments.Segment()
    base_segment.state.numerics.number_of_control_points = 6

    segment = Segments.Cruise.Constant_Speed_Constant_Altitude(base_segment)
    segment.tag = 'level_flight'
    segment.analyses.extend(analyses.base)
    segment.altitude                         = 2.0 * Units.m
    segment.air_speed                        = MIT.flight_speed * Units['m/s']
    segment.distance                         = MIT.flight_distance * Units.m
    segment.initial_battery_state_of_charge  = 1.0
    segment.flight_dynamics.force_x          = True
    segment.flight_dynamics.force_z          = True
    segment.assigned_control_variables.throttle.active               = True
    segment.assigned_control_variables.throttle.assigned_propulsors  = [['ehd_thruster']]
    segment.assigned_control_variables.throttle.initial_guess_values = [[0.3]]
    segment.assigned_control_variables.pitch_angle.active            = True
    segment.assigned_control_variables.pitch_angle.initial_guess_values = [[3.0 * Units.degree]]
    mission.append_segment(segment)
    return mission

def run_mission():
    vehicle  = vehicle_setup()
    configs  = configs_setup(vehicle)
    analyses = RCAIDE.Framework.Analyses.Analysis.Container()
    for tag, config in configs.items():
        analyses[tag] = base_analysis(config)
    results  = mission_setup(analyses).evaluate()
    return vehicle, results

# ----------------------------------------------------------------------------------------------------------------------
#  OpenVSP export with the electrodes drawn in
# ----------------------------------------------------------------------------------------------------------------------
def export_vsp_with_electrodes(vehicle, filename, rows=GUESS.unit_rows, columns=GUESS.unit_columns,
                               column_spacing=GUESS.column_spacing, top_z=GUESS.electrode_top_z):
    """
    Writes the airframe with RCAIDE's OpenVSP exporter, then adds every emitter wire and NACA 0010
    collector as a display-only OpenVSP wing.

    The electrodes are added only to the .vsp3 file, never to the RCAIDE vehicle: their drag is already
    inside the EHD propulsor (spec 3.3, drag bookkeeping rule). Unit layout (rows x columns, column
    spacing, height) is placeholder geometry.
    """
    import openvsp as vsp
    from RCAIDE.Framework.External_Interfaces.OpenVSP.export_vsp_vehicle import export_vsp_vehicle

    # verbose must stay True: export_vsp_vehicle only writes wings inside its `if verbose:` block
    export_vsp_vehicle(vehicle, vehicle.tag, verbose=True, write_file=False)
    array = vehicle.networks.electric.propulsors['ehd_thruster'].electrode_array
    if rows * columns != array.number_of_units:
        raise ValueError('rows x columns must equal the number of EHD units')

    def add_strip(name, chord, x, z, shape):
        g = vsp.AddGeom('WING')
        vsp.SetGeomName(g, name)
        vsp.SetDriverGroup(g, 1, vsp.SPAN_WSECT_DRIVER, vsp.ROOTC_WSECT_DRIVER, vsp.TIPC_WSECT_DRIVER)
        vsp.SetParmVal(g, 'Span', 'XSec_1', array.span / 2)
        vsp.SetParmVal(g, 'Root_Chord', 'XSec_1', chord)
        vsp.SetParmVal(g, 'Tip_Chord', 'XSec_1', chord)
        vsp.SetParmVal(g, 'Sweep', 'XSec_1', 0.0)
        vsp.SetParmVal(g, 'Dihedral', 'XSec_1', 0.0)
        vsp.SetParmVal(g, 'X_Rel_Location', 'XForm', x)
        vsp.SetParmVal(g, 'Z_Rel_Location', 'XForm', z)
        surf = vsp.GetXSecSurf(g, 0)
        for i in range(vsp.GetNumXSec(surf)):
            vsp.ChangeXSecShape(surf, i, shape)
            xsec = vsp.GetXSec(surf, i)
            if shape == vsp.XS_FOUR_SERIES:
                vsp.SetParmVal(vsp.GetXSecParm(xsec, 'ThickChord'), 0.10)
                vsp.SetParmVal(vsp.GetXSecParm(xsec, 'Camber'), 0.0)
            else:
                vsp.SetParmVal(vsp.GetXSecParm(xsec, 'Circle_Diameter'), chord)
        vsp.Update()

    D_w = array.emitter_diameter
    for j in range(columns):
        x_wire = j * column_spacing
        for i in range(rows):
            z = top_z - i * array.unit_spacing
            add_strip('ehd_wire_c%d_r%d' % (j + 1, i + 1),      D_w, x_wire - D_w / 2, z, vsp.XS_CIRCLE)
            add_strip('ehd_collector_c%d_r%d' % (j + 1, i + 1), array.collector_chord, x_wire + array.gap, z, vsp.XS_FOUR_SERIES)
    vsp.WriteVSPFile(filename)
    return filename

# ----------------------------------------------------------------------------------------------------------------------
#  Tests
# ----------------------------------------------------------------------------------------------------------------------
def test_mit_thrust_sweep():
    thruster = vehicle_setup().networks.electric.propulsors['ehd_thruster']
    array    = thruster.electrode_array
    throttle = np.linspace(0., 1., 11)
    # +10 K at sea level gives delta = 1 (25 degC, 1 atm), the state the spec's test vectors use
    state = setup_operating_conditions(thruster, velocity_range=0. * throttle, altitude=0., temperature_deviation=10.)
    state.conditions.energy.propulsors[thruster.tag].throttle[:,0] = throttle
    thruster.compute_performance(state)
    p   = state.conditions.energy.propulsors[thruster.tag]
    arr = state.conditions.energy.converters[array.tag]

    assert np.allclose(arr.relative_air_density, 1.0, atol=1e-3)
    assert np.all(np.diff(p.net_thrust[1:,0]) > 0)
    thrust_density = arr.unit_thrust[-1,0] / (array.unit_spacing * array.span)
    assert round(thrust_density, 1) == 4.3                    # spec 2.4.4 check value at S = 0.10 m
    assert thrust_density > 3.3                               # above the measured R10 ceiling: 1-D cell over-predicts
    print('MIT-like sweep at 40.3 kV: %d units give %.1f N static (%.2f N/m^2 per unit); reported aircraft thrust %.1f N'
          % (array.number_of_units, p.net_thrust[-1,0], thrust_density, MIT.reported_thrust))

def test_mit_level_flight():
    vehicle, results = run_mission()
    seg = results.segments.level_flight
    c   = seg.conditions
    bus = list(vehicle.networks.electric.busses.values())[0]
    b   = c.energy.busses[bus.tag]
    p   = c.energy.propulsors['ehd_thruster']
    arr = c.energy.converters['ehd_electrode_array']
    dt  = np.diff(c.frames.inertial.time[:,0])

    assert seg.converged
    assert np.all((p.throttle > 0) & (p.throttle < 1))
    assert np.all(p.net_thrust > 0)
    assert np.allclose(b.power_draw, p.bus_power / bus.efficiency, rtol=1e-12)
    SOC = b.state_of_charge[:,0]
    assert np.all(np.diff(SOC) < 0)
    dE  = b.energy[0,0] - b.energy[-1,0]
    assert np.isclose(dE, np.sum((b.power_draw[:-1,0] + np.abs(b.heat_energy_generated[:-1,0])) * dt), rtol=1e-6)
    print('MIT-like level flight at %.1f m/s: T_net %.2f N, throttle %.3f, V_a %.1f kV, P_bus %.0f W, SOC %.4f -> %.4f'
          % (MIT.flight_speed, p.net_thrust[0,0], p.throttle[0,0], arr.applied_voltage[0,0] / 1e3, b.power_draw[0,0], SOC[0], SOC[-1]))

def test_mit_vsp_export(tmp_path):
    vsp = pytest.importorskip('openvsp')
    filename = str(tmp_path / 'MIT_like_EHD_aircraft.vsp3')
    export_vsp_with_electrodes(vehicle_setup(), filename)
    assert os.path.isfile(filename)
    vsp.ClearVSPModel()
    vsp.ReadVSPFile(filename)
    names = [vsp.GetGeomName(g) for g in vsp.FindGeoms()]
    assert 'main_wing' in names
    assert sum(n.startswith('ehd_collector_') for n in names) == GUESS.unit_rows * GUESS.unit_columns
    assert sum(n.startswith('ehd_wire_') for n in names) == GUESS.unit_rows * GUESS.unit_columns

# ----------------------------------------------------------------------------------------------------------------------
#  main: run the tests, then write MIT_like_EHD_aircraft.vsp3 to the current folder
# ----------------------------------------------------------------------------------------------------------------------
def main():
    assert pytest.main([__file__, '-q', '-s']) == 0
    try:
        import openvsp  # noqa: F401
    except ImportError:
        print('OpenVSP not installed in this environment; skipping the .vsp3 export')
        return
    print('Wrote ' + os.path.abspath(export_vsp_with_electrodes(vehicle_setup(), 'MIT_like_EHD_aircraft.vsp3')))

if __name__ == '__main__':
    main()
