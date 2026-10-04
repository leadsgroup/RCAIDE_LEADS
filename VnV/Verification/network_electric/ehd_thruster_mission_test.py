# VnV/Verification/network_electric/ehd_thruster_mission_test.py
#
# Created:  Oct 2026, RCAIDE EHD MVP
#
# Mission test for the EHD thruster (spec 3.5 Stage 1 exit criteria): a simple battery-electric UAV with one
# EHD thruster on an electrical bus flies a constant-speed cruise; the segment must converge and the battery
# state of charge must fall consistently with the integrated bus power.

# ----------------------------------------------------------------------------------------------------------------------
#  IMPORT
# ----------------------------------------------------------------------------------------------------------------------
import RCAIDE
from RCAIDE.Framework.Core                                         import Units
from RCAIDE.Library.Methods.Powertrain.Propulsors.EHD_Thruster     import compute_ehd_thruster_mass

import numpy as np
import pytest

# ----------------------------------------------------------------------------------------------------------------------
#  Vehicle
# ----------------------------------------------------------------------------------------------------------------------
def vehicle_setup():
    vehicle                                   = RCAIDE.Vehicle()
    vehicle.tag                               = 'EHD_UAV'
    vehicle.mass_properties.max_takeoff       = 5.0
    vehicle.mass_properties.takeoff           = 5.0
    vehicle.mass_properties.operating_empty   = 5.0
    vehicle.mass_properties.center_of_gravity = [[0.15, 0., 0.]]
    vehicle.reference_area                    = 2.0
    vehicle.flight_envelope.ultimate_load     = 3.0
    vehicle.flight_envelope.limit_load        = 2.0

    wing                         = RCAIDE.Library.Components.Wings.Main_Wing()
    wing.tag                     = 'main_wing'
    wing.areas.reference         = 2.0
    wing.spans.projected         = 5.0
    wing.chords.root             = 0.4
    wing.chords.tip              = 0.4
    wing.chords.mean_aerodynamic = 0.4
    wing.taper                   = 1.0
    wing.aspect_ratio            = wing.spans.projected**2 / wing.areas.reference
    wing.thickness_to_chord      = 0.12
    wing.sweeps.quarter_chord    = 0.0
    wing.twists.root             = 2.0 * Units.degree
    wing.twists.tip              = 0.0
    wing.origin                  = [[0.05, 0., 0.]]
    wing.aerodynamic_center      = [[0.15, 0., 0.]]
    wing.vertical                = False
    wing.xz_plane_symmetric      = True
    wing.dynamic_pressure_ratio  = 1.0
    vehicle.append_component(wing)

    # network
    net = RCAIDE.Framework.Networks.Electric()
    bus = RCAIDE.Library.Components.Powertrain.Distributors.Electrical_Bus()
    bat = RCAIDE.Library.Components.Powertrain.Sources.Battery_Modules.Lithium_Ion_NMC()
    bat.electrical_configuration.series         = 50
    bat.electrical_configuration.parallel       = 1
    bat.geometric_configuration.normal_count    = 50
    bat.geometric_configuration.parallel_count  = 1
    bus.battery_modules.append(bat)
    bus.battery_module_electric_configuration   = 'Series'
    bus.initialize_bus_properties()

    # Two identical EHD thrusters in one propulsor group: the second reuses the first one's results
    for side, y in (('starboard', 0.5), ('port', -0.5)):
        net.propulsors.append(ehd_thruster_setup('ehd_' + side, y, bus.voltage))
    bus.assigned_propulsors = [['ehd_starboard', 'ehd_port']]
    net.busses.append(bus)
    vehicle.append_energy_network(net)
    return vehicle

def ehd_thruster_setup(tag, y, bus_voltage):
    # MIT-like electrodes (spec 2.4.1), chord inside the Kahol-tested 15-100 mm range
    thruster                           = RCAIDE.Library.Components.Powertrain.Propulsors.EHD_Thruster()
    thruster.tag                       = tag
    thruster.origin                    = [[0.0, y, 0.]]
    array                              = RCAIDE.Library.Components.Powertrain.Converters.EHD_Electrode_Array()
    array.tag                          = tag + '_electrode_array'
    array.gap                          = 0.060
    array.emitter_diameter             = 0.2e-3
    array.span                         = 3.0
    array.number_of_units              = 3
    array.unit_spacing                 = 0.10
    array.collector_chord              = 0.05
    array.maximum_voltage              = 40.3e3
    array.collector_foam_density       = 30.
    array.collector_foil_areal_density = 0.05
    hvpc                               = RCAIDE.Library.Components.Powertrain.Modulators.High_Voltage_Converter()
    hvpc.tag                           = tag + '_hvpc'
    hvpc.rated_power                   = 400.
    hvpc.bus_voltage                   = bus_voltage
    thruster.electrode_array           = array
    thruster.high_voltage_converter    = hvpc
    compute_ehd_thruster_mass(thruster)
    return thruster

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
    mission     = RCAIDE.Framework.Mission.Sequential_Segments()
    mission.tag = 'mission'
    Segments    = RCAIDE.Framework.Mission.Segments
    base_segment = Segments.Segment()
    base_segment.state.numerics.number_of_control_points = 8

    segment = Segments.Cruise.Constant_Speed_Constant_Altitude(base_segment)
    segment.tag = 'cruise'
    segment.analyses.extend(analyses.base)
    segment.altitude                         = 100. * Units.m
    segment.air_speed                        = 9.0  * Units['m/s']
    segment.distance                         = 2000. * Units.m
    segment.initial_battery_state_of_charge  = 1.0
    segment.flight_dynamics.force_x          = True
    segment.flight_dynamics.force_z          = True
    segment.assigned_control_variables.throttle.active               = True
    segment.assigned_control_variables.throttle.assigned_propulsors  = [['ehd_starboard', 'ehd_port']]
    segment.assigned_control_variables.throttle.initial_guess_values = [[0.5]]
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
    mission  = mission_setup(analyses)
    results  = mission.evaluate()
    return vehicle, results

# ----------------------------------------------------------------------------------------------------------------------
#  Test
# ----------------------------------------------------------------------------------------------------------------------
def test_ehd_cruise_mission_closes():
    vehicle, results = run_mission()
    seg  = results.segments.cruise
    c    = seg.conditions
    bus  = list(vehicle.networks.electric.busses.values())[0]
    b    = c.energy.busses[bus.tag]
    t    = c.frames.inertial.time[:,0]
    dt   = np.diff(t)
    P_bus_thrusters = 0.
    for tag in ('ehd_starboard', 'ehd_port'):
        thruster = vehicle.networks.electric.propulsors[tag]
        p   = c.energy.propulsors[tag]
        hv  = c.energy.modulators[thruster.high_voltage_converter.tag]
        arr = c.energy.converters[thruster.electrode_array.tag]

        # converged on EHD thrust within the throttle range, without sparkover or input-window flags
        assert np.all((p.throttle > 0) & (p.throttle < 1))
        assert np.all(p.net_thrust > 0) and np.all(arr.unit_thrust > arr.unit_collector_drag + arr.unit_wire_drag)
        assert np.all(arr.sparkover_flag == 0) and np.all(hv.input_voltage_flag == 0)

        # power chain: electrode power -> HVPC -> bus
        assert np.allclose(p.bus_power, p.electrode_power / thruster.high_voltage_converter.efficiency, rtol=1e-12)
        assert np.allclose(hv.inputs.power, p.bus_power, rtol=1e-12)
        P_bus_thrusters = P_bus_thrusters + p.bus_power
    assert seg.converged

    # reuse path: the port thruster copies the starboard results; moments about x cancel
    s_, p_ = c.energy.propulsors['ehd_starboard'], c.energy.propulsors['ehd_port']
    assert np.array_equal(s_.thrust, p_.thrust) and np.array_equal(s_.bus_power, p_.bus_power)
    assert np.allclose(s_.moment[:,2], -p_.moment[:,2]) and np.all(s_.moment[:,2] != 0)
    assert np.allclose(c.energy.thrust_force_vector[:,0], s_.net_thrust[:,0] + p_.net_thrust[:,0])
    assert np.allclose(b.power_draw, P_bus_thrusters / bus.efficiency, rtol=1e-12)

    # state of charge falls monotonically and closes with integrated bus power
    SOC    = b.state_of_charge[:,0]
    E      = b.energy[:,0]
    P_bus  = b.power_draw[:,0]
    Q_heat = b.heat_energy_generated[:,0]
    assert np.all(np.diff(SOC) < 0)
    dE_battery = E[0] - E[-1]
    E_bus      = np.sum(P_bus[:-1] * dt)                       # explicit-Euler integral used by the battery model
    E_heat     = np.sum(np.abs(Q_heat[:-1]) * dt)
    assert np.isclose(dE_battery, E_bus + E_heat, rtol=1e-6)
    assert E_heat / E_bus < 0.05                                # battery losses are a small share
    assert np.isclose((SOC[0] - SOC[-1]) * b.maximum_initial_energy, dE_battery, rtol=1e-6)
    print('EHD cruise: throttle %.3f, V_a %.1f kV, T_net %.2f N, P_bus %.0f W, SOC %.4f -> %.4f over %.0f s'
          % (s_.throttle[0,0], arr.applied_voltage[0,0] / 1e3, c.energy.thrust_force_vector[0,0], P_bus[0], SOC[0], SOC[-1], t[-1] - t[0]))

def main():
    assert pytest.main([__file__, '-q', '-s']) == 0

if __name__ == '__main__':
    main()
