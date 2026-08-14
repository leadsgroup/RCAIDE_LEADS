# empirical_jet_noise_test.py
#
# Created: Jan 2024, M. Clarke
# Modified: Aug 2026, P. Siripun

""" Validation/demo for the semi-empirical airframe + engine noise footprint model.

Builds a representative 737-class vehicle and flies it down a synthetic straight-in 3 degree
glideslope approach, then calls RCAIDE.Framework.Analyses.Aeroacoustics.Semi_Empirical directly
(the same analysis a real mission would use) to compute the noise footprint over a ground
receptor grid, and integrates it into a Sound Exposure Level (SEL) map.
"""

# ----------------------------------------------------------------------
#   Imports
# ----------------------------------------------------------------------
import RCAIDE
from RCAIDE.Framework.Core import Units, Data
from RCAIDE.Framework.Mission.Common import Results
from RCAIDE.Framework.Mission.Segments.Segment import Segment
from RCAIDE.Library.Methods.Geometry.Planform.wing_planform import wing_planform

import matplotlib.pyplot as plt
import matplotlib.tri as tri
import numpy as np

# ----------------------------------------------------------------------
#   Main
# ----------------------------------------------------------------------
def main():
    vehicle = vehicle_setup()
    segment, ctrl_pts = approach_segment_setup(vehicle)

    aeroacoustics_analysis = RCAIDE.Framework.Analyses.Aeroacoustics.Semi_Empirical()
    receptor_grid_setup(aeroacoustics_analysis, x_range=(-500., 3500.), y_range=(-800., 800.), resolution=50)

    aeroacoustics_analysis.evaluate_aeroacoustics(segment, vehicle)

    footprint = compute_sound_exposure_level(aeroacoustics_analysis, segment)
    plot_footprint(aeroacoustics_analysis, footprint)

    return


# ----------------------------------------------------------------------
#   Vehicle
# ----------------------------------------------------------------------
def vehicle_setup():
    vehicle = RCAIDE.Vehicle()
    vehicle.tag = 'b737_800'
    vehicle.mass_properties.max_takeoff = 68038.8555  # kg

    # ------------------------------------------------------------------
    #  Landing Gear
    # ------------------------------------------------------------------
    main_gear                  = RCAIDE.Library.Components.Landing_Gear.Main_Landing_Gear()
    main_gear.tire_diameter    = 1.016   # m
    main_gear.tire_width       = 0.3556  # m
    main_gear.strut_length     = 1.2     # m
    main_gear.strut_diameter   = 0.11811 # m
    main_gear.wheels           = 2
    main_gear.units            = 2       # left and right main gear legs
    main_gear.gear_extended    = True
    vehicle.append_component(main_gear)

    # ------------------------------------------------------------------
    #  Wing, with a deployed flap and slat
    # ------------------------------------------------------------------
    wing                       = RCAIDE.Library.Components.Wings.Main_Wing()
    wing.tag                   = 'main_wing'
    wing.areas.reference       = 124.6                      # m^2
    wing.aspect_ratio          = 34.32**2 / 124.6            # gives ~34.32 m span
    wing.taper                 = 0.2
    wing.thickness_to_chord    = 0.11
    wing.sweeps.leading_edge   = 25. * Units.degrees

    # derives chords.root/tip (and other planform quantities) from the geometry above --
    # required before compute_chord_length_from_span_location can be used on this wing
    wing_planform(wing)

    flap                       = RCAIDE.Library.Components.Wings.Control_Surfaces.Flap()
    flap.span_fraction_start   = 0.15
    flap.span_fraction_end     = 0.6
    flap.chord_fraction        = 0.25
    flap.deflection            = 37.5 * Units.degrees
    wing.append_control_surface(flap)

    slat                       = RCAIDE.Library.Components.Wings.Control_Surfaces.Slat()
    slat.span_fraction_start   = 0.1
    slat.span_fraction_end     = 0.85
    slat.chord_fraction        = 0.12
    slat.deflection            = 20. * Units.degrees
    wing.append_control_surface(slat)

    vehicle.append_component(wing)

    # ------------------------------------------------------------------
    #  Propulsor: Starboard Turbofan
    # ------------------------------------------------------------------
    turbofan                       = RCAIDE.Library.Components.Powertrain.Propulsors.Turbofan()
    turbofan.tag                   = 'starboard_propulsor'
    turbofan.bypass_ratio          = 5.4
    turbofan.design_altitude       = 35000.0 * Units.ft
    turbofan.design_mach_number    = 0.78
    turbofan.design_thrust         = 35000.0 * Units.N
    turbofan.origin                = np.array([[0.0, 0.0, 1.5]])  # core 1.5 m off the ground
    turbofan.length                = 97 * Units.inches
    turbofan.diameter              = 70 * Units.inches
    turbofan.plug_diameter         = 60 * Units.inches
    turbofan.geometry_xe           = 1.0
    turbofan.geometry_ye           = 1.0
    turbofan.geometry_Ce           = 1.0
    turbofan.working_fluid         = RCAIDE.Library.Attributes.Gases.Air()

    ram                             = RCAIDE.Library.Components.Powertrain.Converters.Ram()
    ram.tag                         = 'ram'
    turbofan.ram                    = ram

    fan                             = RCAIDE.Library.Components.Powertrain.Converters.Fan()
    fan.tag                         = 'fan'
    fan.polytropic_efficiency       = 0.93
    fan.pressure_ratio              = 1.7
    fan.angular_velocity            = 4200 * Units.rpm
    fan.number_of_blades            = 22
    turbofan.fan                    = fan

    low_pressure_compressor                    = RCAIDE.Library.Components.Powertrain.Converters.Compressor()
    low_pressure_compressor.tag                = 'low_pressure_compressor'
    low_pressure_compressor.pressure_ratio     = 1.5
    turbofan.low_pressure_compressor           = low_pressure_compressor

    high_pressure_compressor                   = RCAIDE.Library.Components.Powertrain.Converters.Compressor()
    high_pressure_compressor.tag               = 'high_pressure_compressor'
    high_pressure_compressor.pressure_ratio    = 5.137
    turbofan.high_pressure_compressor          = high_pressure_compressor

    combustor                       = RCAIDE.Library.Components.Powertrain.Converters.Combustor()
    combustor.tag                   = 'combustor'
    combustor.number_of_fuel_nozzle = 18
    combustor.diameter              = 0.6858
    turbofan.combustor               = combustor

    core_nozzle                     = RCAIDE.Library.Components.Powertrain.Converters.Expansion_Nozzle()
    core_nozzle.tag                 = 'core_nozzle'
    core_nozzle.polytropic_efficiency = 0.98
    core_nozzle.pressure_ratio      = 0.995
    core_nozzle.diameter            = 0.38118288
    turbofan.core_nozzle            = core_nozzle

    fan_nozzle                      = RCAIDE.Library.Components.Powertrain.Converters.Expansion_Nozzle()
    fan_nozzle.tag                  = 'fan_nozzle'
    fan_nozzle.polytropic_efficiency = 0.98
    fan_nozzle.pressure_ratio       = 0.995
    fan_nozzle.diameter             = 1.5
    turbofan.fan_nozzle             = fan_nozzle

    network = RCAIDE.Framework.Networks.Fuel()
    network.propulsors.append(turbofan)
    vehicle.append_energy_network(network)

    return vehicle


# ----------------------------------------------------------------------
#   Synthetic Approach Segment
# ----------------------------------------------------------------------
def approach_segment_setup(vehicle):
    """Straight-in, constant-speed, constant 3 degree glideslope approach, descending from
    300 m to 10 m AGL, in the local flat-earth frame that Semi_Empirical operates in
    (conditions.frames.inertial.position_vector, z negative-up)."""

    ctrl_pts        = 100
    glide_slope     = 3.0 * Units.degrees
    speed           = 70.0            # m/s, constant approach speed
    altitude_start  = 300.0           # m AGL
    altitude_end    = 10.0            # m AGL
    alpha           = 4.0 * Units.degrees

    horizontal_speed = speed * np.cos(glide_slope)
    descent_rate      = speed * np.sin(glide_slope)
    duration          = (altitude_start - altitude_end) / descent_rate

    t = np.linspace(0.0, duration, ctrl_pts)
    x = (altitude_start / np.tan(glide_slope)) - horizontal_speed * t   # along-track, threshold at x=0
    z = -(altitude_start - descent_rate * t)                            # z negative-up

    position_vector = np.zeros((ctrl_pts, 3))
    position_vector[:, 0] = x
    position_vector[:, 2] = z

    velocity_vector = np.tile(np.array([-horizontal_speed, 0.0, -descent_rate]), (ctrl_pts, 1))

    # --- standard low-altitude atmosphere, build at 1 control point, then broadcast ---
    density, dynamic_viscosity, a, T, P = 1.225, 1.79e-5, 340.3, 288.15, 101325.0

    conditions = Results()
    conditions.aerodynamics.angles.alpha         = np.ones((1, 1)) * alpha
    conditions.freestream.density                = np.ones((1, 1)) * density
    conditions.freestream.dynamic_viscosity       = np.ones((1, 1)) * dynamic_viscosity
    conditions.freestream.speed_of_sound          = np.ones((1, 1)) * a
    conditions.freestream.temperature             = np.ones((1, 1)) * T
    conditions.freestream.pressure                = np.ones((1, 1)) * P
    conditions.freestream.velocity                = np.ones((1, 1)) * speed
    conditions.freestream.mach_number             = np.ones((1, 1)) * speed / a
    conditions.frames.inertial.velocity_vector    = velocity_vector[0:1]

    segment              = Segment()
    segment.state.conditions = conditions
    segment.state.numerics.number_of_control_points = 1

    turbofan = vehicle.networks.fuel.propulsors.starboard_propulsor
    turbofan.append_operating_conditions(segment, conditions.energy, conditions.aeroacoustics)

    # illustrative turbofan cycle exit conditions (representative of a CFM56-class engine at
    # approach power) -- populated directly since this script doesn't run the full cycle solve
    converters = conditions.energy.converters
    converters[turbofan.fan.tag].inputs.static_temperature    = np.ones((1, 1)) * T
    converters[turbofan.fan.tag].outputs.static_temperature   = np.ones((1, 1)) * (T + 80/1.8)
    converters[turbofan.fan_nozzle.tag].outputs.velocity                = np.ones((1, 1)) * 280.0
    converters[turbofan.fan_nozzle.tag].outputs.stagnation_temperature  = np.ones((1, 1)) * 340.0
    converters[turbofan.fan_nozzle.tag].outputs.stagnation_pressure     = np.ones((1, 1)) * 2611.8
    converters[turbofan.core_nozzle.tag].outputs.velocity               = np.ones((1, 1)) * 400.0
    converters[turbofan.core_nozzle.tag].outputs.stagnation_temperature = np.ones((1, 1)) * 800.0
    converters[turbofan.core_nozzle.tag].outputs.stagnation_pressure    = np.ones((1, 1)) * 165000.0
    converters['combustor'].inputs.static_temperature  = np.ones((1, 1)) * 622.7
    converters['combustor'].outputs.static_temperature = np.ones((1, 1)) * 1000.0

    conditions.expand_rows(ctrl_pts)
    segment.state.numerics.number_of_control_points = ctrl_pts

    # overwrite with the real per-control-point trajectory (expand_rows only tiled the
    # single-point placeholder above)
    conditions.frames.inertial.position_vector = position_vector
    conditions.frames.inertial.velocity_vector = velocity_vector
    conditions.frames.inertial.time            = t.reshape(-1, 1)

    return segment, ctrl_pts


# ----------------------------------------------------------------------
#   Ground Receptor Grid
# ----------------------------------------------------------------------
def receptor_grid_setup(aeroacoustics_analysis, x_range, y_range, resolution):
    settings = aeroacoustics_analysis.settings
    settings.microphone_min_x, settings.microphone_max_x = x_range
    settings.microphone_min_y, settings.microphone_max_y = y_range
    settings.microphone_x_resolution = resolution
    settings.microphone_y_resolution = resolution
    settings.noise_receptor_search_radius = 2500.
    return


# ----------------------------------------------------------------------
#   Sound Exposure Level
# ----------------------------------------------------------------------
def compute_sound_exposure_level(aeroacoustics_analysis, segment):
    """Integrates the per-control-point A-weighted hemisphere SPL time history into SEL at
    each ground receptor.

    References
    ----------
    SAE ARP876D: Gas Turbine Jet Exhaust Noise Prediction
    """
    conditions = segment.state.conditions
    SPL_dBA    = conditions.aeroacoustics.hemisphere_SPL_dBA   # (ctrl_pts, n_receptor)
    time       = conditions.frames.inertial.time[:, 0]

    dt = np.gradient(time)
    energy_integral = np.sum((10**(SPL_dBA / 10.0)) * dt[:, None], axis=0)
    SEL = 10 * np.log10(np.maximum(energy_integral, 1e-30))

    from RCAIDE.Library.Methods.Aeroacoustics.Common.generate_zero_elevation_microphone_locations import generate_zero_elevation_microphone_locations
    receptor_locations = generate_zero_elevation_microphone_locations(aeroacoustics_analysis.settings)

    return Data(x=receptor_locations[:, 0], y=receptor_locations[:, 1], SEL=SEL)


# ----------------------------------------------------------------------
#   Plotting
# ----------------------------------------------------------------------
def plot_footprint(aeroacoustics_analysis, footprint):
    fig, ax = plt.subplots(figsize=(10, 8), dpi=120)

    triangulation = tri.Triangulation(footprint.x, footprint.y)
    levels = np.linspace(np.percentile(footprint.SEL, 5), np.percentile(footprint.SEL, 99.5), 40)
    heatmap = ax.tricontourf(triangulation, footprint.SEL, levels=levels, cmap='jet', extend='both')

    cbar = fig.colorbar(heatmap, ax=ax)
    cbar.set_label('Sound Exposure Level, SEL [dBA]', fontsize=12, fontweight='bold')

    ax.set_title('Simulated Approach Noise Footprint', fontsize=14, fontweight='bold', pad=15)
    ax.set_xlabel('Along-track distance [m]', fontsize=12)
    ax.set_ylabel('Cross-track distance [m]', fontsize=12)
    ax.grid(True, linestyle='--', alpha=0.5, color='gray')
    ax.set_aspect('equal')

    return fig


if __name__ == '__main__':
    main()
    plt.show()
