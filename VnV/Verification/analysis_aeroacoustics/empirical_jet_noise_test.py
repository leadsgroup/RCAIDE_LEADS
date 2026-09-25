# empirical_jet_noise_test.py
#
# Created: Jan 2024, M. Clarke
# Modified: Sep 2026, P. Siripun, M. Clarke

""" Verification of the semi-empirical airframe + engine noise footprint model.

Flies the VnV Embraer E190 (landing configuration) down a straight-in 3 degree approach, then calls
RCAIDE.Framework.Analyses.Aeroacoustics.Semi_Empirical directly to compute the noise footprint over a
ground receptor grid. The engine state (nozzle exit velocities and temperatures, fan speed and fan
temperature rise) at every point of the approach is solved by the aircraft's turbofan cycle
(CF34-10E) at a prescribed approach throttle.
"""

# ----------------------------------------------------------------------
#   Imports
# ----------------------------------------------------------------------
import RCAIDE
from RCAIDE.Framework.Core import Units, Data
from RCAIDE.Framework.Mission.Common import Results
from RCAIDE.Framework.Mission.Segments.Segment import Segment
from RCAIDE.Library.Methods.Aeroacoustics.Common.generate_zero_elevation_microphone_locations import generate_zero_elevation_microphone_locations

import matplotlib.pyplot as plt
import matplotlib.tri as tri
import numpy as np
import os, sys

# local imports
vehicles_path = os.path.abspath(os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "..", "Vehicles"))
if vehicles_path not in sys.path:
    sys.path.insert(0, vehicles_path)
from Embraer_190 import vehicle_setup, configs_setup

# ----------------------------------------------------------------------
#   Main
# ----------------------------------------------------------------------
def main():
    # --- Setup Vehicle and Approach ---
    vehicle = configs_setup(vehicle_setup()).landing
    segment = approach_segment_setup(vehicle)

    # --- Setup Analysis and Receptor Grid: 20 km along the approach path by 8 km across ---
    aeroacoustics_analysis = RCAIDE.Framework.Analyses.Aeroacoustics.Semi_Empirical()
    receptor_grid_setup(aeroacoustics_analysis, x_range=(-18000., 2000.), y_range=(-4000., 4000.), resolution=41)

    # --- Execute ---
    aeroacoustics_analysis.evaluate_aeroacoustics(segment, vehicle)
    footprint = compute_sound_exposure_level(aeroacoustics_analysis, segment)
    plot_footprint(footprint, segment)

    # --- Regression check: peak SEL, and SEL under the approach path 2 km before the threshold ---
    approach_point  = np.argmin((footprint.x + 2000.)**2 + footprint.y**2)
    SEL_max         = np.max(footprint.SEL)
    SEL_approach    = footprint.SEL[approach_point]
    print('Maximum SEL [dBA]         : ', SEL_max)
    print('Approach point SEL [dBA]  : ', SEL_approach)

    SEL_max_true      = 108.77467786892782
    SEL_approach_true = 89.82981914100239
    assert np.abs((SEL_max - SEL_max_true) / SEL_max_true) < 1e-3
    assert np.abs((SEL_approach - SEL_approach_true) / SEL_approach_true) < 1e-3
    return


# ----------------------------------------------------------------------
#   Approach Segment Setup
# ----------------------------------------------------------------------
def approach_segment_setup(vehicle):
    """Straight-in approach along +x on a 3 degree glide slope to a threshold at x = 0 with the
    engine state solved by the turbofan cycle at every control point."""

    ctrl_pts          = 60
    glide_slope       = 3.0 * Units.degrees
    threshold_height  = 50. * Units.ft
    ground_speed      = 140. * Units.kts
    alpha             = 6.0 * Units.degrees
    throttle          = 0.5  # prescribed approach setting: fraction of design combustor exit temperature

    # 1. Flight path: z is negative up, receptors at z = 0
    x_path   = np.linspace(-16000., 0., ctrl_pts)
    altitude = (threshold_height - x_path * np.tan(glide_slope)).reshape(-1, 1)

    position_vector       = np.zeros((ctrl_pts, 3))
    position_vector[:, 0] = x_path
    position_vector[:, 2] = -altitude[:, 0]

    velocity_vector       = np.zeros((ctrl_pts, 3))
    velocity_vector[:, 0] = ground_speed * np.cos(glide_slope)
    velocity_vector[:, 2] = ground_speed * np.sin(glide_slope)

    time = (x_path - x_path[0]) / (ground_speed * np.cos(glide_slope))

    # 2. Atmosphere along the flight path (US Standard 1976)
    working_fluid = RCAIDE.Library.Attributes.Gases.Air()
    atmo_data     = RCAIDE.Framework.Analyses.Atmospheric.US_Standard_1976().compute_values(altitude)

    conditions = Results()
    conditions.expand_rows(ctrl_pts)
    segment = Segment()
    segment.state.conditions = conditions
    segment.state.numerics.number_of_control_points = ctrl_pts

    conditions.aerodynamics.angles.alpha              = np.ones((ctrl_pts, 1)) * alpha
    conditions.freestream.altitude                    = altitude
    conditions.freestream.density                     = atmo_data.density
    conditions.freestream.dynamic_viscosity           = atmo_data.dynamic_viscosity
    conditions.freestream.speed_of_sound              = atmo_data.speed_of_sound
    conditions.freestream.temperature                 = atmo_data.temperature
    conditions.freestream.pressure                    = atmo_data.pressure
    conditions.freestream.gravity                     = np.ones((ctrl_pts, 1)) * RCAIDE.Library.Attributes.Planets.Earth().sea_level_gravity
    conditions.freestream.isentropic_expansion_factor = working_fluid.compute_gamma(atmo_data.temperature, atmo_data.pressure)
    conditions.freestream.Cp                          = working_fluid.compute_cp(atmo_data.temperature, atmo_data.pressure)
    conditions.freestream.R                           = np.ones((ctrl_pts, 1)) * working_fluid.gas_specific_constant
    conditions.freestream.velocity                    = np.ones((ctrl_pts, 1)) * ground_speed
    conditions.freestream.mach_number                 = ground_speed / atmo_data.speed_of_sound
    conditions.frames.inertial.position_vector        = position_vector
    conditions.frames.inertial.velocity_vector        = velocity_vector
    conditions.frames.inertial.time                   = time.reshape(-1, 1)

    # 3. Engine states solved by the turbofan cycle at every control point
    network = vehicle.networks.fuel
    for propulsor in network.propulsors:
        propulsor.append_operating_conditions(segment)
        conditions.energy.propulsors[propulsor.tag].throttle = np.ones((ctrl_pts, 1)) * throttle
        propulsor.compute_performance(segment.state, network)

    return segment


# ----------------------------------------------------------------------
#   Ground Receptor Grid
# ----------------------------------------------------------------------
def receptor_grid_setup(aeroacoustics_analysis, x_range, y_range, resolution):
    settings = aeroacoustics_analysis.settings
    settings.microphone_min_x, settings.microphone_max_x = x_range
    settings.microphone_min_y, settings.microphone_max_y = y_range
    settings.microphone_x_resolution = resolution
    settings.microphone_y_resolution = resolution
    settings.noise_receptor_search_radius = 10000.
    return


# ----------------------------------------------------------------------
#   Sound Exposure Level
# ----------------------------------------------------------------------
def compute_sound_exposure_level(aeroacoustics_analysis, segment):
    conditions = segment.state.conditions
    SPL_dBA    = conditions.aeroacoustics.SPL_dBA   # (ctrl_pts, n_receptor)
    time       = conditions.frames.inertial.time[:, 0]

    dt = np.gradient(time)
    energy_integral = np.sum((10**(SPL_dBA / 10.0)) * dt[:, None], axis=0)
    SEL = 10 * np.log10(np.maximum(energy_integral, 1e-30))

    receptor_locations = generate_zero_elevation_microphone_locations(aeroacoustics_analysis.settings)

    return Data(x=receptor_locations[:, 0], y=receptor_locations[:, 1], SEL=SEL)


# ----------------------------------------------------------------------
#   Plotting
# ----------------------------------------------------------------------
def plot_footprint(footprint, segment):
    fig, ax = plt.subplots(figsize=(10, 5), dpi=120)

    triangulation = tri.Triangulation(footprint.x / 1000., footprint.y / 1000.)
    heatmap       = ax.tricontourf(triangulation, footprint.SEL, levels=40, cmap='jet', extend='both')

    flight_path = segment.state.conditions.frames.inertial.position_vector
    ax.plot(flight_path[:, 0] / 1000., flight_path[:, 1] / 1000., 'k--', linewidth=1)

    cbar = fig.colorbar(heatmap, ax=ax)
    cbar.set_label('Level (Exposure) - SEL [dBA]', fontsize=12, fontweight='bold')

    ax.set_title('E190 Approach Noise Footprint', fontsize=14, fontweight='bold', pad=15)
    ax.set_xlabel('Distance from threshold [km]', fontsize=12)
    ax.set_ylabel('Lateral distance [km]', fontsize=12)
    ax.grid(True, linestyle='--', alpha=0.5, color='gray')
    ax.set_aspect('equal')

    return fig


if __name__ == '__main__':
    main()
    plt.show()
