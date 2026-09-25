# test_Boeing_737_approach_noise.py
#
# Created: Jan 2024, M. Clarke
# Modified: Sep 2026, P. Siripun, M. Clarke

""" Validation of the semi-empirical airframe + engine noise footprint model against AEDT.

Flies the VnV Boeing 737-800 (landing configuration) down a true-to-life trajectory mapped from an
interpolated CSV, then calls RCAIDE.Framework.Analyses.Aeroacoustics.Semi_Empirical directly to
compute the noise footprint over a ground receptor grid. The engine state (nozzle exit velocities and
temperatures, fan speed and fan temperature rise) at every point of the trajectory is solved by the
aircraft's turbofan cycle (CFM56-7B26) at a prescribed approach throttle.
"""

# ----------------------------------------------------------------------
#   Imports
# ----------------------------------------------------------------------
import RCAIDE
from RCAIDE.Framework.Core import Units, Data
from RCAIDE.Framework.Mission.Common import Results
from RCAIDE.Framework.Mission.Segments.Segment import Segment

import matplotlib.pyplot as plt
import matplotlib.tri as tri
import numpy as np
import pandas as pd
import os, sys

# local imports
vehicles_path = os.path.abspath(os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "..", "Vehicles"))
if vehicles_path not in sys.path:
    sys.path.insert(0, vehicles_path)
from Boeing_737 import vehicle_setup, configs_setup

# ----------------------------------------------------------------------
#   Main
# ----------------------------------------------------------------------
def main():
    # --- Load and Interpolate CSV Data ---
    #df = pd.read_csv("b737_sim_track_interpolated_pun_original.csv")
    csv_path = os.path.join(os.path.dirname(os.path.abspath(__file__)), "b737_sim_track_interpolated_pun_original.csv")
    df = pd.read_csv(csv_path)
    def interpolate_path(original_list):
        new_length = 100
        old_indices = np.arange(len(original_list))
        new_indices = np.linspace(0, len(original_list) - 1, new_length)
        interpolated_np = np.interp(new_indices, old_indices, original_list)
        return interpolated_np.tolist()

    lat_array              = interpolate_path(df['Latitude (deg)'].to_numpy())
    lon_array              = interpolate_path(df['Longitude (deg)'].to_numpy())
    elevation_msl_array    = interpolate_path(df['Altitude MSL (ft)'].to_numpy())
    ground_speed_kts_array = interpolate_path(df['Ground Speed (kts)'].to_numpy())

    # --- Setup Vehicle and Path ---
    vehicle = configs_setup(vehicle_setup()).landing
    segment, ctrl_pts = approach_segment_setup(vehicle, lat_array, lon_array, elevation_msl_array, ground_speed_kts_array)

    # --- Setup Analysis and Receptor Grid ---
    aeroacoustics_analysis = RCAIDE.Framework.Analyses.Aeroacoustics.Semi_Empirical()
    
    # Map original lat/lon bounding box to the local flat-earth Cartesian grid
    R_earth = 6371000.0
    lon0_rad = np.radians(lon_array[0])
    lat0_rad = np.radians(lat_array[0])
    
    grid_location = [[-88.018902, 41.894352], [-87.797397, 42.059475]]
    x_min = (np.radians(grid_location[0][0]) - lon0_rad) * R_earth * np.cos(lat0_rad)
    x_max = (np.radians(grid_location[1][0]) - lon0_rad) * R_earth * np.cos(lat0_rad)

    # 2. In approach_segment_setup() (approx. line 181)
    y_min = (np.radians(grid_location[0][1]) - lat0_rad) * R_earth
    y_max = (np.radians(grid_location[1][1]) - lat0_rad) * R_earth

    receptor_grid_setup(aeroacoustics_analysis, x_range=(x_min, x_max), y_range=(y_min, y_max), resolution=150)

    # --- Execute ---
    aeroacoustics_analysis.evaluate_aeroacoustics(segment, vehicle)

    footprint = compute_sound_exposure_level(aeroacoustics_analysis, segment)
    
    # FIX: Pass lat_array and lon_array so we can reverse the geographic projection
    plot_footprint(aeroacoustics_analysis, footprint, lat_array, lon_array) 

    return


# ----------------------------------------------------------------------
#   Real Trajectory & Conditions Segment Setup
# ----------------------------------------------------------------------
def approach_segment_setup(vehicle, lat_array, lon_array, elevation_msl_array, ground_speed_kts_array):
    """Maps the interpolated geographic flight path to the flat-earth Cartesian grid and applies 
    exact aerodynamic/engine states matching the original simulation."""
    
    ctrl_pts = len(lat_array)
    R_earth  = 6371000.0
    
    # 1. Geographic to Cartesian Projection (matches original distance vectors)
    lon_rad  = np.radians(lon_array)
    lat_rad  = np.radians(lat_array)
    lon0_rad = lon_rad[0]
    lat0_rad = lat_rad[0]
    
    x_path = (lon_rad - lon0_rad) * R_earth * np.cos(lat0_rad)
    y_path = (lat_rad - lat0_rad) * R_earth
    
    # Z is negative-up. Receptor altitude is 680 ft MSL
    z_path = - (np.array(elevation_msl_array) - 680.0) * Units.ft
    
    position_vector = np.zeros((ctrl_pts, 3))
    position_vector[:, 0] = x_path
    position_vector[:, 1] = y_path
    position_vector[:, 2] = z_path
    
    # 2. Derive Velocity Vectors and Time
    velocity_vector = np.zeros((ctrl_pts, 3))
    time = np.zeros(ctrl_pts)
    ground_speeds = np.array(ground_speed_kts_array) * Units.kts  # kts to m/s
    
    for i in range(ctrl_pts):
        if i > 0 and i < ctrl_pts:
            dx_flight = x_path[i] - x_path[i-1]
            dy_flight = y_path[i] - y_path[i-1]
            dz_flight = z_path[i] - z_path[i-1]
        elif i + 1 < ctrl_pts:
            dx_flight = x_path[i+1] - x_path[i]
            dy_flight = y_path[i+1] - y_path[i]
            dz_flight = z_path[i+1] - z_path[i]
        else:
            dx_flight, dy_flight, dz_flight = 1.0, 0.0, 0.0
            
        mag_flight = np.sqrt(dx_flight**2 + dy_flight**2 + dz_flight**2)
        hx, hy, hz = (dx_flight/mag_flight, dy_flight/mag_flight, dz_flight/mag_flight) if mag_flight > 0 else (1.0, 0.0, 0.0)
        
        velocity_vector[i, 0] = ground_speeds[i] * hx
        velocity_vector[i, 1] = ground_speeds[i] * hy
        velocity_vector[i, 2] = ground_speeds[i] * hz
        
        if i > 0:
            dt = mag_flight / ground_speeds[i] if ground_speeds[i] > 0 else 0
            time[i] = time[i-1] + dt

    # 3. Atmosphere along the trajectory (US Standard 1976 at the MSL altitude of each point)
    alpha         = 10.0 * Units.degrees
    throttle      = 0.55  # prescribed approach setting: fraction of design combustor exit temperature
    altitude      = (np.array(elevation_msl_array) * Units.ft).reshape(-1, 1)
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
    conditions.freestream.velocity                    = ground_speeds.reshape(-1, 1)
    conditions.freestream.mach_number                 = ground_speeds.reshape(-1, 1) / atmo_data.speed_of_sound
    conditions.frames.inertial.position_vector        = position_vector
    conditions.frames.inertial.velocity_vector        = velocity_vector
    conditions.frames.inertial.time                   = time.reshape(-1, 1)

    # 4. Engine states solved by the turbofan cycle at every point of the trajectory
    network = vehicle.networks.fuel
    for propulsor in network.propulsors:
        propulsor.append_operating_conditions(segment)
        conditions.energy.propulsors[propulsor.tag].throttle = np.ones((ctrl_pts, 1)) * throttle
        propulsor.compute_performance(segment.state, network)

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
    
    # FIX: Increase the search radius to prevent the 16km grid from clipping
    settings.noise_receptor_search_radius = 30000. 
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

    from RCAIDE.Library.Methods.Aeroacoustics.Common.generate_zero_elevation_microphone_locations import generate_zero_elevation_microphone_locations
    receptor_locations = generate_zero_elevation_microphone_locations(aeroacoustics_analysis.settings)

    return Data(x=receptor_locations[:, 0], y=receptor_locations[:, 1], SEL=SEL)


# ----------------------------------------------------------------------
#   Plotting
# ----------------------------------------------------------------------


def plot_footprint(aeroacoustics_analysis, footprint, lat_array, lon_array):
    fig, ax = plt.subplots(figsize=(10, 8), dpi=120)

    # FIX: Reverse projection - Local Cartesian (meters) back to Geographic (Lat/Lon)
    R_earth = 6371000.0
    lat0_rad = np.radians(lat_array[0])
    lon0_deg = lon_array[0]
    lat0_deg = lat_array[0]

    footprint_lon = np.degrees(footprint.x / (R_earth * np.clip(np.cos(lat0_rad),-0.99,0.99))) + lon0_deg
    footprint_lat = np.degrees(footprint.y / R_earth) + lat0_deg

    triangulation = tri.Triangulation(footprint_lon, footprint_lat)
    
    # FIX: Force the color levels to match the original model strictly
    heatmap = ax.tricontourf(triangulation, footprint.SEL, levels=40, cmap='jet', extend='both')

    # FIX: Overlay the flight path trajectory
    #ax.plot(lon_array, lat_array, 'ko', markersize=1)

    cbar = fig.colorbar(heatmap, ax=ax)
    cbar.set_label('Level (Exposure) - SEL', fontsize=12, fontweight='bold')

    ax.set_title('B737 Simulated Noise Footprint', fontsize=14, fontweight='bold', pad=15)
    ax.set_xlabel('Longitude', fontsize=12)
    ax.set_ylabel('Latitude', fontsize=12)
    
    ax.grid(True, linestyle='--', alpha=0.5, color='gray')
    
    # Maintain accurate geographic proportions on the plot
    mean_lat = np.mean(footprint_lat)
    ax.set_aspect(1.0 / np.clip(np.cos(np.radians(mean_lat)),-0.99,0.99))

    return fig


if __name__ == '__main__':
    main()
    plt.show()