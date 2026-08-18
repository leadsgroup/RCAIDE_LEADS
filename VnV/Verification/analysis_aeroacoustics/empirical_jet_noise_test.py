# empirical_jet_noise_test.py
#
# Created: Jan 2024, M. Clarke
# Modified: Aug 2026, P. Siripun

""" Validation/demo for the semi-empirical airframe + engine noise footprint model.

Builds a representative 737-class vehicle and flies it down a true-to-life trajectory 
mapped from an interpolated CSV, then calls RCAIDE.Framework.Analyses.Aeroacoustics.Semi_Empirical 
directly to compute the noise footprint over a ground receptor grid.
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
import pandas as pd

# ----------------------------------------------------------------------
#   Main
# ----------------------------------------------------------------------
def main():
    # --- Load and Interpolate CSV Data ---
    df = pd.read_csv("/Users/siripunn/Desktop/LEADS_WORK/LEADS_Research/RCAIDE_LEADS/VnV/Verification/analysis_aeroacoustics/b737_sim_track_interpolated_pun_original.csv")

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
    vehicle = vehicle_setup()
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
    main_gear.units            = 2       
    main_gear.gear_extended    = True
    vehicle.append_component(main_gear)

    # ------------------------------------------------------------------
    #  Wing, with a deployed flap and slat
    # ------------------------------------------------------------------
    wing                       = RCAIDE.Library.Components.Wings.Main_Wing()
    wing.tag                   = 'main_wing'
    wing.areas.reference       = 124.6                     
    wing.aspect_ratio          = 34.32**2 / 124.6            
    wing.taper                 = 0.2
    wing.thickness_to_chord    = 0.11
    wing.sweeps.leading_edge   = 25. * Units.degrees
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
    turbofan.origin                = np.array([[0.0, 0.0, 1.5]])  
    turbofan.length                = 97 * Units.inches
    turbofan.diameter              = 70 * Units.inches
    turbofan.plug_diameter         = 60 * Units.inches
    turbofan.geometry_xe           = 1.0
    turbofan.geometry_ye           = 1.0
    turbofan.geometry_Ce           = 1.0
    turbofan.working_fluid         = RCAIDE.Library.Attributes.Gases.Air()

    ram                            = RCAIDE.Library.Components.Powertrain.Converters.Ram()
    ram.tag                        = 'ram'
    turbofan.ram                   = ram

    fan                            = RCAIDE.Library.Components.Powertrain.Converters.Fan()
    fan.tag                        = 'fan'
    fan.polytropic_efficiency      = 0.93
    fan.pressure_ratio             = 1.7
    fan.angular_velocity           = 4200 * Units.rpm
    fan.number_of_blades           = 22
    turbofan.fan                   = fan

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

    # 3. Apply Environment and Aerodynamic Conditions
    # Fix: Updating properties to exactly match the original model parameters
    alpha                               = 10.0 * Units.degrees 
    density                             = 1.2250
    dynamic_viscosity                   = 1.81e-5
    a                                   = 343.376
    T                                   = 288.16889478
    P                                   = 97717.0

    conditions = Results()
    conditions.aerodynamics.angles.alpha         = np.ones((1, 1)) * alpha
    conditions.freestream.density                = np.ones((1, 1)) * density
    conditions.freestream.dynamic_viscosity      = np.ones((1, 1)) * dynamic_viscosity
    conditions.freestream.speed_of_sound         = np.ones((1, 1)) * a
    conditions.freestream.temperature            = np.ones((1, 1)) * T
    conditions.freestream.pressure               = np.ones((1, 1)) * P
    
    segment = Segment()
    segment.state.conditions = conditions
    segment.state.numerics.number_of_control_points = 1

    # 4. Engine States and Aeroacoustics Sync
    turbofan = vehicle.networks.fuel.propulsors.starboard_propulsor
    turbofan.append_operating_conditions(segment)

    # Fix: Inject the hardcoded operating conditions directly into the aeroacoustics object 
    aero = conditions.aeroacoustics.propulsors[turbofan.tag]
    
    aero.fan.angular_velocity            = np.ones((1, 1)) * 4200
    aero.fan.exit_velocity               = np.ones((1, 1)) * (350 * Units.mph)
    aero.fan.exit_stagnation_temperature = np.ones((1, 1)) * 440
    aero.fan.exit_stagnation_pressure    = np.ones((1, 1)) * 152000
    aero.fan.number_of_blades            = 22 
    aero.fan.diameter                    = 70 * Units.inches 
    aero.fan.static_temperature_output   = np.ones((1, 1)) * (T + 80/1.8)
    aero.fan.static_temperature_input    = np.ones((1, 1)) * T

    aero.fan_nozzle.exit_velocity               = np.ones((1, 1)) * 280.0
    aero.fan_nozzle.exit_stagnation_temperature = np.ones((1, 1)) * 340.0
    aero.fan_nozzle.exit_stagnation_pressure    = np.ones((1, 1)) * 2611.8

    aero.core_nozzle.exit_velocity               = np.ones((1, 1)) * 400.0
    aero.core_nozzle.exit_stagnation_temperature = np.ones((1, 1)) * 800.0
    aero.core_nozzle.exit_stagnation_pressure    = np.ones((1, 1)) * 165000.0

    # Energy framework states 
    converters = conditions.energy.converters
    converters[turbofan.fan.tag].inputs.static_temperature        = np.ones((1, 1)) * T
    converters[turbofan.fan.tag].outputs.static_temperature       = np.ones((1, 1)) * (T + 80/1.8)
    converters[turbofan.fan_nozzle.tag].outputs.velocity          = np.ones((1, 1)) * 280.0
    converters[turbofan.fan_nozzle.tag].outputs.stagnation_temperature = np.ones((1, 1)) * 340.0
    converters[turbofan.fan_nozzle.tag].outputs.stagnation_pressure    = np.ones((1, 1)) * 2611.8
    converters[turbofan.core_nozzle.tag].outputs.velocity         = np.ones((1, 1)) * 400.0
    converters[turbofan.core_nozzle.tag].outputs.stagnation_temperature= np.ones((1, 1)) * 800.0
    converters[turbofan.core_nozzle.tag].outputs.stagnation_pressure   = np.ones((1, 1)) * 165000.0
    converters['combustor'].inputs.static_temperature             = np.ones((1, 1)) * 622.7
    converters['combustor'].outputs.static_temperature            = np.ones((1, 1)) * 1000.0

    # Expand 1x1 base matrices into N_ctrl_pts x 1 matrices
    conditions.expand_rows(ctrl_pts)
    segment.state.numerics.number_of_control_points = ctrl_pts

    # Apply the true 100-point arrays
    conditions.frames.inertial.position_vector = position_vector
    conditions.frames.inertial.velocity_vector = velocity_vector
    conditions.frames.inertial.time            = time.reshape(-1, 1)
    conditions.freestream.velocity             = ground_speeds.reshape(-1, 1)
    conditions.freestream.mach_number          = (ground_speeds / a).reshape(-1, 1)

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