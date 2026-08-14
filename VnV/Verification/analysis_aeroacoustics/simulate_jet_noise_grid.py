# simulate_jet_noise_grid.py
#
# Created: Jan 2024, M. Clarke 

""" setup file for empirical jet noise base on SAE standards 
"""
 
# ----------------------------------------------------------------------
#   Imports
# ----------------------------------------------------------------------

import RCAIDE
from RCAIDE.Framework.Core import Units, Data   
from RCAIDE.Library.Plots import *  
from RCAIDE.Library.Methods.Aeroacoustics.Semi_Empirical.Propulsion.Engine_Noise.vectorized_mixed_noise_components import compute_landing_gear_noise
from RCAIDE.Library.Methods.Aeroacoustics.Semi_Empirical.Propulsion.Engine_Noise.vectorized_mixed_noise_components import flap_noise_model
from RCAIDE.Library.Methods.Aeroacoustics.Semi_Empirical.Propulsion.Engine_Noise.vectorized_mixed_noise_components import slat_noise
from RCAIDE.Library.Methods.Aeroacoustics.Semi_Empirical.Propulsion.Engine_Noise.vectorized_mixed_noise_components import compute_fan_noise
from RCAIDE.Library.Methods.Aeroacoustics.Semi_Empirical.Propulsion.Engine_Noise.vectorized_mixed_noise_components import compute_core_noise
from RCAIDE.Library.Methods.Aeroacoustics.Semi_Empirical.Propulsion.Engine_Noise.compute_jet_noise_new import compute_jet_noise_new
from RCAIDE.Framework.Mission.Common                                              import Results  
from RCAIDE.Framework.Mission.Segments.Segment                                    import Segment 
from RCAIDE.Framework.Mission.Common                                              import Conditions 
from RCAIDE.Library.Methods.Aeroacoustics.Common   import SPL_arithmetic 
from RCAIDE.Library.Methods.Aeroacoustics.Metrics import A_weighting_metric
from RCAIDE.Library.Methods.Aeroacoustics.Common.atmospheric_attenuation import atmospheric_attenuation
from RCAIDE.Library.Plots import * 
 
# Python Imports  
import sys
import matplotlib.pyplot as plt 
import numpy as np     
from copy import deepcopy
import os
import pandas as pd
import scipy.ndimage as ndimage

# Local imports 
base_dir = os.path.dirname(os.path.abspath(__file__))
vehicles_path = os.path.abspath(
    os.path.join(base_dir, "..", "..", "Vehicles")
)

if vehicles_path not in sys.path:
    sys.path.insert(0, vehicles_path)


 
# ----------------------------------------------------------------------
#   Main
# ---------------------------------------------------------------------- 
def main():  
    # define plotting parameters
    df = pd.read_csv("/Users/siripunn/Desktop/LEADS_WORK/LEADS_Research/RCAIDE_LEADS/VnV/Verification/analysis_aeroacoustics/b737_sim_track_interpolated.csv")



    def interpolate_path(original_list):
        new_length = 100

        old_indices = np.arange(len(original_list))
        new_indices = np.linspace(0, len(original_list) - 1, new_length)
        
        # Perform linear interpolation
        interpolated_np = np.interp(new_indices, old_indices, original_list)
        interpolated_list = interpolated_np.tolist()
        return interpolated_list

    lat_array = interpolate_path(df['Latitude (deg)'].to_numpy())
    lon_array = interpolate_path(df['Longitude (deg)'].to_numpy())
    elevation_msl_array = interpolate_path(df['Altitude MSL (ft)'].to_numpy())
    ground_speed_kts_array = interpolate_path(df['Ground Speed (kts)'].to_numpy())
    ground_tr = df['Airplane Thrust Type'].to_numpy()

    ground_tr = []

    for i in elevation_msl_array:
        if i - 680 < 25:
            ground_tr.append('Reversed Thrust')
        else:
            ground_tr.append(None)
            
    print(ground_tr)
 


    PP = plot_parameters()  
    vehicle = None
    path = Data(latitude = lat_array, longitude = lon_array, altitude_MSL_ft = elevation_msl_array, ground_speed_kts = ground_speed_kts_array, thrust_reverse = ground_tr )
    frequency_range = None
    grid_location = [[-88.018902,41.894352],[-87.797397,42.059475]] #[[bottomleft_corner_lon, bottomleft_corner_lat],[topright_corner_lon, topright_corner_lat]]
    grid_dimensions = [120,120] #x,y
    receptor_alt_MSL = 680 #[ft]
    sim_result = simulate_vehicle_noise_grid(vehicle,path,frequency_range,grid_location,grid_dimensions,receptor_alt_MSL,fast = True,plot = True) 
    sound_exposure_level = sim_result.SEL
    grid = sim_result.lat_lon
    return
    
    
# ------------------------------------------------------------------
# Harmonic Noise Validation
# ------------------------------------------------------------------  
def simulate_vehicle_noise_grid(vehicle,path,frequency_range,grid_location,grid_dimensions,receptor_alt_MSL,fast = True,plot = True): 
    '''
    This function calls RCAIDE noise models for individual aircraft components, calculates the total SEL, and plots heatmap of noise over a flight path.
    
    Parameters
    ----------
    vehicle : RCAIDE geometry file
        Essential Aircraft Geometry for Model
    path : Data
        vehicle path file containing:
        - latitude
        - longitude
        - altitude_MSL_ft [ft]
        - ground_speed_kts_array [kts]
        - ground_tr (list of 'Reversed Thrust' labels)
    frequency_range : numpy array
        frequency range you want to sum up the noise dB over
    grid_location : numpy array
        Geographic coordinates of grid corners
    grid_dimensions : numpy array
        number of receptors in grid array
    receptor_alt_MSL : numpy array [ft].
        altitude of the receptors in MSL
    fast : bool
        opt to not simulate noise components domated by jet and slat noise
    plot : bool
        opt to plot the heatmap in addition to returning simulated data
    downsample : bool
        downsample the grid size to decrease computation
    
    Returns
    -------
    total_sel : Data
        contains the SEL level for each receptor in grid
            - lat_lon : numpy array
                receptor grid in geographic coordinates
            - SEL : numpy array [dB]
                sel levels, A-weighted at indexed geographic coordinates
    
    Notes
    -----
    The function assumes standard atmospheric attenuation rates from SAE-AIR-1845 

    **Definitions**

    'SEL'
        Sound Exposure Level, the total acoustic energy of a noise event 
        normalized to a duration of 1 second.

    References
    ----------
    [1] SAE ARP876D: Gas Turbine Jet Exhaust Noise Prediction (original)
    [2] de Almeida, Odenir. "Semi-empirical methods for coaxial jet noise prediction." (2008). (adapted)
    [3] Yueping Guo. "A Semi-Empirical Model for Aircraft Landing Gear Noise Prediction." (2012)
    [4] Yueping Guo. "Aircraft Flap Side Edge Noise Modeling and Prediction" (2012)
    [5] Yueping Guo. "Aircraft Slat Noise Modeling and Prediction" (2010)
    [6] Enhanced Core Noise Modeling for Turbofan Engines (NASA)
    [7] Enhanced Fan Noise Modeling for Turbofan Engines (NASA) 
    '''

    if frequency_range == None:
        #use in-built frequency spectrum
        frequency = np.array([
        50.0, 63.0, 80.0, 100.0, 125.0, 160.0, 200.0, 250.0, 
        315.0, 400.0, 500.0, 630.0, 800.0, 1000.0, 1250.0, 1600.0, 
        2000.0, 2500.0, 3150.0, 4000.0, 5000.0, 6300.0, 8000.0, 10000.0
    ])

    if vehicle == None:
        #default to typical 737-800 parameters
        #define params for landing gear model
        D = 1.016 #m - Wheel Diameter
        H = 1.2 #m - Strut Length
        wheels = 2 # number of wheels
        Weight = 68038.8555 #kg Aircraft total weight
        strut_diameter = 0.11811 #m
        W = 0.3556#m Wheel Width (tyre front)
        
        #define params for flap model
        thickness = 0.1 #m flap thickness (average, at the side edge)
        cf = 0.9 #m Flap Chord Length
        deltaf = np.radians(37.5) # Flap Deployment Angle

        #define params for slat model
        phi = 0 #Radians Azimuthal Angle (bankangle = 0 during approach assumptions)
        Ls = 0.08128 #Slat Chord Length
        gamma_s = np.radians(20) # Slat Deployment Angle
        sigma_s = np.radians(25) # Slat Sweep Angle
        alpha = np.radians(10) # Aircraft wing angle of attack

        #define param for core noise model
        pr = 13.1
        m = None # predefined mass flow rate
        
        # define operating conditions                                            
        a                       = 343.376
        T                       = 288.16889478  
        density                 = 1.2250	
        dynamic_viscosity       = 1.81E-5   
        ctrl_pts                = 1
        AoA                     = 4
        U = 103 # aircraft velocity
        M = 0.2 # mach number

    #------------------------------------------------------------------------------------------------------------------------------------
    # Propulsor: Starboard Propulsor
    #------------------------------------------------------------------------------------------------------------------------------------
    turbofan                                    = RCAIDE.Library.Components.Powertrain.Propulsors.Turbofan()
    turbofan.tag                                = 'starboard_propulsor'
    turbofan.bypass_ratio                       = 5.4
    turbofan.design_altitude                    = 35000.0*Units.ft
    turbofan.design_mach_number                 = 0.78
    turbofan.design_thrust                      = 35000.0* Units.N             

    # fan
    fan                                         = RCAIDE.Library.Components.Powertrain.Converters.Fan()
    fan.tag                                     = 'fan'
    fan.polytropic_efficiency                   = 0.93
    fan.pressure_ratio                          = 1.7
    turbofan.fan                                = fan

    # working fluid
    turbofan.working_fluid                      = RCAIDE.Library.Attributes.Gases.Air()
    ram                                         = RCAIDE.Library.Components.Powertrain.Converters.Ram()
    ram.tag                                     = 'ram'
    turbofan.ram                                = ram

    # combustor
    combustor                                         = RCAIDE.Library.Components.Powertrain.Converters.Combustor()
    combustor.tag                                     = 'combustor'
    combustor.number_of_fuel_nozzle                   = 18
    combustor.diameter                                = 0.6858
    turbofan.combustor                                = combustor

    # core nozzle
    core_nozzle                                    = RCAIDE.Library.Components.Powertrain.Converters.Expansion_Nozzle()   
    core_nozzle.tag                                = 'core nozzle'
    core_nozzle.polytropic_efficiency              = 0.98                    
    core_nozzle.pressure_ratio                     = 0.995 
    core_nozzle.diameter                           = 0.38118288
    turbofan.core_nozzle                           = core_nozzle
             
    # fan nozzle             
    fan_nozzle                                     = RCAIDE.Library.Components.Powertrain.Converters.Expansion_Nozzle()   
    fan_nozzle.tag                                 = 'fan nozzle'
    fan_nozzle.polytropic_efficiency               = 0.98                    
    fan_nozzle.pressure_ratio                      = 0.995
    turbofan.fan_nozzle                            = fan_nozzle 

    # define segment 
    segment                                                = Segment()  
    conditions                                             = Results()  
    conditions.aerodynamics.angles.alpha                   = alpha
    conditions.freestream.density                          = np.ones((ctrl_pts,1)) * density
    conditions.freestream.dynamic_viscosity                = np.ones((ctrl_pts,1)) * dynamic_viscosity   
    conditions.freestream.speed_of_sound                   = np.ones((ctrl_pts,1)) * a 
    conditions.freestream.temperature                      = np.ones((ctrl_pts,1)) * T
    conditions.freestream.pressure                         = 97717 #pa at 300m
    conditions.freestream.velocity                         = 85 
    conditions.frames.inertial.velocity_vector             = np.array([[U, 0. ,0.]]) 
    conditions.freestream.mach_number                      = np.atleast_2d(np.linalg.norm(conditions.frames.inertial.velocity_vector,axis = 1)).T/ a
    conditions.frames.planet.true_course                   = np.zeros((ctrl_pts,3,3)) 
    conditions.frames.planet.true_course[:,2,2]            = 1 
    conditions.frames.wind.transform_to_inertial           = np.zeros((ctrl_pts,3,3))    
    conditions.frames.body.transform_to_inertial           = np.zeros((ctrl_pts,3,3))
    conditions.frames.body.transform_to_inertial[:,0,0]    = np.cos(AoA)
    conditions.frames.body.transform_to_inertial[:,0,2]    = np.sin(AoA)
    conditions.frames.body.transform_to_inertial[:,1,1]    = 1
    conditions.frames.body.transform_to_inertial[:,2,0]    = -np.sin(AoA)
    conditions.frames.body.transform_to_inertial[:,2,2]    = np.cos(AoA)     

    segment.state.conditions                                 = conditions 


    turbofan.append_operating_conditions(segment, segment.state.conditions.energy,segment.state.conditions.aeroacoustics)
 
    segment.state.conditions.aeroacoustics.propulsors[turbofan.tag].fan.angular_velocity = 4200 # rpm
    segment.state.conditions.aeroacoustics.propulsors[turbofan.tag].fan.exit_velocity = 350 * Units.mph # m/s to mph
    segment.state.conditions.aeroacoustics.propulsors[turbofan.tag].fan.exit_stagnation_temperature = 440 # deg C
    segment.state.conditions.aeroacoustics.propulsors[turbofan.tag].fan.exit_stagnation_pressure = 152*1000 #Pa

    turbofan.origin = np.array([[0.0, 0.0, 1.5]]) # Core 1.5 m off the ground
    turbofan.length = 97*Units.inches
    turbofan.diameter = 70*Units.inches
    turbofan.plug_diameter = 60*Units.inches
    turbofan.geometry_xe = 1.0 #constants, no need to change
    turbofan.geometry_ye = 1.0
    turbofan.geometry_Ce = 1.0
    
    segment.state.conditions.aeroacoustics.propulsors[turbofan.tag].fan.number_of_blades = 22
    segment.state.conditions.aeroacoustics.propulsors[turbofan.tag].fan.diameter = 70*Units.inches

    segment.state.conditions.aeroacoustics.propulsors[turbofan.tag].fan.static_temperature_output = T + (80/1.8)
    segment.state.conditions.aeroacoustics.propulsors[turbofan.tag].fan.static_temperature_input  = T

    segment.state.conditions.aeroacoustics.propulsors[turbofan.tag].fan_nozzle.exit_velocity = 280.0
    segment.state.conditions.aeroacoustics.propulsors[turbofan.tag].fan_nozzle.exit_stagnation_temperature = 340.0
    segment.state.conditions.aeroacoustics.propulsors[turbofan.tag].fan_nozzle.exit_stagnation_pressure = 2611.8

    # Core Nozzle (Primary) Parameters - Realistic for CFM56
    segment.state.conditions.aeroacoustics.propulsors[turbofan.tag].core_nozzle.exit_velocity = 400.0
    segment.state.conditions.aeroacoustics.propulsors[turbofan.tag].core_nozzle.exit_stagnation_temperature = 800.0
    segment.state.conditions.aeroacoustics.propulsors[turbofan.tag].core_nozzle.exit_stagnation_pressure = 165000.0
        
    segment.state.conditions.energy.converters['combustor'].inputs.static_temperature = 622.7
    segment.state.conditions.energy.converters['combustor'].outputs.static_temperature = 1000
    
      
    segment.state.conditions.expand_rows(ctrl_pts)
    # -------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
    # Read Path Data and Receptor Data (Using Relative Paths)
    # -------------------------------------------------------------------------------------------------------------------------------------------------------------------------------  
    def calc_grid(num_x, num_y, geo_coords):
        """
        Generates a grid of coordinates inside a bounding box based on the number of points.
        """
        lon_min, lat_min = geo_coords[0]
        lon_max, lat_max = geo_coords[1]
        
        # Generate evenly spaced arrays based on the number of requested points
        lons = np.linspace(lon_min, lon_max, num_x)
        lats = np.linspace(lat_min, lat_max, num_y)
        
        # Create a 2D meshgrid
        lon_grid, lat_grid = np.meshgrid(lons, lats)
        
        # Flatten the arrays to return [all_lons, all_lats]
        return [lon_grid.flatten(), lat_grid.flatten()]

    # -------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
    # Vectorized Distance & Angle Calculator
    # ------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- 
    def calc_3d_dist_vectorized(ac_lat, ac_lon, ac_alt_ft, rec_lats, rec_lons, rec_elevs_ft):
        R_earth = 6371000.0  # Earth's radius in meters
        ft_to_meters = Units.feet

        lat1, lon1 = np.radians(ac_lat), np.radians(ac_lon)
        lat2, lon2 = np.radians(rec_lats), np.radians(rec_lons)

        dlat = lat2 - lat1
        dlon = lon2 - lon1
        a = np.sin(dlat / 2.0)**2 + np.cos(lat1) * np.cos(lat2) * np.sin(dlon / 2.0)**2
        c = 2 * np.arctan2(np.sqrt(a), np.sqrt(1 - a))
        ground_distance_m = R_earth * c

        delta_h_m = (ac_alt_ft - rec_elevs_ft) * ft_to_meters

        slant_distance_m = np.sqrt(ground_distance_m**2 + delta_h_m**2)
        elevation_angle_rad = np.arctan2(delta_h_m, ground_distance_m)

        return ground_distance_m, slant_distance_m, elevation_angle_rad

    # Extract receptor arrays for fast computation
    rec_grid = calc_grid(grid_dimensions[0], grid_dimensions[1], grid_location)
    #df = downsample_spatial_grid(df, stride_factor=2)
    rec_lats = rec_grid[1]
    rec_lons = rec_grid[0]
    rec_elevs = np.full(len(rec_lats),receptor_alt_MSL) #assume flat grid
    num_receptors = len(rec_lats)

    # -------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
    # Run Simulation Loop
    # ------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
    positionsx = []
    positionsy = []
    sim_results = []
    dt_array = [] # Store time steps for SEL integration
    
    # Aircraft speed from your conditions
    U_flight = path.ground_speed_kts
    U_flight = [x * 0.5144 for x in U_flight] # kts to m/s
    settings = Data() # initialize inputs for Jet Noise Model
    settings.center_frequencies = np.pad(frequency, (5, 0), mode='constant')
    altflight = []
    tr=[]
    ac_lat = path.latitude #degrees
    ac_lon = path.longitude #degrees
    ac_alt = path.altitude_MSL_ft

    for index in range(0,len(ac_lat)):
        if grid_location[0][1] < ac_lat[index] < grid_location[1][1] and grid_location[0][0] < ac_lon[index] < grid_location[1][0]+0.05:
        #if the aircraft crosses into the grid, begin simulations for the receptors
        #of not, skip to save computation time
            
            #trp = None
            if index > 22:

                positionsx.append(ac_lat[index])
                positionsy.append(ac_lon[index])
            #tr.append(round(trp))

            # 1. Calculate distances
            ground_dist, los_distance, angle_to_ground = calc_3d_dist_vectorized(
                ac_lat[index], ac_lon[index], ac_alt[index], rec_lats, rec_lons, rec_elevs
            )
            
            # 2. Proper 3D Polar Angle and dt calculation
            R_earth = 6371000.0
            
            if index > 0 and index < len(ac_alt)+1:
                # Use current minus previous for forward motion vector
                
                dx_flight = np.radians(ac_lon[index] - ac_lon[index-1]) * R_earth * np.cos(np.radians(ac_lat[index]))
                dy_flight = np.radians(ac_lat[index] - ac_lat[index-1]) * R_earth
                dz_flight = (ac_alt[index] - ac_alt[index-1]) * Units.feet
            else:
                # Fallback for index == 0: look ahead to index + 1 instead of pointing backward (-1.0) CHECK THIS
                if index + 1 < len(ac_alt):
                    dx_flight = np.radians(ac_lon[index+1] - ac_lon[index]) * R_earth * np.cos(np.radians(ac_lat[index]))
                    dy_flight = np.radians(ac_lat[index+1] - ac_lat[index]) * R_earth
                    dz_flight = (ac_alt[index+1] - ac_alt [index]) * Units.feet
                else:
                    dx_flight, dy_flight, dz_flight = 1.0, 0.0, 0.0
                
            mag_flight = np.sqrt(dx_flight**2 + dy_flight**2 + dz_flight**2)
            
            # Ensure unit vector points forward (default to positive x if magnitude is 0)
            hx, hy, hz = (dx_flight/mag_flight, dy_flight/mag_flight, dz_flight/mag_flight) if mag_flight > 0 else (1.0, 0.0, 0.0)
            
            # Calculate time delta for SEL (dt = distance / speed)
            dt = mag_flight / U_flight[index]
            dt_array.append(dt)

            # B. Determine Observer Vector (Aircraft -> Receptor)
            dx_obs = np.radians(rec_lons - ac_lon[index]) * R_earth * np.cos(np.radians(ac_lat[index]))
            dy_obs = np.radians(rec_lats - ac_lat[index]) * R_earth
            dz_obs = (-(rec_elevs - ac_alt[index]) * Units.feet)
            altflight.append(round(dz_obs[0]))
            l_seg_m = ground_dist

            d_AS = ((dx_obs * dx_flight) + (dy_obs * dy_flight)) / mag_flight
            d_AS = np.clip(d_AS, 0, mag_flight)
            
            # d_seg_m is your vertical AGL altitude component at CPA
            d_seg_m = (ac_alt[index] * Units.feet) + d_AS * (dz_flight / mag_flight) - (rec_elevs * Units.feet)
            d_seg_m = np.maximum(d_seg_m, Units.feet) # AEDT uses a minimum distance/altitude limit of 1 ft (~0.3m)

            # C. Dot Product to find True Polar Angle (0 = Nose, 180 = Tail)
            dot_prod = (dx_obs * hx) + (dy_obs * hy) + (dz_obs * hz)
            cos_theta = np.clip(dot_prod / los_distance, -1.0, 1.0)
            
            theta_proper = np.arccos(cos_theta)
            
            # Assign the true polar angle to the components
            tf = theta_proper
            te = theta_proper
            theta_raw_arr = theta_proper

            # 3. Pre-allocate array to store the final scalar noise value (OASPL) for this timestep
            total_SPL_map = np.zeros(num_receptors)
            LADJ_dB,beta_deg = AEDT_LADJ_Attenuation(l_seg_m, d_seg_m, bank_angle_deg=0.0)
            

            # 4. Loop through each receptor to run RCAIDE noise models
            for i in range(num_receptors):
                # Extract scalar values and format as 2D arrays
                R_val = np.array([[los_distance[i]]]) #passed in meters
                
                # FIX: Use the true polar angle for landing gear as well
                theta_raw = np.array([[theta_raw_arr[i]]])
                theta_flap = np.array([[tf[i]]])
                theta_engine = np.array([[te[i]]])
                print('computing noise step',index)
                # Pass segment.state to match internal RCAIDE condition structure
                aero_data = segment.state.conditions.aeroacoustics.propulsors[turbofan.tag]
                
                # Convert polar theta to Cartesian coordinates for jet_noise function logic
                mic_x = los_distance[i] * np.cos(theta_proper[i])
                mic_y = los_distance[i] * np.sin(theta_proper[i])
                mic_locations = np.array([[mic_x, mic_y, 0.0]])

                if fast == True:
                
                # --- RUN NOISE MODELS ---
                    slat_noise_val = slat_noise(R_val, phi, theta_flap[0][0], Ls, gamma_s, sigma_s, alpha, segment, frequency, A=1e-5)

                    if path.thrust_reverse[index] == 'Reversed Thrust' or path.thrust_reverse[index] == 'Idle Approach':
                        jet_noise = compute_jet_noise_new(mic_locations, turbofan, aero_data, segment.state, frequency,1)
                    else:
                        jet_noise = compute_jet_noise_new(mic_locations, turbofan, aero_data, segment.state, frequency,0)

                    jet_spec_dBA = jet_noise.SPL_1_3_spectrum[0][0]
                    jet_spec_raw = jet_spec_dBA
                    
                    # Combine spectra logarithmically
                    spectra = np.array([
                        slat_noise_val[0],
                        jet_spec_raw
                    ])
                    total_spectrum = SPL_arithmetic(spectra, sum_axis=0)

                else:
                    lg_noise = compute_landing_gear_noise(R_val, theta_raw, D, H, W, wheels, M, Weight, strut_diameter, frequency, segment)
                    flap_noise = flap_noise_model(R_val, theta_flap, cf, thickness, deltaf, frequency, segment)
                    slat_noise_val = slat_noise(R_val, phi, theta_flap[0][0], Ls, gamma_s, sigma_s, alpha, segment, frequency, A=1e-5)
                    
                    if path.thrust_reverse[index] == 'Reversed Thrust' or path.thrust_reverse[index] == 'Idle Approach':
                        jet_noise = compute_jet_noise_new(mic_locations, turbofan, aero_data, segment.state, frequency,1)
                    else:
                        jet_noise = compute_jet_noise_new(mic_locations, turbofan, aero_data, segment.state, frequency,0)

                    #Run turbofan models
                    fan_noise = compute_fan_noise(R_val[0], theta_engine[0][0], turbofan, m, segment.state.conditions.aeroacoustics, segment, frequency)
                    core_noise = compute_core_noise(R_val, theta_engine, turbofan, pr, segment.state.conditions.aeroacoustics, segment, frequency)
                    
                    jet_spec_dBA = jet_noise.SPL_1_3_spectrum[0][0]
                    jet_spec_raw = jet_spec_dBA
                    
                    # Combine spectra logarithmically
                    spectra = np.array([
                        lg_noise.Total[0], 
                        flap_noise[0], 
                        slat_noise_val[0],
                        fan_noise.SPL_1_3_spectrum[0][0], 
                        core_noise.SPL_1_3_spectrum[0][0],
                        jet_spec_raw
                    ])
                    total_spectrum = SPL_arithmetic(spectra, sum_axis=0)
                
                # 2. Package the scalar distance into an array to satisfy RCAIDE's len(dist) check
                dist_array = np.array([los_distance[i]])
                
                # 3. Calculate atmospheric attenuation
                att_dB = atmospheric_attenuation(dist_array, frequency)[0]
                
                # subtract attenuation factors
                attenuated_spectrum = total_spectrum - (att_dB) - LADJ_dB[i] #1.1 squeeze multiplier
                # 4. Apply A-weighting filter to the spectrum
                a_weighted_spectrum = A_weighting_metric(attenuated_spectrum, frequency)

                # 5. Convert the A-weighted spectrum into a single scalar dBA / SEL value
                oaspl = 10 * np.log10(np.sum(10 ** (a_weighted_spectrum / 10)))
                total_SPL_map[i] = oaspl

            # -------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
            # Plotting the results for this timestep
            # -------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
            sim_results.append(total_SPL_map)

    if len(sim_results) == 0:
            print("Warning: The track data did not intersect the requested grid boundaries.")
            return None
    sim_results_arr = np.array(sim_results)
    dt_arr = np.array(dt_array).reshape(-1, 1)        
    energy_integral = np.sum((10**(sim_results_arr / 10.0)) * dt_arr, axis=0)
    z = 10 * np.log10(energy_integral)

    x = rec_lons
    y = rec_lats

    np.savez('b737_high_res_footprint_4.npz', 
                        longitude=x, 
                        latitude=y, 
                        sel_dBA=z)

    if plot == True:
        import matplotlib.tri as tri
        
        # 3. Setup the plot
        fig, ax = plt.subplots(figsize=(10, 8), dpi=120)
    
        # 4. Create an unstructured triangulation grid and plot the heatmap
        triangulation = tri.Triangulation(x, y)
        
        # Generate the contour
        levels = np.linspace(42,120, 40) # 40 colors

        heatmap = ax.tricontourf(triangulation, z, levels = levels, cmap='jet', extend='both')
        plt.plot(positionsy,positionsx,'ko',markersize=1)

        #uncomment to annotate flight path.

        #for index, (x, y) in enumerate(zip(positionsy, positionsx), start=0):
            #plt.annotate(f"{tr[index]}", (x, y), textcoords="offset points", xytext=(1, 1),fontsize=3)

        # 5. Add colorbar and labels
        cbar = fig.colorbar(heatmap, ax=ax)
        cbar.set_label(f'Level (Exposure) - SEL', fontsize=12, fontweight='bold')
    
        ax.set_title('B737 Simulated Noise Footprint', fontsize=14, fontweight='bold', pad=15)
        ax.set_xlabel('Longitude', fontsize=12)
        ax.set_ylabel('Latitude', fontsize=12)
        
        # Format axes with a subtle grid
        ax.grid(True, linestyle='--', alpha=0.5, color='gray')
        
        # Keep the geographic spatial scales proportional based on the center latitude
        mean_lat = np.mean(y)
        ax.set_aspect(1.0 / np.cos(np.radians(mean_lat)))

    
        # 6. Display the plot
        plt.show()
        return Data(lat_lon = [x,y], SEL = z)

    return Data(lat_lon = [x,y], SEL = z)
            
def plot_parameters():
     
    plt.rcParams.update({'font.size': 12})
    plt.rcParams['axes.linewidth'] = 1. 
 
    PP = Data(  
        fig_size_width  = 14 ,
        fig_size_height = 9 ,       
        lw  = 1,                             # line_width               
        m   = 5,                             # markersize               
        lf  = 10,                            # legend_font_size         
        Slc = ['black','green','yellow'],    # line_colors        
        Slm = ['^','o','s'],                 # line_markers       
        Sls = '-',                           # line_styles        
        Elc = ['darkred','red','tomato'],    # Experimental_line_colors 
        Elm = ['s'],                         # Experimental_line_markers
        Els = '',                            # Experimental_line_styles 
        Rlc = ['darkblue','blue','cyan'],    # Ref_Code_line_colors     
        Rlm = ['o'],                         # Ref_Code_line_markers    
        Rls = ':',                           # Ref_Code_line_styles     
    )   
    
    return PP  



def AEDT_LADJ_Attenuation(l_seg_m, d_seg_m, bank_angle_deg=0.0):
    """
    Computes the AEDT Lateral Attenuation Adjustment (LA_ADJ) for a wing-mounted 
    civil aircraft according to SAE-AIR-5662.
    
    Parameters:
    - l_seg_m: numpy array of horizontal sideline distances from the segment to the receptor [meters]
    - d_seg_m: numpy array of AGL altitudes of the aircraft at the CPA [meters]
    - bank_angle_deg: aircraft bank angle [degrees] (default is 0.0 for straight flight)
    
    Returns:
    - LA_ADJ: numpy array of the total lateral attenuation adjustment [dB]
    """
    
    # 1. Geometry Processing
    # Calculate slant range and prevent divide-by-zero errors
    SLR_seg = np.sqrt(d_seg_m**2 + l_seg_m**2)
    SLR_seg = np.maximum(SLR_seg, 1e-6)
    
    # Elevation angle (beta)
    beta_rad = np.arcsin(d_seg_m / SLR_seg)
    beta_deg = np.degrees(beta_rad)
    
    # Depression angle (phi)
    phi_deg = bank_angle_deg + beta_deg
    phi_rad = np.radians(phi_deg)
    
    # 2. Engine Installation Effect (E_WING) for Wing-Mounted Jets
    E_WING = np.zeros_like(phi_deg)
    
    # Masks for valid phi ranges
    mask_pos = (phi_deg >= 0.0) & (phi_deg <= 180.0)
    mask_neg = (phi_deg < 0.0) & (phi_deg >= -180.0)
    
    # Calculate positive phi 
    cos2_phi = np.cos(phi_rad[mask_pos])**2
    sin2_phi = np.sin(phi_rad[mask_pos])**2
    sin2_2phi = np.sin(2.0 * phi_rad[mask_pos])**2
    cos2_2phi = np.cos(2.0 * phi_rad[mask_pos])**2
    
    num = (0.0039 * cos2_phi + sin2_phi)**0.062
    den = (0.8786 * sin2_2phi + cos2_2phi)
    
    E_WING[mask_pos] = 10.0 * np.log10(num / den)
    
    # Calculate negative phi
    E_WING[mask_neg] = -1.49
    
    # 3. Ground-to-Ground Effect (G)
    G = np.full_like(l_seg_m, 10.86) # Default to > 914m case
    mask_G = (l_seg_m >= 0.0) & (l_seg_m <= 914.0)
    
    G[mask_G] = 11.83 * (1.0 - np.exp(-0.00274 * l_seg_m[mask_G]))
    
    # 4. Air-to-Ground Effect (Lambda)
    Lambda = np.zeros_like(beta_deg)
    
    # Enforce beta bounds (beta <= 0 is treated as 0)
    beta_eff = np.maximum(beta_deg, 0.0)
    
    mask_L = (beta_eff >= 0.0) & (beta_eff <= 50.0)
    Lambda[mask_L] = 1.137 - (0.0229 * beta_eff[mask_L]) + (9.72 * np.exp(-0.142 * beta_eff[mask_L]))
    # For beta > 50, Lambda remains 0.0
    
    # 5. Overall Lateral Attenuation Adjustment (LA_ADJ)
    LA_ADJ = -(E_WING - ((G * Lambda) / 10.86))
    
    return LA_ADJ, beta_deg


def downsample_spatial_grid(df, stride_factor):
    """
    Downsamples a spatial latitude/longitude grid dataframe by a given stride factor.
    If the dataframe is structured as a regular grid, this reduces resolution uniformly.
    
    Parameters:
    - df: pandas DataFrame containing 'Latitude' and 'Longitude' columns.
    - stride_factor: integer step size (e.g., 4 means keep every 4th point along the grid lines).
    
    Returns:
    - Downsampled pandas DataFrame.
    """
    # If the dataframe has a known grid structure, we can downsample by unique coordinate sorting or simple slicing
    # Assuming it's a flattened meshgrid of points:
    if 'Latitude' in df.columns and 'Longitude' in df.columns:
        # Sort by latitude and longitude to ensure predictable ordering
        df_sorted = df.sort_values(by=['Latitude', 'Longitude']).reset_index(drop=True)
        
        # Get unique lats and lons to reconstruct grid dimensions if needed
        unique_lats = np.sort(df['Latitude'].unique())
        unique_lons = np.sort(df['Longitude'].unique())
        
        # Downsample unique coordinates by the stride factor
        ds_lats = unique_lats[::stride_factor]
        ds_lons = unique_lons[::stride_factor]
        
        # Filter the dataframe to only include these downsampled coordinates
        df_downsampled = df[df['Latitude'].isin(ds_lats) & df['Longitude'].isin(ds_lons)].reset_index(drop=True)
        
        return df_downsampled
    else:
        # Fallback: simple row slicing if columns aren't standard
        return df.iloc[::stride_factor, :].reset_index(drop=True)
 
if __name__ == '__main__': 
    main()  
    plt.show()
    