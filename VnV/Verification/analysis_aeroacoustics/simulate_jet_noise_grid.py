# empirical_jet_noise_test.py
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
from RCAIDE.Framework.Mission.Common                                              import Results  
from RCAIDE.Framework.Mission.Segments.Segment                                    import Segment 
from RCAIDE.Framework.Mission.Common                                              import Conditions 
from RCAIDE.Library.Methods.Aeroacoustics.Common   import SPL_arithmetic 
from RCAIDE.Library.Plots import * 
 
# Python Imports  
import sys
import matplotlib.pyplot as plt 
import numpy as np     
from copy import deepcopy
import os
import pandas as pd
# local imports 
base_dir = os.path.dirname(os.path.abspath(__file__))

vehicles_path = os.path.abspath(
    os.path.join(base_dir, "..", "..", "Vehicles")
)

if vehicles_path not in sys.path:
    sys.path.insert(0, vehicles_path)

# ----------------------------------------------------------------------
#   Main
# ----------------------------------------------------------------------
 
# ----------------------------------------------------------------------
#   Main
# ---------------------------------------------------------------------- 
def main():  
    # define plotting parameters 
    PP = plot_parameters()  

    # landing gear noise validation
    Landing_Gear_Validation(PP) 
    
    return  
    
    
# ------------------------------------------------------------------ 
# Harmonic Noise Validation
# ------------------------------------------------------------------  
def Landing_Gear_Validation(PP): 
    
    # define aircraft properties 
    '''gear_params: dict
    - num_wheels (Nw): Number of wheels
    - wheel_diam (d): Wheel diameter [inches]
    - wheel_width (w): Wheel width [inches]
    - strut_lengths (L_j): List of lengths of struts [inches]
    - strut_dims (dim_j): List of diameters/widths of struts [inches]
    - aircraft_weight (W_ac): Max Takeoff Weight [lbs]
    - track_angle (gamma): Wheel track alignment angle [degrees]
    
flight_params: dict
    - M_flight: Flight Mach number
    - theta: Emission angle [degrees] (90 is overhead)
    - R: Distance to observer [ft]
    - c0: Speed of sound [ft/s] (default 1116)
    - rho0: Air density [slugs/ft^3] (default 0.00237)'''

    #define params for landing gear model
    D = 1.016 #m
    H = 8.0518 #m
    wheels= 2
    Weight = 68038.8555 #kg
    strut_diameter=0.11811#m
    theta =(np.pi)/2 #deg 
    frequency = np.logspace(1.5, 4.5, 100)
    W = 0.3556#m
    
    #define params for flap model
    thickness = 0.1#m
    cf = 0.9#m
    deltaf = np.radians(37.5)

    #define params for slat model
    phi= 0
    Ls = 0.08128
    gamma_s = np.radians(20)
    sigma_s =  np.radians(25)
    alpha =  np.radians(10)

    #define param for core noise model
    pr = 13.1
    


 # define operating conditions                                            
    a                       = 343.376
    T                       = 288.16889478  
    density                 = 1.2250	
    dynamic_viscosity       = 1.81E-5   
    ctrl_pts                = 1
    AoA                     = 4
    U = 103 #aircraft velocity
    M = 0.2 #mach number
    frequency_flp = np.logspace(1, 4, 100)

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

    #combustor
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

    m = None #mass flow rate


    # define microphone locations

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
 
    segment.state.conditions.aeroacoustics.propulsors[turbofan.tag].fan.angular_velocity = 4200
    segment.state.conditions.aeroacoustics.propulsors[turbofan.tag].fan.exit_velocity = 350 * Units.mph
    segment.state.conditions.aeroacoustics.propulsors[turbofan.tag].fan.exit_stagnation_temperature = 440
    segment.state.conditions.aeroacoustics.propulsors[turbofan.tag].fan.exit_stagnation_pressure = 152*1000
    
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
    # Take in Reference AEDT run and plot heatmap
    # -------------------------------------------------------------------------------------------------------------------------------------------------------------------------------  

    #df = pd.read_csv('/Users/siripunn/Desktop/LEADS_WORK/LEADS_Research/RCAIDE_LEADS/VnV/Verification/analysis_aeroacoustics/b737_sim_noise.csv')

    # -------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
    # Read Path Data and Receptor Data (Using Relative Paths)
    # -------------------------------------------------------------------------------------------------------------------------------------------------------------------------------  

    df = pd.read_csv('/Users/siripunn/Desktop/LEADS_WORK/LEADS_Research/RCAIDE_LEADS/VnV/Verification/analysis_aeroacoustics/b737_sim_noise.csv')
    track_df = pd.read_csv('/Users/siripunn/Desktop/LEADS_WORK/LEADS_Research/RCAIDE_LEADS/VnV/Verification/analysis_aeroacoustics/b737_sim_track_interpolated.csv')

    # -------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
    # Vectorized Distance & Angle Calculator
    # ------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- 
    def calc_3d_dist_vectorized(ac_lat, ac_lon, ac_alt_ft, rec_lats, rec_lons, rec_elevs_ft):
        R_earth = 6371000.0  # Earth's radius in meters
        ft_to_meters = 0.3048

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
    df = downsample_spatial_grid(df, stride_factor=2)
    rec_lats = df['Latitude'].values
    rec_lons = df['Longitude'].values
    rec_elevs = df['Elevation MSL (ft)'].values #USE OTHER ELEV.
    num_receptors = len(df)

    # -------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
    # Run Simulation Loop
    # ------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
    positionsx = []
    positionsy = []
    sim_results = []
    # Loop over the first 5 steps in the aircraft path
# Loop over the first 5 steps in the aircraft path
    for index, track_point in track_df.head(45).iterrows(): 
        ac_lat = track_point['Latitude (deg)']
        ac_lon = track_point['Longitude (deg)']
        ac_alt = track_point['Altitude MSL (ft)'] 
        
        print(f"Aircraft Position: Lat {ac_lat}, Lon {ac_lon}, Alt {ac_alt} ft MSL")
        print(f"Grid Center: Lat {np.mean(rec_lats)}, Lon {np.mean(rec_lons)}")

        if index > 15:
            positionsx.append(ac_lat)
            positionsy.append(ac_lon)

            # 1. Calculate distances and angles for ALL receptors at once for this timestep
            ground_dist, los_distance, angle_to_ground = calc_3d_dist_vectorized(
                ac_lat, ac_lon, ac_alt, rec_lats, rec_lons, rec_elevs
            )
            
            # -------------------------------------------------------------------------
            # 2. PROPER 3D POLAR ANGLE (THETA) CALCULATION 
            # -------------------------------------------------------------------------
            R_earth = 6371000.0
            
            # A. Determine Aircraft Heading Vector 
            if index > 0:
                prev_pt = track_df.iloc[index - 1]
                # FIX 2: Added R_earth to dx and dy so the flight vector is in meters, not radians
                dx_flight = np.radians(ac_lon - prev_pt['Longitude (deg)']) * R_earth * np.cos(np.radians(ac_lat))
                dy_flight = np.radians(ac_lat - prev_pt['Latitude (deg)']) * R_earth
                dz_flight = (ac_alt - prev_pt['Altitude MSL (ft)']) * 0.3048
            else:
                dx_flight, dy_flight, dz_flight = -1.0, 0.0, 0.0
                
            mag_flight = np.sqrt(dx_flight**2 + dy_flight**2 + dz_flight**2)
            hx, hy, hz = (dx_flight/mag_flight, dy_flight/mag_flight, dz_flight/mag_flight) if mag_flight > 0 else (-1.0, 0.0, 0.0)

            # B. Determine Observer Vector (Aircraft -> Receptor)
            dx_obs = np.radians(rec_lons - ac_lon) * R_earth * np.cos(np.radians(ac_lat))
            dy_obs = np.radians(rec_lats - ac_lat) * R_earth
            dz_obs = (rec_elevs - ac_alt) * 0.3048

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

            # 4. Loop through each receptor to run RCAIDE noise models
            for i in range(num_receptors):
                # Extract scalar values and format as 2D arrays
                R_val = np.array([[los_distance[i]]]) #passed in meters
                
                # FIX: Use the true polar angle for landing gear as well
                theta_raw = np.array([[theta_raw_arr[i]]]) 
                theta_flap = np.array([[tf[i]]])
                theta_engine = np.array([[te[i]]])
                print('step',index,'iteration',i, 'ac_true_h')
                
                # --- RUN NOISE MODELS ---
                lg_noise = compute_landing_gear_noise(R_val, theta_raw, D, H, W, wheels, M, Weight, strut_diameter, frequency, segment)
                flap_noise = flap_noise_model(R_val, theta_flap, cf, thickness, deltaf, frequency, segment)
                slat_noise_val = slat_noise(R_val, phi, theta_flap[0][0], Ls, gamma_s, sigma_s, alpha, segment, frequency, A=1e-5)
                
                fan_noise = compute_fan_noise(R_val[0], theta_engine[0][0], turbofan, m, segment.state.conditions.aeroacoustics, segment, frequency)
                core_noise = compute_core_noise(R_val, theta_engine, turbofan, pr, segment.state.conditions.aeroacoustics, segment, frequency)
                
                # Combine spectra logarithmically
                # Ensure slicing matches the output shape of your RCAIDE models (usually [0][0] for 1st ctrl pt, 1st observer)
                spectra = np.array([
                    lg_noise.Total[0], 
                    flap_noise[0], 
                    slat_noise_val[0], # problematic - overpredicting to 107 dB
                    fan_noise.SPL_1_3_spectrum[0][0], #slightly overpredicting - 84dB
                    core_noise.SPL_1_3_spectrum[0][0], #bottleneck here
                ])
                
                total_spectrum = SPL_arithmetic(spectra, sum_axis=0)
                
                # Convert the 1/3 octave band spectrum into a single Overall Sound Pressure Level (OASPL)
                # You cannot map a 100-length array to a color dot; you need a single dB value per coordinate.
                oaspl = 10 * np.log10(np.sum(10 ** (total_spectrum / 10)))
                total_SPL_map[i] = oaspl

            # -------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
            # Plotting the results for this timestep
            # -------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
            sim_results.append(total_SPL_map)
            

        if len(sim_results) == 9+20:
            print(positionsx,positionsy)
            import matplotlib.tri as tri
            
            # FIX: Logarithmic energy sum for Sound Exposure
            # Converts dB back to linear Pascals, sums them across time, converts back to dB
            z = 10 * np.log10(np.sum(10**(np.array(sim_results) / 10.0), axis=0))
            
            x = rec_lons
            y = rec_lats
            # 3. Setup the plot
            fig, ax = plt.subplots(figsize=(10, 8), dpi=120)
        
            # 4. Create an unstructured triangulation grid and plot the heatmap
            triangulation = tri.Triangulation(x, y)
            
            # Generate the filled contour (heatmap)
            # 'jet' is a standard colormap for aeroacoustic footprints
            levels = np.linspace(np.min(z), np.max(z), 20) # 40 smooth color transitions
            heatmap = ax.tricontourf(triangulation, z, levels=levels, cmap='jet', extend='both')
            plt.plot(positionsy,positionsx,'ko',markersize=1)
        
            # 5. Add colorbar and labels
            cbar = fig.colorbar(heatmap, ax=ax)
            cbar.set_label(f'Level (Exposure)', fontsize=12, fontweight='bold')
        
            ax.set_title('B737 Simulated Noise Footprint', fontsize=14, fontweight='bold', pad=15)
            ax.set_xlabel('Longitude', fontsize=12)
            ax.set_ylabel('Latitude', fontsize=12)
            
            # Format axes with a subtle grid
            ax.grid(True, linestyle='--', alpha=0.5, color='gray')
            
            # Keep the geographic spatial scales proportional based on the center latitude
            # This approximates a Mercator projection so the map isn't stretched
            mean_lat = np.mean(y)
            ax.set_aspect(1.0 / np.cos(np.radians(mean_lat)))
        
            # 6. Display the plot
            plt.show()
            print(np.min(los_distance))
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
        
        print(f"Original Grid Resolution: {len(unique_lats)} lats x {len(unique_lons)} lons ({len(df)} total points)")
        
        # Downsample unique coordinates by the stride factor
        ds_lats = unique_lats[::stride_factor]
        ds_lons = unique_lons[::stride_factor]
        
        # Filter the dataframe to only include these downsampled coordinates
        df_downsampled = df[df['Latitude'].isin(ds_lats) & df['Longitude'].isin(ds_lons)].reset_index(drop=True)
        
        print(f"Downsampled Grid Resolution: {len(ds_lats)} lats x {len(ds_lons)} lons ({len(df_downsampled)} total points)")
        return df_downsampled
    else:
        # Fallback: simple row slicing if columns aren't standard
        return df.iloc[::stride_factor, :].reset_index(drop=True)
 
if __name__ == '__main__': 
    main()  
    plt.show()
    