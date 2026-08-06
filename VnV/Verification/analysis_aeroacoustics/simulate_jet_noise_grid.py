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
    PP = plot_parameters()  

    # landing gear noise validation
    simulate_vehicle_noise_grid(PP) 
    
    return  
    
    
# ------------------------------------------------------------------ 
# Harmonic Noise Validation
# ------------------------------------------------------------------  
def simulate_vehicle_noise_grid(PP): 
    
    #define params for landing gear model
    D = 1.016 #m
    H = 1.2 #m
    wheels= 2
    Weight = 68038.8555 #kg
    strut_diameter=0.11811#m
    theta =(np.pi)/2 #deg 

    frequency = frequency = np.array([
        50.0, 63.0, 80.0, 100.0, 125.0, 160.0, 200.0, 250.0, 
        315.0, 400.0, 500.0, 630.0, 800.0, 1000.0, 1250.0, 1600.0, 
        2000.0, 2500.0, 3150.0, 4000.0, 5000.0, 6300.0, 8000.0, 10000.0
    ])

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
    m = None # predefined mass flow rate
    


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

    turbofan.origin = np.array([[0.0, 0.0, 1.5]]) # Engine height assumed 1.0m
    turbofan.length = 97*Units.inches
    turbofan.diameter = 70*Units.inches
    turbofan.plug_diameter = 60*Units.inches
    turbofan.geometry_xe = 1.0
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

    df = pd.read_csv('/Users/siripunn/Desktop/LEADS_WORK/LEADS_Research/RCAIDE_LEADS/VnV/Verification/analysis_aeroacoustics/b737_sim_noise_SEL_N_TR (1).csv')
    track_df = pd.read_csv('/Users/siripunn/Desktop/LEADS_WORK/LEADS_Research/RCAIDE_LEADS/VnV/Verification/analysis_aeroacoustics/new_track.csv')

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
    rec_lats = df['Latitude (deg)'].values
    rec_lons = df['Longitude (deg)'].values
    rec_elevs = df['Elevation MSL (ft)'].values
    num_receptors = len(df)

    # -------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
    # Run Simulation Loop
    # ------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
    positionsx = []
    positionsy = []
    sim_results = []
    dt_array = [] # Store time steps for SEL integration
    
    # Aircraft speed from your conditions
    U_flight = track_df['Ground Speed (kts)'].tolist()
    U_flight = [x * 0.5144 for x in U_flight] # kts to m/s
    settings = Data() # initialize inputs for Jet Noise Model
    settings.center_frequencies = np.pad(frequency, (5, 0), mode='constant')
    altflight = []
    tr=[]
    for index, track_point in track_df.head(50).iterrows(): 
        ac_lat = track_point['Latitude (deg)']
        ac_lon = track_point['Longitude (deg)']
        ac_alt = track_point['Altitude MSL (ft)']
        trp = track_point['Noise Thrust per Engine (lbs)']

        if index > 10:
            
            if index > 17:
                positionsx.append(ac_lat)
                positionsy.append(ac_lon)
                tr.append(round(trp))

            # 1. Calculate distances
            ground_dist, los_distance, angle_to_ground = calc_3d_dist_vectorized(
                ac_lat, ac_lon, ac_alt, rec_lats, rec_lons, rec_elevs
            )
            
            # 2. Proper 3D Polar Angle and dt calculation
            R_earth = 6371000.0
            
            if index > 0 and index < len(track_df)+1:
                # Use current minus previous for forward motion vector
                curr_pt = track_df.iloc[index]
                prev_pt = track_df.iloc[index - 1]
                
                dx_flight = np.radians(curr_pt['Longitude (deg)'] - prev_pt['Longitude (deg)']) * R_earth * np.cos(np.radians(ac_lat))
                dy_flight = np.radians(curr_pt['Latitude (deg)'] - prev_pt['Latitude (deg)']) * R_earth
                dz_flight = (curr_pt['Altitude MSL (ft)'] - prev_pt['Altitude MSL (ft)']) * Units.feet
            else:
                # Fallback for index == 0: look ahead to index + 1 instead of pointing backward (-1.0)
                if index + 1 < len(track_df):
                    next_pt = track_df.iloc[index + 1]
                    dx_flight = np.radians(next_pt['Longitude (deg)'] - ac_lon) * R_earth * np.cos(np.radians(ac_lat))
                    dy_flight = np.radians(next_pt['Latitude (deg)'] - ac_lat) * R_earth
                    dz_flight = (next_pt['Altitude MSL (ft)'] - ac_alt) * Units.feet
                else:
                    dx_flight, dy_flight, dz_flight = 1.0, 0.0, 0.0
                
            mag_flight = np.sqrt(dx_flight**2 + dy_flight**2 + dz_flight**2)
            
            # Ensure unit vector points forward (default to positive x if magnitude is 0)
            hx, hy, hz = (dx_flight/mag_flight, dy_flight/mag_flight, dz_flight/mag_flight) if mag_flight > 0 else (1.0, 0.0, 0.0)
            
            # Calculate time delta for SEL (dt = distance / speed)
            dt = mag_flight / U_flight[index]
            dt_array.append(dt)

            # B. Determine Observer Vector (Aircraft -> Receptor)
            dx_obs = np.radians(rec_lons - ac_lon) * R_earth * np.cos(np.radians(ac_lat))
            dy_obs = np.radians(rec_lats - ac_lat) * R_earth
            dz_obs = (-(rec_elevs - ac_alt) * Units.feet)
            altflight.append(round(dz_obs[0]))
            l_seg_m = ground_dist

            d_AS = ((dx_obs * dx_flight) + (dy_obs * dy_flight)) / mag_flight
            d_AS = np.clip(d_AS, 0, mag_flight)
            
            # d_seg_m is your vertical AGL altitude component at CPA
            d_seg_m = (ac_alt * Units.feet) + d_AS * (dz_flight / mag_flight) - (rec_elevs * Units.feet)
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
                print('step',index,'iteration',i, 'ac_true_h', dz_obs[0])
                
                # --- RUN NOISE MODELS ---
                #lg_noise = compute_landing_gear_noise(R_val, theta_raw, D, H, W, wheels, M, Weight, strut_diameter, frequency, segment)
                #flap_noise = flap_noise_model(R_val, theta_flap, cf, thickness, deltaf, frequency, segment)
                slat_noise_val = slat_noise(R_val, phi, theta_flap[0][0], Ls, gamma_s, sigma_s, alpha, segment, frequency, A=1e-5)

                # Pass segment.state to match internal RCAIDE condition structure
                aero_data = segment.state.conditions.aeroacoustics.propulsors[turbofan.tag]
                
                # Convert polar theta to Cartesian coordinates for jet_noise function logic
                mic_x = los_distance[i] * np.cos(theta_proper[i])
                mic_y = los_distance[i] * np.sin(theta_proper[i])
                mic_locations = np.array([[mic_x, mic_y, 0.0]])

                current_thrust = track_point['Airplane Thrust Type']

                if current_thrust == 'Reversed Thrust' or current_thrust == 'Idle Approach':
                    jet_noise = compute_jet_noise_new(mic_locations, turbofan, aero_data, segment.state, frequency,1)
                else:
                    jet_noise = compute_jet_noise_new(mic_locations, turbofan, aero_data, segment.state, frequency,0)

                #Run turbofan models
                
                #fan_noise = compute_fan_noise(R_val[0], theta_engine[0][0], turbofan, m, segment.state.conditions.aeroacoustics, segment, frequency)
                #core_noise = compute_core_noise(R_val, theta_engine, turbofan, pr, segment.state.conditions.aeroacoustics, segment, frequency)

                jet_spec_dBA = jet_noise.SPL_1_3_spectrum[0][0]
                jet_spec_raw = jet_spec_dBA
                
                # Combine spectra logarithmically
                spectra = np.array([
                    #lg_noise.Total[0], 
                    #flap_noise[0], 
                    slat_noise_val[0],
                    #fan_noise.SPL_1_3_spectrum[0][0], 
                    #core_noise.SPL_1_3_spectrum[0][0],
                    jet_spec_raw
                ])
                total_spectrum = SPL_arithmetic(spectra, sum_axis=0)
                
                # 2. Package the scalar distance into an array to satisfy RCAIDE's len(dist) check
                dist_array = np.array([los_distance[i]])
                
                # 3. Calculate atmospheric attenuation
                att_dB = atmospheric_attenuation(dist_array, frequency)[0]
                
                # subtract attenuation factors
                attenuated_spectrum = total_spectrum - (att_dB * 1.1) - LADJ_dB[i] #1.1 squeeze multiplier
                # 4. Apply A-weighting filter to the spectrum
                a_weighted_spectrum = A_weighting_metric(attenuated_spectrum, frequency)

                # 5. Convert the A-weighted spectrum into a single scalar dBA / SEL value
                oaspl = 10 * np.log10(np.sum(10 ** (a_weighted_spectrum / 10)))
                total_SPL_map[i] = oaspl

            # -------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
            # Plotting the results for this timestep
            # -------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
            sim_results.append(total_SPL_map)

        
        if len(sim_results) == 28+10:
            print(positionsx,positionsy)
            import matplotlib.tri as tri
            sim_results_arr = np.array(sim_results)
            dt_arr = np.array(dt_array).reshape(-1, 1)        
            energy_integral = np.sum((10**(sim_results_arr / 10.0)) * dt_arr, axis=0)
            z = 10 * np.log10(energy_integral)

            x = rec_lons
            y = rec_lats

            np.savez('b737_high_res_footprint_3.npz', 
                                longitude=x, 
                                latitude=y, 
                                sel_dBA=z)
            
            # 3. Setup the plot
            fig, ax = plt.subplots(figsize=(10, 8), dpi=120)
        
            # 4. Create an unstructured triangulation grid and plot the heatmap
            triangulation = tri.Triangulation(x, y)
            
            # Generate the contour
            levels = np.linspace(42,120, 40) # 40 colors

            heatmap = ax.tricontourf(triangulation, np.clip(z,20,120), levels = levels, cmap='jet', extend='both')
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
            print(dt_array)
            plt.show()
            
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
    