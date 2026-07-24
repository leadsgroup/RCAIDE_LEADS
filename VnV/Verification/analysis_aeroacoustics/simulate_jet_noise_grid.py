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
from RCAIDE.Library.Methods.Aeroacoustics.Semi_Empirical.Airframe.landing_gear_noise_model import compute_landing_gear_noise
from RCAIDE.Library.Methods.Aeroacoustics.Semi_Empirical.Airframe.flap_noise_model import flap_noise_model
from RCAIDE.Library.Methods.Aeroacoustics.Semi_Empirical.Airframe.slat_noise_model import slat_noise
from RCAIDE.Library.Methods.Aeroacoustics.Semi_Empirical.Propulsion.Engine_Noise.compute_fan_noise import compute_fan_noise
from RCAIDE.Library.Methods.Aeroacoustics.Semi_Empirical.Propulsion.Engine_Noise.compute_core_noise import compute_core_noise
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
    track_df = pd.read_csv('/Users/siripunn/Desktop/LEADS_WORK/LEADS_Research/RCAIDE_LEADS/VnV/Verification/analysis_aeroacoustics/b737_sim_track.csv')

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
    rec_lats = df['Latitude'].values
    rec_lons = df['Longitude'].values
    rec_elevs = df['Elevation MSL (ft)'].values
    num_receptors = len(df)

    # -------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
    # Run Simulation Loop
    # ------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
    # Loop over the first 5 steps in the aircraft path
    for index, track_point in track_df.head(20).iterrows(): 
        ac_lat = track_point['Latitude (deg)']
        ac_lon = track_point['Longitude (deg)']
        ac_alt = track_point['Altitude MSL (ft)']
        print(f"Aircraft Position: Lat {ac_lat}, Lon {ac_lon}, Alt {ac_alt} ft")
        print(f"Grid Center: Lat {np.mean(rec_lats)}, Lon {np.mean(rec_lons)}")

        if index > 10:

            # 1. Calculate distances and angles for ALL receptors at once for this timestep
            ground_dist, los_distance, angle_to_ground = calc_3d_dist_vectorized(
                ac_lat, ac_lon, ac_alt, rec_lats, rec_lons, rec_elevs
            )
            
            # 2. Calculate theta values (tf: Flap angle, te: Engine angle)
            # Create masks to determine if the aircraft is ahead or behind the receptor
            mask_behind = ac_lon <= rec_lons
            mask_ahead = ~mask_behind
            
            tf = np.zeros(num_receptors)
            te = np.zeros(num_receptors)
            
            # Prevent invalid arcsin domain due to floating point inaccuracies
            ratio_los = np.clip(ground_dist / los_distance, -1.0, 1.0)
            ratio_alt = ground_dist / (ac_alt * 0.3048) # Converting aircraft altitude to meters
            
            # Vectorized assignment
            tf[mask_behind] = (np.pi/2) - np.arcsin(ratio_los[mask_behind])
            te[mask_behind] = (np.pi/2) - np.arctan(ratio_alt[mask_behind])
            
            tf[mask_ahead] = (np.pi/2) + np.arcsin(ratio_los[mask_ahead])
            te[mask_ahead] = (np.pi/2) + np.arctan(ratio_alt[mask_ahead])

            # 3. Pre-allocate array to store the final scalar noise value (OASPL) for this timestep
            total_SPL_map = np.zeros(num_receptors)

            # 4. Loop through each receptor to run RCAIDE noise models
            for i in range(num_receptors):
                # Extract scalar values and format as 2D arrays (RCAIDE generally expects 2D inputs for these states)
                R_val = np.array([[los_distance[i]*Units.feet]]) #passed in meters
                theta_raw = np.array([[angle_to_ground[i]]])
                theta_flap = np.array([[tf[i]]])
                theta_engine = np.array([[te[i]]])
                
                # --- RUN NOISE MODELS ---
                lg_noise = compute_landing_gear_noise(R_val, theta_raw, D, H, W, wheels, M, Weight, strut_diameter, frequency, segment)
                flap_noise = flap_noise_model(R_val, theta_flap, cf, thickness, deltaf, frequency, segment)
                slat_noise_val = slat_noise(R_val, phi, theta_flap[0][0], Ls, gamma_s, sigma_s, alpha, segment, frequency, A=1e-5)
                
                fan_noise = compute_fan_noise(R_val[0], theta_engine[0][0], turbofan, m, segment.state.conditions.aeroacoustics, segment, frequency)
                core_noise = compute_core_noise(R_val, theta_engine, turbofan, pr, segment.state.conditions.aeroacoustics, segment, frequency)
                
                # Combine spectra logarithmically
                # Ensure slicing matches the output shape of your RCAIDE models (usually [0][0] for 1st ctrl pt, 1st observer)
                spectra = np.array([
                    lg_noise.Total, 
                    #flap_noise[0], 
                    slat_noise_val[0], 
                    fan_noise.SPL_1_3_spectrum[0][0], 
                    core_noise.SPL_1_3_spectrum[0][0]
                ])
                
                total_spectrum = SPL_arithmetic(spectra, sum_axis=0)
                
                # Convert the 1/3 octave band spectrum into a single Overall Sound Pressure Level (OASPL)
                # You cannot map a 100-length array to a color dot; you need a single dB value per coordinate.
                oaspl = 10 * np.log10(np.sum(10 ** (total_spectrum / 10)))
                total_SPL_map[i] = oaspl

            # -------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
            # Plotting the results for this timestep
            # -------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
            scatter = plt.scatter(
                x=rec_lons, 
                y=rec_lats, 
                c=total_SPL_map,    # Now mapping a 1D array of OASPL floats
                cmap='jet',         
                marker='s',         
                s=15,               
                edgecolors='none' 
            )
            cbar = plt.colorbar(scatter)
            cbar.set_label('Total Noise Level (OASPL dB)', rotation=270, labelpad=15)

            plt.title(f'RCAIDE Simulation Noise - Timestep {index}')
            plt.xlabel('Longitude')
            plt.ylabel('Latitude')
            plt.gca().set_aspect('equal', adjustable='datalim')

            plt.tight_layout()
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
 
if __name__ == '__main__': 
    main()  
    plt.show()