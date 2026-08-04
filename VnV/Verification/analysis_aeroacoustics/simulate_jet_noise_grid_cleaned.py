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
def main():  
    PP = plot_parameters()  
    Landing_Gear_Validation(PP) 
    return  
    
# ------------------------------------------------------------------ 
# Harmonic Noise Validation
# ------------------------------------------------------------------  
def Landing_Gear_Validation(PP): 
    # Define params for landing gear model
    D = 1.016 # m
    H = 1.2 # m
    wheels = 2
    Weight = 68038.8555 # kg
    strut_diameter = 0.11811 # m
    frequency = np.array([
        50.0, 63.0, 80.0, 100.0, 125.0, 160.0, 200.0, 250.0, 
        315.0, 400.0, 500.0, 630.0, 800.0, 1000.0, 1250.0, 1600.0, 
        2000.0, 2500.0, 3150.0, 4000.0, 5000.0, 6300.0, 8000.0, 10000.0
    ])
    W = 0.3556 # m
    
    # Define params for flap model
    thickness = 0.1 # m
    cf = 0.9 # m
    deltaf = np.radians(37.5)

    # Define params for slat model
    phi = 0
    Ls = 0.08128
    gamma_s = np.radians(20)
    sigma_s = np.radians(25)
    alpha = np.radians(10)

    # Define param for core noise model
    pr = 13.1

    # Define operating conditions                                            
    a = 343.376
    T = 288.16889478  
    density = 1.2250	
    dynamic_viscosity = 1.81E-5   
    ctrl_pts = 1
    AoA = 4
    U = 103 # aircraft velocity
    M = 0.2 # mach number

    # ------------------------------------------------------------------------------------------------------------------------------------
    # Propulsor Setup
    # ------------------------------------------------------------------------------------------------------------------------------------
    turbofan = RCAIDE.Library.Components.Powertrain.Propulsors.Turbofan()
    turbofan.tag = 'starboard_propulsor'
    turbofan.bypass_ratio = 5.4
    turbofan.design_altitude = 35000.0 * Units.ft
    turbofan.design_mach_number = 0.78
    turbofan.design_thrust = 35000.0 * Units.N             

    fan = RCAIDE.Library.Components.Powertrain.Converters.Fan()
    fan.tag = 'fan'
    fan.polytropic_efficiency = 0.93
    fan.pressure_ratio = 1.7
    turbofan.fan = fan

    turbofan.working_fluid = RCAIDE.Library.Attributes.Gases.Air()
    ram = RCAIDE.Library.Components.Powertrain.Converters.Ram()
    ram.tag = 'ram'
    turbofan.ram = ram

    combustor = RCAIDE.Library.Components.Powertrain.Converters.Combustor()
    combustor.tag = 'combustor'
    combustor.number_of_fuel_nozzle = 18
    combustor.diameter = 0.6858
    turbofan.combustor = combustor

    core_nozzle = RCAIDE.Library.Components.Powertrain.Converters.Expansion_Nozzle()   
    core_nozzle.tag = 'core nozzle'
    core_nozzle.polytropic_efficiency = 0.98                    
    core_nozzle.pressure_ratio = 0.995 
    core_nozzle.diameter = 0.38118288
    turbofan.core_nozzle = core_nozzle
             
    fan_nozzle = RCAIDE.Library.Components.Powertrain.Converters.Expansion_Nozzle()   
    fan_nozzle.tag = 'fan nozzle'
    fan_nozzle.polytropic_efficiency = 0.98                    
    fan_nozzle.pressure_ratio = 0.995
    turbofan.fan_nozzle = fan_nozzle 

    m = None 

    segment = Segment()  
    conditions = Results()  
    conditions.aerodynamics.angles.alpha = alpha
    conditions.freestream.density = np.ones((ctrl_pts,1)) * density
    conditions.freestream.dynamic_viscosity = np.ones((ctrl_pts,1)) * dynamic_viscosity   
    conditions.freestream.speed_of_sound = np.ones((ctrl_pts,1)) * a 
    conditions.freestream.temperature = np.ones((ctrl_pts,1)) * T
    conditions.freestream.pressure = 97717 
    conditions.freestream.velocity = 85 
    conditions.frames.inertial.velocity_vector = np.array([[U, 0. ,0.]]) 
    conditions.freestream.mach_number = np.atleast_2d(np.linalg.norm(conditions.frames.inertial.velocity_vector,axis = 1)).T/ a
    conditions.frames.planet.true_course = np.zeros((ctrl_pts,3,3)) 
    conditions.frames.planet.true_course[:,2,2] = 1 
    conditions.frames.wind.transform_to_inertial = np.zeros((ctrl_pts,3,3))    
    conditions.frames.body.transform_to_inertial = np.zeros((ctrl_pts,3,3))
    conditions.frames.body.transform_to_inertial[:,0,0] = np.cos(AoA)
    conditions.frames.body.transform_to_inertial[:,0,2] = np.sin(AoA)
    conditions.frames.body.transform_to_inertial[:,1,1] = 1
    conditions.frames.body.transform_to_inertial[:,2,0] = -np.sin(AoA)
    conditions.frames.body.transform_to_inertial[:,2,2] = np.cos(AoA)     

    segment.state.conditions = conditions 
    turbofan.append_operating_conditions(segment, segment.state.conditions.energy, segment.state.conditions.aeroacoustics)
 
    segment.state.conditions.aeroacoustics.propulsors[turbofan.tag].fan.angular_velocity = 4200
    segment.state.conditions.aeroacoustics.propulsors[turbofan.tag].fan.exit_velocity = 350 * Units.mph
    segment.state.conditions.aeroacoustics.propulsors[turbofan.tag].fan.exit_stagnation_temperature = 440
    segment.state.conditions.aeroacoustics.propulsors[turbofan.tag].fan.exit_stagnation_pressure = 152*1000

    turbofan.origin = np.array([[0.0, 0.0, 1.5]])
    turbofan.length = 97 * Units.inches
    turbofan.diameter = 70 * Units.inches
    turbofan.plug_diameter = 60 * Units.inches
    turbofan.geometry_xe = 1.0
    turbofan.geometry_ye = 1.0
    turbofan.geometry_Ce = 1.0
    
    segment.state.conditions.aeroacoustics.propulsors[turbofan.tag].fan.number_of_blades = 22
    segment.state.conditions.aeroacoustics.propulsors[turbofan.tag].fan.diameter = 70 * Units.inches

    segment.state.conditions.aeroacoustics.propulsors[turbofan.tag].fan.static_temperature_output = T + (80/1.8)
    segment.state.conditions.aeroacoustics.propulsors[turbofan.tag].fan.static_temperature_input  = T

    segment.state.conditions.aeroacoustics.propulsors[turbofan.tag].fan_nozzle.exit_velocity = 280.0
    segment.state.conditions.aeroacoustics.propulsors[turbofan.tag].fan_nozzle.exit_stagnation_temperature = 340.0
    segment.state.conditions.aeroacoustics.propulsors[turbofan.tag].fan_nozzle.exit_stagnation_pressure = 2611.8

    segment.state.conditions.aeroacoustics.propulsors[turbofan.tag].core_nozzle.exit_velocity = 400.0
    segment.state.conditions.aeroacoustics.propulsors[turbofan.tag].core_nozzle.exit_stagnation_temperature = 800.0
    segment.state.conditions.aeroacoustics.propulsors[turbofan.tag].core_nozzle.exit_stagnation_pressure = 165000.0
        
    segment.state.conditions.energy.converters['combustor'].inputs.static_temperature = 622.7
    segment.state.conditions.energy.converters['combustor'].outputs.static_temperature = 1000
    segment.state.conditions.expand_rows(ctrl_pts)  

    # -------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
    # Read Path Data and Receptor Data
    # -------------------------------------------------------------------------------------------------------------------------------------------------------------------------------  
    df = pd.read_csv('/Users/siripunn/Desktop/LEADS_WORK/LEADS_Research/RCAIDE_LEADS/VnV/Verification/analysis_aeroacoustics/b737_sim_noise_SEL_N_TR (1).csv')
    track_df = pd.read_csv('/Users/siripunn/Desktop/LEADS_WORK/LEADS_Research/RCAIDE_LEADS/VnV/Verification/analysis_aeroacoustics/b737_sim_track_interpolated.csv')

    df = downsample_spatial_grid(df, stride_factor=2)
    
    # -------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
    # Clean Local Tangent Plane (ENU) Coordinate Projection 
    # -------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
    # Anchor origin to the first track point to eliminate global lat/lon distortion
    lat0 = track_df['Latitude (deg)'].iloc[0]
    lon0 = track_df['Longitude (deg)'].iloc[0]
    
    deg_to_rad = np.radians(1.0)
    m_per_deg_lat = 111132.92 - 559.82 * np.cos(2 * lat0 * deg_to_rad) + 1.17 * np.cos(4 * lat0 * deg_to_rad)
    m_per_deg_lon = 111412.84 * np.cos(lat0 * deg_to_rad) - 93.5 * np.cos(3 * lat0 * deg_to_rad)

    def latlon_to_cartesian(lats, lons):
        """Projects lat/lon/elev directly to local East-North-Up Cartesian coordinates (meters)."""
        x_east = (lons - lon0) * m_per_deg_lon
        y_north = (lats - lat0) * m_per_deg_lat
        return x_east, y_north

    # Project all receptors into clean local meters once
    rec_x, rec_y = latlon_to_cartesian(df['Latitude (deg)'].values, df['Longitude (deg)'].values)
    rec_elevs_m = df['Elevation MSL (ft)'].values * 0.3048
    num_receptors = len(df)

    # Project track path into local meters
    track_x, track_y = latlon_to_cartesian(track_df['Latitude (deg)'].values, track_df['Longitude (deg)'].values)
    track_z = track_df['Altitude MSL (ft)'].values * 0.3048

    # -------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
    # Run Simulation Loop
    # ------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
    positionsx = []
    positionsy = []
    sim_results = []
    dt_array = [] 
    U_flight = 72.0 # m/s 

    altflight = []
    tr = []
    
    max_steps = min(48, len(track_df))
    for index in range(1, max_steps):
        # Current and previous track points in local Cartesian frame
        x1, y1, z1 = track_x[index-1], track_y[index-1], track_z[index-1]
        x2, y2, z2 = track_x[index], track_y[index], track_z[index]
        
        trp = track_df.iloc[index]['Noise Thrust per Engine (lbs)']

        if index > 10:
            if index > 17:
                positionsx.append(track_df['Latitude (deg)'].iloc[index])
                positionsy.append(track_df['Longitude (deg)'].iloc[index])
                tr.append(round(trp))

            # Segment vector components (dx, dy, dz)
            dx_flight = x2 - x1
            dy_flight = y2 - y1
            dz_flight = z2 - z1
            mag_flight = np.sqrt(dx_flight**2 + dy_flight**2 + dz_flight**2)
            
            if mag_flight == 0:
                continue

            # Unit heading vector of the segment
            hx, hy, hz = dx_flight / mag_flight, dy_flight / mag_flight, dz_flight / mag_flight
            
            dt = mag_flight / U_flight
            dt_array.append(dt)

            # Vector from segment start (x1, y1, z1) to all receptors
            dx_obs = rec_x - x1
            dy_obs = rec_y - y1
            
            # Projection length along segment (d_AS)
            d_AS = (dx_obs * hx) + (dy_obs * hy) + (dz_flight * 0.0) # Flat Earth approximation for ground projection
            d_AS = np.clip(d_AS, 0, mag_flight)
            
            # Closest Point of Approach (CPA) coordinates on the segment
            cpa_x = x1 + d_AS * hx
            cpa_y = y1 + d_AS * hy
            cpa_z = z1 + d_AS * hz
            
            # Horizontal Sideline Distance (l_seg) from receptor to CPA
            l_seg_m = np.sqrt((rec_x - cpa_x)**2 + (rec_y - cpa_y)**2)
            
            # AGL Altitude at CPA (d_seg_m)
            d_seg_m = cpa_z - rec_elevs_m
            d_seg_m = np.maximum(d_seg_m, 0.3) # Floor limit (~1 ft)

            # 3D Slant Range from receptor to aircraft position/CPA
            los_distance = np.sqrt((rec_x - x2)**2 + (rec_y - y2)**2 + (rec_elevs_m - z2)**2)
            los_distance = np.maximum(los_distance, 0.3)

            # Vector from Aircraft to Receptors for emission angle calculations
            dx_ac = rec_x - x2
            dy_ac = rec_y - y2
            dz_ac = rec_elevs_m - z2

            dot_prod = (dx_ac * hx) + (dy_ac * hy) + (dz_ac * hz)
            cos_theta = np.clip(dot_prod / los_distance, -1.0, 1.0)
            theta_proper = np.arccos(cos_theta)

            total_SPL_map = np.zeros(num_receptors)
            LADJ_dB, beta_deg = AEDT_LADJ_Attenuation(l_seg_m, d_seg_m, bank_angle_deg=0.0)

            # Receptor iteration loop
            for i in range(num_receptors):
                R_val = np.array([[los_distance[i]]])
                theta_raw = np.array([[theta_proper[i]]])
                theta_flap = np.array([[theta_proper[i]]])
                theta_engine = np.array([[theta_proper[i]]])
                
                # --- RUN NOISE MODELS ---
                lg_noise = compute_landing_gear_noise(R_val, theta_raw, D, H, W, wheels, M, Weight, strut_diameter, frequency, segment)
                flap_noise = flap_noise_model(R_val, theta_flap, cf, thickness, deltaf, frequency, segment)
                slat_noise_val = slat_noise(R_val, phi, theta_flap[0][0], Ls, gamma_s, sigma_s, alpha, segment, frequency, A=1e-5)

                aero_data = segment.state.conditions.aeroacoustics.propulsors[turbofan.tag]
                
                mic_x = los_distance[i] * np.cos(theta_proper[i])
                mic_y = los_distance[i] * np.sin(theta_proper[i])
                mic_locations = np.array([[mic_x, mic_y, 0.0]])

                current_thrust = track_df.iloc[index]['Airplane Thrust Type']
                if current_thrust == 'Reversed Thrust' or current_thrust == 'Idle Approach':
                    jet_noise = compute_jet_noise_new(mic_locations, turbofan, aero_data, segment.state, frequency, 1)
                else:
                    jet_noise = compute_jet_noise_new(mic_locations, turbofan, aero_data, segment.state, frequency, 0)
                
                fan_noise = compute_fan_noise(R_val[0], theta_engine[0][0], turbofan, m, segment.state.conditions.aeroacoustics, segment, frequency)
                core_noise = compute_core_noise(R_val, theta_engine, turbofan, pr, segment.state.conditions.aeroacoustics, segment, frequency)

                jet_spec_raw = jet_noise.SPL_1_3_spectrum[0][0]
                
                spectra = np.array([
                    lg_noise.Total[0], 
                    flap_noise[0], 
                    slat_noise_val[0],
                    fan_noise.SPL_1_3_spectrum[0][0], 
                    core_noise.SPL_1_3_spectrum[0][0],
                    jet_spec_raw
                ])
                total_spectrum = SPL_arithmetic(spectra, sum_axis=0)
                
                dist_array = np.array([los_distance[i]])
                att_dB = atmospheric_attenuation(dist_array, frequency)[0]
                
                attenuated_spectrum = total_spectrum - (att_dB * 1.5) - LADJ_dB[i]
                a_weighted_spectrum = A_weighting_metric(attenuated_spectrum, frequency)

                oaspl = 10 * np.log10(np.sum(10 ** (a_weighted_spectrum / 10)))
                total_SPL_map[i] = oaspl

            sim_results.append(total_SPL_map)

        if len(sim_results) == 28 + 8:
            import matplotlib.tri as tri
            sim_results_arr = np.array(sim_results)
            dt_arr = np.array(dt_array).reshape(-1, 1) 
            
            energy_integral = np.sum((10**(sim_results_arr / 10.0)) * dt_arr, axis=0)
            z = 10 * np.log10(energy_integral)
            
            x = df['Longitude (deg)'].values
            y = df['Latitude (deg)'].values

            np.savez('b737_high_res_footprint_3.npz', longitude=x, latitude=y, sel_dBA=z)
            
            fig, ax = plt.subplots(figsize=(10, 8), dpi=120)
            triangulation = tri.Triangulation(x, y)
            
            heatmap = ax.tricontourf(triangulation, z, levels=40, cmap='jet', extend='both')
            plt.plot(positionsy, positionsx, 'ko', markersize=1)
            for idx, (px, py) in enumerate(zip(positionsy, positionsx), start=0):
                plt.annotate(f"{tr[idx]}", (px, py), textcoords="offset points", xytext=(1, 1), fontsize=3)

            cbar = fig.colorbar(heatmap, ax=ax)
            cbar.set_label('Level (Exposure) - SEL', fontsize=12, fontweight='bold')
        
            ax.set_title('B737 Simulated Noise Footprint', fontsize=14, fontweight='bold', pad=15)
            ax.set_xlabel('Longitude', fontsize=12)
            ax.set_ylabel('Latitude', fontsize=12)
            ax.grid(True, linestyle='--', alpha=0.5, color='gray')
            
            mean_lat = np.mean(y)
            ax.set_aspect(1.0 / np.cos(np.radians(mean_lat)))
            plt.show()
            
def plot_parameters():
    plt.rcParams.update({'font.size': 12})
    plt.rcParams['axes.linewidth'] = 1. 
    return Data(fig_size_width=14, fig_size_height=9)  

def AEDT_LADJ_Attenuation(l_seg_m, d_seg_m, bank_angle_deg=0.0):
    """Computes the AEDT Lateral Attenuation Adjustment (LA_ADJ) per SAE-AIR-5662."""
    SLR_seg = np.sqrt(d_seg_m**2 + l_seg_m**2)
    SLR_seg = np.maximum(SLR_seg, 1e-6)
    
    beta_rad = np.arcsin(d_seg_m / SLR_seg)
    beta_deg = np.degrees(beta_rad)
    
    phi_deg = bank_angle_deg + beta_deg
    phi_rad = np.radians(phi_deg)
    
    E_WING = np.zeros_like(phi_deg)
    mask_pos = (phi_deg >= 0.0) & (phi_deg <= 180.0)
    mask_neg = (phi_deg < 0.0) & (phi_deg >= -180.0)
    
    cos2_phi = np.cos(phi_rad[mask_pos])**2
    sin2_phi = np.sin(phi_rad[mask_pos])**2
    sin2_2phi = np.sin(2.0 * phi_rad[mask_pos])**2
    cos2_2phi = np.cos(2.0 * phi_rad[mask_pos])**2
    
    num = (0.0039 * cos2_phi + sin2_phi)**0.062
    den = (0.8786 * sin2_2phi + cos2_2phi)
    E_WING[mask_pos] = 10.0 * np.log10(num / den)
    E_WING[mask_neg] = -1.49
    
    G = np.full_like(l_seg_m, 10.86) 
    mask_G = (l_seg_m >= 0.0) & (l_seg_m <= 914.0)
    G[mask_G] = 11.83 * (1.0 - np.exp(-0.00274 * l_seg_m[mask_G]))
    
    Lambda = np.zeros_like(beta_deg)
    beta_eff = np.maximum(beta_deg, 0.0)
    mask_L = (beta_eff >= 0.0) & (beta_eff <= 50.0)
    Lambda[mask_L] = 1.137 - (0.0229 * beta_eff[mask_L]) + (9.72 * np.exp(-0.142 * beta_eff[mask_L]))
    
    LA_ADJ = -(E_WING - ((G * Lambda) / 10.86))
    return LA_ADJ, beta_deg

def downsample_spatial_grid(df, stride_factor):
    if 'Latitude' in df.columns and 'Longitude' in df.columns:
        unique_lats = np.sort(df['Latitude'].unique())
        unique_lons = np.sort(df['Longitude'].unique())
        ds_lats = unique_lats[::stride_factor]
        ds_lons = unique_lons[::stride_factor]
        return df[df['Latitude'].isin(ds_lats) & df['Longitude'].isin(ds_lons)].reset_index(drop=True)
    else:
        return df.iloc[::stride_factor, :].reset_index(drop=True)
 
if __name__ == '__main__': 
    main()  
    plt.show()