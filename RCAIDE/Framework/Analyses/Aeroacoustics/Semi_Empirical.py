# RCAIDE/Framework/Analyses/Aeroacoustics/Semi_Empirical.py
# 
# Created:  Jul 2023, M. Clarke
# Last Edited: Aug 14 2026, 11:51AM P. Siripun

# ----------------------------------------------------------------------------------------------------------------------
#  IMPORT
# ----------------------------------------------------------------------------------------------------------------------  
import RCAIDE
from RCAIDE.Library.Methods.Aeroacoustics.Common.decibel_arithmetic import SPL_arithmetic  
from RCAIDE.Library.Methods.Aeroacoustics.Common.generate_hemisphere_microphone_locations import generate_hemisphere_microphone_locations  
import RCAIDE.Framework.Analyses.Aeroacoustics.Aeroacoustics as Aeroacoustic_Module
from RCAIDE.Framework.Core import Data, Units
from RCAIDE.Framework.Mission.Segments.Segment import Segment
from RCAIDE.Framework.Mission.Segments.Evaluate import Results   
from RCAIDE.Library.Methods.Aeroacoustics.Semi_Empirical.Propulsion.Engine_Noise.vectorized_mixed_noise_components import compute_landing_gear_noise
from RCAIDE.Library.Methods.Aeroacoustics.Semi_Empirical.Propulsion.Engine_Noise.vectorized_mixed_noise_components import flap_noise_model
from RCAIDE.Library.Methods.Aeroacoustics.Semi_Empirical.Propulsion.Engine_Noise.vectorized_mixed_noise_components import slat_noise
from RCAIDE.Library.Methods.Aeroacoustics.Semi_Empirical.Propulsion.Engine_Noise.vectorized_mixed_noise_components import compute_fan_noise
from RCAIDE.Library.Methods.Aeroacoustics.Semi_Empirical.Propulsion.Engine_Noise.vectorized_mixed_noise_components import compute_core_noise
from RCAIDE.Library.Methods.Aeroacoustics.Semi_Empirical.Propulsion.Engine_Noise.compute_jet_noise_new import compute_jet_noise_new
from RCAIDE.Framework.Mission.Common                                              import Results  
from RCAIDE.Framework.Mission.Segments.Segment                                    import Segment  
from RCAIDE.Library.Methods.Aeroacoustics.Common   import SPL_arithmetic 
from RCAIDE.Library.Methods.Aeroacoustics.Metrics import A_weighting_metric
from RCAIDE.Library.Methods.Aeroacoustics.Common.atmospheric_attenuation import atmospheric_attenuation
from RCAIDE.Library.Plots import * 
 
# Python Imports  
import matplotlib.pyplot as plt 
import numpy as np     
import pandas as pd

import numpy as np
import pandas as pd
import matplotlib.pyplot as plt

# ----------------------------------------------------------------------------------------------------------------------
#  Helper Functions
# ---------------------------------------------------------------------------------------------------------------------- 
def interpolate_path(original_list):
    new_length = 100
    old_indices = np.arange(len(original_list))
    new_indices = np.linspace(0, len(original_list) - 1, new_length)
    interpolated_np = np.interp(new_indices, old_indices, original_list)
    return interpolated_np.tolist()

def calc_grid(num_x, num_y, geo_coords):
    lon_min, lat_min = geo_coords[0]
    lon_max, lat_max = geo_coords[1]
    lons = np.linspace(lon_min, lon_max, num_x)
    lats = np.linspace(lat_min, lat_max, num_y)
    lon_grid, lat_grid = np.meshgrid(lons, lats)
    return [lon_grid.flatten(), lat_grid.flatten()]

def calc_3d_dist_vectorized(ac_lat, ac_lon, ac_alt_ft, rec_lats, rec_lons, rec_elevs_ft):
    R_earth = 6371000.0  
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

def AEDT_LADJ_Attenuation(l_seg_m, d_seg_m, bank_angle_deg=0.0):
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
        df_sorted = df.sort_values(by=['Latitude', 'Longitude']).reset_index(drop=True)
        unique_lats = np.sort(df['Latitude'].unique())
        unique_lons = np.sort(df['Longitude'].unique())
        ds_lats = unique_lats[::stride_factor]
        ds_lons = unique_lons[::stride_factor]
        return df[df['Latitude'].isin(ds_lats) & df['Longitude'].isin(ds_lons)].reset_index(drop=True)
    else:
        return df.iloc[::stride_factor, :].reset_index(drop=True)

def plot_parameters():
    plt.rcParams.update({'font.size': 12})
    plt.rcParams['axes.linewidth'] = 1. 
    PP = Data(  
        fig_size_width  = 14 ,
        fig_size_height = 9 ,       
        lw  = 1,                                           
        m   = 5,                                           
        lf  = 10,                                          
        Slc = ['black','green','yellow'],          
        Slm = ['^','o','s'],                       
        Sls = '-',                                         
        Elc = ['darkred','red','tomato'],          
        Elm = ['s'],                               
        Els = '',                                          
        Rlc = ['darkblue','blue','cyan'],          
        Rlm = ['o'],                               
        Rls = ':',                                         
    )   
    return PP  

# ----------------------------------------------------------------------------------------------------------------------
#  Semi_Empirical
# ---------------------------------------------------------------------------------------------------------------------- 
class Semi_Empirical(Aeroacoustic_Module): 
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
    
    def __defaults__(self):
        self.tag                              = "Semi_Empirical"
        self.settings.noise_hemisphere_radius = 50
        return
            
    def evaluate_aeroacoustics(self, segment, vehicle):
        # unpack  
        settings      = self.settings     
        conditions    = segment.state.conditions  
        dim_cf        = len(settings.center_frequencies ) 
        ctrl_pts      = int(segment.state.numerics.number_of_control_points) 
         
        microphone_locations = generate_hemisphere_microphone_locations(settings)      
        N_hemisphere_mics    = len(microphone_locations)
        
        # create empty arrays for results      
        total_SPL_dBA        = np.ones((ctrl_pts,N_hemisphere_mics))*1E-16 
        total_SPL_spectra    = np.ones((ctrl_pts,N_hemisphere_mics,dim_cf))*1E-16

        # --- INTEGRATED NOISE CODE START ---
        # Note: adjust file path as necessary for the host machine
        df = pd.read_csv("/Users/siripunn/Desktop/LEADS_WORK/LEADS_Research/RCAIDE_LEADS/VnV/Verification/analysis_aeroacoustics/b737_sim_track_interpolated.csv")

        lat_array = interpolate_path(df['Latitude (deg)'].to_numpy())
        lon_array = interpolate_path(df['Longitude (deg)'].to_numpy())
        elevation_msl_array = interpolate_path(df['Altitude MSL (ft)'].to_numpy())
        ground_speed_kts_array = interpolate_path(df['Ground Speed (kts)'].to_numpy())
        
        ground_tr = []
        for i in elevation_msl_array:
            if i - 680 < 25:
                ground_tr.append('Reversed Thrust')
            else:
                ground_tr.append(None)
                
        path = Data(latitude=lat_array, longitude=lon_array, altitude_MSL_ft=elevation_msl_array, ground_speed_kts=ground_speed_kts_array, thrust_reverse=ground_tr)
        
        frequency_range = settings.center_frequencies
        frequency = frequency_range
        grid_location = [[-88.018902,41.894352],[-87.797397,42.059475]] 
        grid_dimensions = [120,120] 
        receptor_alt_MSL = 680 
        fast = True
        plot = True
        ctrl_pts                = 1


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

        # Propulsor setup
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

        # Overwrite segment conditions for simulation script compatibility
        sim_segment = Segment()  
        sim_conditions = Results()  
        sim_conditions.aerodynamics.angles.alpha = alpha
        sim_conditions.freestream.density = np.ones((ctrl_pts,1)) * density
        sim_conditions.freestream.dynamic_viscosity = np.ones((ctrl_pts,1)) * dynamic_viscosity   
        sim_conditions.freestream.speed_of_sound = np.ones((ctrl_pts,1)) * a 
        sim_conditions.freestream.temperature = np.ones((ctrl_pts,1)) * T
        sim_conditions.freestream.pressure = 97717 
        sim_conditions.freestream.velocity = 85 
        sim_conditions.frames.inertial.velocity_vector = np.array([[U, 0. ,0.]]) 
        sim_conditions.freestream.mach_number = np.atleast_2d(np.linalg.norm(sim_conditions.frames.inertial.velocity_vector, axis=1)).T / a
        sim_conditions.frames.planet.true_course = np.zeros((ctrl_pts,3,3)) 
        sim_conditions.frames.planet.true_course[:,2,2] = 1 
        sim_conditions.frames.wind.transform_to_inertial = np.zeros((ctrl_pts,3,3))    
        sim_conditions.frames.body.transform_to_inertial = np.zeros((ctrl_pts,3,3))
        sim_conditions.frames.body.transform_to_inertial[:,0,0] = np.cos(AoA)
        sim_conditions.frames.body.transform_to_inertial[:,0,2] = np.sin(AoA)
        sim_conditions.frames.body.transform_to_inertial[:,1,1] = 1
        sim_conditions.frames.body.transform_to_inertial[:,2,0] = -np.sin(AoA)
        sim_conditions.frames.body.transform_to_inertial[:,2,2] = np.cos(AoA)     
        sim_segment.state.conditions = sim_conditions 

        turbofan.append_operating_conditions(sim_segment, sim_segment.state.conditions.energy, sim_segment.state.conditions.aeroacoustics)
     
        sim_segment.state.conditions.aeroacoustics.propulsors[turbofan.tag].fan.angular_velocity = 4200 
        sim_segment.state.conditions.aeroacoustics.propulsors[turbofan.tag].fan.exit_velocity = 350 * Units.mph 
        sim_segment.state.conditions.aeroacoustics.propulsors[turbofan.tag].fan.exit_stagnation_temperature = 440 
        sim_segment.state.conditions.aeroacoustics.propulsors[turbofan.tag].fan.exit_stagnation_pressure = 152*1000 

        turbofan.origin = np.array([[0.0, 0.0, 1.5]]) 
        turbofan.length = 97*Units.inches
        turbofan.diameter = 70*Units.inches
        turbofan.plug_diameter = 60*Units.inches
        turbofan.geometry_xe = 1.0 
        turbofan.geometry_ye = 1.0
        turbofan.geometry_Ce = 1.0
        
        sim_segment.state.conditions.aeroacoustics.propulsors[turbofan.tag].fan.number_of_blades = 22
        sim_segment.state.conditions.aeroacoustics.propulsors[turbofan.tag].fan.diameter = 70*Units.inches
        sim_segment.state.conditions.aeroacoustics.propulsors[turbofan.tag].fan.static_temperature_output = T + (80/1.8)
        sim_segment.state.conditions.aeroacoustics.propulsors[turbofan.tag].fan.static_temperature_input  = T
        sim_segment.state.conditions.aeroacoustics.propulsors[turbofan.tag].fan_nozzle.exit_velocity = 280.0
        sim_segment.state.conditions.aeroacoustics.propulsors[turbofan.tag].fan_nozzle.exit_stagnation_temperature = 340.0
        sim_segment.state.conditions.aeroacoustics.propulsors[turbofan.tag].fan_nozzle.exit_stagnation_pressure = 2611.8
        sim_segment.state.conditions.aeroacoustics.propulsors[turbofan.tag].core_nozzle.exit_velocity = 400.0
        sim_segment.state.conditions.aeroacoustics.propulsors[turbofan.tag].core_nozzle.exit_stagnation_temperature = 800.0
        sim_segment.state.conditions.aeroacoustics.propulsors[turbofan.tag].core_nozzle.exit_stagnation_pressure = 165000.0
        sim_segment.state.conditions.energy.converters['combustor'].inputs.static_temperature = 622.7
        sim_segment.state.conditions.energy.converters['combustor'].outputs.static_temperature = 1000
        
        sim_segment.state.conditions.expand_rows(ctrl_pts)

        rec_grid = calc_grid(grid_dimensions[0], grid_dimensions[1], grid_location)
        rec_lats = rec_grid[1]
        rec_lons = rec_grid[0]
        rec_elevs = np.full(len(rec_lats), receptor_alt_MSL) 
        num_receptors = len(rec_lats)

        positionsx = []
        positionsy = []
        sim_results = []
        dt_array = [] 
        
        U_flight = [x * 0.5144 for x in path.ground_speed_kts] 
        settings_jet = Data() 
        settings_jet.center_frequencies = np.pad(frequency, (5, 0), mode='constant')
        altflight = []
        
        ac_lat = path.latitude 
        ac_lon = path.longitude 
        ac_alt = path.altitude_MSL_ft

        for index in range(0,len(ac_lat)):
            if grid_location[0][1] < ac_lat[index] < grid_location[1][1] and grid_location[0][0] < ac_lon[index] < grid_location[1][0]+0.05:
                if index > 22:
                    positionsx.append(ac_lat[index])
                    positionsy.append(ac_lon[index])

                ground_dist, los_distance, angle_to_ground = calc_3d_dist_vectorized(
                    ac_lat[index], ac_lon[index], ac_alt[index], rec_lats, rec_lons, rec_elevs
                )
                
                R_earth = 6371000.0
                if index > 0 and index < len(ac_alt):
                    dx_flight = np.radians(ac_lon[index] - ac_lon[index-1]) * R_earth * np.cos(np.radians(ac_lat[index]))
                    dy_flight = np.radians(ac_lat[index] - ac_lat[index-1]) * R_earth
                    dz_flight = (ac_alt[index] - ac_alt[index-1]) * Units.feet
                else:
                    if index + 1 < len(ac_alt):
                        dx_flight = np.radians(ac_lon[index+1] - ac_lon[index]) * R_earth * np.cos(np.radians(ac_lat[index]))
                        dy_flight = np.radians(ac_lat[index+1] - ac_lat[index]) * R_earth
                        dz_flight = (ac_alt[index+1] - ac_alt [index]) * Units.feet
                    else:
                        dx_flight, dy_flight, dz_flight = 1.0, 0.0, 0.0
                    
                mag_flight = np.sqrt(dx_flight**2 + dy_flight**2 + dz_flight**2)
                hx, hy, hz = (dx_flight/mag_flight, dy_flight/mag_flight, dz_flight/mag_flight) if mag_flight > 0 else (1.0, 0.0, 0.0)
                
                dt = mag_flight / U_flight[index]
                dt_array.append(dt)

                dx_obs = np.radians(rec_lons - ac_lon[index]) * R_earth * np.cos(np.radians(ac_lat[index]))
                dy_obs = np.radians(rec_lats - ac_lat[index]) * R_earth
                dz_obs = (-(rec_elevs - ac_alt[index]) * Units.feet)
                altflight.append(round(dz_obs[0]))
                l_seg_m = ground_dist

                d_AS = ((dx_obs * dx_flight) + (dy_obs * dy_flight)) / mag_flight
                d_AS = np.clip(d_AS, 0, mag_flight)
                
                d_seg_m = (ac_alt[index] * Units.feet) + d_AS * (dz_flight / mag_flight) - (rec_elevs * Units.feet)
                d_seg_m = np.maximum(d_seg_m, Units.feet) 

                dot_prod = (dx_obs * hx) + (dy_obs * hy) + (dz_obs * hz)
                cos_theta = np.clip(dot_prod / los_distance, -1.0, 1.0)
                
                theta_proper = np.arccos(cos_theta)
                tf = theta_proper
                te = theta_proper
                theta_raw_arr = theta_proper

                total_SPL_map = np.zeros(num_receptors)
                LADJ_dB, beta_deg = AEDT_LADJ_Attenuation(l_seg_m, d_seg_m, bank_angle_deg=0.0)
                
                for i in range(num_receptors):
                    R_val = np.array([[los_distance[i]]]) 
                    theta_raw = np.array([[theta_raw_arr[i]]])
                    theta_flap = np.array([[tf[i]]])
                    theta_engine = np.array([[te[i]]])
                    
                    aero_data = sim_segment.state.conditions.aeroacoustics.propulsors[turbofan.tag]
                    mic_x = los_distance[i] * np.cos(theta_proper[i])
                    mic_y = los_distance[i] * np.sin(theta_proper[i])
                    mic_locations = np.array([[mic_x, mic_y, 0.0]])

                    # NOTE: External acoustic computation methods (slat_noise, compute_jet_noise_new, etc.)
                    # must be properly imported or defined within the RCAIDE environment to execute here.
                    # Currently represented conceptually per the provided raw script logic.

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

                sim_results.append(total_SPL_map)

        if len(sim_results) > 0:
            sim_results_arr = np.array(sim_results)
            dt_arr = np.array(dt_array).reshape(-1, 1)        
            energy_integral = np.sum((10**(sim_results_arr / 10.0)) * dt_arr, axis=0)
            z = 10 * np.log10(energy_integral)

            if plot:
                import matplotlib.tri as tri
                fig, ax = plt.subplots(figsize=(10, 8), dpi=120)
                triangulation = tri.Triangulation(rec_lons, rec_lats)
                levels = np.linspace(42, 120, 40)
                heatmap = ax.tricontourf(triangulation, z, levels=levels, cmap='jet', extend='both')
                plt.plot(positionsy, positionsx, 'ko', markersize=1)
                
                cbar = fig.colorbar(heatmap, ax=ax)
                cbar.set_label('Level (Exposure) - SEL', fontsize=12, fontweight='bold')
                ax.set_title('B737 Simulated Noise Footprint', fontsize=14, fontweight='bold', pad=15)
                ax.set_xlabel('Longitude', fontsize=12)
                ax.set_ylabel('Latitude', fontsize=12)
                ax.grid(True, linestyle='--', alpha=0.5, color='gray')
                mean_lat = np.mean(rec_lats)
                ax.set_aspect(1.0 / np.cos(np.radians(mean_lat)))
                plt.show()

        # Wrap up default RCAIDE conditions assignment
        conditions.aeroacoustics.hemisphere_SPL_dBA = total_SPL_dBA * (1 - settings.noise_reduction_factors.SPL_dbA)
        conditions.aeroacoustics.hemisphere_SPL_1_3_spectrum_dBA = total_SPL_spectra * (1 - settings.noise_reduction_factors.SPL_dbA)                                                    
        return