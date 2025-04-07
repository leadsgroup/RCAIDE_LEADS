# RCAIDE/Framework/Analyses/Noise/Frequency_Domain_Buildup.py
# 
# Mod:  Oct 2024, A. Molloy
# Created:  Jul 2023, M. Clarke

# ----------------------------------------------------------------------------------------------------------------------
#  IMPORT
# ----------------------------------------------------------------------------------------------------------------------  
# noise imports
from RCAIDE.Framework.Core import  Data
from RCAIDE.Library.Methods.Noise.Common.background_noise     import background_noise
from RCAIDE.Library.Methods.Noise.Metrics import * 
from RCAIDE.Library.Methods.Noise.Common.generate_zero_elevation_microphone_locations import generate_zero_elevation_microphone_locations 
from RCAIDE.Library.Methods.Noise.Common.generate_terrain_microphone_locations        import generate_terrain_microphone_locations     
from RCAIDE.Library.Methods.Noise.Common.compute_relative_noise_evaluation_locations  import compute_relative_noise_evaluation_locations
from RCAIDE.Library.Methods.Geodesics.compute_point_to_point_geospacial_data          import compute_point_to_point_geospacial_data

# package imports
import numpy as np
from scipy.interpolate                                           import RegularGridInterpolator


# ----------------------------------------------------------------------------------------------------------------------
#  PLOTS
# ---------------------------------------------------------------------------------------------------------------------- 
def post_process_noise_data(results,
                            flight_times = np.array(['06:00:00','06:30:00','07:00:00','07:30:00',
                                                   '08:00:00','08:30:00','09:00:00','09:30:00',
                                                   '10:00:00','10:30:00','11:00:00','11:30:00',
                                                   '12:00:00','12:30:00','13:00:00','13:30:00',
                                                   '14:00:00','14:30:00','15:00:00']),
                            time_period = ['06:00:00','20:00:00'], 
                            evalaute_noise_metrics = True):
    """
    Processes raw noise simulation results into formatted data for visualization.

    Parameters
    ----------
    results : Results
        RCAIDE results data structure containing:
            - segments[i].analyses.noise.settings
                Noise analysis settings including:
                    - number_of_microphone_in_stencil
                    - microphone_x_resolution
                    - microphone_y_resolution
                    - topography_file
                    - noise_times_steps
            - segments[i].state.conditions
                Flight conditions including:
                    - frames.inertial.time
                    - noise.hemisphere_SPL_dBA
                
    flight_times : ndarray of str, optional
        Array of time strings for noise evaluation (default: hourly from 06:00 to 15:00)
        
    time_period : list of str, optional
        Start and end times for analysis period (default: ['06:00:00','20:00:00'])
        
    evalaute_noise_metrics : bool, optional
        Flag to compute additional noise metrics (default: True)

    Returns
    -------
    noise_data : Data
        Processed noise data structure containing:
            - SPL_dBA : ndarray
                Sound pressure levels at each grid point
            - time : ndarray
                Time history array
            - aircraft_position : ndarray
                Aircraft trajectory points
            - microphone_locations : ndarray
                Measurement grid coordinates
            - topography_file : str, optional
                Path to terrain data if used
            - microphone_coordinates : ndarray, optional
                Geographic coordinates if terrain used

    Notes
    -----
    Processing steps include:
        1. Unpacking analysis settings
        2. Generating microphone grid
        3. Interpolating noise levels
        4. Computing metrics
        5. Formatting for visualization
    
    **Major Assumptions**
        * Regular measurement grid spacing
        * Continuous noise between segments
        * Background noise floor exists
        * Spherical spreading of sound
    
    **Definitions**
    
    'SPL_dBA'
        A-weighted Sound Pressure Level
    'Hemisphere'
        Directivity pattern around source
    'Stencil'
        Local group of evaluation points
    
    See Also
    --------
    RCAIDE.Library.Plots.Noise.plot_noise_level : Visualization of processed data
    RCAIDE.Library.Methods.Noise.Common.background_noise : Background noise floor function
    """

    # Step 1: Unpack settings      
    settings   = results.segments[0].analyses.noise.settings
    n          = settings.number_of_microphone_in_stencil
    N_gm_x     = settings.microphone_x_resolution
    N_gm_y     = settings.microphone_y_resolution    
    noise_data = Data()   
     
    # Step 2: Determing microhpone points where noise is to be computed
    microphone_coordinates =  None
    if settings.topography_file !=  None:
        compute_point_to_point_geospacial_data(settings)
        microphone_locations ,microphone_coordinates = generate_terrain_microphone_locations(settings) 
        noise_data.topography_file                    = settings.topography_file  
        noise_data.microphone_coordinates             = microphone_coordinates.reshape(N_gm_x,N_gm_y,3)     
        noise_data.aircraft_origin_coordinates        = settings.aircraft_origin_coordinates          
        noise_data.aircraft_destination_coordinates   = settings.aircraft_destination_coordinates         
    else:    
        microphone_locations =  generate_zero_elevation_microphone_locations(settings)   
    noise_data.microphone_y_resolution       = N_gm_y
    noise_data.microphone_x_resolution       = N_gm_x              
    noise_data.microphone_locations          = microphone_locations.reshape(N_gm_x,N_gm_y,3)         
    
    # Step 3: Create empty arrays to store noise data 
    N_segs = len(results.segments)
    num_gm_mic      = len(microphone_locations)
    num_noise_time  = settings.noise_times_steps
    num_f           = len(settings.center_frequencies)
    
    # Step 4: Initalize Arrays 
    N_ctrl_pts            = ( N_segs-1) * (num_noise_time -1) + num_noise_time # ensures that noise is computed continuously across segments 
    SPL_dBA               = np.ones((N_ctrl_pts,N_gm_x,N_gm_y))*background_noise()  
    SPL_dBA_1_3_spectrum  = np.ones((N_ctrl_pts,N_gm_x,N_gm_y,num_f))*background_noise()  
    Aircraft_pos          = np.empty((0,3))
    Time                  = np.empty((0))
    mic_locs              = np.zeros((N_ctrl_pts,n))   
 
    idx =  0
    
    # Step 5: loop through segments and store noise 
    for seg in range(N_segs):  
        segment    = results.segments[seg]
        settings   = segment.analyses.noise.settings  
        phi        = settings.noise_hemisphere_phi_angles
        theta      = settings.noise_hemisphere_theta_angles
        conditions = segment.state.conditions  
        time       = conditions.frames.inertial.time[:,0]
        
        # Step 5.1 : Compute relative microhpone locations 
        noise_time,noise_pos,RML,PHI,THETA,num_gm_mic  = compute_relative_noise_evaluation_locations(settings, microphone_locations,segment) 
         
        # Step 5.2: Compute aircraft position and npose at interpolated hemisphere locations
        cpt   = 0 
        if seg == (N_segs - 1):
            noise_time_ = noise_time 
        else:
            noise_time_ = noise_time[:-1]
             
        Aircraft_pos = np.vstack((Aircraft_pos,noise_pos))
        Time         = np.hstack((Time,noise_time_))
        
        for i in range(len(noise_time_)):
            # Step 5.2.1 :Noise interpolation 
            delta_t         = (noise_time[i] -time[cpt]) / (time[cpt+1] - time[cpt])
            SPL_lower       = conditions.noise.hemisphere_SPL_dBA[cpt].reshape(len(phi),len(theta))
            SPL_uppper      = conditions.noise.hemisphere_SPL_dBA[cpt+1].reshape(len(phi),len(theta))
            SPL_gradient    = SPL_uppper -  SPL_lower
            SPL_interp      = SPL_lower + SPL_gradient *delta_t
            

            SPL_lower_1_3_spectrum       = conditions.noise.hemisphere_SPL_1_3_spectrum_dBA[cpt].reshape(len(phi),len(theta),num_f)
            SPL_uppper_1_3_spectrum      = conditions.noise.hemisphere_SPL_1_3_spectrum_dBA[cpt+1].reshape(len(phi),len(theta),num_f)
            SPL_gradient_1_3_spectrum    = SPL_uppper_1_3_spectrum -  SPL_lower_1_3_spectrum
            SPL_interp_1_3_spectrum      = SPL_lower_1_3_spectrum + SPL_gradient_1_3_spectrum *delta_t
            
     
            #  Step 5.2.2 Create surrogate   
            SPL_dBA_surrogate              = RegularGridInterpolator((phi, theta),SPL_interp  ,method = 'linear',   bounds_error=False, fill_value=None) 
            SPL_dBA_1_3_spectrum_surrogate = RegularGridInterpolator((phi, theta),SPL_interp_1_3_spectrum  ,method = 'linear',   bounds_error=False, fill_value=None)       
            
            #  Step 5.2.3 Query surrogate
            R                              = np.linalg.norm(RML[i], axis=1) 
            locs                           = np.argsort(R)[:n]
            pts                            = (PHI[i][locs],THETA[i][locs]) 
            SPL_dBA_unscaled               = SPL_dBA_surrogate(pts) 
            SPL_dBA_1_3_spectrum_unscaled  = SPL_dBA_1_3_spectrum_surrogate(pts) 
            
            #  Step 5.2.4 Scale data using radius  
            R_ref                          = settings.noise_hemisphere_radius  
            SPL_dBA_scaled                 = SPL_dBA_unscaled - 20*np.log10(R[locs]/R_ref)
            SPL_dBA_1_3_spectrum_scaled    = SPL_dBA_1_3_spectrum_unscaled -  np.tile(20*np.log10(R[locs]/R_ref)[:, None], (1, num_f))
            
            # insert noise incorrect mic locations 
            SPL_dBA_temp         = SPL_dBA[idx].flatten()
            SPL_dBA_temp[locs]   = SPL_dBA_scaled
            SPL_dBA[idx]         = SPL_dBA_temp.reshape(N_gm_x,N_gm_y) 

            SPL_dBA_1_3_spectrum_temp         = SPL_dBA_1_3_spectrum[idx].reshape(N_gm_x*N_gm_y,num_f)
            SPL_dBA_1_3_spectrum_temp[locs]   = SPL_dBA_1_3_spectrum_scaled
            SPL_dBA_1_3_spectrum[idx]         = SPL_dBA_1_3_spectrum_temp.reshape(N_gm_x,N_gm_y,num_f) 

            mic_locs[idx]        = locs 
            idx += 1
            
            if noise_time[i] >= time[cpt+1]:
                cpt += 1             
                
    # Step 6: Make any readings less that background noise equal to background noise
    SPL_dBA                             = np.nan_to_num(SPL_dBA) 
    SPL_dBA[SPL_dBA<background_noise()] = background_noise() 
    SPL_dBA_1_3_spectrum                             = np.nan_to_num(SPL_dBA_1_3_spectrum) 
    SPL_dBA_1_3_spectrum[SPL_dBA_1_3_spectrum<background_noise()] = background_noise()
    
     
    # Step 7: Store data 
    noise_data.SPL_dBA               = SPL_dBA
    noise_data.SPL_dBA_1_3_spectrum  = SPL_dBA_1_3_spectrum
    noise_data.time                  = Time 
    noise_data.aircraft_position     = Aircraft_pos
    noise_data.microhpone_locations  = mic_locs
    
    # Step 8: Perform noise metric calculations
    if evalaute_noise_metrics:
        compute_noise_metrics(noise_data, flight_times)
    
    return noise_data