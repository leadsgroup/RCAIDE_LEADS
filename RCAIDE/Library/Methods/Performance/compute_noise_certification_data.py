# RCAIDE/Library/Methods/Performance/compute_noise_certification_data.py
# 
# 
# Created:  Jul 2023, M. Clarke

# ----------------------------------------------------------------------------------------------------------------------
#  IMPORT
# ----------------------------------------------------------------------------------------------------------------------

# RCAIDE imports 
from RCAIDE.Framework.Core import   Data    
from RCAIDE.Library.Methods.Noise.Common import post_process_noise_data
 
# Pacakge imports 
import numpy as np 
 
# ----------------------------------------------------------------------
#  Compute Aircraft Noise Certification Data  
# ----------------------------------------------------------------------  
def compute_noise_certification_data(approach_mission  = None, takeoff_mission   = None):
    """Calculates the noise at certification points as well as the noise contours of approach and takeoff.
    A combined approach-takeoff noisec contour is also created 
    """ 
            
    if approach_mission == None:
        raise AssertionError('Approach mission not specifed!')
    if takeoff_mission == None:
        raise AssertionError('Takeoff mission not specifed!')
     
    microphone_x_resolution                = 401 
    microphone_y_resolution                = 9  
    noise_times_steps                      = 51 
    number_of_microphone_in_stencil        = 1800
    
    # update weights analysis
    for segment in approach_mission.segments:
        if segment.analyses.noise == None:
            raise AssertionError('Noise analysis not specifed!')
        noise_analysis = segment.analyses.noise
        noise_analysis.settings.microphone_x_resolution                = microphone_x_resolution
        noise_analysis.settings.microphone_y_resolution                = microphone_y_resolution
        noise_analysis.settings.noise_times_steps                      = noise_times_steps
        noise_analysis.settings.number_of_microphone_in_stencil        = number_of_microphone_in_stencil
        noise_analysis.settings.microphone_min_y                       = 1E-6   
        noise_analysis.settings.microphone_max_y                       = 1800
        noise_analysis.settings.microphone_min_x                       = 1E-6   
        noise_analysis.settings.microphone_max_x                       = 8000
    
    # update weights analysis
    for segment in takeoff_mission.segments:
        if segment.analyses.noise == None:
            raise AssertionError('Noise analysis not specifed!')
        noise_analysis = segment.analyses.noise 
        noise_analysis.settings.microphone_x_resolution                = microphone_x_resolution
        noise_analysis.settings.microphone_y_resolution                = microphone_y_resolution
        noise_analysis.settings.noise_times_steps                      = noise_times_steps
        noise_analysis.settings.number_of_microphone_in_stencil        = number_of_microphone_in_stencil
        noise_analysis.settings.microphone_min_y                       = 1E-6   
        noise_analysis.settings.microphone_max_y                       = 1800
        noise_analysis.settings.microphone_min_x                       = -2000 + 1E-6  
        noise_analysis.settings.microphone_max_x                       = 6000
    
    # evaluate both missions
    approach_results = approach_mission.evaluate() 
    takeoff_results  = takeoff_mission.evaluate() 
    
    # post process noise data 
    approach_noise_data   = post_process_noise_data(approach_results, compute_EPNL=True)
    takeoff_noise_data    = post_process_noise_data(takeoff_results, compute_EPNL=True) 
    
    # append approach noise                                
    approach_pos         = approach_noise_data.aircraft_position
    approach_pos[:,0]   -= 2000 
    
    # append takeoff noise  
    cert_SPL_dBA_max  = np.max(np.concatenate((approach_noise_data.SPL_dBA,takeoff_noise_data.SPL_dBA), axis = 0) ,axis = 0)     
    cert_EPNL_max     = np.max(np.concatenate((approach_noise_data.EPNL[None,:, :],takeoff_noise_data.EPNL[None,:, :]), axis = 0) ,axis = 0)                 
    cert_pos          = np.concatenate((approach_pos, takeoff_noise_data.aircraft_position), axis = 0)     
    cert_mic_locs     = takeoff_noise_data.microphone_locations 
     
    noise_data = Data(
        certification_SPL_dBA_max         = cert_SPL_dBA_max, 
        certification_trajectory          = cert_pos,     
        certification_microphone_locations= cert_mic_locs,

        approach_SPL_dBA_max              = np.max(approach_noise_data.SPL_dBA,axis = 0),
        approach_SPL_dBA                  = approach_noise_data.SPL_dBA, 
        approach_time                     = approach_noise_data.time,   
        approach_trajectory               = approach_pos,     
        approach_microphone_locations     = takeoff_noise_data.microphone_locations, 

        takeoff_SPL_dBA_max               = np.max(takeoff_noise_data.SPL_dBA,axis = 0),
        takeoff_SPL_dBA                   = takeoff_noise_data.SPL_dBA, 
        takeoff_time                      = takeoff_noise_data.time,   
        takeoff_trajectory                = takeoff_noise_data.aircraft_position,     
        takeoff_microphone_locations      = takeoff_noise_data.microphone_locations     
    
    )
    
    print('Certification Noise')
    print('-----------------------------------------------')
    print('2000 m  Approach Noise   :', round(cert_EPNL_max[0, 0], 2)) 
    print('6000 m  Flyover Noise    :', round(cert_EPNL_max[-1, 0], 2))
    print('450  m  Sideline Noise   :', round(max(cert_EPNL_max[:, 2]), 2)) 
    
    noise_data.approach_noise_2000m = cert_EPNL_max[0, 0] 
    noise_data.flyover_noise_6000m  = cert_EPNL_max[-1, 0] 
    noise_data.sideline_noise_450m  = max(cert_EPNL_max[:, 2])
    
    return noise_data

