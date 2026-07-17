# RCAIDE/Methods/Aeroacoustics/Common/generate_microphone_locations.py
# 
# 
# Created:  Oct 2023, A. Molloy  

# ----------------------------------------------------------------------------------------------------------------------
#  IMPORT
# ----------------------------------------------------------------------------------------------------------------------  
# package imports  
import numpy as np 

# ---------------------------------------------------------------------------------------------------------------------- 
#  generate_zero_elevation_microphone_locations
# ---------------------------------------------------------------------------------------------------------------------- 
def generate_zero_elevation_microphone_locations(settings):
    """This computes the absolute microphone/observer locations on level ground. 
            
    Assumptions:
        None

    Source:
        N/A  

    Inputs:   
    settings.
        min_x - minimum x coordinate of noise evaluation plane [meters]
        max_x - maximum x coordinate of noise evaluation plane [meters]
        min_y - minimum y coordinate of noise evaluation plane [meters]
        max_x - maximim y coordinate of noise evaluation plane [meters]
        N_x   - number of microphones on x-axis 
        N_y   - number of microphones on y-axis 
    
    Outputs: 
        gm_mic_locations   - cartesian coordiates of all microphones defined  [meters] 
    
    Properties Used:
        N/A       
    """       

    N_x                   = settings.microphone_x_resolution 
    N_y                   = settings.microphone_y_resolution
    num_gm                = N_x*N_y
    gm_mic_locations      = np.zeros((num_gm,3))     
    min_x                 = settings.microphone_min_x         
    max_x                 = settings.microphone_max_x         
    min_y                 = settings.microphone_min_y         
    max_y                 = settings.microphone_max_y   
    x_coords_0            = np.repeat(np.linspace(min_x,max_x,N_x)[:,np.newaxis],N_y, axis = 1)
    y_coords_0            = np.repeat(np.linspace(min_y,max_y,N_y)[:,np.newaxis],N_x, axis = 1).T
    z_coords_0            = np.zeros_like(x_coords_0) 
    gm_mic_locations[:,0] = x_coords_0.reshape(num_gm)
    gm_mic_locations[:,1] = y_coords_0.reshape(num_gm)
    gm_mic_locations[:,2] = z_coords_0.reshape(num_gm) 
     
    return gm_mic_locations 