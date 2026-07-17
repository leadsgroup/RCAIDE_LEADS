# RCAIDE/Methods/Aeroacoustics/Common/compute_relative_noise_evaluation_locations.py
# 
# 
# Created:  Jul 2023, M. Clarke  

# ----------------------------------------------------------------------------------------------------------------------
#  IMPORT
# ---------------------------------------------------------------------------------------------------------------------- 

# Python package imports   
import numpy as np  
    
# ----------------------------------------------------------------------------------------------------------------------  
#  Relative Noise Evaluatation Locations
# ----------------------------------------------------------------------------------------------------------------------      
def compute_relative_noise_evaluation_locations(settings,microphone_locations, segment):
    """This computes the relative locations on the surface in the computational domain where the 
    propogated sound is computed. Vectors point from observer/microphone to aircraft/source  
            
    Assumptions: 
        Acoustic scattering is not modeled

    Source:
        N/A  

    Inputs:  
        settings.microphone_locations                - array of microphone locations on the ground  [meters] 
        segment.conditions.frames.inertial.position_vector  - position of aircraft                         [boolean]                                          

    Outputs: 
    GM_THETA   - angle measured from ground microphone in the x-z plane from microphone to aircraft 
    GM_PHI     - angle measured from ground microphone in the y-z plane from microphone to aircraft 
    RML        - relative microphone locations  
    num_gm_mic - number of ground microphones
 
    Properties Used:
        N/A       
    """       
  
    MSL_altitude      = settings.mean_sea_level_altitude
    N                 = settings.noise_times_steps
    pos               = segment.state.conditions.frames.inertial.position_vector
    
    # rediscretize time and aircraft position to get finer resolution 
    time              = segment.state.conditions.frames.inertial.time[:,0]
    noise_time        = np.linspace(time[0], time[-1], N) 
    noise_pos         = np.zeros((N,3)) 
    noise_pos[:,0]    = np.interp(noise_time,time,pos[:,0])
    noise_pos[:,1]    = np.interp(noise_time,time,pos[:,1])
    noise_pos[:,2]    = np.interp(noise_time,time,pos[:,2])
    
    num_gm_mic        = len(microphone_locations)  
    RML               = np.zeros((N,num_gm_mic,3)) 
    PHI               = np.zeros((N,num_gm_mic))
    THETA             = np.zeros((N,num_gm_mic)) 
    
    for cpt in range(N):  
        relative_locations         = np.zeros((num_gm_mic,3))
        relative_locations[:,0]    = microphone_locations[:,0] - (settings.aircraft_origin_location[0] + noise_pos[cpt,0])    
        relative_locations[:,1]    = microphone_locations[:,1] - (settings.aircraft_origin_location[1] + noise_pos[cpt,1]) 
        if MSL_altitude:
            relative_locations[:,2]    = -(noise_pos[cpt,2])  - microphone_locations[:,2] 
        else:
            relative_locations[:,2]    = -(noise_pos[cpt,2])
            
        RML[cpt,:,:]   = relative_locations 
        PHI[cpt,:]     =  np.arctan2(np.sqrt(np.square(relative_locations[:, 0]) + np.square(relative_locations[:, 1])),  relative_locations[:, 2])  
        THETA[cpt,:]   =  np.arctan2(relative_locations[:, 1], relative_locations[:, 0]) 
    
    return noise_time,noise_pos,RML,PHI,THETA,num_gm_mic 
 