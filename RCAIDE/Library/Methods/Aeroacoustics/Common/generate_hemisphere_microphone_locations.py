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
#  generate_hemisphere_microphone_locations
# ---------------------------------------------------------------------------------------------------------------------- 
def generate_hemisphere_microphone_locations(settings): 
    """This computes the microphones locations in a noise hemisphere
            
    Assumptions:
        None

    Source:
        N/A  

    Inputs:   
    settings.
    
        r     - noise hemisphere radius                   [meters]
        n     - noise hemisphere microphone resolution    [unitless]
        phi   - noise hemisphere phi angle bounds         [radians]
        theta - noise hemisphere theta angle bounds       [radians] 
    
    Outputs: 
        gm_mic_locations   - cartesian coordiates of all microphones defined  [meters] 
    
    Properties Used:
        N/A       
    """     
    r     = settings.noise_hemisphere_radius                  
    phi   = settings.noise_hemisphere_phi_angles   
    theta = settings.noise_hemisphere_theta_angles   
 
    x     = r * np.outer(np.sin(phi), np.cos(theta))
    y     = r * np.outer(np.sin(phi), np.sin(theta))
    z     = r * np.outer(np.cos(phi), np.ones(np.size(theta))) 
 
    num_gm                = len(z.flatten())
    gm_mic_locations      = np.zeros((num_gm,3))  
    gm_mic_locations[:,0] = x.flatten() 
    gm_mic_locations[:,1] = y.flatten() 
    gm_mic_locations[:,2] = z.flatten()
    
    return gm_mic_locations   