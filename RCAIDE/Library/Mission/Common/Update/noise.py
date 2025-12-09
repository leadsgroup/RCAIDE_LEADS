# RCAIDE/Library/Missions/Common/Update/noise.py
# 
# 
# Created:  Jul 2023, M. Clarke

# ----------------------------------------------------------------------------------------------------------------------
# noise
# ---------------------------------------------------------------------------------------------------------------------- 
def noise(segment):
    """ Updates the noise produced by the vehicle
        
        Assumptions:
        N/A
        
        Inputs:
            None 
                 
        Outputs: 
            None
      
        Properties Used:
        N/A
                    
    """   
    noise_model = segment.analyses.noise
    
    if noise_model:
        noise_model.evaluate_noise(segment, segment.analyses.vehicle)    