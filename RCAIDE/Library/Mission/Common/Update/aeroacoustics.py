# RCAIDE/Library/Missions/Common/Update/aeroacoustics.py
# 
# 
# Created:  Jul 2023, M. Clarke

# ----------------------------------------------------------------------------------------------------------------------
# aeroacoustics
# ---------------------------------------------------------------------------------------------------------------------- 
def aeroacoustics(segment):
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
    aeroacoustics_model = segment.analyses.aeroacoustics
    
    if aeroacoustics_model:
        aeroacoustics_model.evaluate_aeroacoustics(segment, segment.analyses.vehicle)    