# RCAIDE/Library/Missions/Common/Update/emissions.py
# 
# 
# Created:  Jul 2023, M. Clarke

# ----------------------------------------------------------------------------------------------------------------------
#  emissions
# ----------------------------------------------------------------------------------------------------------------------

def emissions(segment):
    """ Updates the emissions produced by the vehicle
        
        Assumptions:
        N/A
        
        Inputs:
            None 
                 
        Outputs: 
            None
      
        Properties Used:
        N/A
                    
    """   
    emissions_model = segment.analyses.emissions
    
    if emissions_model:
        emissions_model.evaluate(segment,segment.analyses.vehicle)    