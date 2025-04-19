# RCAIDE/Library/Missions/Common/Pre_Process/emissions.py
# 
# 
# Created:  Jul 2023, M. Clarke

# ----------------------------------------------------------------------------------------------------------------------
#  IMPORT
# ---------------------------------------------------------------------------------------------------------------------- 
# RCAIDE imports   
# ----------------------------------------------------------------------------------------------------------------------
#  stability
# ----------------------------------------------------------------------------------------------------------------------  
def emissions(mission):
    """ Runs emissions model and build surrogate
    
        Assumptions:
            N/A
        
        Inputs:
            None
            
        Outputs:
            None             
    """
        
    last_tag = None
    for tag,segment in mission.segments.items():
        if segment.analyses.emissions != None:
            if last_tag and  'compute' in mission.segments[last_tag].analyses.emissions.process: 
                segment.analyses.emissions.process.emissions = mission.segments[last_tag].analyses.emissions.process.emissions
                segment.analyses.emissions.surrogates        = mission.segments[last_tag].analyses.emissions.surrogates  
            else:          
                em   = segment.analyses.emissions
                em.initialize()   
                last_tag = tag
    return 