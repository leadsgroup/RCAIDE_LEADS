# RCAIDE/Library/Missions/Common/Update/altitude.py
# 
# 
# Created:  Jul 2023, M. Clarke 
# Modified: Nov 2026, M. Clarke

# ----------------------------------------------------------------------------------------------------------------------
#  Update Altitude
# ---------------------------------------------------------------------------------------------------------------------- 
def altitude(segment):
    """ Updates freestream altitude from inertial position
        
        Assumptions:
        N/A
        
        Inputs:
            segment.state.conditions:
                frames.inertial.position_vector [meters]
        Outputs:
            segment.state.conditions:
                freestream.altitude             [meters]
      
        Properties Used:
        N/A
                    
    """    
    altitude = -segment.state.conditions.frames.inertial.position_vector[:,2]
    segment.state.conditions.freestream.altitude = altitude[:,None] 