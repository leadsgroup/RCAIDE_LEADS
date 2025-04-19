# RCAIDE/Library/Missions/Segments/Common/Initialize/Frames.py
# 
# 
# Created:  Jul 2023, M. Clarke 

# ----------------------------------------------------------------------------------------------------------------------
#  Initialize Planet Position
# ----------------------------------------------------------------------------------------------------------------------
def planet_position(segment):
    """ Sets the initial location of the vehicle relative to the planet at the start of the segment
    
        Assumptions:
        Only used if there is an initial condition
        
        Inputs:
            segment.state.initials.conditions:
                frames.planet.longitude [Radians]
                frames.planet.latitude  [Radians]
            segment.longitude           [Radians]
            segment.latitude            [Radians]

        Outputs:
            segment.state.conditions:           
                frames.planet.latitude  [Radians]
                frames.planet.longitude [Radians]

        Properties Used:
        N/A
                                
    """        
    
    if segment.state.initials:
        longitude_initial = segment.state.initials.conditions.frames.planet.longitude[-1,0]
        latitude_initial  = segment.state.initials.conditions.frames.planet.latitude[-1,0] 
    elif 'latitude' in segment:
        longitude_initial = segment.longitude
        latitude_initial  = segment.latitude      
    else:
        longitude_initial = 0.0
        latitude_initial  = 0.0

    segment.state.conditions.frames.planet.longitude[:,0] = longitude_initial
    segment.state.conditions.frames.planet.latitude[:,0]  = latitude_initial    

    return 