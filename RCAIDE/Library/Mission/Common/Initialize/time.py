# RCAIDE/Library/Missions/Common/Initialize/time.py
# 
# 
# Created:  Jul 2023, M. Clarke
 
# ----------------------------------------------------------------------------------------------------------------------
#  Initialize Time
# ---------------------------------------------------------------------------------------------------------------------- 
def time(segment):
    """ Sets the initial time of the vehicle at the start of the segment
    
        Assumptions:
        Only used if there is an initial condition
        
        Inputs:
            state.initials.conditions:
                frames.inertial.time     [seconds]
                frames.planet.start_time [seconds]
            state.conditions:           
                frames.inertial.time     [seconds]
            segment.start_time           [seconds]
            
        Outputs:
            state.conditions:           
                frames.inertial.time     [seconds]
                frames.planet.start_time [seconds]

        Properties Used:
        N/A
                                
    """        
    
    if segment.state.initials:
        t_initial = segment.state.initials.conditions.frames.inertial.time
        t_current = segment.state.conditions.frames.inertial.time 
        segment.state.conditions.frames.inertial.time[:,:] = t_current + (t_initial[-1,0] - t_current[0,0])
        
    else:
        t_initial = segment.state.conditions.frames.inertial.time[0,0]
        
    if segment.state.initials:
        segment.state.conditions.frames.planet.start_time = segment.state.initials.conditions.frames.planet.start_time
        
    elif 'start_time' in segment:
        segment.state.conditions.frames.planet.start_time = segment.start_time
    
    return 