# RCAIDE/Library/Missions/Common/Unknowns/ground.py
# 
# 
# Created:  Jul 2023, M. Clarke
 
# ----------------------------------------------------------------------------------------------------------------------
#  Unpack Unknowns
# ---------------------------------------------------------------------------------------------------------------------- 
def ground(segment):
    """ Unpacks the times and velocities from the solver to the mission
    
        Assumptions:
        Overrides the velocities if they go to zero
        
        Inputs:
            segment.state.unknowns:
                ground_velocity         [meters/second]
                time               [second]
            segment.velocity_start [meters/second]
            
        Outputs:
            segment.state.conditions:
                frames.inertial.velocity_vector [meters/second]
                frames.inertial.time            [second]
        Properties Used:
        N/A
                                
    """       
    
    # unpack unknowns 
    ground_velocity = segment.state.unknowns.ground_velocity
    time            = segment.state.unknowns.elapsed_time
    
    # unpack givens
    v0         = segment.air_speed_start  
    t_initial  = segment.state.conditions.frames.inertial.time[0,0]
    t_nondim   = segment.state.numerics.dimensionless.control_points
    
    # time
    t_final    = t_initial + time  
    times      = t_nondim * (t_final-t_initial) + t_initial     

    # apply unknowns
    conditions = segment.state.conditions
    conditions.frames.inertial.velocity_vector[1:,0] = ground_velocity
    conditions.frames.inertial.velocity_vector[0,0]  = v0
    conditions.frames.inertial.time[:,0]             = times[:,0]
