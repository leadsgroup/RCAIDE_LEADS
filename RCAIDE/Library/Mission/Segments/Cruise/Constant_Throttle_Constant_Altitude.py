# RCAIDE/Library/Missions/Segments/Cruise/Constant_Throttle_Constant_Altitude.py
# 
# 
# Created:  Jul 2023, M. Clarke

# ----------------------------------------------------------------------------------------------------------------------
#  IMPORT
# ---------------------------------------------------------------------------------------------------------------------- 
# Package imports 
import numpy as np
 
# ----------------------------------------------------------------------------------------------------------------------
#  Initialize Conditions
# ----------------------------------------------------------------------------------------------------------------------
def unpack_unknowns(segment):
    
    # unpack unknowns
    unknowns   = segment.state.unknowns 
    accel_x    = unknowns.acceleration 
    time       = unknowns.elapsed_time
     
    # rescale time
    t_initial  = segment.state.conditions.frames.inertial.time[0,0]
    t_final    = t_initial + time  
    t_nondim   = segment.state.numerics.dimensionless.control_points
    time       = t_nondim * (t_final-t_initial) + t_initial     

    # build acceleration
    N          = segment.state.numerics.number_of_control_points
    a          = np.zeros((N, 3))
    a[:, 0]    = accel_x[:,0]
    
    # apply unknowns
    conditions = segment.state.conditions 
    conditions.frames.inertial.acceleration_vector  = a
    conditions.frames.inertial.time[:,0]            = time[:,0]
    
    return 

# ----------------------------------------------------------------------
#  Integrate Velocity
# ---------------------------------------------------------------------- 

def integrate_velocity(segment):
    
    # unpack 
    conditions = segment.state.conditions
    v0         = segment.air_speed_start 
    beta       = segment.sideslip_angle
    I          = segment.state.numerics.time.integrate
    a          = conditions.frames.inertial.acceleration_vector
    
    # compute x-velocity
    velocity_xy = v0 + np.dot(I, a)[:,0]   
    v_x         = np.cos(beta)*velocity_xy
    v_y         = np.sin(beta)*velocity_xy

    # pack velocity
    conditions.frames.inertial.velocity_vector[:,0] = v_x
    conditions.frames.inertial.velocity_vector[:,1] = v_y
    
    return

# ----------------------------------------------------------------------
#  Initialize Conditions
# ----------------------------------------------------------------------    

def initialize_conditions(segment):
    """Sets the specified conditions which are given for the segment type.

    Assumptions:
    Constant throttle and constant altitude, allows for acceleration

    Source:
    N/A

    Inputs:
    segment.altitude                             [meters]
    segment.air_speed_start                      [meters/second]
    segment.air_speed_end                        [meters/second] 
    segment.state.numerics.number_of_control_points [int]

    Outputs:
    state.conditions.energy.throttle        [unitless]
    conditions.frames.inertial.position_vector  [meters]
    conditions.freestream.altitude              [meters]

    Properties Used:
    N/A
    """    
    # unpack inputs
    alt      = segment.altitude 
    v0       = segment.air_speed_start
    vf       = segment.air_speed_end    
    
    # check for initial altitude
    if alt is None:
        if not segment.state.initials: raise AttributeError('altitude not set')
        alt = -1.0 *segment.state.initials.conditions.frames.inertial.position_vector[-1,2]   

    if v0  is None: 
        if not segment.state.initials: raise AttributeError('airspeed not set')
        v0 = np.linalg.norm(segment.state.initials.conditions.frames.inertial.velocity_vector[-1])
        
    # avoid having zero velocity since aero and propulsion models need non-zero Reynolds number
    if v0 == 0.0: v0 = 0.01
    if vf == 0.0: vf = 0.01
    
    # intial and final speed cannot be the same
    if v0 == vf:
        vf = vf + 0.01
        
    # repack
    segment.air_speed_start = v0
    segment.air_speed_end   = vf
    
    # pack conditions   
    segment.state.conditions.freestream.altitude[:,0] = alt
    segment.state.conditions.frames.inertial.position_vector[:,2] = -alt # z points down    
    
# ----------------------------------------------------------------------
#  Solve Residuals
# ----------------------------------------------------------------------    

def solve_velocity(segment):
    """ Calculates the additional velocity residual
    
        Assumptions:
        The vehicle accelerates, residual on forces and to get it to the final speed
        
        Inputs:
        segment.air_speed_end                  [meters/second]
        segment.state.conditions: 
            frames.inertial.velocity_vector    [meters/second] 
        segment.state.numerics.time.differentiate
            
        Outputs:
        segment.state.residuals:
            forces               [meters/second^2]
            final_velocity_error [meters/second] 

        Properties Used:
        N/A
                                
    """    

    # unpack inputs
    conditions = segment.state.conditions 
    vf         = segment.air_speed_end
    v          = conditions.frames.inertial.velocity_vector 
    
    segment.state.residuals.final_velocity_error = (np.linalg.norm(v[-1,:])- vf)

    return