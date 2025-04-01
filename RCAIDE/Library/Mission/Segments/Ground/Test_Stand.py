# RCAIDE/Library/Missions/Segments/Ground/Test_Stand.py
# 
# 
# Created:  Jul 2023, M. Clarke  

# ----------------------------------------------------------------------------------------------------------------------
#  IMPORT
# ----------------------------------------------------------------------------------------------------------------------
# RCAIDE Imports  
import numpy as np 

# ----------------------------------------------------------------------------------------------------------------------
# unpack unknowns
# ---------------------------------------------------------------------------------------------------------------------- 
def initialize_conditions(segment):
    """Sets the specified conditions which are given for the segment type.

    Assumptions:
    Builds on the initialize conditions for common 
    """  

    # use the common initialization # unpack inputs
    alt           = segment.altitude 
    v             = segment.velocity
    elapsed_time  = segment.time 
    
    # check for initial altitude
    if alt is None:
        if not segment.state.initials: raise AttributeError('altitude not set')
        alt = -1.0 *segment.state.initials.conditions.frames.inertial.position_vector[-1,2]   

    if v  is None: 
        v = np.linalg.norm(segment.state.initials.conditions.frames.inertial.velocity_vector[-1])
          
    # dimensionalize time
    conditions = segment.state.conditions    
    t_initial  = conditions.frames.inertial.time[0,0]
    t_final    = elapsed_time + t_initial
    t_nondim   = segment.state.numerics.dimensionless.control_points
    time       =  t_nondim * (t_final-t_initial) + t_initial 
    
    # pack
    segment.state.conditions.freestream.altitude[:,0]             = alt 
    segment.state.conditions.frames.inertial.velocity_vector[:,0] = v 
    segment.state.conditions.frames.inertial.time[:,0]            = time[:,0]
    