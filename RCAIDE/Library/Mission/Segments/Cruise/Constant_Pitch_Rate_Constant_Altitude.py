# RCAIDE/Library/Missions/Segments/Cruise/Constant_Pitch_Rate_Constant_Altitude.py
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
def initialize_conditions(segment):
    """Sets the specified conditions which are given for the segment type.

    Assumptions:
    Constant acceleration and constant altitude

    Source:
    N/A

    Inputs:
    segment.altitude                [meters]
    segment.pitch_initial           [radians]
    segment.pitch_final             [radians]
    segment.pitch_rate              [radians/second]

    Outputs:
    conditions.frames.body.inertial_rotations   [radians/second]
    conditions.frames.inertial.position_vector  [meters]
    conditions.freestream.altitude              [meters]
    conditions.frames.inertial.time             [seconds]

    Properties Used:
    N/A
    """       
    
    # unpack
    alt        = segment.altitude 
    T0         = segment.pitch_initial
    Tf         = segment.pitch_final 
    theta_dot  = segment.pitch_rate   
    conditions = segment.state.conditions 
    state      = segment.state
    
    # check for initial altitude
    if alt is None:
        if not segment.state.initials: raise AttributeError('altitude not set')
        alt = -1.0 * segment.state.initials.conditions.frames.inertial.position_vector[-1,2]
        
    # check for initial pitch
    if T0 is None:
        T0  =  state.initials.conditions.frames.body.inertial_rotations[-1,1]
        segment.pitch_initial = T0
    
    # dimensionalize time
    t_initial = conditions.frames.inertial.time[0,0]
    t_final   = (Tf-T0)/theta_dot + t_initial
    t_nondim  = state.numerics.dimensionless.control_points
    time      = t_nondim * (t_final-t_initial) + t_initial
    
    # set the body angle
    body_angle = theta_dot*time + T0
    segment.state.conditions.frames.body.inertial_rotations[:,1] = body_angle[:,0]    
    
    # pack
    segment.state.conditions.freestream.altitude[:,0]             = alt
    segment.state.conditions.frames.inertial.position_vector[:,2] = -alt # z points down
    segment.state.conditions.frames.inertial.time[:,0]            = time[:,0]
    
    