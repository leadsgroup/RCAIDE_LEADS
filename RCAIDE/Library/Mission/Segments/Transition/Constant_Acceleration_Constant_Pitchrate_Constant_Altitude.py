# RCAIDE/Library/Missions/Segments/Transition/Constant_Acceleration_Constant_Pitchrate_Constant_Altitude.py
# 
# 
# Created:  Jul 2023, M. Clarke 
  
# ----------------------------------------------------------------------------------------------------------------------  
#  Initialize Conditions
# ----------------------------------------------------------------------------------------------------------------------   
# Package imports 
import numpy as np

def initialize_conditions(segment):
    """Sets the specified conditions which are given for the segment type.

    Assumptions:
    Constant acceleration and constant altitude

    Source:
    N/A

    Inputs:
    segment.altitude                [meters]
    segment.air_speed_start         [meters/second]
    segment.air_speed_end           [meters/second]
    segment.acceleration            [meters/second^2]
    conditions.frames.inertial.time [seconds]

    Outputs:
    conditions.frames.inertial.velocity_vector  [meters/second]
    conditions.frames.inertial.position_vector  [meters]
    conditions.freestream.altitude              [meters]
    conditions.frames.inertial.time             [seconds]

    Properties Used:
    N/A
    """      
    
    # unpack
    alt = segment.altitude 
    v0  = segment.air_speed_start
    vf  = segment.air_speed_end  
    beta= segment.sideslip_angle
    ax  = segment.acceleration   
    T0  = segment.pitch_initial
    Tf  = segment.pitch_final     
    
    # check for initial altitude
    if alt is None:
        if not segment.state.initials: raise AttributeError('altitude not set')
        alt = -1.0 * segment.state.initials.conditions.frames.inertial.position_vector[-1,2]
        segment.altitude = alt
        
    # check for initial pitch
    if T0 is None:
        T0  =  segment.state.initials.conditions.frames.body.inertial_rotations[-1,1]
        segment.pitch_initial = T0    
    
    # check for initial velocity vector
    if v0 is None:
        if not segment.state.initials: raise AttributeError('initial airspeed not set')
        v0  =  segment.state.initials.conditions.frames.inertial.velocity_vector[-1,0] # x direction velocity         

    # dimensionalize time
    t_initial = segment.state.conditions.frames.inertial.time[0,0]
    t_final   = (vf-v0)/ax + t_initial
    t_nondim  = segment.state.numerics.dimensionless.control_points
    time      = t_nondim * (t_final-t_initial) + t_initial
    
    # Figure out vx
    v_mag = v0+(time - t_initial)*ax
    v_x   = np.cos(beta)*v_mag
    v_y   = np.sin(beta)*v_mag
    
    # set the body angle
    if Tf > T0:
        body_angle = T0 + time*(Tf-T0)/(t_final-t_initial)
    else:
        body_angle = T0 - time*(T0-Tf)/(t_final-t_initial)
    segment.state.conditions.frames.body.inertial_rotations[:,1] = body_angle[:,0]     
    
    # pack
    segment.state.conditions.freestream.altitude[:,0] = alt
    segment.state.conditions.frames.inertial.position_vector[:,2] = -alt # z points down
    segment.state.conditions.frames.inertial.velocity_vector[:,0] = v_x[:,0]
    segment.state.conditions.frames.inertial.velocity_vector[:,1] = v_y[:,0]
    segment.state.conditions.frames.inertial.time[:,0] = time[:,0]
