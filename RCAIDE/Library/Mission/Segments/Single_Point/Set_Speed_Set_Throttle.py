# RCAIDE/Library/Missions/Segments/Single_Point/Set_Speed_Set_Throttle.py
# 
# 
# Created:  Jul 2023, M. Clarke 
 
# ----------------------------------------------------------------------------------------------------------------------  
#  Initialize Conditions
# ----------------------------------------------------------------------------------------------------------------------  

import numpy as np

# ----------------------------------------------------------------------------------------------------------------------  
#  Initialize Conditions
# ----------------------------------------------------------------------------------------------------------------------   
def initialize_conditions(segment):
    """Sets the specified conditions which are given for the segment type.

    Assumptions:
    A fixed speed and throttle

    Source:
    N/A

    Inputs:
    segment.altitude                               [meters]
    segment.air_speed                              [meters/second]
    segment.throttle                               [unitless]
    segment.linear_acceleration_z                         [meters/second^2]
    segment.state.unknowns.acceleration            [meters/second^2]

    Outputs:
    conditions.frames.inertial.acceleration_vector [meters/second^2]
    conditions.frames.inertial.velocity_vector     [meters/second]
    conditions.frames.inertial.position_vector     [meters]
    conditions.freestream.altitude                 [meters]
    conditions.frames.inertial.time                [seconds]

    Properties Used:
    N/A
    """      
    
    # unpack
    alt                     = segment.altitude
    air_speed               = segment.air_speed
    beta                    = segment.sideslip_angle  
    linear_acceleration_z   = segment.linear_acceleration_z
    roll_rate               = segment.roll_rate
    pitch_rate              = segment.pitch_rate
    yaw_rate                = segment.yaw_rate
    acceleration            = segment.state.unknowns.acceleration[0][0]
    
    # check for initial altitude
    if alt is None:
        if not segment.state.initials: raise AttributeError('altitude not set')
        alt = -1.0 *segment.state.initials.conditions.frames.inertial.position_vector[-1,2]
    
    v_x  = np.cos(beta)*air_speed 
    v_y  = np.sin(beta)*air_speed
        
    # pack
    segment.state.conditions.freestream.altitude[:,0]             = alt
    segment.state.conditions.frames.inertial.position_vector[:,2] = -alt # z points down
    segment.state.conditions.frames.inertial.velocity_vector[:,0] = v_x 
    segment.state.conditions.frames.inertial.velocity_vector[:,1] = v_y 
    segment.state.conditions.frames.inertial.acceleration_vector  = np.array([[acceleration,0.0,linear_acceleration_z]])  
    segment.state.conditions.static_stability.roll_rate[:,0]      = roll_rate         
    segment.state.conditions.static_stability.pitch_rate[:,0]     = pitch_rate
    segment.state.conditions.static_stability.yaw_rate[:,0]       = yaw_rate      

    
# ----------------------------------------------------------------------------------------------------------------------  
#  Unpack Unknowns 
# ----------------------------------------------------------------------------------------------------------------------  
def unpack_unknowns(segment):
    """ Unpacks the x accleration and body angle from the solver to the mission
    
        Assumptions:
        N/A
        
        Inputs:
            segment.state.unknowns:
                acceleration                        [meters/second^2]
                body_angle                          [radians]
            
        Outputs:
            segment.state.conditions:
                frames.inertial.acceleration_vector [meters/second^2]
                frames.body.inertial_rotations      [radians]

        Properties Used:
        N/A
                                
    """      
    
    # unpack unknowns  
    acceleration  = segment.state.unknowns.acceleration 
    segment.state.conditions.frames.inertial.acceleration_vector[0,0] = acceleration         