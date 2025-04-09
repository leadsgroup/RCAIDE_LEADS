# RCAIDE/Library/Missions/Segments/Untrimmed/Untrimmed.py
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
    """
    Initializes conditions for untrimmed flight analysis at fixed state

    Parameters
    ----------
    segment : Segment
        The mission segment being analyzed
            - altitude : float
                Flight altitude [m]
            - air_speed : float
                True airspeed [m/s]
            - sideslip_angle : float
                Aircraft sideslip angle [rad]
            - linear_acceleration_x : float
                Acceleration in x-direction [m/s^2]
            - linear_acceleration_y : float
                Acceleration in y-direction [m/s^2]
            - linear_acceleration_z : float
                Acceleration in z-direction [m/s^2]
            - roll_rate : float
                Aircraft roll rate [rad/s]
            - pitch_rate : float
                Aircraft pitch rate [rad/s]
            - yaw_rate : float
                Aircraft yaw rate [rad/s]
            - state:
                conditions : Data
                    State conditions container
                initials : Data, optional
                    Initial conditions from previous segment

    Returns
    -------
    None
        Updates segment conditions directly:
            - conditions.freestream.altitude [m]
            - conditions.frames.inertial.position_vector [m]
            - conditions.frames.inertial.velocity_vector [m/s]
            - conditions.frames.inertial.acceleration_vector [m/s^2]
            - conditions.static_stability.roll_rate [rad/s]
            - conditions.static_stability.pitch_rate [rad/s]
            - conditions.static_stability.yaw_rate [rad/s]

    Notes
    -----
    This function sets up the initial conditions for an untrimmed flight analysis
    at fixed speed and altitude. It allows specification of full 6-DOF motion 
    states without enforcing trim constraints.

    **Calculation Process**
        1. Check initial altitude
        2. Decompose velocity into components using sideslip angle:
            - v_x = V * cos(β)
            - v_y = V * sin(β)
            where:
            - V is true airspeed
            - β is sideslip angle
        3. Set position and altitude
        4. Set full acceleration vector
        5. Set angular rates

    **Major Assumptions**
        * Fixed speed and altitude point
        * No trim constraints enforced
        * Full 6-DOF motion allowed
        * Small angle approximations
        * No atmospheric variations

    See Also
    --------
    RCAIDE.Framework.Mission.Segments
    """      
    
    # unpack
    alt                    = segment.altitude
    air_speed              = segment.air_speed  
    sideslip               = segment.sideslip_angle
    linear_acceleration_x  = segment.linear_acceleration_x
    linear_acceleration_y  = segment.linear_acceleration_y 
    linear_acceleration_z  = segment.linear_acceleration_z
    roll_rate              = segment.roll_rate
    pitch_rate             = segment.pitch_rate
    yaw_rate               = segment.yaw_rate
    
    
    # check for initial altitude
    if alt is None:
        if not segment.state.initials: raise AttributeError('altitude not set')
        alt = -1.0 *segment.state.initials.conditions.frames.inertial.position_vector[-1,2]
    
    # pack
    air_speed_x                                                   = np.cos(sideslip)*air_speed 
    air_speed_y                                                   = np.sin(sideslip)*air_speed 
    segment.state.conditions.freestream.altitude[:,0]             = alt
    segment.state.conditions.frames.inertial.position_vector[:,2] = -alt # z points down
    segment.state.conditions.frames.inertial.velocity_vector[:,0] = air_speed_x
    segment.state.conditions.frames.inertial.velocity_vector[:,1] = air_speed_y
    segment.state.conditions.frames.inertial.acceleration_vector  = np.array([[linear_acceleration_x,linear_acceleration_y,linear_acceleration_z]])  
    segment.state.conditions.static_stability.roll_rate[:,0]      = roll_rate         
    segment.state.conditions.static_stability.pitch_rate[:,0]     = pitch_rate
    segment.state.conditions.static_stability.yaw_rate[:,0]       = yaw_rate      