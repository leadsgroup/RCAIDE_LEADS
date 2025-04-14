# RCAIDE/Library/Missions/Segments/Cruise/Constant_Acceleration_Constant_Altitude.py
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
    """
    Initializes conditions for constant acceleration cruise at fixed altitude

    Parameters
    ----------
    segment : Segment
        The mission segment being analyzed

    Notes
    -----
    This function sets up the initial conditions for a cruise segment with constant
    acceleration and constant altitude. The velocity changes linearly with time
    based on the specified acceleration.

    **Required Segment Components**

    segment:
        - altitude : float
            Cruise altitude [m]. If not specified, the altitude from the previous segment is used.
        - air_speed_start : float
            Initial true airspeed [m/s]. If not specified, the airspeed from the previous segment is used.
        - air_speed_end : float
            Final true airspeed [m/s]
        - acceleration : float
            Constant acceleration [m/s^2]
        - sideslip_angle : float
            Aircraft sideslip angle [rad]
        - state:
            numerics.dimensionless.control_points : array
                Discretization points [-]
            conditions : Data
                State conditions container

    **Calculation Process**
        1. Calculate time required for speed change
        2. Discretize time points
        3. Calculate velocity profile using:
            - Initial velocity
            - Constant acceleration
            - Sideslip angle for lateral components

    **Major Assumptions**
        * Constant acceleration
        * Constant altitude
        * Small angle approximations
        * Quasi-steady flight
        * No wind effects

    Returns
    -------
    None
        Updates segment conditions directly:
        - conditions.frames.inertial.velocity_vector [m/s]
        - conditions.frames.inertial.position_vector [m]
        - conditions.freestream.altitude [m]
        - conditions.frames.inertial.time [s]

    See Also
    --------
    RCAIDE.Framework.Mission.Segments
    """      
    
    # unpack
    alt        = segment.altitude 
    v0         = segment.air_speed_start
    vf         = segment.air_speed_end
    ax         = segment.acceleration    
    beta       = segment.sideslip_angle
    conditions = segment.state.conditions 
    
    # check for initial altitude
    if alt is None:
        if not segment.state.initials: raise AttributeError('altitude not set')
        alt = -1.0 * segment.state.initials.conditions.frames.inertial.position_vector[-1,2]
    
    # check for initial velocity
    if v0 is None: 
        if not segment.state.initials: raise AttributeError('airspeed not set')
        v0 = np.linalg.norm(segment.state.initials.conditions.frames.inertial.velocity_vector[-1])
        
    # dimensionalize time
    t_initial = conditions.frames.inertial.time[0,0]
    t_final   = (vf-v0)/ax + t_initial
    t_nondim  = segment.state.numerics.dimensionless.control_points
    time      = t_nondim * (t_final-t_initial) + t_initial 
    v_mag = v0+(time - t_initial)*ax
    v_x   = np.cos(beta)*v_mag
    v_y   = np.sin(beta)*v_mag
    
    # pack
    segment.state.conditions.freestream.altitude[:,0]             = alt
    segment.state.conditions.frames.inertial.position_vector[:,2] = -alt # z points down
    segment.state.conditions.frames.inertial.velocity_vector[:,0] = v_x[:,0]
    segment.state.conditions.frames.inertial.velocity_vector[:,1] = v_y[:,0]
    segment.state.conditions.frames.inertial.time[:,0]            = time[:,0] 
