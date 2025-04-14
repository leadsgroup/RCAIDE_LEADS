# RCAIDE/Library/Missions/Segments/Cruise/Constant_Speed_Constant_Altitude.py
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
    Initializes conditions for constant speed cruise at fixed altitude

    Parameters
    ----------
    segment : Segment
        The mission segment being analyzed
            - altitude : float
                Cruise altitude [m]
            - distance : float
                Ground distance to cover [m]
            - air_speed : float
                True airspeed to maintain [m/s]
            - sideslip_angle : float
                Aircraft sideslip angle [rad]
            - state:
                numerics.dimensionless.control_points : array
                    Discretization points [-]
                conditions : Data
                    State conditions container
                initials : Data, optional
                    Initial conditions from previous segment
    
    Returns
    -------
    None
        Updates segment conditions directly:
            - conditions.frames.inertial.velocity_vector [m/s]
            - conditions.frames.inertial.position_vector [m]
            - conditions.freestream.altitude [m]
            - conditions.frames.inertial.time [s]

    Notes
    -----
    This function sets up the initial conditions for a cruise segment with constant
    true airspeed and constant altitude. The segment duration is determined by the
    specified ground distance to cover.

    **Calculation Process**
        1. Check for initial conditions
        2. Calculate time required based on distance and speed:
            t = x/V where:
                - x is ground distance
                - V is true airspeed
        3. Discretize time points
        4. Decompose velocity into components using sideslip angle

    **Major Assumptions**
        * Constant true airspeed
        * Constant altitude
        * Small angle approximations
        * Quasi-steady flight
        * No wind effects

    See Also
    --------
    RCAIDE.Framework.Mission.Segments
    """        
    
    # unpack 
    alt        = segment.altitude
    xf         = segment.distance
    air_speed  = segment.air_speed       
    beta       = segment.sideslip_angle
    conditions = segment.state.conditions 

    # check for initial velocity
    if air_speed is None: 
        if not segment.state.initials: raise AttributeError('airspeed not set')
        air_speed = np.linalg.norm(segment.state.initials.conditions.frames.inertial.velocity_vector[-1])
        
    # check for initial altitude
    if alt is None:
        if not segment.state.initials: raise AttributeError('altitude not set')
        alt = -1.0 * segment.state.initials.conditions.frames.inertial.position_vector[-1,2]
    
    # dimensionalize time
    v_x         = np.cos(beta)*air_speed 
    v_y         = np.sin(beta)*air_speed 
    t_initial   = conditions.frames.inertial.time[0,0]
    t_final     = xf /air_speed + t_initial
    t_nondim    = segment.state.numerics.dimensionless.control_points
    time        = t_nondim * (t_final-t_initial) + t_initial
    
    # pack
    segment.state.conditions.freestream.altitude[:,0]             = alt
    segment.state.conditions.frames.inertial.position_vector[:,2] = -alt # z points down
    segment.state.conditions.frames.inertial.velocity_vector[:,0] = v_x
    segment.state.conditions.frames.inertial.velocity_vector[:,1] = v_y
    segment.state.conditions.frames.inertial.time[:,0]            = time[:,0]