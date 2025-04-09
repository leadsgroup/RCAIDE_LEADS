# RCAIDE/Library/Missions/Segments/Cruise/Constant_Mach_Constant_Altitude_Loiter.py
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
    Initializes conditions for constant speed loiter at fixed altitude

    Parameters
    ----------
    segment : Segment
        The mission segment being analyzed
            - altitude : float
                Loiter altitude [m]
            - time : float
                Loiter duration [s]
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
    This function sets up the initial conditions for a loiter segment with constant
    true airspeed and constant altitude. The segment duration is specified by time
    rather than distance.

    **Calculation Process**
        1. Check for initial conditions
        2. Discretize time points over loiter duration
        3. Decompose velocity into components using:
            - Constant airspeed
            - Sideslip angle

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
    final_time = segment.time
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
    t_initial = conditions.frames.inertial.time[0,0]
    t_final   = final_time + t_initial
    t_nondim  = segment.state.numerics.dimensionless.control_points
    time      = t_nondim * (t_final-t_initial) + t_initial
    v_x       = np.cos(beta)*air_speed 
    v_y       = np.sin(beta)*air_speed 
    
    # pack
    segment.state.conditions.freestream.altitude[:,0]             = alt
    segment.state.conditions.frames.inertial.position_vector[:,2] = -alt # z points down
    segment.state.conditions.frames.inertial.velocity_vector[:,0] = v_x
    segment.state.conditions.frames.inertial.velocity_vector[:,1] = v_y
    segment.state.conditions.frames.inertial.time[:,0]            = time[:,0]
    
