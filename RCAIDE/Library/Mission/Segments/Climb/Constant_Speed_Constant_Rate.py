# RCAIDE/Library/Missions/Segments/Climb/Constant_Speed_Constant_Rate.py
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
    Initializes conditions for constant speed climb segment

    Parameters
    ----------
    segment : Segment
        The mission segment being analyzed

    Notes
    -----
    This function sets up the initial conditions for a climb segment with constant
    true airspeed and constant rate of climb.

    **Required Segment Components**

    segment:
        - climb_rate : float
            Rate of climb [m/s]
        - air_speed : float
            True airspeed to maintain [m/s]
        - altitude_start : float
            Initial altitude [m]
        - altitude_end : float
            Final altitude [m]
        - sideslip_angle : float
            Aircraft sideslip angle [rad]
        - state:
            numerics.dimensionless.control_points : array
                Discretization points [-]
            conditions : Data
                State conditions container

    **Calculation Process**
        1. Discretize altitude profile
        2. Decompose constant velocity into components using:
            - Climb rate constraint
            - Sideslip angle
            - Constant speed requirement

    **Major Assumptions**
        * Constant true airspeed
        * Constant rate of climb
        * Small angle approximations
        * Quasi-steady flight

    Returns
    -------
    None
        Updates segment conditions directly:
        - conditions.frames.inertial.velocity_vector [m/s]
        - conditions.frames.inertial.position_vector [m]
        - conditions.freestream.altitude [m]

    See Also
    --------
    RCAIDE.Framework.Mission.Segments
    """            
    
    # unpack
    climb_rate = segment.climb_rate
    air_speed  = segment.air_speed   
    alt0       = segment.altitude_start 
    altf       = segment.altitude_end
    beta       = segment.sideslip_angle
    t_nondim   = segment.state.numerics.dimensionless.control_points
    conditions = segment.state.conditions  

    # check for initial velocity
    if air_speed is None: 
        if not segment.state.initials: raise AttributeError('airspeed not set')
        air_speed = np.linalg.norm(segment.state.initials.conditions.frames.inertial.velocity_vector[-1])
        
    # check for initial altitude
    if alt0 is None:
        if not segment.state.initials: raise AttributeError('initial altitude not set')
        alt0 = -1.0 *segment.state.initials.conditions.frames.inertial.position_vector[-1,2]

    # discretize on altitude
    alt = t_nondim * (altf-alt0) + alt0
    
    # process velocity vector
    v_mag = air_speed
    v_z   = -climb_rate 
    v_xy  = np.sqrt( v_mag**2 - v_z**2 )
    v_x   = np.cos(beta)*v_xy
    v_y   = np.sin(beta)*v_xy
    
    # pack conditions    
    conditions.frames.inertial.velocity_vector[:,0] = v_x
    conditions.frames.inertial.velocity_vector[:,1] = v_y
    conditions.frames.inertial.velocity_vector[:,2] = v_z
    conditions.frames.inertial.position_vector[:,2] = -alt[:,0] # z points down
    conditions.freestream.altitude[:,0]             =  alt[:,0] # positive altitude in this context