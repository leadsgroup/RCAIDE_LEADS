# RCAIDE/Library/Missions/Segments/Climb/Constant_Speed_Linear_Altitude.py 
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
    Initializes conditions for constant speed climb with linear altitude change

    Parameters
    ----------
    segment : Segment
        The mission segment being analyzed

    Notes
    -----
    This function sets up the initial conditions for a climb segment with constant
    true airspeed and linear altitude variation. The climb angle is determined by
    the distance and altitude change.

    **Required Segment Components**

    segment:
        - air_speed : float
            True airspeed to maintain [m/s]
        - altitude_start : float
            Initial altitude [m]
        - altitude_end : float
            Final altitude [m]
        - distance : float
            Ground distance to cover [m]
        - sideslip_angle : float
            Aircraft sideslip angle [rad]
        - state:
            numerics.dimensionless.control_points : array
                Discretization points [-]
            conditions : Data
                State conditions container

    **Calculation Process**
        1. Calculate climb angle from altitude change and distance
        2. Discretize altitude profile
        3. Decompose constant velocity into components using:
            - Computed climb angle
            - Sideslip angle
            - Constant speed requirement

    **Major Assumptions**
        * Constant true airspeed
        * Linear altitude change
        * Small angle approximations
        * Quasi-steady flight

    Returns
    -------
    None
        Updates segment conditions directly:

    See Also
    --------
    RCAIDE.Framework.Mission.Segments
    """        
    
    # unpack
    alt0       = segment.altitude_start 
    altf       = segment.altitude_end
    xf         = segment.distance
    air_speed  = segment.air_speed    
    beta       = segment.sideslip_angle    
    conditions = segment.state.conditions 

    # check for initial velocity
    if air_speed is None: 
        if not segment.state.initials: raise AttributeError('airspeed not set')
        air_speed = np.linalg.norm(segment.state.initials.conditions.frames.inertial.velocity_vector[-1])
        
    # check for initial altitude
    if alt0 is None:
        if not segment.state.initials: raise AttributeError('altitude not set')
        alt0 = -1.0 *segment.state.initials.conditions.frames.inertial.position_vector[-1,2]
     
    climb_angle  = np.arctan((altf-alt0)/xf)
    v_x          = np.cos(beta)*np.cos(climb_angle)*air_speed
    v_y          = np.sin(beta)*np.cos(climb_angle)*air_speed
    v_z          = np.sin(climb_angle)*air_speed 
    t_nondim     = segment.state.numerics.dimensionless.control_points
    
    # discretize on altitude
    alt = t_nondim * (altf-alt0) + alt0    
    
    # pack
    conditions.freestream.altitude[:,0]             = alt[:,0]
    conditions.frames.inertial.position_vector[:,2] = -alt[:,0] # z points down 
    conditions.frames.inertial.velocity_vector[:,0] = v_x
    conditions.frames.inertial.velocity_vector[:,1] = v_y
    conditions.frames.inertial.velocity_vector[:,2] = -v_z  