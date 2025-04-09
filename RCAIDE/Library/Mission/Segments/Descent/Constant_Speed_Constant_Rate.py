# RCAIDE/Library/Missions/Segments/Descent/Constant_Speed_Constant_Rate.py
# 
# 
# Created:  Jul 2023, M. Clarke 

# ----------------------------------------------------------------------------------------------------------------------  
#  IMPORT 
# ----------------------------------------------------------------------------------------------------------------------  
# package imports 
import numpy as np

# ---------------------------------------------------------------------------------------------------------------------- 
#  Initialize Conditions
# ----------------------------------------------------------------------------------------------------------------------  
def initialize_conditions(segment):
    """
    Initializes conditions for constant speed descent at fixed rate

    Parameters
    ----------
    segment : Segment
        The mission segment being analyzed
            - descent_rate : float
                Rate of descent [m/s]
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
    
    Returns
    -------
    None
    
    Notes
    -----
    This function sets up the initial conditions for a descent segment with constant
    true airspeed and constant descent rate. The horizontal velocity components are
    determined from the airspeed and descent rate constraints. Updates segment conditions
    directly with velocity_vector [m/s], altitude [m], and position_vector [m].

    **Calculation Process**
        1. Discretize altitude profile
        2. Calculate horizontal velocity magnitude:
        v_xy = sqrt(V^2 - v_z^2) where:
            - V is true airspeed
            - v_z is descent rate
        3. Decompose horizontal velocity using sideslip angle:
            - v_x = v_xy * cos(β)
            - v_y = v_xy * sin(β)

            where β is sideslip angle

    **Major Assumptions**
        * Constant true airspeed
        * Constant descent rate
        * Small angle approximations
        * Quasi-steady flight
        * No wind effects

    See Also
    --------
    RCAIDE.Framework.Mission.Segments
    """     
    
    # unpack
    descent_rate = segment.descent_rate
    air_speed    = segment.air_speed   
    alt0         = segment.altitude_start 
    altf         = segment.altitude_end
    beta         = segment.sideslip_angle
    t_nondim     = segment.state.numerics.dimensionless.control_points
    conditions   = segment.state.conditions  

    # check for initial altitude
    if alt0 is None:
        if not segment.state.initials: raise AttributeError('initial altitude not set')
        alt0 = -1.0 *segment.state.initials.conditions.frames.inertial.position_vector[-1,2] 
        
    # check for initial velocity vector
    if air_speed is None:
        if not segment.state.initials: raise AttributeError('initial airspeed not set')
        air_speed  =  np.linalg.norm(segment.state.initials.conditions.frames.inertial.velocity_vector[-1,:])   
            
    # discretize on altitude
    alt = t_nondim * (altf-alt0) + alt0
    
    # process velocity vector
    v_xy_mag    = air_speed
    v_z         = descent_rate # z points down
    v_xy        = np.sqrt(v_xy_mag**2 - v_z**2 )
    v_x         = np.cos(beta)*v_xy
    v_y         = np.sin(beta)*v_xy
    
    # pack conditions    
    conditions.frames.inertial.velocity_vector[:,0] = v_x
    conditions.frames.inertial.velocity_vector[:,1] = v_y
    conditions.frames.inertial.velocity_vector[:,2] = v_z
    conditions.frames.inertial.position_vector[:,2] = -alt[:,0] # z points down
    conditions.freestream.altitude[:,0]             =  alt[:,0] # positive altitude in this context