# RCAIDE/Library/Missions/Segments/Descent/Constant_Speed_Constant_Angle.py
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
    Initializes conditions for constant speed descent at fixed angle

    Parameters
    ----------
    segment : Segment
        The mission segment being analyzed
            - descent_angle : float
                Fixed descent angle [rad]
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
    true airspeed and constant descent angle. The vertical speed is determined by
    the descent angle. Updates segment conditions directly with velocity_vector [m/s],
    altitude [m], and position_vector [m].

    **Calculation Process**
        1. Discretize altitude profile
        2. Decompose velocity into components using:
            - Fixed descent angle
            - Sideslip angle
            - Constant true airspeed
        3. Components calculated as:
                - v_x = V * cos(β) * cos(-γ)
                - v_y = V * sin(β) * cos(-γ)
                - v_z = -V * sin(-γ)
            where:
                - V is true airspeed
                - β is sideslip angle
                - γ is descent angle

    **Major Assumptions**
        * Constant true airspeed
        * Fixed descent angle
        * Small angle approximations
        * Quasi-steady flight
        * No wind effects

    See Also
    --------
    RCAIDE.Framework.Mission.Segments
    """        
    
    # unpack
    descent_angle= segment.descent_angle
    air_speed    = segment.air_speed   
    alt0         = segment.altitude_start 
    altf         = segment.altitude_end 
    beta         = segment.sideslip_angle
    t_nondim     = segment.state.numerics.dimensionless.control_points
    conditions   = segment.state.conditions  

    # check for initial altitude
    if alt0 is None:
        if not segment.state.initials: raise AttributeError('initial altitude not set')
        alt0 = -1.0 * segment.state.initials.conditions.frames.inertial.position_vector[-1,2]
    
    # check for initial velocity vector
    if air_speed is None:
        if not segment.state.initials: raise AttributeError('initial airspeed not set')
        air_speed  =  np.linalg.norm(segment.state.initials.conditions.frames.inertial.velocity_vector[-1,:])     
            
    # discretize on altitude
    alt = t_nondim * (altf-alt0) + alt0
    
    # process velocity vector
    v_mag = air_speed
    v_x   = np.cos(beta)* v_mag * np.cos(-descent_angle)
    v_y   = np.sin(beta)* v_mag * np.cos(-descent_angle)
    v_z   = -v_mag * np.sin(-descent_angle)
    
    # pack conditions    
    conditions.frames.inertial.velocity_vector[:,0] = v_x
    conditions.frames.inertial.velocity_vector[:,1] = v_y
    conditions.frames.inertial.velocity_vector[:,2] = v_z
    conditions.frames.inertial.position_vector[:,2] = -alt[:,0] # z points down
    conditions.freestream.altitude[:,0]             =  alt[:,0] # positive altitude in this context