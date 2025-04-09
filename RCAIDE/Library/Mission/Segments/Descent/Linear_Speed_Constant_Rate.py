# RCAIDE/Library/Missions/Segments/Descent/Linear_Speed_Constant_Rate.py
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
    Initializes conditions for linear speed descent at fixed rate

    Parameters
    ----------
    segment : Segment
        The mission segment being analyzed
            - descent_rate : float
                Rate of descent [m/s]
            - air_speed_start : float
                Initial true airspeed [m/s]
            - air_speed_end : float
                Final true airspeed [m/s]
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
                initials : Data, optional
                    Initial conditions from previous segment

    Returns
    -------
    None

    Notes
    -----
    This function sets up the initial conditions for a descent segment with linearly
    varying true airspeed and constant descent rate. The horizontal velocity 
    components vary with the changing airspeed. Updates segment with velocity
    vector, altitude, and position vector.

    **Calculation Process**
        1. Discretize altitude profile
        2. Calculate true airspeed variation:
        V = V0 + (Vf - V0)*t where:
            - V0 is initial airspeed
            - Vf is final airspeed
            - t is normalized time/distance
        3. Calculate horizontal velocity magnitude:
        v_xy = sqrt(V^2 - v_z^2) where:
            - V is local true airspeed
            - v_z is descent rate
        4. Decompose horizontal velocity using sideslip angle:
            - v_x = v_xy * cos(β)
            - v_y = v_xy * sin(β)
            where β is sideslip angle

    **Major Assumptions**
        * Linear true airspeed variation
        * Constant descent rate
        * Small angle approximations
        * Quasi-steady flight
        * No wind effects


    See Also
    --------
    RCAIDE.Framework.Mission.Segments
    """      
     
    # unpack User Inputs
    descent_rate = segment.descent_rate
    v0           = segment.air_speed_start
    vf           = segment.air_speed_end
    alt0         = segment.altitude_start 
    altf         = segment.altitude_end 
    beta         = segment.sideslip_angle
    t_nondim     = segment.state.numerics.dimensionless.control_points
    conditions   = segment.state.conditions  

    # check for initial velocity
    if v0 is None: 
        if not segment.state.initials: raise AttributeError('airspeed not set')
        v0 = np.linalg.norm(segment.state.initials.conditions.frames.inertial.velocity_vector[-1])
        
    # check for initial altitude
    if alt0 is None:
        if not segment.state.initials: raise AttributeError('initial altitude not set')
        alt0 = -1.0 *segment.state.initials.conditions.frames.inertial.position_vector[-1,2]

    # discretize on altitude
    alt = t_nondim * (altf-alt0) + alt0
    
    # process velocity vector
    v_xy_mag = (vf-v0)*t_nondim + v0
    v_z   = descent_rate # z points down
    v_xy_mag = np.sqrt(v_xy_mag**2 - v_z**2 )

    v_x         = np.cos(beta)*v_xy_mag
    v_y         = np.sin(beta)*v_xy_mag    
    
    # pack conditions    
    conditions.frames.inertial.velocity_vector[:,0] = v_x[:,0]
    conditions.frames.inertial.velocity_vector[:,1] = v_y[:,0]
    conditions.frames.inertial.velocity_vector[:,2] = v_z
    conditions.frames.inertial.position_vector[:,2] = -alt[:,0] # z points down
    conditions.freestream.altitude[:,0]             =  alt[:,0] # positive altitude in this context