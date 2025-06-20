# RCAIDE/Library/Missions/Segments/Cruise/Curved_Constant_Radius_Constant_Speed_Constant_Altitude/initialize_conditions.py
# 
# 
# Created:  September 2024, A. Molloy + M. Clarke

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
    Initializes conditions for constant radius curved flight at fixed altitude and speed.

    Parameters
    ----------
    segment : Segment
        The mission segment being analyzed
            - altitude : float
                Cruise altitude [m]
            - air_speed : float
                True airspeed to maintain [m/s]
            - true_course : float
                Initial true course angle [deg]
            - turn_angle : float
                Total turn angle to execute [deg]
                Positive for right turn, negative for left turn
            - turn_radius : float
                Radius of the turn [m]
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
            - conditions.frames.body.velocity_vector [m/s]
            - conditions.freestream.altitude [m]
            - conditions.frames.inertial.time [s]
            - conditions.frames.planet.true_heading [rad]
            - conditions.frames.planet.true_course [rad]
    
    Notes
    -----
    This function sets up the initial conditions for a coordinated turn segment with 
    constant radius, constant speed, and constant altitude. The turn direction and 
    magnitude are specified by the turn angle.

    **Calculation Process**
    1. Check initial conditions
    2. Calculate turn rate from speed and radius:
        ω = V/R where:
            - V is true airspeed
            - R is turn radius
    3. Calculate time required for turn:
        t = |θ|/ω where:
            - θ is turn angle
            - ω is turn rate
    4. Discretize time points
    5. Calculate true course progression
    6. Decompose velocity into inertial and body components

    **Major Assumptions**
        * Coordinated turn (true course aligned with true heading)
        * Constant true airspeed
        * Constant altitude
        * Constant turn radius
        * Small angle approximations
        * Quasi-steady flight

    See Also
    --------
    RCAIDE.Framework.Mission.Segments
    """        
    
    # unpack 
    alt               = segment.altitude
    air_speed         = segment.air_speed       
    beta              = segment.sideslip_angle
    radius            = segment.turn_radius
    start_true_course = segment.true_course
    arc_sector        = segment.turn_angle
    conditions        = segment.state.conditions 

    # check for initial velocity
    if air_speed is None: 
        if not segment.state.initials: raise AttributeError('airspeed not set')
        air_speed = np.linalg.norm(segment.state.initials.conditions.frames.inertial.velocity_vector[-1])
        
    # check for initial altitude
    if alt is None:
        if not segment.state.initials: raise AttributeError('altitude not set')
        alt = -1.0 * segment.state.initials.conditions.frames.inertial.position_vector[-1,2]
    
    # check for turn radius
    if radius is None:
        if not segment.state.initials: raise AttributeError('radius not set')
        radius = 0.1 # minimum radius so as to approximate a near instantaneous curve. This will likely break the dynamics.
    
    # check for turn angle
    if arc_sector is None:
        if not segment.state.initials: raise AttributeError('turn angle not set')
        arc_sector = 0.0 # aircraft does not turn    

    # dimensionalize time
    v_body_x    = np.cos(beta)*air_speed # x-velocity in the body frame. 
    v_body_y    = np.sin(beta)*air_speed # y-velocity in the body frame
    t_initial   = conditions.frames.inertial.time[0,0]
    omega       = v_body_x / radius
    t_final     = abs(arc_sector) / omega + t_initial  # Time to complete the turn
    t_nondim    = segment.state.numerics.dimensionless.control_points
    time        = t_nondim * (t_final-t_initial) + t_initial
    
    true_course_control_points = start_true_course + t_nondim * arc_sector
    
    v_inertial_x = air_speed * np.cos(true_course_control_points)
    v_inertial_y = air_speed * np.sin(true_course_control_points)
    
    # pack
    segment.state.conditions.freestream.altitude[:,0]                 = alt
    segment.state.conditions.frames.inertial.position_vector[:,2]     = -alt # z points down
    segment.state.conditions.frames.inertial.velocity_vector[:,0]     = v_inertial_x[:,0]
    segment.state.conditions.frames.inertial.velocity_vector[:,1]     = v_inertial_y[:,0]
    #segment.state.conditions.frames.inertial.acceleration_vector[:,0] = -(air_speed ** 2) / radius * np.sin(true_course_control_points)[:,0] # Check sign
    #segment.state.conditions.frames.inertial.acceleration_vector[:,1] = (air_speed ** 2) / radius * np.cos(true_course_control_points)[:,0] # Check sign
    segment.state.conditions.frames.body.velocity_vector[:,0]         = v_body_x
    segment.state.conditions.frames.body.velocity_vector[:,1]         = v_body_y
    #segment.state.conditions.frames.body.acceleration_vector[:,1]     = (air_speed ** 2) / radius
    segment.state.conditions.frames.inertial.time[:,0]                = time[:,0]
    segment.state.conditions.frames.planet.true_heading[:,0]          = true_course_control_points[:,0]
    segment.state.conditions.frames.planet.true_course[:,0]           = true_course_control_points[:,0]