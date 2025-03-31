# RCAIDE/Library/Missions/Segments/Ground/Takeoff.py
# 
# 
# Created:  Jul 2023, M. Clarke  

# ----------------------------------------------------------------------------------------------------------------------
#  IMPORT
# ----------------------------------------------------------------------------------------------------------------------
# RCAIDE Imports  
import numpy as np 

# ----------------------------------------------------------------------------------------------------------------------
# unpack unknowns
# ---------------------------------------------------------------------------------------------------------------------- 
def initialize_conditions(segment):
    """
    Initializes conditions for aircraft takeoff ground roll

    Parameters
    ----------
    segment : Segment
        The mission segment being analyzed
            - altitude : float
                Ground altitude [m]
            - velocity_start : float
                Initial ground speed [m/s]
            - velocity_end : float
                Rotation speed [m/s]
            - ground_incline : float
                Runway incline angle [rad]
            - friction_coefficient : float
                Ground friction coefficient [-]
            - state:
                numerics:
                    dimensionless:
                        control_points : array
                            Discretization points [-]
                conditions : Data
                    State conditions container
                unknowns:
                    ground_velocity : array
                        Ground velocity profile [m/s]
                initials : Data, optional
                    Initial conditions from previous segment
            - analyses:
                weights:
                    vehicle:
                        mass_properties:
                            takeoff : float
                                Aircraft takeoff mass [kg]

    Returns
    -------
    None
        Updates segment conditions directly:
            - conditions.frames.inertial.velocity_vector [m/s]
            - conditions.ground.incline [rad]
            - conditions.ground.friction_coefficient [-]
            - conditions.freestream.altitude [m]
            - conditions.frames.inertial.position_vector [m]
            - conditions.weights.total_mass [kg]
    
    Notes
    -----
    This function sets up the initial conditions for a ground takeoff segment with
    acceleration from initial speed to rotation speed. The segment handles ground
    effects, friction, and incline during the takeoff roll.

    **Calculation Process**
        1. Check initial conditions
        2. Initialize velocity profile:
            v = v0 + (vf - v0)*t where:
                - v0 is initial speed
                - vf is rotation speed
                - t is normalized time/distance
        3. Set ground conditions:
            - Friction coefficient
            - Ground incline
        4. Set position and altitude
        5. Set aircraft mass

    **Major Assumptions**
        * Constant friction coefficient
        * Constant ground incline
        * Linear velocity increase
        * Quasi-steady acceleration
        * Small minimum velocity for numerical stability

    See Also
    --------
    RCAIDE.Framework.Mission.Segments
    """  

    # use the common initialization # unpack inputs
    alt      = segment.altitude 
    v0       = segment.velocity_start
    vf       = segment.velocity_end 
    
    # check for initial altitude
    if alt is None:
        if not segment.state.initials: raise AttributeError('altitude not set')
        alt = -1.0 *segment.state.initials.conditions.frames.inertial.position_vector[-1,2]   

    if v0  is None: 
        v0 = np.linalg.norm(segment.state.initials.conditions.frames.inertial.velocity_vector[-1])
        
    # avoid having zero velocity since aero and propulsion models need non-zero Reynolds number
    if v0 == 0.0: v0 = 0.01
    if vf == 0.0: vf = 0.01
    
    # intial and final speed cannot be the same
    if v0 == vf:
        vf = vf + 0.01
        
    # repack
    segment.air_speed_start = v0
    segment.air_speed_end   = vf
    
    initialized_velocity = (vf - v0)*segment.state.numerics.dimensionless.control_points + v0
    
    # Initialize the x velocity unknowns to speed convergence:
    segment.state.unknowns.ground_velocity = initialized_velocity[1:,0]    

    # pack conditions 
    conditions = segment.state.conditions    
    conditions.frames.inertial.velocity_vector[:,0] = initialized_velocity[:,0]
    conditions.ground.incline[:,0]                  = segment.ground_incline
    conditions.ground.friction_coefficient[:,0]     = segment.friction_coefficient   
    conditions.freestream.altitude[:,0]             = alt
    conditions.frames.inertial.position_vector[:,2] = -alt   
    conditions.weights.total_mass[:,0]              = segment.analyses.weights.vehicle.mass_properties.takeoff
    conditions.frames.inertial.position_vector[:,:] = conditions.frames.inertial.position_vector[0,:][None,:][:,:]