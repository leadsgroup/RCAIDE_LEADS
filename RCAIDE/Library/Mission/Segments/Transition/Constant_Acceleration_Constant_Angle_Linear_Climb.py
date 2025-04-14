# RCAIDE/Library/Missions/Segments/Transition/Constant_Acceleration_Constant_Angle_Linear_Climb.py
# 
# 
# Created:  Jul 2023, M. Clarke 
 
# ----------------------------------------------------------------------------------------------------------------------  
#  Initialize Conditions
# ----------------------------------------------------------------------------------------------------------------------  

import numpy as np

# ----------------------------------------------------------------------------------------------------------------------  
#  Initialize Conditions
# ----------------------------------------------------------------------------------------------------------------------   
def initialize_conditions(segment):
    """
    Initializes conditions for transition segment with constant acceleration and climb angle

    Parameters
    ----------
    segment : Segment
        The mission segment being analyzed
            - altitude_start : float
                Initial altitude [m]
            - altitude_end : float
                Final altitude [m]
            - climb_angle : float
                Fixed climb angle [rad]
            - air_speed_start : float
                Initial true airspeed [m/s]
            - acceleration : float
                Constant acceleration [m/s^2]
            - sideslip_angle : float
                Aircraft sideslip angle [rad]
            - pitch_initial : float
                Initial pitch angle [rad]
            - pitch_final : float
                Final pitch angle [rad]
            - state:
                numerics:
                    dimensionless:
                        control_points : array
                            Discretization points [-]
                conditions : Data
                    State conditions container
                initials : Data, optional
                    Initial conditions from previous segment
    
    Returns
    -------
    None
        Updates segment conditions directly:
            - conditions.freestream.altitude [m]
            - conditions.frames.inertial.position_vector [m]
            - conditions.frames.inertial.velocity_vector [m/s]
            - conditions.frames.body.inertial_rotations [rad]
            - conditions.frames.inertial.time [s]

    Notes
    -----
    This function sets up the initial conditions for a transition segment with constant
    acceleration, constant climb angle, and linear pitch variation. The segment handles
    the transition between different flight phases while climbing.

    **Calculation Process**
        1. Check required inputs
        2. Calculate trajectory geometry:
                - Ground distance = (alt_f - alt_0)/tan(γ)
                - True distance = sqrt((alt_f - alt_0)^2 + ground_distance^2)
        where γ is climb angle
        3. Calculate time required:
                t = (-V0 + sqrt(V0^2 + 2ax))/ax
            where:
                - V0 is initial velocity
                - ax is acceleration
        4. Compute velocity components:
            - vx = V*cos(β)*cos(γ)
            - vy = V*sin(β)*cos(γ)
            - vz = V*sin(γ)
        5. Linear pitch transition

    **Major Assumptions**
        * Constant acceleration
        * Constant climb angle
        * Linear pitch variation
        * Coordinated flight
        * Small angle approximations

    See Also
    --------
    RCAIDE.Framework.Mission.Segments
    """      
    
    # unpack
    alt0        = segment.altitude_start 
    altf        = segment.altitude_end 
    climb_angle = segment.climb_angle
    v0          = segment.air_speed_start 
    beta        = segment.sideslip_angle    
    ax          = segment.acceleration   
    T0          = segment.pitch_initial
    Tf          = segment.pitch_final     
    t_nondim    = segment.state.numerics.dimensionless.control_points
    conditions  = segment.state.conditions 
    
    # check for climb angle     
    if climb_angle is None:
        raise AttributeError('set climb')
    
    if ax is None: 
        raise AttributeError('set acceleration') 
    
    # check for initial and final altitude 
    if alt0 is None:
        if not segment.state.initials: raise AttributeError('altitude not set')
        alt0 = -1.0 * segment.state.initials.conditions.frames.inertial.position_vector[-1,2] 
    
    if altf is None:
        raise AttributeError('final altitude not set')
        
    # check for initial pitch
    if T0 is None:
        T0  =  segment.state.initials.conditions.frames.body.inertial_rotations[-1,1] 
        
    # check for initial velocity vector
    if v0 is None:
        if not segment.state.initials: raise AttributeError('initial airspeed not set')
        v0  =  segment.state.initials.conditions.frames.inertial.velocity_vector[-1,:] 
        segment.velocity_vector = v0
    
    elif len(np.shape(v0)) == 0:
        v0 = np.array([v0, 0, 0]) 
         
    # discretize on altitude
    v0_mag          = np.linalg.norm(v0)
    alt             = t_nondim * (altf-alt0) + alt0   
    ground_distance = (altf-alt0)/np.tan(climb_angle)
    true_distance   = np.sqrt((altf-alt0)**2 + ground_distance**2)
    t_initial       = conditions.frames.inertial.time[0,0]    
    elapsed_time    = (-v0_mag + np.sqrt(v0_mag**2 + 2*ax*true_distance))/(ax)   
    vf_mag          = v0_mag + ax*(elapsed_time)   
    
    # dimensionalize time        
    t_final   = t_initial + elapsed_time
    t_nondim  = segment.state.numerics.dimensionless.control_points
    time      = t_nondim * (t_final-t_initial)
    
    # Figure out vx
    V  = (vf_mag-v0_mag) 
    vx = t_nondim *  V  * np.cos(beta) * np.cos(climb_angle) + v0[0] * np.cos(climb_angle) 
    vy = t_nondim *  V  * np.sin(beta) * np.cos(climb_angle) + v0[1] * np.cos(climb_angle) 
    vz = t_nondim *  V  * np.sin(climb_angle) + v0[2] * np.sin(climb_angle)  
    
    # set the body angle
    body_angle = time*(Tf-T0)/(t_final-t_initial) + T0
    segment.state.conditions.frames.body.inertial_rotations[:,1] = body_angle[:,0]     
    
    # pack
    segment.state.conditions.freestream.altitude[:,0]             = alt[:,0]
    segment.state.conditions.frames.inertial.position_vector[:,2] = -alt[:,0] # z points down
    segment.state.conditions.frames.inertial.velocity_vector[:,0] = vx[:,0] 
    segment.state.conditions.frames.inertial.velocity_vector[:,1] = vy[:,0] 
    segment.state.conditions.frames.inertial.velocity_vector[:,2] = -vz[:,0] 
    segment.state.conditions.frames.inertial.time[:,0]            = time[:,0]