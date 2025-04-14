# RCAIDE/Library/Missions/Segments/Transition/Constant_Acceleration_Constant_Pitchrate_Constant_Altitude.py
# 
# 
# Created:  Jul 2023, M. Clarke 
  
# ----------------------------------------------------------------------------------------------------------------------  
#  Initialize Conditions
# ----------------------------------------------------------------------------------------------------------------------   
# Package imports 
import numpy as np

def initialize_conditions(segment):
    """
    Initializes conditions for transition segment with constant acceleration and pitch rate

    Parameters
    ----------
    segment : Segment
        The mission segment being analyzed
            - altitude : float
                Fixed flight altitude [m]
            - air_speed_start : float
                Initial true airspeed [m/s]
            - air_speed_end : float
                Final true airspeed [m/s]
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
    acceleration and pitch rate at fixed altitude. The segment handles transitions
    between different flight phases while maintaining altitude.

    **Calculation Process**
        1. Check initial conditions
        2. Calculate time required based on acceleration:
            t = (Vf - V0)/ax where:
                - V0 is initial airspeed
                - Vf is final airspeed
                - ax is acceleration
        3. Calculate velocity magnitude profile:
        V = V0 + ax*t
        4. Decompose velocity using sideslip:
            - v_x = V*cos(β)
            - v_y = V*sin(β)
        where β is sideslip angle
        5. Linear pitch transition:
            θ = θ0 + (θf - θ0)*t/tf for θf > θ0
            θ = θ0 - (θ0 - θf)*t/tf for θf < θ0

    **Major Assumptions**
        * Constant acceleration
        * Constant pitch rate
        * Fixed altitude
        * Linear pitch variation
        * Coordinated flight
        * Small angle approximations

    See Also
    --------
    RCAIDE.Framework.Mission.Segments
    """      
    
    # unpack
    alt = segment.altitude 
    v0  = segment.air_speed_start
    vf  = segment.air_speed_end  
    beta= segment.sideslip_angle
    ax  = segment.acceleration   
    T0  = segment.pitch_initial
    Tf  = segment.pitch_final     
    
    # check for initial altitude
    if alt is None:
        if not segment.state.initials: raise AttributeError('altitude not set')
        alt = -1.0 * segment.state.initials.conditions.frames.inertial.position_vector[-1,2]
        segment.altitude = alt
        
    # check for initial pitch
    if T0 is None:
        T0  =  segment.state.initials.conditions.frames.body.inertial_rotations[-1,1]
        segment.pitch_initial = T0    
    
    # check for initial velocity vector
    if v0 is None:
        if not segment.state.initials: raise AttributeError('initial airspeed not set')
        v0  =  segment.state.initials.conditions.frames.inertial.velocity_vector[-1,0] # x direction velocity         

    # dimensionalize time
    t_initial = segment.state.conditions.frames.inertial.time[0,0]
    t_final   = (vf-v0)/ax + t_initial
    t_nondim  = segment.state.numerics.dimensionless.control_points
    time      = t_nondim * (t_final-t_initial) + t_initial
    
    # Figure out vx
    v_mag = v0+(time - t_initial)*ax
    v_x   = np.cos(beta)*v_mag
    v_y   = np.sin(beta)*v_mag
    
    # set the body angle
    if Tf > T0:
        body_angle = T0 + time*(Tf-T0)/(t_final-t_initial)
    else:
        body_angle = T0 - time*(T0-Tf)/(t_final-t_initial)
    segment.state.conditions.frames.body.inertial_rotations[:,1] = body_angle[:,0]     
    
    # pack
    segment.state.conditions.freestream.altitude[:,0] = alt
    segment.state.conditions.frames.inertial.position_vector[:,2] = -alt # z points down
    segment.state.conditions.frames.inertial.velocity_vector[:,0] = v_x[:,0]
    segment.state.conditions.frames.inertial.velocity_vector[:,1] = v_y[:,0]
    segment.state.conditions.frames.inertial.time[:,0] = time[:,0]
