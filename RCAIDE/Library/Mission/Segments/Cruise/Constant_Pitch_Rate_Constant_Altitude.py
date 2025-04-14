# RCAIDE/Library/Missions/Segments/Cruise/Constant_Pitch_Rate_Constant_Altitude.py
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
    Initializes conditions for constant pitch rate maneuver at fixed altitude

    Parameters
    ----------
    segment : Segment
        The mission segment being analyzed
            - altitude : float
                Cruise altitude [m]
            - pitch_initial : float
                Initial pitch angle [rad]
            - pitch_final : float
                Final pitch angle [rad]
            - pitch_rate : float
                Constant pitch rate [rad/s]
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
            - conditions.frames.body.inertial_rotations [rad]
            - conditions.frames.inertial.position_vector [m]
            - conditions.freestream.altitude [m]
            - conditions.frames.inertial.time [s]
    
    Notes
    -----
    This function sets up the initial conditions for a cruise segment with constant
    pitch rate and constant altitude. The pitch angle varies linearly with time
    between initial and final values.
        
    **Calculation Process**
        1. Calculate time required for pitch change:
            t = (θf - θ0)/θ_dot where:
                - θf is final pitch angle
                - θ0 is initial pitch angle
                - θ_dot is pitch rate
        2. Discretize time points
        3. Calculate pitch angle profile:
            θ(t) = θ_dot * t + θ0

    **Major Assumptions**
        * Constant pitch rate
        * Constant altitude
        * Linear pitch angle variation
        * No coupling with other rotational axes
        * Quasi-steady flight

    See Also
    --------
    RCAIDE.Framework.Mission.Segments
    """       
    
    # unpack
    alt        = segment.altitude 
    T0         = segment.pitch_initial
    Tf         = segment.pitch_final 
    theta_dot  = segment.pitch_rate   
    conditions = segment.state.conditions 
    state      = segment.state
    
    # check for initial altitude
    if alt is None:
        if not segment.state.initials: raise AttributeError('altitude not set')
        alt = -1.0 * segment.state.initials.conditions.frames.inertial.position_vector[-1,2]
        
    # check for initial pitch
    if T0 is None:
        T0  =  state.initials.conditions.frames.body.inertial_rotations[-1,1]
        segment.pitch_initial = T0
    
    # dimensionalize time
    t_initial = conditions.frames.inertial.time[0,0]
    t_final   = (Tf-T0)/theta_dot + t_initial
    t_nondim  = state.numerics.dimensionless.control_points
    time      = t_nondim * (t_final-t_initial) + t_initial
    
    # set the body angle
    body_angle = theta_dot*time + T0
    segment.state.conditions.frames.body.inertial_rotations[:,1] = body_angle[:,0]    
    
    # pack
    segment.state.conditions.freestream.altitude[:,0]             = alt
    segment.state.conditions.frames.inertial.position_vector[:,2] = -alt # z points down
    segment.state.conditions.frames.inertial.time[:,0]            = time[:,0]
    
    