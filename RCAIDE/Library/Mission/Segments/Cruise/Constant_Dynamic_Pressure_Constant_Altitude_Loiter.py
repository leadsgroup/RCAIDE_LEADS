# RCAIDE/Library/Missions/Segments/Cruise/Constant_Dynamic_Pressure_Constant_Altitude_Loiter.py
# 
# 
# Created:  Jul 2023, M. Clarke

# ----------------------------------------------------------------------------------------------------------------------
#  IMPORT
# ----------------------------------------------------------------------------------------------------------------------
# RCAIDE 
import RCAIDE 

# Package imports  
import numpy as np
 
# ----------------------------------------------------------------------------------------------------------------------
#  Initialize Conditions
# ----------------------------------------------------------------------------------------------------------------------
def initialize_conditions(segment):
    """
    Initializes conditions for constant dynamic pressure loiter at fixed altitude

    Parameters
    ----------
    segment : Segment
        The mission segment being analyzed
            - altitude : float
                Loiter altitude [m]
            - time : float
                Loiter duration [s]
            - dynamic_pressure : float
                Dynamic pressure to maintain [Pa]
            - sideslip_angle : float
                Aircraft sideslip angle [rad]
            - state:
                numerics.dimensionless.control_points : array
                    Discretization points [-]
                conditions : Data
                    State conditions container
            - analyses:
                atmosphere : Model
                    Atmospheric model for property calculations

    Returns
    -------
    None
        Updates segment conditions directly:
            - conditions.frames.inertial.velocity_vector [m/s]
            - conditions.frames.inertial.position_vector [m]
            - conditions.freestream.altitude [m]
            - conditions.frames.inertial.time [s]

                    
    Notes
    -----
    This function sets up the initial conditions for a loiter segment with constant
    dynamic pressure and constant altitude. The airspeed is determined from the
    dynamic pressure constraint.

    **Calculation Process**
        1. Get atmospheric properties at altitude
        2. Calculate true airspeed from dynamic pressure:
            V = sqrt(2q/ρ) where:
                - q is dynamic pressure
                - ρ is air density
        3. Discretize time points
        4. Decompose velocity into components using sideslip angle

    **Major Assumptions**
        * Constant dynamic pressure
        * Constant altitude
        * Standard atmosphere model
        * Small angle approximations
        * Quasi-steady flight
        * No wind effects

    See Also
    --------
    RCAIDE.Framework.Mission.Segments
    RCAIDE.Library.Mission.Common.Update.atmosphere
    """   
    
    # unpack
    alt        = segment.altitude
    final_time = segment.time
    q          = segment.dynamic_pressure
    beta       = segment.sideslip_angle
    conditions = segment.state.conditions   
    
    # Update freestream to get density 
    RCAIDE.Library.Mission.Common.Update.atmosphere(segment) 
    rho        = conditions.freestream.density[:,0]   
    
    # check for initial altitude
    if alt is None:
        if not segment.state.initials: raise AttributeError('altitude not set')
        alt = -1.0 * segment.state.initials.conditions.frames.inertial.position_vector[-1,2]       
    
    # check for initial velocity
    if q is None: 
        if not segment.state.initials: raise AttributeError('dynamic pressure not set')
        air_speed = np.linalg.norm(segment.state.initials.conditions.frames.inertial.velocity_vector[-1])
        
    else: # compute speed, constant with constant altitude
        air_speed = np.sqrt(q/(rho*0.5))
    
    # dimensionalize time
    t_initial = conditions.frames.inertial.time[0,0]
    t_final   = final_time + t_initial
    t_nondim  = segment.state.numerics.dimensionless.control_points
    time      = t_nondim * (t_final-t_initial) + t_initial
    v_x       = np.cos(beta)*air_speed 
    v_y       = np.sin(beta)*air_speed 
    
    # pack
    segment.state.conditions.freestream.altitude[:,0]             = alt
    segment.state.conditions.frames.inertial.position_vector[:,2] = -alt # z points down
    segment.state.conditions.frames.inertial.velocity_vector[:,0] = v_x
    segment.state.conditions.frames.inertial.velocity_vector[:,1] = v_y
    segment.state.conditions.frames.inertial.time[:,0]            = time[:,0]