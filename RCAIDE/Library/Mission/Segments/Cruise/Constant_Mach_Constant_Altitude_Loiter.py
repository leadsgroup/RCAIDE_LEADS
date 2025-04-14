# RCAIDE/Library/Missions/Segments/Cruise/Constant_Mach_Constant_Altitude_Loiter.py
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
    Initializes conditions for constant Mach loiter at fixed altitude

    Parameters
    ----------
    segment : Segment
        The mission segment being analyzed
            - altitude : float
                Loiter altitude [m]
            - time : float
                Loiter duration [s]
            - mach_number : float
                Mach number to maintain [-]
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
    Mach number and constant altitude. The true airspeed is determined from the
    Mach number and local speed of sound.      

    **Calculation Process**
        1. Get atmospheric properties at altitude
        2. Calculate true airspeed from Mach number:
            V = M * a where:
                - M is Mach number
                - a is speed of sound
        3. Discretize time points
        4. Decompose velocity into components using sideslip angle

    **Major Assumptions**
        * Constant Mach number
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
    mach       = segment.mach_number
    beta       = segment.sideslip_angle
    conditions = segment.state.conditions   
    
    # check for initial altitude
    if alt is None:
        if not segment.state.initials: raise AttributeError('altitude not set')
        alt = -1.0 * segment.state.initials.conditions.frames.inertial.position_vector[-1,2]  
    segment.state.conditions.freestream.altitude[:,0] = alt
        
    # Update freestream to get speed of sound
    RCAIDE.Library.Mission.Common.Update.atmosphere(segment)  
    a          = conditions.freestream.speed_of_sound         
    
    # check for initial velocity
    if mach is None: 
        if not segment.state.initials: raise AttributeError('airspeed not set')
        air_speed = np.linalg.norm(segment.state.initials.conditions.frames.inertial.velocity_vector[-1])
        
    else: # compute speed, constant with constant altitude
        air_speed = mach * a
    
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
    segment.state.conditions.frames.inertial.velocity_vector[:,0] = v_x[:,0]
    segment.state.conditions.frames.inertial.velocity_vector[:,1] = v_y[:,0]
    segment.state.conditions.frames.inertial.time[:,0]            = time[:,0]
    
