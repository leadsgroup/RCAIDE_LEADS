# RCAIDE/Library/Missions/Segments/Climb/Constant_Mach_Linear_Altitude.py
# 
# 
# Created:  Jul 2023, M. Clarke

# ----------------------------------------------------------------------------------------------------------------------
#  IMPORT
# ----------------------------------------------------------------------------------------------------------------------
# RCAIDE  
from RCAIDE.Library.Mission.Common.Update.atmosphere import atmosphere

# Package imports  
import numpy as np
 
# ----------------------------------------------------------------------------------------------------------------------
#  Initialize Conditions
# ----------------------------------------------------------------------------------------------------------------------
def initialize_conditions(segment):
    """
    Initializes conditions for constant Mach climb with linear altitude change

    Parameters
    ----------
    segment : Segment
        The mission segment being analyzed

    Notes
    -----
    This function sets up the initial conditions for a climb segment with constant
    Mach number and linear altitude variation. The climb angle is determined by
    the distance and altitude change.

    **Required Segment Components**

    segment:
        - mach_number : float
            Mach number to maintain [-]
        - altitude_start : float
            Initial altitude [m]
        - altitude_end : float
            Final altitude [m]
        - distance : float
            Ground distance to cover [m]
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

    **Calculation Process**
        1. Calculate climb angle from altitude change and distance
        2. Get atmospheric properties for speed of sound
        3. Calculate true airspeed from Mach number
        4. Decompose velocity into components using:
            - Computed climb angle
            - Sideslip angle
            - Constant Mach requirement

    **Major Assumptions**
        * Constant Mach number
        * Linear altitude change
        * Standard atmosphere model
        * Small angle approximations
        * Quasi-steady flight

    Returns
    -------
    None
        Updates segment conditions directly:
        - conditions.frames.inertial.velocity_vector [m/s]
        - conditions.frames.inertial.position_vector [m]
        - conditions.freestream.altitude [m]

    See Also
    --------
    RCAIDE.Framework.Mission.Segments
    RCAIDE.Library.Mission.Common.Update.atmosphere
    """        
    
    # unpack
    alt0       = segment.altitude_start 
    altf       = segment.altitude_end
    xf         = segment.distance
    mach       = segment.mach_number
    beta       = segment.sideslip_angle
    conditions = segment.state.conditions  
    t_nondim   = segment.state.numerics.dimensionless.control_points
    
    # check for initial altitude
    if alt0 is None:
        if not segment.state.initials: raise AttributeError('altitude not set')
        alt0 = -1.0 * segment.state.initials.conditions.frames.inertial.position_vector[-1,2]
        
    # discretize on altitude
    alt = t_nondim * (altf-alt0) + alt0      
    segment.state.conditions.freestream.altitude[:,0] = alt[:,0]

    # check for initial velocity
    if mach is None: 
        if not segment.state.initials: raise AttributeError('mach not set')
        air_speed = np.linalg.norm(segment.state.initials.conditions.frames.inertial.velocity_vector[-1])   
    else:
        
        # Update freestream to get speed of sound
        atmosphere(segment)
        a          = conditions.freestream.speed_of_sound    
    
        # compute speed, constant with constant altitude
        air_speed    = mach * a   
        
    climb_angle  = np.arctan((altf-alt0)/xf)
    v_x          = np.cos(beta)*np.cos(climb_angle)*air_speed
    v_y          = np.sin(beta)*np.cos(climb_angle)*air_speed
    v_z          = np.sin(climb_angle)*air_speed 
    t_nondim     = segment.state.numerics.dimensionless.control_points
    
    # discretize on altitude
    alt = t_nondim * (altf-alt0) + alt0    
    
    # pack
    conditions.freestream.altitude[:,0]             = alt[:,0]
    conditions.frames.inertial.position_vector[:,2] = -alt[:,0] # z points down 
    conditions.frames.inertial.velocity_vector[:,0] = v_x[:,0]
    conditions.frames.inertial.velocity_vector[:,1] = v_y[:,0]
    conditions.frames.inertial.velocity_vector[:,2] = -v_z[:,0]      