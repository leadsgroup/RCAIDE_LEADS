# RCAIDE/Library/Missions/Segments/Climb/Linear_Mach_Constant_Rate.py
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
    Initializes conditions for linear Mach number climb segment

    Parameters
    ----------
    segment : Segment
        The mission segment being analyzed

    Notes
    -----
    This function sets up the initial conditions for a climb segment with linearly
    varying Mach number and constant rate of climb. The Mach number varies linearly
    between the start and end values.

    **Required Segment Components**

    segment:
        - climb_rate : float
            Rate of climb [m/s]
        - mach_number_start : float
            Initial Mach number [-]
        - mach_number_end : float
            Final Mach number [-]
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
        - analyses:
            atmosphere : Model
                Atmospheric model for property calculations

    **Calculation Process**
        1. Discretize altitude profile
        2. Get atmospheric properties for speed of sound
        3. Calculate velocity from Mach number variation
        4. Decompose velocity into components using:
            - Climb rate constraint
            - Sideslip angle
            - Linear Mach profile

    **Major Assumptions**
        * Linear Mach number variation
        * Constant rate of climb
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
    climb_rate = segment.climb_rate
    M0         = segment.mach_number_start
    Mf         = segment.mach_number_end
    alt0       = segment.altitude_start 
    altf       = segment.altitude_end 
    beta       = segment.sideslip_angle
    t_nondim   = segment.state.numerics.dimensionless.control_points
    conditions = segment.state.conditions
    
    # check for initial altitude
    if alt0 is None:
        if not segment.state.initials: raise AttributeError('initial altitude not set')
        alt0 = -1.0 *segment.state.initials.conditions.frames.inertial.position_vector[-1,2]

    # discretize on altitude
    alt = t_nondim * (altf-alt0) + alt0
    conditions.freestream.altitude[:,0]             =  alt[:,0] # positive altitude in this context
    

    # Update freestream to get speed of sound
    atmosphere(segment)
    a          = conditions.freestream.speed_of_sound
    
    # check for initial velocity
    if M0 is None: 
        if not segment.state.initials: raise AttributeError('airspeed not set')
        M0 = np.linalg.norm(segment.state.initials.conditions.frames.inertial.velocity_vector[-1])/ a
         
    # process velocity vector
    mach_number = (Mf-M0)*t_nondim + M0
    v_mag       = mach_number * a
    v_z         = -climb_rate # z points down
    v_xy        = np.sqrt( v_mag**2 - v_z**2 )
    v_x         = np.cos(beta)*v_xy
    v_y         = np.sin(beta)*v_xy
    
    # pack conditions    
    conditions.frames.inertial.velocity_vector[:,0] = v_x[:,0]
    conditions.frames.inertial.velocity_vector[:,1] = v_y[:,0]
    conditions.frames.inertial.velocity_vector[:,2] = v_z
    conditions.frames.inertial.position_vector[:,2] = -alt[:,0] # z points down
    conditions.freestream.altitude[:,0]             =  alt[:,0] # positive altitude in this context
    