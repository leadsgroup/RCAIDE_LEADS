# RCAIDE/Library/Missions/Segments/Climb/Constant_EAS_Constant_Rate.py
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
    Initializes conditions for constant equivalent airspeed climb segment

    Parameters
    ----------
    segment : Segment
        The mission segment being analyzed

    Notes
    -----
    This function sets up the initial conditions for a climb segment with constant
    equivalent airspeed (EAS) and constant rate of climb. It handles the conversion
    between EAS and true airspeed accounting for density variations with altitude.

    **Required Segment Components**

    segment:
        - climb_rate : float
            Rate of climb [m/s]
        - equivalent_air_speed : float
            Equivalent airspeed [m/s]
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

    **Conversion Process**
        1. Compute atmospheric properties at altitude
        2. Convert EAS to true airspeed (TAS) using density ratio
        3. Decompose TAS into velocity components

    **Major Assumptions**
        * Constant equivalent airspeed
        * Constant rate of climb
        * Standard atmosphere model
        * Small angle approximations
        * Incompressible flow

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
    eas        = segment.equivalent_air_speed    
    beta       = segment.sideslip_angle
    alt0       = segment.altitude_start 
    altf       = segment.altitude_end
    t_nondim   = segment.state.numerics.dimensionless.control_points
    conditions = segment.state.conditions  

    # check for initial altitude
    if alt0 is None:
        if not segment.state.initials: raise AttributeError('initial altitude not set')
        alt0 = -1.0 * segment.state.initials.conditions.frames.inertial.position_vector[-1,2]

    # discretize on altitude
    alt = t_nondim * (altf-alt0) + alt0
    
    # pack conditions
    conditions.freestream.altitude[:,0]             =  alt[:,0] # positive altitude in this context


    # check for initial velocity vector
    if eas is None:
        if not segment.state.initials: raise AttributeError('initial equivalent airspeed not set')
        air_speed  = np.linalg.norm(segment.state.initials.conditions.frames.inertial.velocity_vector[-1,:])  
    else: 
        # determine airspeed from equivalent airspeed
        atmosphere(segment) # get density for airspeed
        density   = conditions.freestream.density[:,0]   
        MSL_data  = segment.analyses.atmosphere.compute_values(0.0,0.0)
        air_speed = eas/np.sqrt(density/MSL_data.density[0])    
    
    # process velocity vector
    v_mag  = air_speed
    v_z    = -climb_rate # z points down
    v_xy   = np.sqrt( v_mag**2 - v_z**2 )
    v_x    = np.cos(beta)*v_xy
    v_y    = np.sin(beta)*v_xy
    
    # pack conditions    
    conditions.frames.inertial.velocity_vector[:,0] = v_x
    conditions.frames.inertial.velocity_vector[:,1] = v_y
    conditions.frames.inertial.velocity_vector[:,2] = v_z
    conditions.frames.inertial.position_vector[:,2] = -alt[:,0] # z points down