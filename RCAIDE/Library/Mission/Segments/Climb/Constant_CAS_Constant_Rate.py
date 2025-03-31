# RCAIDE/Library/Missions/Segments/Climb/Constant_CAS_Constant_Rate.py
# 
# 
# Created:  Jul 2023, M. Clarke

# ----------------------------------------------------------------------------------------------------------------------
#  IMPORT
# ----------------------------------------------------------------------------------------------------------------------
# RCAIDE 
from RCAIDE.Library.Mission.Common.Update.atmosphere import atmosphere
from RCAIDE.Framework.Core import Units

# Package imports  
import numpy as np
 
# ----------------------------------------------------------------------------------------------------------------------
#  Initialize Conditions
# ----------------------------------------------------------------------------------------------------------------------
def initialize_conditions(segment):
    """
    Initializes conditions for constant calibrated airspeed climb segment

    Parameters
    ----------
    segment : Segment
        The mission segment being analyzed

    Notes
    -----
    This function sets up the initial conditions for a climb segment with constant
    calibrated airspeed (CAS) and constant rate of climb. It handles the conversion
    between CAS and true airspeed accounting for atmospheric effects. Also updates segment 
    conditions with velocity vector, position vector, and altitude. 

    **Required Segment Components**

    segment:
        - climb_rate : float
            Rate of climb [m/s]
        - calibrated_air_speed : float
            Calibrated airspeed [m/s]
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
        2. Convert CAS to equivalent airspeed (EAS)
        3. Convert EAS to true airspeed (TAS)
        4. Decompose TAS into velocity components

    **Major Assumptions**
        * Constant calibrated airspeed
        * Constant rate of climb
        * Standard atmosphere model
        * Small angle approximations
        * Subsonic flow

    Returns
    -------
    None
        
    See Also
    --------
    RCAIDE.Framework.Mission.Segments
    RCAIDE.Library.Mission.Common.Update.atmosphere
    """         
    
    # unpack
    climb_rate = segment.climb_rate
    cas        = segment.calibrated_air_speed   
    alt0       = segment.altitude_start 
    altf       = segment.altitude_end
    beta       = segment.sideslip_angle
    t_nondim   = segment.state.numerics.dimensionless.control_points
    conditions = segment.state.conditions  

    # check for initial altitude
    if alt0 is None:
        if not segment.state.initials: raise AttributeError('initial altitude not set')
        alt0 = -1.0 * segment.state.initials.conditions.frames.inertial.position_vector[-1,2]

    # discretize on altitude
    alt = t_nondim * (altf-alt0) + alt0
    
    # pack conditions
    conditions.freestream.altitude[:,0] = alt[:,0] # positive altitude in this context
    

    if cas is None:
        if not segment.state.initials: raise AttributeError('initial equivalent airspeed not set')
        v_mag =  np.linalg.norm(segment.state.initials.conditions.frames.inertial.velocity_vector[-1,:])    
    else:  
        
        # determine airspeed from calibrated airspeed
        atmosphere(segment) # get density for airspeed
        density  = conditions.freestream.density[:,0]  
        pressure = conditions.freestream.pressure[:,0] 
    
        MSL_data  = segment.analyses.atmosphere.compute_values(0.0,0.0)
        pressure0 = MSL_data.pressure[0]
    
        kcas  = cas / Units.knots
        delta = pressure / pressure0 
    
        mach = 2.236*((((1+4.575e-7*kcas**2)**3.5-1)/delta + 1)**0.2857 - 1)**0.5
    
        qc  = pressure * ((1+0.2*mach**2)**3.5 - 1)
        eas = cas * (pressure/pressure0)**0.5*(((qc/pressure+1)**0.286-1)/((qc/pressure0+1)**0.286-1))**0.5
        
        v_mag = eas/np.sqrt(density/MSL_data.density[0])    
    
    # process velocity vector 
    v_z   = -climb_rate # z points down
    v_xy  = np.sqrt( v_mag**2 - v_z**2 )
    v_x   = np.cos(beta)*v_xy 
    v_y   = np.sin(beta)*v_xy 
    
    # pack conditions    
    conditions.frames.inertial.velocity_vector[:,0] = v_x
    conditions.frames.inertial.velocity_vector[:,1] = v_y
    conditions.frames.inertial.velocity_vector[:,2] = v_z
    conditions.frames.inertial.position_vector[:,2] = -alt[:,0] # z points down