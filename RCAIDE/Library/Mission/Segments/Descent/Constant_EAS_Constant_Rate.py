# RCAIDE/Library/Missions/Segments/Descent/Constant_EAS_Constant_Rate.py
# 
# 
# Created:  Jul 2023, M. Clarke 

# ----------------------------------------------------------------------------------------------------------------------  
#  IMPORT 
# ----------------------------------------------------------------------------------------------------------------------  
# RCAIDE imports 
import RCAIDE

# pacakge imports 
import numpy as np

# ----------------------------------------------------------------------------------------------------------------------  
#  Initialize Conditions
# ----------------------------------------------------------------------------------------------------------------------  
def initialize_conditions(segment):
    """
    Initializes conditions for constant equivalent airspeed descent at fixed rate

    Parameters
    ----------
    segment : Segment
        The mission segment being analyzed
            - descent_rate : float
                Rate of descent [m/s]
            - equivalent_air_speed : float
                Equivalent airspeed to maintain [m/s]
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
    
    Returns
    -------
    None

    Notes
    -----
    This function sets up the initial conditions for a descent segment with constant
    equivalent airspeed (EAS) and constant descent rate. It handles the conversion
    between EAS and true airspeed accounting for density variations with altitude.
    Updates segment conditions directly with velocity_vector [m/s], altitude [m],
    and position_vector [m].

    **Calculation Process**
        1. Discretize altitude profile
        2. Get atmospheric properties at each altitude
        3. Convert EAS to true airspeed:
            V = EAS/sqrt(ρ/ρ0) where:
                - ρ is local density
                - ρ0 is sea level density
        4. Decompose velocity into components using:
            - Fixed descent rate
            - Sideslip angle
            - Computed true airspeed

    **Major Assumptions**
        * Constant equivalent airspeed
        * Constant descent rate
        * Standard atmosphere model
        * Small angle approximations
        * Quasi-steady flight

    See Also
    --------
    RCAIDE.Framework.Mission.Segments
    RCAIDE.Library.Mission.Common.Update.atmosphere
    """       
    
    # unpack
    descent_rate = segment.descent_rate
    eas          = segment.equivalent_air_speed   
    alt0         = segment.altitude_start 
    altf         = segment.altitude_end 
    beta         = segment.sideslip_angle
    t_nondim     = segment.state.numerics.dimensionless.control_points
    conditions   = segment.state.conditions  

    # check for initial altitude
    if alt0 is None:
        if not segment.state.initials: raise AttributeError('initial altitude not set')
        alt0 = -1.0 * segment.state.initials.conditions.frames.inertial.position_vector[-1,2]


    # discretize on altitude
    alt = t_nondim * (altf-alt0) + alt0
    
    # Pack altitude before updating atmosphere
    conditions.freestream.altitude[:,0]             =  alt[:,0] # positive altitude in this context
    
    # determine airspeed from equivalent airspeed
    RCAIDE.Library.Mission.Common.Update.atmosphere(segment) # get density for airspeed
    density   = conditions.freestream.density[:,0]   
    MSL_data  = segment.analyses.atmosphere.compute_values(0.0,0.0)

    # check for initial velocity vector
    if eas is None:
        if not segment.state.initials: raise AttributeError('initial equivalent airspeed not set')
        air_speed  = np.linalg.norm(segment.state.initials.conditions.frames.inertial.velocity_vector[-1,:])  
    else: 
        air_speed  = eas/np.sqrt(density/MSL_data.density[0])    
    
    # process velocity vector
    v_mag = air_speed
    v_z   = descent_rate # z points down
    v_xy  = np.sqrt( v_mag**2 - v_z**2 )
    v_x   = np.cos(beta)*v_xy
    v_y   = np.sin(beta)*v_xy
    
    # pack conditions    
    conditions.frames.inertial.velocity_vector[:,0] = v_x
    conditions.frames.inertial.velocity_vector[:,1] = v_y
    conditions.frames.inertial.velocity_vector[:,2] = v_z
    conditions.frames.inertial.position_vector[:,2] = -alt[:,0] # z points down
