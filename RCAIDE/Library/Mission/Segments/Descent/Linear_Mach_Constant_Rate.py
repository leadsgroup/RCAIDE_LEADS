 # RCAIDE/Library/Missions/Segments/Descent/Linear_Mach_Constant_Rate.py
# 
# 
# Created:  Jul 2023, M. Clarke 

# ----------------------------------------------------------------------------------------------------------------------  
#  IMPORT 
# ----------------------------------------------------------------------------------------------------------------------  
# RCAIDE imports  
from RCAIDE.Library.Mission.Common.Update.atmosphere import atmosphere

# pacakge imports 
import numpy as np

# ---------------------------------------------------------------------------------------------------------------------- 
#  Initialize Conditions
# ----------------------------------------------------------------------------------------------------------------------  
def initialize_conditions(segment):
    """
    Initializes conditions for linear Mach descent at fixed rate

    Parameters
    ----------
    segment : Segment
        The mission segment being analyzed
            - descent_rate : float
                Rate of descent [m/s]
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
                initials : Data, optional
                    Initial conditions from previous segment

    Returns
    -------
    None

    Notes
    -----
    This function sets up the initial conditions for a descent segment with linearly
    varying Mach number and constant descent rate. The true airspeed varies with both
    Mach number and local speed of sound. Updates segment with velocity vector, position
    vector, and altitude.

    **Calculation Process**
        1. Get atmospheric properties for speed of sound
        2. Discretize altitude profile
        3. Calculate Mach number variation:
        M = M0 + (Mf - M0)*t where:
            - M0 is initial Mach number
            - Mf is final Mach number
            - t is normalized time/distance
        4. Calculate true airspeed:
            V = M * a where:
                - M is local Mach number
                - a is local speed of sound
        5. Decompose velocity using:
            - Fixed descent rate
            - Sideslip angle
            - Computed true airspeed

    **Major Assumptions**
        * Linear Mach number variation
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
    M0           = segment.mach_number_start
    Mf           = segment.mach_number_end
    alt0         = segment.altitude_start 
    altf         = segment.altitude_end 
    beta         = segment.sideslip_angle    
    t_nondim     = segment.state.numerics.dimensionless.control_points
    conditions   = segment.state.conditions  

    # Update freestream to get speed of sound
    atmosphere(segment)
    a          = conditions.freestream.speed_of_sound        
    
    # check for initial altitude
    if alt0 is None:
        if not segment.state.initials: raise AttributeError('initial altitude not set')
        alt0 = -1.0 *segment.state.initials.conditions.frames.inertial.position_vector[-1,2]
    
    # check for initial velocity vector
    if M0 is None:
        if not segment.state.initials: raise AttributeError('initial mach number not set')
        M0  =  np.linalg.norm(segment.state.initials.conditions.frames.inertial.velocity_vector[-1,:])/a[0,:]         
        
    # discretize on altitude
    alt = t_nondim * (altf-alt0) + alt0
    conditions.freestream.altitude[:,0] =  alt[:,0]  # positive altitude 

    # process velocity vector
    mach_number = (Mf-M0)*t_nondim + M0
    v_xy_mag    = mach_number * a
    v_z         = descent_rate # z points down
    v_xy        = np.sqrt( v_xy_mag**2 - v_z**2 ) 
    v_x         = np.cos(beta)*v_xy
    v_y         = np.sin(beta)*v_xy
    
    # pack conditions    
    conditions.frames.inertial.velocity_vector[:,0] = v_x[:,0]
    conditions.frames.inertial.velocity_vector[:,1] = v_y[:,0]
    conditions.frames.inertial.velocity_vector[:,2] = v_z
    conditions.frames.inertial.position_vector[:,2] = -alt[:,0] # z points down
    conditions.freestream.altitude[:,0]             =  alt[:,0] # positive altitude t
