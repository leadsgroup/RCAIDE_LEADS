# RCAIDE/Library/Missions/Segments/Hover/Hover.py
# 
# 
# Created:  Jul 2023, M. Clarke 
 
# ----------------------------------------------------------------------------------------------------------------------  
#  Initialize Conditions
# ----------------------------------------------------------------------------------------------------------------------  

def initialize_conditions(segment):
    """"
    Initializes conditions for hover segment at fixed altitude

    Parameters
    ----------
    segment : Segment
        The mission segment being analyzed
            - altitude : float
                Hover altitude [m]
            - time : float
                Hover duration [s]
            - state:
                numerics:
                    dimensionless:
                        control_points : array
                            Discretization points [-]
                conditions:
                    frames:
                        inertial:
                            time : array
                                Time points [s]
                initials : Data, optional
                    Initial conditions from previous segment
    
    Returns
    -------
    None
        Updates segment conditions directly:
            - conditions.freestream.altitude [m]
            - conditions.frames.inertial.position_vector [m]
            - conditions.frames.inertial.velocity_vector [m/s]
            - conditions.frames.inertial.time [s]

    Notes
    -----
    This function sets up the initial conditions for a hover segment with fixed
    altitude and duration. The segment represents stationary flight with no
    translational velocity components.

    **Calculation Process**
        1. Check initial altitude
        2. Scale time points over hover duration:
            t = t_norm * duration + t_initial
        3. Set velocity components to zero:
            - v_x = 0
            - v_y = 0
            - v_z = 0
        4. Set position at hover altitude

    **Major Assumptions**
        * Perfect hover (no drift)
        * Fixed altitude
        * No translational velocity
        * No atmospheric variations
        * Quasi-steady state

    See Also
    --------
    RCAIDE.Framework.Mission.Segments
    """       
    
    # unpack
    alt        = segment.altitude
    duration   = segment.time
    conditions = segment.state.conditions   
    
    
    # check for initial altitude
    if alt is None:
        if not segment.state.initials: raise AttributeError('altitude not set')
        alt = -1.0 *segment.state.initials.conditions.frames.inertial.position_vector[-1,2]      
    
    # dimensionalize time
    t_initial = conditions.frames.inertial.time[0,0]
    t_nondim  = segment.state.numerics.dimensionless.control_points
    time      =  t_nondim * (duration) + t_initial
    
    # pack
    segment.state.conditions.freestream.altitude[:,0]             = alt
    segment.state.conditions.frames.inertial.position_vector[:,2] = -alt # z points down
    segment.state.conditions.frames.inertial.velocity_vector[:,0] = 0.
    segment.state.conditions.frames.inertial.time[:,0]            = time[:,0]    
