# RCAIDE/Library/Missions/Segments/Hover/Climb.py
# 
# 
# Created:  Jul 2023, M. Clarke 
 
# ----------------------------------------------------------------------------------------------------------------------  
#  Initialize Conditions
# ----------------------------------------------------------------------------------------------------------------------   
def initialize_conditions(segment):
    """
    Initializes conditions for vertical climb segment

    Parameters
    ----------
    segment : Segment
        The mission segment being analyzed
            - altitude_start : float
                Initial altitude [m]
            - altitude_end : float
                Final altitude [m]
            - climb_rate : float
                Vertical climb rate [m/s]
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
            - conditions.frames.inertial.velocity_vector [m/s]
            - conditions.frames.inertial.position_vector [m]
            - conditions.freestream.altitude [m]
            - conditions.frames.inertial.time [s]

    Notes
    -----
    This function sets up the initial conditions for a vertical climb segment with
    constant climb rate. The segment handles pure vertical motion with no horizontal
    velocity components.

    **Calculation Process**
        1. Check initial conditions
        2. Discretize altitude profile:
            alt = alt0 + (altf - alt0)*t_norm
        3. Calculate time required:
            dt = (altf - alt0)/climb_rate
        4. Set velocity components:
            - v_x = 0
            - v_z = -climb_rate (z points down)
        5. Scale time points:
        t = t_norm * dt

    **Major Assumptions**
        * Constant climb rate
        * Pure vertical motion
        * No horizontal velocity
        * No atmospheric variations
        * Quasi-steady climb

    See Also
    --------
    RCAIDE.Framework.Mission.Segments
    """        
    
    # unpack
    climb_rate = segment.climb_rate
    alt0       = segment.altitude_start 
    altf       = segment.altitude_end
    t_nondim   = segment.state.numerics.dimensionless.control_points
    t_initial  = segment.state.conditions.frames.inertial.time[0,0]
    conditions = segment.state.conditions  

    # check for initial altitude
    if alt0 is None:
        if not segment.state.initials: raise AttributeError('initial altitude not set')
        alt0 = -1.0 * segment.state.initials.conditions.frames.inertial.position_vector[-1,2]

    # discretize on altitude
    alt = t_nondim * (altf-alt0) + alt0
    
    # process velocity vector
    v_z   = -climb_rate # z points down    
    dt = (altf - alt0)/climb_rate

    # rescale operators
    t = t_nondim * dt

    # pack
    t_initial = segment.state.conditions.frames.inertial.time[0,0]
    segment.state.conditions.frames.inertial.time[:,0] = t_initial + t[:,0]    
    
    # pack conditions    
    conditions.frames.inertial.velocity_vector[:,0]         = 0.
    conditions.frames.inertial.velocity_vector[:,2]         = v_z
    conditions.frames.inertial.position_vector[:,2]         = -alt[:,0] # z points down
    conditions.freestream.altitude[:,0]                     =  alt[:,0] # positive altitude in this context
    segment.state.conditions.frames.inertial.time[:,0]      = t_initial + t[:,0]