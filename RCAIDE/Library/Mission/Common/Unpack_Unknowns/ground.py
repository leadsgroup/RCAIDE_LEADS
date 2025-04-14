# RCAIDE/Library/Missions/Common/Unknowns/ground.py
# 
# 
# Created:  Jul 2023, M. Clarke
 
# ----------------------------------------------------------------------------------------------------------------------
#  Unpack Unknowns
# ---------------------------------------------------------------------------------------------------------------------- 
def ground(segment):
    """
    Updates ground segment states from solver unknowns

    Parameters
    ----------
    segment : Segment
        The mission segment being analyzed
            - state:
                - unknowns:
                    - ground_velocity : array
                        Solved ground velocity [m/s]
                    - elapsed_time : float
                        Segment duration [s]
                - conditions.frames.inertial:
                    - velocity_vector : array
                        Vehicle velocity [m/s]
                    - time : array
                        Segment time points [s]
                - numerics.dimensionless:
                    - control_points : array
                        Normalized time points [-]
            - air_speed_start : float
                Initial airspeed [m/s]

    Returns
    -------
    None
        Updates segment conditions directly

    Notes
    -----
    This function applies ground-specific solver values to the segment state,
    handling velocity and time variables for ground operations. It ensures
    proper velocity transitions and time scaling.

    The function processes:
        1. Ground velocity application
        2. Time vector computation
        3. Initial velocity preservation

    **Major Assumptions**
        * Continuous velocity transitions
        * Valid time scaling
        * Non-negative velocities
        * Well-defined initial conditions

    See Also
    --------
    RCAIDE.Framework.Mission.Segments.Ground
    """       
    
    # unpack unknowns 
    ground_velocity = segment.state.unknowns.ground_velocity
    time            = segment.state.unknowns.elapsed_time
    
    # unpack givens
    v0         = segment.air_speed_start  
    t_initial  = segment.state.conditions.frames.inertial.time[0,0]
    t_nondim   = segment.state.numerics.dimensionless.control_points
    
    # time
    t_final    = t_initial + time  
    times      = t_nondim * (t_final-t_initial) + t_initial     

    # apply unknowns
    conditions = segment.state.conditions
    conditions.frames.inertial.velocity_vector[1:,0] = ground_velocity
    conditions.frames.inertial.velocity_vector[0,0]  = v0
    conditions.frames.inertial.time[:,0]             = times[:,0]
