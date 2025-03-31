# RCAIDE/Library/Missions/Segments/Common/Initialize/inertial_position.py
# 
# 
# Created:  Jul 2023, M. Clarke 

# ----------------------------------------------------------------------------------------------------------------------
#  Initialize Inertial Position
# ----------------------------------------------------------------------------------------------------------------------
def inertial_position(segment): 
    """
    Initializes the inertial position vector for mission segment analysis

    Parameters
    ----------
    segment : Segment
        The mission segment being analyzed

    Notes
    -----
    This function sets up the initial position vector and aircraft range in
    the inertial reference frame. It handles position continuity between
    segments and applies altitude constraints when specified.

    The function performs the following steps:
        1. Retrieves initial conditions from previous segment if available
        2. Applies altitude constraints from segment parameters
        3. Updates position vector and aircraft range for continuity

    **Required Segment State Variables**

    If segment.state.initials exists:
        state.initials.conditions.frames.inertial:
            - position_vector : array
                Previous segment final position [m]
            - aircraft_range : array
                Previous segment final range [m]

    state.conditions.frames.inertial:
        - position_vector : array
            Current segment position vector [m]
        - aircraft_range : array
            Current segment range [m]

    **Segment Parameters**
    
    Either:
    - altitude : float
        Fixed altitude for segment [m]
    Or:
    - altitude_start : float
        Initial altitude for segment [m]

    **Major Assumptions**
        * Initial conditions available if needed
        * Valid altitude constraints
        * Continuous position tracking
        * Earth-fixed inertial frame

    Returns
    -------
    None
        Updates segment conditions directly

    Raises
    ------
    AssertionError
        If neither altitude nor altitude_start is specified

    See Also
    --------
    RCAIDE.Framework.Mission.Segments
    """      
    if segment.state.initials:
        r_initial = segment.state.initials.conditions.frames.inertial.position_vector
        r_current = segment.state.conditions.frames.inertial.position_vector
        R_initial = segment.state.initials.conditions.frames.inertial.aircraft_range
        R_current = segment.state.conditions.frames.inertial.aircraft_range
        
        if 'altitude' in segment.keys() and segment.altitude is not None:
            r_initial[-1,None,-1] = -segment.altitude
        elif 'altitude_start' in segment.keys() and segment.altitude_start is not None:
            r_initial[-1,None,-1] = -segment.altitude_start
        else:
            assert('Altitude not set')
            
        segment.state.conditions.frames.inertial.position_vector[:,:] = r_current + (r_initial[-1,None,:] - r_current[0,None,:])
        segment.state.conditions.frames.inertial.aircraft_range[:,:]  = R_current + (R_initial[-1,None,:] - R_current[0,None,:])
        
    return 