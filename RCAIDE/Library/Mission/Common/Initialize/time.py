# RCAIDE/Library/Missions/Common/Initialize/time.py
# 
# 
# Created:  Jul 2023, M. Clarke
 
# ----------------------------------------------------------------------------------------------------------------------
#  Initialize Time
# ---------------------------------------------------------------------------------------------------------------------- 
def time(segment):
    """
    Initializes time variables for mission segment analysis

    Parameters
    ----------
    segment : Segment
        The mission segment being analyzed

    Notes
    -----
    This function sets up the initial time values for both inertial and
    planetary reference frames. It handles time continuity between segments
    and establishes start times for new segments.

    The function follows this priority for time initialization:
        1. Previous segment final time (if initials exist)
        2. Explicit segment start time (if specified)
        3. Current initial time value

    **Required Segment State Variables**

    If segment.state.initials exists:
        state.initials.conditions.frames:
            inertial:
                - time : array
                    Previous segment final time [s]
            planet:
                - start_time : float
                    Previous segment start time [s]

    state.conditions.frames:
        inertial:
            - time : array
                Current segment time array [s]
        planet:
            - start_time : float
                Current segment start time [s]

    **Optional Segment Parameters**
    
    segment:
        - start_time : float
            Explicit segment start time [s]

    **Major Assumptions**
        * Continuous time tracking when using initials
        * Valid time values (non-negative)
        * Proper time synchronization between frames
        * Time measured in seconds

    Returns
    -------
    None
        Updates segment conditions directly

    See Also
    --------
    RCAIDE.Framework.Mission.Segments
    RCAIDE.Library.Mission.Common.Initialize.planet_position
    """        
    
    if segment.state.initials:
        t_initial = segment.state.initials.conditions.frames.inertial.time
        t_current = segment.state.conditions.frames.inertial.time 
        segment.state.conditions.frames.inertial.time[:,:] = t_current + (t_initial[-1,0] - t_current[0,0])
        
    else:
        t_initial = segment.state.conditions.frames.inertial.time[0,0]
        
    if segment.state.initials:
        segment.state.conditions.frames.planet.start_time = segment.state.initials.conditions.frames.planet.start_time
        
    elif 'start_time' in segment:
        segment.state.conditions.frames.planet.start_time = segment.start_time
    
    return 