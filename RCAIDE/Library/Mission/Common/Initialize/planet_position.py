# RCAIDE/Library/Missions/Segments/Common/Initialize/Frames.py
# 
# 
# Created:  Jul 2023, M. Clarke 

# ----------------------------------------------------------------------------------------------------------------------
#  Initialize Planet Position
# ----------------------------------------------------------------------------------------------------------------------
def planet_position(segment):
    """
    Initializes the vehicle's planetary position coordinates for mission segment

    Parameters
    ----------
    segment : Segment
        The mission segment being analyzed

    Notes
    -----
    This function sets up the initial latitude and longitude coordinates
    of the vehicle relative to the planet. It handles position initialization
    from either previous segment conditions or explicit segment parameters.

    The function follows this priority for position initialization:
        1. Previous segment final position (if initials exist)
        2. Explicit segment coordinates (if specified)
        3. Default coordinates (0,0)

    **Required Segment State Variables**

    If segment.state.initials exists:
        state.initials.conditions.frames.planet:
            - longitude : array
                Previous segment final longitude [rad]
            - latitude : array
                Previous segment final latitude [rad]

    state.conditions.frames.planet:
        - longitude : array
            Current segment longitude array [rad]
        - latitude : array
            Current segment latitude array [rad]

    **Optional Segment Parameters**
    
    segment:
        - longitude : float
            Initial longitude [rad]
        - latitude : float
            Initial latitude [rad]

    **Major Assumptions**
        * Spherical planet model
        * Continuous position tracking when using initials
        * Valid coordinate values
        * Radians for angular measurements

    Returns
    -------
    None
        Updates segment conditions directly

    See Also
    --------
    RCAIDE.Framework.Mission.Segments
    RCAIDE.Library.Mission.Common.Initialize.inertial_position
    """        
    
    if segment.state.initials:
        longitude_initial = segment.state.initials.conditions.frames.planet.longitude[-1,0]
        latitude_initial  = segment.state.initials.conditions.frames.planet.latitude[-1,0] 
    elif 'latitude' in segment:
        longitude_initial = segment.longitude
        latitude_initial  = segment.latitude      
    else:
        longitude_initial = 0.0
        latitude_initial  = 0.0

    segment.state.conditions.frames.planet.longitude[:,0] = longitude_initial
    segment.state.conditions.frames.planet.latitude[:,0]  = latitude_initial    

    return 