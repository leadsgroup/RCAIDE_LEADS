# RCAIDE/Library/Missions/Common/Pre_Process/emissions.py
# 
# 
# Created:  Jul 2023, M. Clarke

# ----------------------------------------------------------------------------------------------------------------------
#  emissions
# ----------------------------------------------------------------------------------------------------------------------  
def emissions(mission):
    """
    Initializes and processes emissions models for mission segments

    Parameters
    ----------
    mission : Mission
        The mission containing segments to be analyzed
            - analyses.emissions : Analysis
                Emissions analysis module
                - process.emissions : Process
                    Emissions computation process
            - surrogates : Data
                Emissions surrogate models
    
    Returns
    -------
    None
        Updates mission segment analyses directly

    Notes
    -----
    This function prepares the emissions analysis for each mission segment.
    It manages emissions models and surrogate data across segments for
    computational efficiency by reusing previously computed data when possible.

    The function performs the following steps:
        1. Identifies segments requiring emissions analysis
        2. Reuses previous segment's emissions data when possible
        3. Initializes new emissions analyses when needed

    **Process Flow**
    
    For each segment:
        1. Check if emissions analysis exists
        2. If previous segment exists with computed data:
            - Reuse process and surrogate data
        3. Otherwise:
            - Initialize new emissions analysis
            - Store segment tag for future reference

    **Major Assumptions**
        * Compatible emissions models between segments
        * Valid initialization of first segment
        * Continuous emissions characteristics
        * Proper surrogate model compatibility

    See Also
    --------
    RCAIDE.Framework.Mission.Segments
    """
        
    last_tag = None
    for tag,segment in mission.segments.items():
        if segment.analyses.emissions != None:
            if last_tag and  'compute' in mission.segments[last_tag].analyses.emissions.process: 
                segment.analyses.emissions.process.emissions = mission.segments[last_tag].analyses.emissions.process.emissions
                segment.analyses.emissions.surrogates        = mission.segments[last_tag].analyses.emissions.surrogates  
            else:          
                em   = segment.analyses.emissions
                em.initialize()   
                last_tag = tag
    return 