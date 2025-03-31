# RCAIDE/Library/Missions/Common/Initialize/weights.py
# 
# 
# Created:  Jul 2023, M. Clarke
 

# ----------------------------------------------------------------------------------------------------------------------
#  Initialize Weights
# ---------------------------------------------------------------------------------------------------------------------- 
def weights(segment):
    """
    Initializes vehicle mass properties for mission segment analysis

    Parameters
    ----------
    segment : Segment
        The mission segment being analyzed

    Notes
    -----
    This function sets up the initial mass values for the vehicle. It determines
    the initial mass through a priority system and maintains mass continuity
    between segments.

    The function follows this priority for mass initialization:
        1. Previous segment final mass (if initials exist)
        2. Vehicle takeoff mass (if weight analysis exists)
        3. Network mass properties (fallback option)

    **Required Segment State Variables**

    If segment.state.initials exists:
        state.initials.conditions.weights:
            - total_mass : array
                Previous segment final mass [kg]

    state.conditions.weights:
        - total_mass : array
            Current segment mass array [kg]

    **Required Analysis Components**
    
    Either:
    segment.analyses.weights:
        - vehicle.mass_properties.takeoff : float
            Vehicle takeoff mass [kg]
    Or:
    segment.analyses.energy.vehicle.networks:
        - mass_properties.mass : float
            Network mass properties [kg]

    **Major Assumptions**
        * Continuous mass tracking when using initials
        * Valid mass values (positive)
        * At least one mass property source available
        * Mass measured in kilograms

    Returns
    -------
    None
        Updates segment conditions directly

    See Also
    --------
    RCAIDE.Framework.Mission.Segments
    """      
 
    if segment.state.initials:
        m_initial = segment.state.initials.conditions.weights.total_mass[-1,0] 
    else: 
        if segment.analyses.weights != None: 
            m_initial = segment.analyses.weights.vehicle.mass_properties.takeoff
        else: 
            m_initial = segment.analyses.energy.vehicle.networks[list(segment.analyses.energy.vehicle.networks.keys())[0]].mass_properties.mass

    m_current = segment.state.conditions.weights.total_mass
    
    segment.state.conditions.weights.total_mass[:,:] = m_current + (m_initial - m_current[0,0])
        
    return 