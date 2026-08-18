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
            - mass.total : array
                Previous segment final mass [kg]

    state.conditions.weights:
        - mass.total : array
            Current segment mass array [kg]

    **Required Analysis Components**
    
    Either:
    segment.analyses.weights:
        - vehicle.mass_properties.takeoff : float
            Vehicle takeoff mass [kg]
    Or:
    segment.analyses.vehicle.networks:
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
        m_initial = segment.state.initials.conditions.weights.vehicle.mass[-1,0]
        
        segment.state.conditions.weights.vehicle.mass[:,0]                         = segment.state.initials.conditions.weights.vehicle.mass[-1,0] 
        segment.state.conditions.weights.vehicle.global_center_of_gravity[:,0]     = segment.state.initials.conditions.weights.vehicle.global_center_of_gravity[-1,0] 
        segment.state.conditions.weights.vehicle.mass_rate[0,:]                    = segment.state.initials.conditions.weights.vehicle.mass_rate[-1,0]
        segment.state.conditions.weights.vehicle.moments_of_inertia_Ixx[0,:]       = segment.state.initials.conditions.weights.vehicle.moments_of_inertia_Ixx[-1,0]
        segment.state.conditions.weights.vehicle.moments_of_inertia_Ixy[0,:]       = segment.state.initials.conditions.weights.vehicle.moments_of_inertia_Ixy[-1,0]
        segment.state.conditions.weights.vehicle.moments_of_inertia_Ixz[0,:]       = segment.state.initials.conditions.weights.vehicle.moments_of_inertia_Ixz[-1,0]
        segment.state.conditions.weights.vehicle.moments_of_inertia_Iyx[0,:]       = segment.state.initials.conditions.weights.vehicle.moments_of_inertia_Iyx[-1,0]
        segment.state.conditions.weights.vehicle.moments_of_inertia_Iyy[0,:]       = segment.state.initials.conditions.weights.vehicle.moments_of_inertia_Iyy[-1,0]
        segment.state.conditions.weights.vehicle.moments_of_inertia_Iyz[0,:]       = segment.state.initials.conditions.weights.vehicle.moments_of_inertia_Iyz[-1,0]
        segment.state.conditions.weights.vehicle.moments_of_inertia_Izx[0,:]       = segment.state.initials.conditions.weights.vehicle.moments_of_inertia_Izx[-1,0]
        segment.state.conditions.weights.vehicle.moments_of_inertia_Izy[0,:]       = segment.state.initials.conditions.weights.vehicle.moments_of_inertia_Izy[-1,0]
        segment.state.conditions.weights.vehicle.moments_of_inertia_Izz[0,:]       = segment.state.initials.conditions.weights.vehicle.moments_of_inertia_Izz[-1,0]
      
        for tag,item in segment.state.initials.conditions.weights.components.mass.items():
            if segment.analyses.weights.settings.run_center_of_gravity_analysis:  
                segment.state.conditions.weights.components.mass[tag][:,0]                         = segment.state.initials.conditions.weights.components.mass[tag][-1,0] 
                segment.state.conditions.weights.components.global_center_of_gravity[tag][:,0]     = segment.state.initials.conditions.weights.components.global_center_of_gravity[tag][-1,0]  
            if segment.analyses.weights.settings.run_moments_of_inertia_analysis:
                segment.state.conditions.weights.components.moments_of_inertia_Ixx[tag][:,0]       = segment.state.initials.conditions.weights.components.moments_of_inertia_Ixx[tag][-1,0]
                segment.state.conditions.weights.components.moments_of_inertia_Ixy[tag][:,0]       = segment.state.initials.conditions.weights.components.moments_of_inertia_Ixy[tag][-1,0]
                segment.state.conditions.weights.components.moments_of_inertia_Ixz[tag][:,0]       = segment.state.initials.conditions.weights.components.moments_of_inertia_Ixz[tag][-1,0]
                segment.state.conditions.weights.components.moments_of_inertia_Iyx[tag][:,0]       = segment.state.initials.conditions.weights.components.moments_of_inertia_Iyx[tag][-1,0]
                segment.state.conditions.weights.components.moments_of_inertia_Iyy[tag][:,0]       = segment.state.initials.conditions.weights.components.moments_of_inertia_Iyy[tag][-1,0]
                segment.state.conditions.weights.components.moments_of_inertia_Iyz[tag][:,0]       = segment.state.initials.conditions.weights.components.moments_of_inertia_Iyz[tag][-1,0]
                segment.state.conditions.weights.components.moments_of_inertia_Izx[tag][:,0]       = segment.state.initials.conditions.weights.components.moments_of_inertia_Izx[tag][-1,0]
                segment.state.conditions.weights.components.moments_of_inertia_Izy[tag][:,0]       = segment.state.initials.conditions.weights.components.moments_of_inertia_Izy[tag][-1,0]
                segment.state.conditions.weights.components.moments_of_inertia_Izz[tag][:,0]       = segment.state.initials.conditions.weights.components.moments_of_inertia_Izz[tag][-1,0]
     
    else: 
        m_initial = segment.analyses.vehicle.mass_properties.takeoff 
    m_current = segment.state.conditions.weights.vehicle.mass
    
    segment.state.conditions.weights.vehicle.mass[:,:] = m_current + (m_initial - m_current[0,0])
        
    return 