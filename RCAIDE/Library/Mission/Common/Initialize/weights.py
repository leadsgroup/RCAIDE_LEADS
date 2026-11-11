# RCAIDE/Library/Missions/Common/Initialize/weights.py
# 
# 
# Created:  Jul 2023, M. Clarke
# Modified: Nov 2026, M. Clarke

from RCAIDE.Framework.Core import set_at
import numpy as np

MOMENTS_OF_INERTIA = ['moments_of_inertia_' + axes for axes in ('Ixx', 'Ixy', 'Ixz', 'Iyx', 'Iyy', 'Iyz', 'Izx', 'Izy', 'Izz')]
 

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
    conditions = segment.state.conditions.weights
    settings   = segment.analyses.weights.settings
    if segment.state.initials:
        initial   = segment.state.initials.conditions.weights
        m_initial = initial.vehicle.mass[-1,0]

        # the vehicle starts where the previous segment ended
        vehicle                          = conditions.vehicle
        vehicle.mass                     = set_at(vehicle.mass, np.s_[:,0], initial.vehicle.mass[-1,0])
        vehicle.global_center_of_gravity = set_at(vehicle.global_center_of_gravity, np.s_[:,0], initial.vehicle.global_center_of_gravity[-1,0])
        for name in ['mass_rate'] + MOMENTS_OF_INERTIA:
            vehicle[name] = set_at(vehicle[name], np.s_[0,:], initial.vehicle[name][-1,0])

        # and so do its components
        components = conditions.components
        for tag in initial.components.mass.keys():
            if settings.run_center_of_gravity_analysis or settings.run_moments_of_inertia_analysis:
                components.mass[tag] = set_at(components.mass[tag], np.s_[:,0], initial.components.mass[tag][-1,0])
            if settings.run_center_of_gravity_analysis:
                cg = components.global_center_of_gravity
                cg[tag] = set_at(cg[tag], np.s_[:,0], initial.components.global_center_of_gravity[tag][-1,0])
            if settings.run_moments_of_inertia_analysis:
                for name in MOMENTS_OF_INERTIA:
                    components[name][tag] = set_at(components[name][tag], np.s_[:,0], initial.components[name][tag][-1,0])
    else:
        m_initial = segment.analyses.vehicle.mass_properties.takeoff

    m_current = conditions.vehicle.mass
    conditions.vehicle.mass = m_current + (m_initial - m_current[0,0])

    return
