# RCAIDE/Library/Missions/Common/Pre_Process/energy.py
# 
# 
# Created:  Jul 2023, M. Clarke
 
# ----------------------------------------------------------------------------------------------------------------------
#  energy
# ----------------------------------------------------------------------------------------------------------------------  
def energy(mission):
    """
    Initializes energy network unknowns and residuals for mission segments

    Parameters
    ----------
    mission : Mission
        The mission containing segments to be analyzed
            - analyses.energy.vehicle.networks : list
                Collection of energy networks
                Each network must implement:
                - add_unknowns_and_residuals_to_segment(segment)
                    Method to configure network analysis
    
    Returns
    -------
    None
        Updates mission segment analyses directly

    Notes
    -----
    This function sets up the energy analysis framework for each mission segment
    by adding the appropriate unknowns and residuals to each network in the
    vehicle's energy system.

    The function performs the following steps:
        1. Iterates through all mission segments
        2. For each segment, processes all energy networks
        3. Adds network-specific unknowns and residuals to segment state

    **Major Assumptions**
        * Valid network configurations
        * Proper network initialization
        * Compatible energy systems
        * Well-defined residuals and unknowns

    See Also
    --------
    RCAIDE.Framework.Mission.Segments
    """       
    for segment in mission.segments:
        for network in segment.analyses.energy.vehicle.networks:
            network.add_unknowns_and_residuals_to_segment(segment) 
    return 