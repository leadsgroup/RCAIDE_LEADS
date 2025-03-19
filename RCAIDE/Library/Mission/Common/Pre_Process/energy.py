# RCAIDE/Library/Missions/Common/Pre_Process/energy.py
# 
# 
# Created:  Jul 2023, M. Clarke
# ----------------------------------------------------------------------------------------------------------------------
# RCAIDE
# ----------------------------------------------------------------------------------------------------------------------  
import RCAIDE
 
# ----------------------------------------------------------------------------------------------------------------------
#  energy
# ----------------------------------------------------------------------------------------------------------------------  
def energy(mission):
    """ Appends all unknows and residuals to the network 
    
        Assumptions:
            N/A
        
        Inputs:
            None
            
        Outputs:
            None 

        Properties Used:
        N/A                
    """       
    for segment in mission.segments:

        for network in segment.analyses.energy.vehicle.networks:

            if  type(network) == RCAIDE.Framework.Networks.Fuel:
                segment.state.conditions.energy.hybrid_power_split_ratio = 0 * segment.state.ones_row(1)            
    
            elif  type(network) == RCAIDE.Framework.Networks.Electric:
                segment.state.conditions.energy.hybrid_power_split_ratio = segment.state.ones_row(1)            
    
            else:
                segment.state.conditions.energy.hybrid_power_split_ratio = segment.hybrid_power_split_ratio * segment.state.ones_row(1) 
            network.add_unknowns_and_residuals_to_segment(segment) 
    return 