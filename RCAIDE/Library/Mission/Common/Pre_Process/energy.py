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
    """ Pre-processes energy network by appending all unknowns and residuals             
    """       
    for segment in mission.segments: 
        for network in segment.analyses.energy.vehicle.networks: 
            if type(network) == RCAIDE.Framework.Networks.Hybrid:
                if segment.hybrid_power_split_ratio == None:
                    raise AssertionError('Hybridization power split ratio not set! Specify in mission segment') 
            elif type(network) == RCAIDE.Framework.Networks.Fuel: 
                if segment.hybrid_power_split_ratio == None:                
                    segment.hybrid_power_split_ratio = 0.0 
            elif type(network) == RCAIDE.Framework.Networks.Electric: 
                if segment.hybrid_power_split_ratio == None:                
                    segment.hybrid_power_split_ratio = 1.0  
            segment.state.conditions.energy.hybrid_power_split_ratio = segment.hybrid_power_split_ratio * segment.state.ones_row(1)                    
            network.add_unknowns_and_residuals_to_segment(segment) 
    return 