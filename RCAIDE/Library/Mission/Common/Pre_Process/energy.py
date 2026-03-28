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
        for network in segment.analyses.vehicle.networks: 
            segment.state.conditions.energy.hybrid_power_split_ratio            = network.hybrid_power_split_ratio * segment.state.ones_row(1)  
            segment.state.conditions.energy.battery_fuel_cell_power_split_ratio = network.battery_fuel_cell_power_split_ratio * segment.state.ones_row(1)  
            
            for propulsor in network.propulsors: 
                propulsor.initialize(network) 
                propulsor.append_operating_conditions(segment)
    
            for converter in network.converters: 
                converter.initialize(network) 
                converter.append_operating_conditions(segment)  

            for modulator in network.modulators: 
                modulator.append_operating_conditions(segment)  

            for source in  network.sources: 
                source.initialize(network)  
                source.append_operating_conditions(segment)  

            for system in network.systems:
                system.append_operating_conditions(segment)             
    
            for distributor in network.distributors:
                distributor.initialize(network)
                distributor.append_operating_conditions(segment) 
                
    return 