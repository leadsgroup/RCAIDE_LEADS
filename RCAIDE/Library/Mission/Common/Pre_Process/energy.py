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
    idx = 0      
    for segment in mission.segments: 
        for network in segment.analyses.vehicle.networks:  
            
            #'''not sure I want to keep this but this basically is where matteo assigns the voltage of the bus a compoment is on may not need '''
            #for converter in network.converters:  
                #for distributor_tag in converter.assigned_distributors:
                    #if isinstance(network.distributors[distributor_tag[0]], RCAIDE.Library.Components.Powertrain.Distributors.Electrical_Bus):
                        #converter.bus_voltage = network.distributors[distributor_tag[0]].voltage

            #for modulator in network.modulators:
                #for distributor_tag in modulator.assigned_distributors:
                    #if isinstance(network.distributors[distributor_tag[0]], RCAIDE.Library.Components.Powertrain.Distributors.Electrical_Bus):
                        #modulator.bus_voltage = network.distributors[distributor_tag[0]].voltage

            #for system in network.systems:
                #for distributor_tag in system.assigned_distributors:
                    #if isinstance(network.distributors[distributor_tag[0]], RCAIDE.Library.Components.Powertrain.Distributors.Electrical_Bus):
                        #system.bus_voltage = network.distributors[distributor_tag[0]].voltage               
                           
            if segment.hybrid_power_split_ratio == None:                
                segment.hybrid_power_split_ratio            = 0.0
                segment.battery_fuel_cell_power_split_ratio = 0.0              

            segment.state.conditions.energy.hybrid_power_split_ratio            = segment.hybrid_power_split_ratio * segment.state.ones_row(1)  
            segment.state.conditions.energy.battery_fuel_cell_power_split_ratio = segment.battery_fuel_cell_power_split_ratio * segment.state.ones_row(1)                    
            network.add_unknowns_and_residuals_to_segment(segment) 

        idx += 1

    return 