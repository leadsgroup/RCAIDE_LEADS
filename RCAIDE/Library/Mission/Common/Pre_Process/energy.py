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

            if isinstance(network.distributors, RCAIDE.Library.Components.Powertrain.Distributors.Electrical_Bus):
            
                # determine bus properties
                for distributor in network.distributors:
                    distributor.initialize_bus_properties()                  
                
                    # update bus voltage on each electrical component
                    for converter in distributor.assigned_converters[0]:
                        converter.bus_voltage = distributor.voltage
                        
                    for modulator in distributor.assigned_modulators[0]:
                        modulator.bus_voltage = distributor.voltage
        
                    for system in distributor.assigned_systems[0]:
                        system.bus_voltage = distributor.voltage                            
                    
            # design propulsor 
            for propulsor in network.propulsors:
                propulsor.intialize_propulsor_design(network)  
                           
            if segment.hybrid_power_split_ratio == None:                
                segment.hybrid_power_split_ratio = 0.0
                segment.battery_fuel_cell_power_split_ratio = 0.0              

            segment.state.conditions.energy.hybrid_power_split_ratio            = segment.hybrid_power_split_ratio * segment.state.ones_row(1)  
            segment.state.conditions.energy.battery_fuel_cell_power_split_ratio = segment.battery_fuel_cell_power_split_ratio * segment.state.ones_row(1)                    
            network.add_unknowns_and_residuals_to_segment(segment) 
    return 