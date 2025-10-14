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
                for bus in  network.busses:
                    bus.initialize_bus_properties()                  
                
                    # update bus voltage on each electrical component
                    for converter in bus.assigned_converters[0]:
                        converter.bus_voltage = bus.voltage
                        
                    for modulator in bus.assigned_modulators[0]:
                        modulator.bus_voltage = bus.voltage
        
                    for system in bus.assigned_systems[0]:
                        system.bus_voltage = bus.voltage                            
                    
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