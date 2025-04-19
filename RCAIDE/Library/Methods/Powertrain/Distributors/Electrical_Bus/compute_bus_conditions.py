# RCAIDE/Methods/Energy/Distributors/Electrical_Bus.py
# 
# 
# Created: Sep 2024, S. Shekar

# ----------------------------------------------------------------------------------------------------------------------
#  IMPORT
# ----------------------------------------------------------------------------------------------------------------------
# imports 
import numpy as np

 
# ----------------------------------------------------------------------------------------------------------------------
# compute_bus_conditions
# ----------------------------------------------------------------------------------------------------------------------
def compute_bus_conditions(bus, state, t_idx, delta_t): 
    """Computes the conditions of the bus based on the response of the battery modules
    
    Args:
        bus: Electrical bus object
        state: Current system state
        t_idx: Time index
        delta_t: Time step
    """  
    bus_conditions = state.conditions.energy[bus.tag]
     
    if len(bus.battery_modules) != 0: 
        if bus.battery_module_electric_configuration == 'Series':
            bm_conditions                               = [bus_conditions.battery_modules[bm.tag] for bm in bus.battery_modules]
            bus_conditions.voltage_open_circuit[t_idx]  = sum(bm.voltage_open_circuit[t_idx] for bm in bm_conditions)
            bus_conditions.voltage_under_load[t_idx]    = sum(bm.voltage_under_load[t_idx] for bm in bm_conditions)
            bus_conditions.heat_energy_generated[t_idx] = sum(bm.heat_energy_generated[t_idx] for bm in bm_conditions)
            bus_conditions.efficiency[t_idx]            = (bus_conditions.power_draw[t_idx] + bus_conditions.heat_energy_generated[t_idx])/bus_conditions.power_draw[t_idx]
            if t_idx != state.numerics.number_of_control_points-1:  
                bm_conditions                              = [bus_conditions.battery_modules[bm.tag] for bm in bus.battery_modules]
                bus_conditions.temperature[t_idx+1]        = sum(bm.temperature[t_idx+1] for bm in bm_conditions)/ len(bus.battery_modules)
                bus_conditions.energy[t_idx+1]             = sum(bm.energy[t_idx+1] for bm in bm_conditions)
                bus_conditions.state_of_charge[t_idx+1]    = bm_conditions[-1].state_of_charge[t_idx+1]
    
        elif bus.battery_module_electric_configuration == 'Parallel':
            bm_conditions                               = [bus_conditions.battery_modules[bm.tag] for bm in bus.battery_modules]
            bus_conditions.heat_energy_generated[t_idx] = sum(bm.heat_energy_generated[t_idx] for bm in bm_conditions)
            bus_conditions.voltage_open_circuit[t_idx]  = bm_conditions[-1].voltage_open_circuit[t_idx]
            bus_conditions.voltage_under_load[t_idx]    = bm_conditions[-1].voltage_under_load[t_idx]             
            bus_conditions.efficiency[t_idx]            = (bus_conditions.power_draw[t_idx] +  bus_conditions.heat_energy_generated[t_idx])/bus_conditions.power_draw[t_idx]
            if t_idx != state.numerics.number_of_control_points-1:  
                bus_conditions.heat_energy_generated[t_idx] = sum(bm.heat_energy_generated[t_idx] for bm in bm_conditions)
                bus_conditions.temperature[t_idx+1]         = sum(bm.temperature[t_idx+1] for bm in bm_conditions)/len(bus.battery_modules)
                bus_conditions.energy[t_idx+1]              = sum(bm.energy[t_idx+1] for bm in bm_conditions)
                bus_conditions.state_of_charge[t_idx+1]     = bm_conditions[-1].cell.state_of_charge[t_idx+1]
        
    if t_idx != state.numerics.number_of_control_points-1:  
        # Handle fully charged state
        if state.conditions.energy.recharging and np.float16(bus_conditions.state_of_charge[t_idx+1]) == 1:
            bus_conditions.charging_current[t_idx+1] = 0
            bus_conditions.power_draw[t_idx+1]       = 0
            bus_conditions.current_draw[t_idx+1]     = 0
    return