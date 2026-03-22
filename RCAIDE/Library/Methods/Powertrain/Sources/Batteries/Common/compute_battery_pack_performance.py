# RCAIDE/Methods/Powertrain/Sources/Batteries/Common/compute_battery_pack_performance.py
# 
# 
# Created:  Feb 2024, M. Clarke
# Modified: Sep 2024, S. Shekar

# ----------------------------------------------------------------------------------------------------------------------
#  IMPORT
# ----------------------------------------------------------------------------------------------------------------------
import RCAIDE
from RCAIDE.Framework.Core import Units 
import numpy as np
from copy import deepcopy
 
# ----------------------------------------------------------------------------------------------------------------------
# compute_nmc_cell_performance
# ---------------------------------------------------------------------------------------------------------------------- 
def compute_battery_pack_performance(battery,state,network):
    """ 
    """ 
    
    battery_conditions = state.conditions.energy.sources[battery.tag]
    psi                = state.conditions.energy.battery_fuel_cell_power_split_ratio  
    phi                = state.conditions.energy.hybrid_power_split_ratio
    
    for m_i, module in enumerate(battery.modules):
        
        if module.active: 
            voltage = battery.voltage * state.ones_row(1)
            
            if state.conditions.energy.recharging:    
                battery_conditions.inputs.power.electrical             =  state.unknowns.network['electrical_power'] *  battery_conditions.power_split_ratio * psi * phi
                battery_conditions.current                             =  battery_conditions.inputs.power.electrical / voltage
                battery_conditions[module.tag].inputs.power.electrical =  battery_conditions.inputs.power.electrical / battery.number_of_active_modules     
            else:
                battery_conditions.outputs.power.electrical              = state.unknowns.network['electrical_power'] *  battery_conditions.power_split_ratio * psi * phi
                battery_conditions.current                               = battery_conditions.outputs.power.electrical /voltage 
                battery_conditions[module.tag].outputs.power.electrical  = battery_conditions.outputs.power.electrical  / battery.number_of_active_modules   
          
          
            battery_conditions[module.tag].power   = battery_conditions[module.tag].inputs.power.electrical -  battery_conditions[module.tag].outputs.power.electrical      
            battery_conditions[module.tag].current = battery_conditions.current / battery.number_of_active_modules 
            if (battery.identical_modules == False or m_i == 0):  
                module_inputs, module_outputs, stored_results_flag, stored_module_tag = module.compute_performance(battery,state,network)
            else: 
                module_inputs, module_outputs = module.reuse_stored_data(state,network,battery.tag,stored_module_tag)                
                
            #battery_conditions.inputs.power.electrical += module_inputs.power.electrical
            #battery_conditions.outputs.power.electrical += module_outputs.power.electrical 
            
    stored_results_flag     = True
    stored_battery_tag      = battery.tag  
            
    return battery_conditions.inputs, battery_conditions.outputs, stored_results_flag, stored_battery_tag 
 