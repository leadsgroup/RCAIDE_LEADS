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
                charging_current                 = module.nominal_capacity * battery.charging_c_rate 
                charging_power                   = (charging_current*voltage*module.power_split_ratio) 
                #battery_conditions[module.tag].power_draw   -= charging_power 
                battery_conditions[module.tag].current_draw  = -battery_conditions[module.tag].power_draw /battery.voltage
                
                
                 
                #battery_conditions.inputs.power.electrical             =  state.unknowns.network['electrical_power'] *  battery_conditions.power_split_ratio * psi * phi
                battery_conditions[module.tag].inputs.power.electrical  = charging_power 
                battery_conditions[module.tag].current_draw             =  battery_conditions[module.tag].inputs.power.electrical  /battery.voltage 
            else:
                battery_conditions.outputs.power.electrical              = state.unknowns.network['electrical_power'] *  battery.power_split_ratio * psi * phi
                battery_conditions.current_draw                          = battery_conditions.outputs.power.electrical /voltage 
                battery_conditions[module.tag].outputs.power.electrical  = battery_conditions.outputs.power.electrical  / battery.number_of_active_modules   
          
          
            battery_conditions[module.tag].power_draw   = battery_conditions[module.tag].outputs.power.electrical -  battery_conditions[module.tag].inputs.power.electrical      
            #battery_conditions[module.tag].current_draw  = battery_conditions.current / battery.number_of_active_modules 
            if (battery.identical_modules == False or m_i == 0):  
                module_inputs, module_outputs, stored_results_flag, stored_module_tag = module.compute_performance(battery,state,network)
            else: 
                module_inputs, module_outputs = module.reuse_stored_data(state,network,battery.tag,stored_module_tag)                
                
            #battery_conditions.inputs.power.electrical += module_inputs.power.electrical
            #battery_conditions.outputs.power.electrical += module_outputs.power.electrical
            

            battery_conditions.temperature        = battery_conditions[module.tag].temperature 
            battery_conditions.energy             +=  battery_conditions[module.tag].energy
            battery_conditions.state_of_charge    = battery_conditions[module.tag].state_of_charge  
            battery_conditions.heat_energy_generated +=  battery_conditions[module.tag].heat_energy_generated       

            if battery.battery_module_electric_configuration == 'Series': 
                battery_conditions.voltage_open_circuit  +=  battery_conditions[module.tag].voltage_open_circuit 
                battery_conditions.voltage_under_load    +=  battery_conditions[module.tag].voltage_under_load 
            elif battery.battery_module_electric_configuration == 'Parallel': 
                battery_conditions.voltage_open_circuit  = battery_conditions[module.tag].voltage_open_circuit 
                battery_conditions.voltage_under_load    = battery_conditions[module.tag].voltage_under_load  
            #battery_conditions.efficiency            = (battery_conditions[module.tag].power  +  battery_conditions.heat_energy_generated)/(battery_conditions[module.tag].power)
       
            if state.conditions.energy.recharging:
                fully_charged = battery_conditions.state_of_charge == 1
                battery_conditions.charging_current[fully_charged]  = 0
                battery_conditions[module.tag].power[fully_charged]        = 0
                battery_conditions[module.tag].current[fully_charged]      = 0
                battery_conditions[module.tag].inputs.power.electrical[fully_charged]      = 0
            
    stored_results_flag     = True
    stored_battery_tag      = battery.tag
    
    return battery_conditions.inputs, battery_conditions.outputs, stored_results_flag, stored_battery_tag 
 