# RCAIDE/Library/Methods/Powertrain/Sources/Batteries/Lithium_Ion_NMC/unpack_lithium_ion_nmc_unknowns.py
# (c) Copyright 2023 Aerospace Research Community LLC
# 
# Created:  Jun 2024, M. Clarke   

# ---------------------------------------------------------------------------------------------------------------------- 
#  unpack_lithium_ion_nmc_unknowns
# ----------------------------------------------------------------------------------------------------------------------  
def unpack_lithium_ion_nmc_unknowns(module,battery,segment):  
    battery_module_conditions = segment.state.conditions.energy.sources[battery.tag][module.tag] 
    battery_module_conditions.cell.temperature     = segment.state.unknowns.network[battery.tag + '_' + module.tag  +  '_cell_temperature']
    battery_module_conditions.cell.state_of_charge = segment.state.unknowns.network[battery.tag + '_' + module.tag  + '_cell_state_of_charge']    
    return 