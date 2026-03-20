# RCAIDE/Library/Methods/Powertrain/Systems/compute_systems_power_draw.py
# 
# Created:  Oct 2025, M. Guidotti

# ----------------------------------------------------------------------------------------------------------------------
#  compute_systems_power_draw
# ----------------------------------------------------------------------------------------------------------------------    

def compute_systems_power_draw(system, state):
    """
    
    """
    system_conditions              = state.conditions.energy.systems[system.tag]      
    system_conditions.inputs.power.electrical    = system.power_draw * state.ones_row(1) 

    return system_conditions.inputs, system_conditions.outputs