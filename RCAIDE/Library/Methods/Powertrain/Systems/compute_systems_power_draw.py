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

    inputs                         = system_conditions.inputs
    outputs                        = system_conditions.outputs

    inputs.power.electrical        = system.power_draw * state.ones_row(1) 

    return inputs, outputs