# RCAIDE/Library/Methods/Powertrain/Modulators/Electronic_Speed_Controller/append_esc_conditions.py
# (c) Copyright 2023 Aerospace Research Community LLC
# 
# Created:  Jun 2024, M. Clarke  

from RCAIDE.Framework.Mission.Common     import   Conditions

# ---------------------------------------------------------------------------------------------------------------------- 
#  append_esc_conditions
# ----------------------------------------------------------------------------------------------------------------------    
def append_esc_conditions(esc,segment,energy_conditions): 
    ones_row    = segment.state.ones_row 
    energy_conditions.modulators[esc.tag]                  = Conditions()
    energy_conditions.modulators[esc.tag].inputs           = Conditions()
    energy_conditions.modulators[esc.tag].outputs          = Conditions()
    energy_conditions.modulators[esc.tag].throttle         = 0. * ones_row(1)  
    energy_conditions.modulators[esc.tag].outputs.voltage  = 0. * ones_row(1)  
    energy_conditions.modulators[esc.tag].inputs.voltage   = esc.bus_voltage * ones_row(1)   
    return 
