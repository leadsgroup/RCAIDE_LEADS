# RCAIDE/Library/Methods/Powertrain/Modulators/Inverter/append_inverter_conditions.py
# 
#
# Created:  Sep 2025, M. Guidotti

from RCAIDE.Framework.Mission.Common     import   Conditions

# ---------------------------------------------------------------------------------------------------------------------- 
#  append_inverter_conditions
# ----------------------------------------------------------------------------------------------------------------------    
def append_inverter_conditions(esc,segment,energy_conditions): 
    """
   
    """
    
    ones_row    = segment.state.ones_row 
    energy_conditions.modulators[esc.tag]                  = Conditions()
    energy_conditions.modulators[esc.tag].inputs           = Conditions()
    energy_conditions.modulators[esc.tag].outputs          = Conditions()
    energy_conditions.modulators[esc.tag].throttle         = 0. * ones_row(1)  
    energy_conditions.modulators[esc.tag].outputs.voltage  = 0. * ones_row(1)  
    energy_conditions.modulators[esc.tag].inputs.voltage   = esc.bus_voltage * ones_row(1)   
    return 
