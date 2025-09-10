# RCAIDE/Library/Methods/Powertrain/Converters/Pump/append_pump_conditions.py
# 
# Created:  Sep. 2025, M. Guidotti

from RCAIDE.Framework.Mission.Common     import   Conditions

# ---------------------------------------------------------------------------------------------------------------------- 
#  append_pump_conditions
# ----------------------------------------------------------------------------------------------------------------------    
def append_pump_conditions(pump,segment,energy_conditions): 

    """
    
    """

    ones_row                                                       = segment.state.ones_row 
    energy_conditions.converters[pump.tag]                         = Conditions()
    energy_conditions.converters[pump.tag].inputs                  = Conditions()
    energy_conditions.converters[pump.tag].outputs                 = Conditions() 
    energy_conditions.converters[pump.tag].inputs.p_in             = 0. * ones_row(1) 
    energy_conditions.converters[pump.tag].outputs.p_out           = 0. * ones_row(1)
    energy_conditions.converters[pump.tag].outputs.power           = 0. * ones_row(1)

    return 