# RCAIDE/Library/Methods/Powertrain/Converters/Pump/append_pump_conditions.py
# 
# Created:  Sep. 2025, M. Guidotti

from RCAIDE.Framework.Mission.Common     import   Conditions

# ---------------------------------------------------------------------------------------------------------------------- 
#  append_pump_conditions
# ----------------------------------------------------------------------------------------------------------------------    
def append_pump_conditions(pump,segment): 

    """
    
    """

    ones_row                                                       = segment.state.ones_row 
    segment.state.conditions.energy.converters[pump.tag]                         = Conditions()
    segment.state.conditions.energy.converters[pump.tag].inputs                  = Conditions()
    segment.state.conditions.energy.converters[pump.tag].outputs                 = Conditions() 
    segment.state.conditions.energy.converters[pump.tag].inputs.p_in             = 0. * ones_row(1) 
    segment.state.conditions.energy.converters[pump.tag].outputs.p_out           = 0. * ones_row(1) 
    segment.state.conditions.energy.converters[pump.tag].outputs.power                   = Conditions() 
    segment.state.conditions.energy.converters[pump.tag].outputs.power.mechanical        = 0 * ones_row(1)
    segment.state.conditions.energy.converters[pump.tag].outputs.power.electrical        = 0 * ones_row(1)
    segment.state.conditions.energy.converters[pump.tag].outputs.power.chemical          = 0 * ones_row(1)
    segment.state.conditions.energy.converters[pump.tag].outputs.power.pneumatic         = 0 * ones_row(1)
    segment.state.conditions.energy.converters[pump.tag].outputs.power.hydraulic         = 0 * ones_row(1)
    segment.state.conditions.energy.converters[pump.tag].outputs.power.thermal           = 0 * ones_row(1)
    segment.state.conditions.energy.converters[pump.tag].inputs.power                   = Conditions() 
    segment.state.conditions.energy.converters[pump.tag].inputs.power.mechanical        = 0 * ones_row(1)
    segment.state.conditions.energy.converters[pump.tag].inputs.power.electrical        = 0 * ones_row(1)
    segment.state.conditions.energy.converters[pump.tag].inputs.power.chemical          = 0 * ones_row(1)
    segment.state.conditions.energy.converters[pump.tag].inputs.power.pneumatic         = 0 * ones_row(1)
    segment.state.conditions.energy.converters[pump.tag].inputs.power.hydraulic         = 0 * ones_row(1)
    segment.state.conditions.energy.converters[pump.tag].inputs.power.thermal           = 0 * ones_row(1)
    return 