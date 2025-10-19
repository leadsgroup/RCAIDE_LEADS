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
    segment.state.conditions.energy.converters[pump.tag].outputs.power           = 0. * ones_row(1)

    segment.state.conditions.energy.converters[pump.tag].power                   = Conditions()
    segment.state.conditions.energy.converters[pump.tag].power.propulsive        = 0 * ones_row(1)
    segment.state.conditions.energy.converters[pump.tag].power.mechanical        = 0 * ones_row(1)
    segment.state.conditions.energy.converters[pump.tag].power.electrical        = 0 * ones_row(1)
    segment.state.conditions.energy.converters[pump.tag].power.chemical          = 0 * ones_row(1)
    segment.state.conditions.energy.converters[pump.tag].power.pneumatic         = 0 * ones_row(1)
    segment.state.conditions.energy.converters[pump.tag].power.hydraulic         = 0 * ones_row(1)
    segment.state.conditions.energy.converters[pump.tag].power.thermal           = 0 * ones_row(1)
    return 