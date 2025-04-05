# RCAIDE/Library/Methods/Powertrain/Converters/turbine/append_turbine_conditions.py
# (c) Copyright 2023 Aerospace Research Community LLC
# 
# Created:  Jun 2024, M. Clarke  

from RCAIDE.Framework.Mission.Common     import   Conditions

# ---------------------------------------------------------------------------------------------------------------------- 
#  
# ----------------------------------------------------------------------------------------------------------------------    
def append_turbine_conditions(turbine,segment,energy_conditions): 
    ones_row    = segment.state.ones_row 
    energy_conditions.converters[turbine.tag]                                             = Conditions()
    energy_conditions.converters[turbine.tag].inputs                                      = Conditions()
    energy_conditions.converters[turbine.tag].outputs                                     = Conditions()
    energy_conditions.converters[turbine.tag].inputs.fan                                  = Conditions()
    energy_conditions.converters[turbine.tag].inputs.fan.work_done                        = 0*ones_row(1)
    return 