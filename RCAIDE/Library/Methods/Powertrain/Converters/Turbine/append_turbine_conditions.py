# RCAIDE/Library/Methods/Powertrain/Converters/turbine/append_turbine_conditions.py
# (c) Copyright 2023 Aerospace Research Community LLC
# 
# Created:  Jun 2024, M. Clarke  

from RCAIDE.Framework.Mission.Common     import   Conditions

# ---------------------------------------------------------------------------------------------------------------------- 
#  
# ----------------------------------------------------------------------------------------------------------------------    
def append_turbine_conditions(turbine,segment,propulsor_conditions): 
    ones_row    = segment.state.ones_row 
    propulsor_conditions[turbine.tag]                                       = Conditions()
    propulsor_conditions[turbine.tag].inputs                                = Conditions()
    propulsor_conditions[turbine.tag].outputs                               = Conditions()
    propulsor_conditions[turbine.tag].inputs.fan                            = Conditions()
    propulsor_conditions[turbine.tag].inputs.fan.work_done                  = 0*ones_row(1)  
    propulsor_conditions[turbine.tag].inputs.shaft_power_off_take           = Conditions()
    propulsor_conditions[turbine.tag].inputs.shaft_power_off_take.work_done = 0*ones_row(1) 
    return 