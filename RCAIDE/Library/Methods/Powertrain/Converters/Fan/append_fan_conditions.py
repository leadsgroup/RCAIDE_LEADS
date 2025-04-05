# RCAIDE/Library/Methods/Powertrain/Converters/fan/append_fan_conditions.py
# (c) Copyright 2023 Aerospace Research Community LLC
# 
# Created:  Jun 2024, M. Clarke  

from RCAIDE.Framework.Mission.Common     import   Conditions

# ---------------------------------------------------------------------------------------------------------------------- 
#  append_fan_conditions
# ----------------------------------------------------------------------------------------------------------------------    
def append_fan_conditions(fan,segment,energy_conditions): 
    ones_row    = segment.state.ones_row                  
    energy_conditions.converters[fan.tag]                              = Conditions() 
    energy_conditions.converters[fan.tag].inputs                       = Conditions() 
    energy_conditions.converters[fan.tag].outputs                      = Conditions()
    energy_conditions.converters[fan.tag].rpm                          = 0. * ones_row(1) 
    
    return 