# RCAIDE/Library/Methods/Powertrain/Converters/fan/append_fan_conditions.py
# (c) Copyright 2023 Aerospace Research Community LLC
# 
# Created:  Jun 2024, M. Clarke  

from RCAIDE.Framework.Mission.Common     import   Conditions

# ---------------------------------------------------------------------------------------------------------------------- 
#  append_fan_conditions
# ----------------------------------------------------------------------------------------------------------------------    
def append_fan_conditions(fan,segment,propulsor_conditions): 
    propulsor_conditions[fan.tag]                              = Conditions() 
    propulsor_conditions[fan.tag].inputs                       = Conditions() 
    propulsor_conditions[fan.tag].outputs                      = Conditions() 
    return 