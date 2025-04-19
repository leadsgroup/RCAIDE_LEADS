# RCAIDE/Library/Methods/Powertrain/Converters/ram/append_ram_conditions.py
# (c) Copyright 2023 Aerospace Research Community LLC
# 
# Created:  Jun 2024, M. Clarke  

from RCAIDE.Framework.Mission.Common     import   Conditions

# ---------------------------------------------------------------------------------------------------------------------- 
#  append_ram_conditions
# ----------------------------------------------------------------------------------------------------------------------    
def append_ram_conditions(ram,segment,propulsor_conditions): 
    propulsor_conditions[ram.tag]                              = Conditions() 
    propulsor_conditions[ram.tag].inputs                       = Conditions() 
    propulsor_conditions[ram.tag].outputs                      = Conditions() 
    return 