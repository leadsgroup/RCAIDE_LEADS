# RCAIDE/Library/Methods/Powertrain/Converters/offtake_shaft/append_offtake_shaft_conditions.py
# (c) Copyright 2023 Aerospace Research Community LLC
# 
# Created:  Jun 2024, M. Clarke  

from RCAIDE.Framework.Mission.Common     import   Conditions

# ---------------------------------------------------------------------------------------------------------------------- 
#  append_ram_conditions
# ----------------------------------------------------------------------------------------------------------------------    
def append_offtake_shaft_conditions(offtake_shaft,segment,propulsor_conditions): 
    propulsor_conditions[offtake_shaft.tag]                              = Conditions() 
    propulsor_conditions[offtake_shaft.tag].inputs                       = Conditions() 
    propulsor_conditions[offtake_shaft.tag].outputs                      = Conditions() 
    return 