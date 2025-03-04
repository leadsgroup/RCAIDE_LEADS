# RCAIDE/Library/Methods/Powertrain/Converters/Engine/append_engine_conditions.py
# (c) Copyright 2023 Aerospace Research Community LLC
# 
# Created:  Jun 2024, M. Clarke  

from RCAIDE.Framework.Mission.Common     import   Conditions

# ---------------------------------------------------------------------------------------------------------------------- 
#  append_engine_conditions
# ----------------------------------------------------------------------------------------------------------------------    
def append_engine_conditions(engine,segment,propulsor_conditions): 
    propulsor_conditions[engine.tag]                      = Conditions() 
    propulsor_conditions[engine.tag].inputs               = Conditions()
    propulsor_conditions[engine.tag].outputs              = Conditions()      
    return 
