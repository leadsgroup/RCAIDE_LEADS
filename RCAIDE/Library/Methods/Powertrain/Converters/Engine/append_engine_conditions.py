# RCAIDE/Library/Methods/Powertrain/Converters/Engine/append_engine_conditions.py
# (c) Copyright 2023 Aerospace Research Community LLC
# 
# Created:  Jun 2024, M. Clarke  

from RCAIDE.Framework.Mission.Common     import   Conditions

# ---------------------------------------------------------------------------------------------------------------------- 
#  append_engine_conditions
# ----------------------------------------------------------------------------------------------------------------------    
def append_engine_conditions(engine,segment,energy_conditions,noise_conditions=None): 
    energy_conditions.converters[engine.tag]                      = Conditions() 
    energy_conditions.converters[engine.tag].inputs               = Conditions()
    energy_conditions.converters[engine.tag].outputs              = Conditions()      
    return 
