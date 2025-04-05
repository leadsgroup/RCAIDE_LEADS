# RCAIDE/Library/Methods/Powertrain/Converters/Engine/append_engine_conditions.py
# (c) Copyright 2023 Aerospace Research Community LLC
# 
# Created:  Jun 2024, M. Clarke  

from RCAIDE.Framework.Mission.Common     import   Conditions

# ---------------------------------------------------------------------------------------------------------------------- 
#  append_engine_conditions
# ----------------------------------------------------------------------------------------------------------------------    
def append_engine_conditions(engine,segment,energy_conditions,noise_conditions=None): 
    # unpack 
    ones_row          = segment.state.ones_row
    
    energy_conditions.converters[engine.tag]                      = Conditions() 
    energy_conditions.converters[engine.tag].inputs               = Conditions()
    energy_conditions.converters[engine.tag].outputs              = Conditions()
    energy_conditions.converters[engine.tag].omega                = engine.rated_speed * ones_row(1)   
    
    return 
