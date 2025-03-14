# RCAIDE/Library/Methods/Powertrain/Propulsors/Internal_Combustion_Engine/unpack_internal_combustion_engine_unknowns.py
# (c) Copyright 2023 Aerospace Research Community LLC
# 
# Created:  Jun 2024, M. Clarke   

# ---------------------------------------------------------------------------------------------------------------------- 
#  unpack ice propeller network unknowns 
# ----------------------------------------------------------------------------------------------------------------------  

def unpack_internal_combustion_engine_unknowns(propulsor,segment):  
    engine            = propulsor.engine 
    segment.state.conditions.energy.converters[engine.tag].rpm = segment.state.unknowns[propulsor.tag + '_propeller_rpm'] 
    return 