# RCAIDE/Library/Methods/Powertrain/Propulsors/Electric_Rotor_Propulsor/unpack_ice_propeller_unknowns.py
# (c) Copyright 2023 Aerospace Research Community LLC
# 
# Created:  Jun 2024, M. Clarke   

# ---------------------------------------------------------------------------------------------------------------------- 
#  unpack ice propeller network unknowns 
# ----------------------------------------------------------------------------------------------------------------------  

def unpack_ice_propeller_unknowns(propulsor,segment):  
    engine            = propulsor.engine 
    segment.state.conditions.energy[propulsor.tag][engine.tag].rpm = segment.state.unknowns[propulsor.tag + '_propeller_rpm'] 
    return 