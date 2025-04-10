# RCAIDE/Library/Methods/Powertrain/Propulsors/Internal_Combustion_Engine/unpack_internal_combustion_engine_unknowns.py
# 
# Created:  Jun 2024, M. Clarke   

# ---------------------------------------------------------------------------------------------------------------------- 
#  unpack ice propeller network unknowns 
# ----------------------------------------------------------------------------------------------------------------------  

def unpack_internal_combustion_engine_unknowns(propulsor,segment):  
    """Unpack internal combustion engine unknowns and assigns them to the specfic
    compoment each interation of the mission solver
    """
    engine            = propulsor.engine 
    segment.state.conditions.energy.converters[engine.tag].omega = segment.state.unknowns[propulsor.tag + '_propeller_omega'] 
    return 