# RCAIDE/Library/Methods/Powertrain/Propulsors/Electric_Rotor_Propulsor/pack_ice_propeller_residuals.py
# (c) Copyright 2023 Aerospace Research Community LLC
# 
# Created:  Jun 2024, M. Clarke   

# ---------------------------------------------------------------------------------------------------------------------- 
#  pack ice propeller residuals
# ----------------------------------------------------------------------------------------------------------------------  

def pack_ice_propeller_residuals(propulsor,segment):  
    engine             = propulsor.engine
    propeller          = propulsor.propeller  
    q_engine           = segment.state.conditions.energy[propulsor.tag][engine.tag].torque
    q_prop             = segment.state.conditions.energy[propulsor.tag][propeller.tag].torque 
    segment.state.residuals.network[propulsor.tag + '_rotor_engine_torque'] = q_engine - q_prop 
    return 
