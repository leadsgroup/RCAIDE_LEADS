# RCAIDE/Library/Methods/Powertrain/Propulsors/Internal_Combustion_Engine/pack_internal_combustion_engine_residuals.py
# (c) Copyright 2023 Aerospace Research Community LLC
# 
# Created:  Jun 2024, M. Clarke   

# ---------------------------------------------------------------------------------------------------------------------- 
#  pack ice propeller residuals
# ----------------------------------------------------------------------------------------------------------------------  

def pack_internal_combustion_engine_residuals(propulsor,segment):
    ''' Computes the torque-matching residual between the engine and the propeller
    to be evalauted by the mission solver 
    '''
    # unpack
    engine             = propulsor.engine
    propeller          = propulsor.propeller  
    q_engine           = segment.state.conditions.energy.converters[engine.tag].torque
    q_prop             = segment.state.conditions.energy.converters[propeller.tag].torque

    # compute torque matching residual        
    segment.state.residuals.network[propulsor.tag + '_rotor_engine_torque'] = q_engine - q_prop 
    return 
