# RCAIDE/Library/Methods/Powertrain/Propulsors/Internal_Combustion_Engine/pack_internal_combustion_engine_residuals.py
# 
# Created:  Jun 2024, M. Clarke   

# ---------------------------------------------------------------------------------------------------------------------- 
#  pack ice propeller residuals
# ----------------------------------------------------------------------------------------------------------------------  
from RCAIDE.Framework.Core import set_at
import numpy as np

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
    residuals = segment.state.residuals.network
    key       = propulsor.tag + '_rotor_engine_torque'
    residuals[key] = set_at(residuals[key], np.s_[...], q_engine - q_prop)
    return 
