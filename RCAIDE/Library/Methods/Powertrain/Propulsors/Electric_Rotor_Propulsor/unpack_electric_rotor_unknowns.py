# RCAIDE/Library/Methods/Powertrain/Propulsors/Electric_Rotor_Propulsor/unpack_electric_rotor_unknowns.py
# (c) Copyright 2023 Aerospace Research Community LLC
# 
# Created:  Jun 2024, M. Clarke   

# ----------------------------------------------------------------------------------------------------------------------
#  IMPORT
# ----------------------------------------------------------------------------------------------------------------------
# RCAIDE imports

import RCAIDE

# ---------------------------------------------------------------------------------------------------------------------- 
#  unpack electric rotor network unknowns 
# ----------------------------------------------------------------------------------------------------------------------  

def unpack_electric_rotor_unknowns(propulsor,segment): 
    results = segment.state.conditions.energy[propulsor.tag]
    motor   = propulsor.motor  
    if (type(motor) == RCAIDE.Library.Components.Powertrain.Converters.PMSM_Motor):
        results[motor.tag].current = segment.state.unknowns[propulsor.tag + '_current'] 
    elif (type(motor) == RCAIDE.Library.Components.Powertrain.Converters.DC_Motor):
        results[motor.tag].rotor_power_coefficient = segment.state.unknowns[propulsor.tag + '_rotor_cp'] 
    return 