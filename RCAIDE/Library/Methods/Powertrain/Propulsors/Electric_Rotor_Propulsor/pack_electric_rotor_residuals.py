# RCAIDE/Library/Methods/Powertrain/Propulsors/Electric_Rotor_Propulsor/pack_electric_rotor_residuals.py
# (c) Copyright 2023 Aerospace Research Community LLC
# 
# Created:  Jun 2024, M. Clarke   

# ---------------------------------------------------------------------------------------------------------------------- 
#  pack electric rotor network residuals
# ----------------------------------------------------------------------------------------------------------------------  

def pack_electric_rotor_residuals(propulsor,segment): 
    propulsor_results   = segment.state.conditions.energy
    motor               = propulsor.motor
    rotor               = propulsor.rotor 
    q_motor             = propulsor_results[propulsor.tag][motor.tag].torque
    q_prop              = propulsor_results[propulsor.tag][rotor.tag].torque 
    segment.state.residuals.network[ propulsor.tag + '_rotor_motor_torque'] = q_motor - q_prop 
    return 
