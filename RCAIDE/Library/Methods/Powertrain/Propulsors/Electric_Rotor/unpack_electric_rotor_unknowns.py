# RCAIDE/Library/Methods/Powertrain/Propulsors/Electric_Rotor_Propulsor/unpack_electric_rotor_unknowns.py
# (c) Copyright 2023 Aerospace Research Community LLC
# 
# Created:  Jun 2024, M. Clarke   

# ---------------------------------------------------------------------------------------------------------------------- 
#  unpack electric rotor network unknowns 
# ----------------------------------------------------------------------------------------------------------------------  

def unpack_electric_rotor_unknowns(propulsor,segment): 
    results = segment.state.conditions.energy[propulsor.tag]
    motor   = propulsor.motor  
    rotor   = propulsor.rotor
    if rotor.fidelity == 'Blade_Element_Momentum_Theory_Helmholtz_Wake':
        results[motor.tag].inputs.current = segment.state.unknowns[propulsor.tag + '_motor_current'] 
    return 