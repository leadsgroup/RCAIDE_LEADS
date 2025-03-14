# RCAIDE/Library/Methods/Powertrain/Propulsors/Electric_Rotor_Propulsor/unpack_electric_rotor_unknowns.py
# (c) Copyright 2023 Aerospace Research Community LLC
# 
# Created:  Jun 2024, M. Clarke   

# ---------------------------------------------------------------------------------------------------------------------- 
#  unpack electric rotor network unknowns 
# ----------------------------------------------------------------------------------------------------------------------  

def unpack_electric_rotor_unknowns(propulsor,segment): 
    '''
    Unpack residuals for electric rotor and assigns them to the specfic
    compoment each interation of the mission solver   
    '''
    motor_conditions = segment.state.conditions.energy.converters[motor.tag]
    motor   = propulsor.motor  
    rotor   = propulsor.rotor
    if rotor.fidelity == 'Blade_Element_Momentum_Theory_Helmholtz_Wake':
        motor_conditions.inputs.current = segment.state.unknowns[propulsor.tag + '_motor_current'] 
    return 