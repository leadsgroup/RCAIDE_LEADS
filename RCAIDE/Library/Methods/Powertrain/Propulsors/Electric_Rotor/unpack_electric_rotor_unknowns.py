# RCAIDE/Library/Methods/Powertrain/Propulsors/Electric_Rotor_Propulsor/unpack_electric_rotor_unknowns.py

# Created:  Jun 2024, M. Clarke   

# ---------------------------------------------------------------------------------------------------------------------- 
#  unpack electric rotor network unknowns 
# ----------------------------------------------------------------------------------------------------------------------  

def unpack_electric_rotor_unknowns(propulsor,segment): 
    '''
    Unpack residuals for electric rotor and assigns them to the specfic
    compoment each interation of the mission solver   
    '''
    motor   = propulsor.motor   
    motor_conditions = segment.state.conditions.energy.converters[motor.tag]
    motor_conditions.inputs.current = segment.state.unknowns[propulsor.tag + '_motor_current'] 
    return 