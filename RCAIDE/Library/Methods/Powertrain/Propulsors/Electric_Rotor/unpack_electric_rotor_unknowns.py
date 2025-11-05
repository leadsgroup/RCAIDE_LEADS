# RCAIDE/Library/Methods/Powertrain/Propulsors/Electric_Rotor_Propulsor/unpack_electric_rotor_unknowns.py

# Created:  Jun 2024, M. Clarke   

import RCAIDE

# ---------------------------------------------------------------------------------------------------------------------- 
#  unpack electric rotor network unknowns 
# ----------------------------------------------------------------------------------------------------------------------  

def unpack_electric_rotor_unknowns(propulsor,segment, network): 
    '''
    Unpack residuals for electric rotor and assigns them to the specfic
    compoment each interation of the mission solver   
    '''
    for assigned_converter_tag in propulsor.assigned_converters:
        if isinstance(network.converters[assigned_converter_tag[0][0]], RCAIDE.Library.Components.Powertrain.Converters.Motor):
            motor = network.converters[assigned_converter_tag[0][0]]
 
    motor_conditions = segment.state.conditions.energy.converters[motor.tag]
    motor_conditions.inputs.current = segment.state.unknowns[propulsor.tag + '_motor_current'] 
    return 