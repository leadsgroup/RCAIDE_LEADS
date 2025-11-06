# RCAIDE/Library/Methods/Powertrain/Propulsors/Electric_Rotor_Propulsor/pack_electric_rotor_residuals.py
# 
# Created:  Jun 2024, M. Clarke   

import RCAIDE

# ---------------------------------------------------------------------------------------------------------------------- 
#  pack electric rotor network residuals
# ----------------------------------------------------------------------------------------------------------------------  

def pack_electric_rotor_residuals(propulsor,segment, network): 
    ''' Computes the torque-matching residual between the motor and the rotor
    to be evalauted by the mission solver 
    '''    
    # unpack
    propulsor_results   = segment.state.conditions.energy

    for assigned_converter_tag in propulsor.assigned_converters:
        if isinstance(network.converters[assigned_converter_tag[0][0]], RCAIDE.Library.Components.Powertrain.Converters.Motor):
            motor = network.converters[assigned_converter_tag[0][0]]
        elif isinstance(network.converters[assigned_converter_tag[0][0]], RCAIDE.Library.Components.Powertrain.Converters.Rotor):
            rotor = network.converters[assigned_converter_tag[0][0]]

    q_motor             = propulsor_results.converters[motor.tag].outputs.torque
    q_prop              = propulsor_results.converters[rotor.tag].torque
    segment.state.residuals.network[ propulsor.tag + '_rotor_motor_torque'] = q_motor - q_prop 
    return 
