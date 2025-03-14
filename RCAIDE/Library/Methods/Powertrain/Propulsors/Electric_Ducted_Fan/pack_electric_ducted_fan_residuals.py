# RCAIDE/Library/Methods/Powertrain/Propulsors/Electric_Rotor_Propulsor/pack_electric_ducted_fan_residuals.py 
# 
# Created:  Jun 2024, M. Clarke   

# ---------------------------------------------------------------------------------------------------------------------- 
#  pack electric ducted_fan network residuals
# ----------------------------------------------------------------------------------------------------------------------  

def pack_electric_ducted_fan_residuals(propulsor,segment):
    ''' Computes the torque-matching residual between the motor and the ducted fan
    to be evalauted by the mission solver 
    '''
    # unpack 
    motor         = propulsor.motor
    ducted_fan    = propulsor.ducted_fan 
    q_motor       = segment.state.conditions.energy.converters[motor.tag].torque
    q_ducted_fan  = segment.state.conditions.energy.converters[ducted_fan.tag].torque
    
    # compute torque matching residual
    segment.state.residuals.network[propulsor.tag  + '_ducted_fan_motor_torque'] = q_motor - q_ducted_fan
    return 
