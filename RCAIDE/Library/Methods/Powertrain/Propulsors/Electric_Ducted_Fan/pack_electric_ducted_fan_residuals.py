# RCAIDE/Library/Methods/Powertrain/Propulsors/Electric_Rotor_Propulsor/pack_electric_ducted_fan_residuals.py 
# 
# Created:  Jun 2024, M. Clarke   

# ---------------------------------------------------------------------------------------------------------------------- 
#  pack electric ducted_fan network residuals
# ----------------------------------------------------------------------------------------------------------------------  
from RCAIDE.Framework.Core import set_at
import numpy as np

def pack_electric_ducted_fan_residuals(propulsor,segment):
    ''' Computes the torque-matching residual between the motor and the ducted fan
    to be evalauted by the mission solver 
    '''
    # unpack 
    motor         = propulsor.motor
    ducted_fan    = propulsor.ducted_fan 
    q_motor       = segment.state.conditions.energy.converters[motor.tag].outputs.torque
    q_ducted_fan  = segment.state.conditions.energy.converters[ducted_fan.tag].torque
    
    # compute torque matching residual
    residuals = segment.state.residuals.network
    key       = propulsor.tag  + '_ducted_fan_motor_torque'
    residuals[key] = set_at(residuals[key], np.s_[...], q_motor - q_ducted_fan)
    return
