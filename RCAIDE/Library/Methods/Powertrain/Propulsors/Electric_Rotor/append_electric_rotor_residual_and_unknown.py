# RCAIDE/Library/Methods/Powertrain/Propulsors/Electric_Rotor_Propulsor/append_electric_rotor_residual_and_unknown.py
# (c) Copyright 2023 Aerospace Research Community LLC
# 
# Created:  Jun 2024, M. Clarke  
 
# ---------------------------------------------------------------------------------------------------------------------- 
#  append_electric_rotor_residual_and_unknown
# ----------------------------------------------------------------------------------------------------------------------  
def append_electric_rotor_residual_and_unknown(propulsor,segment):
    ''' 
    appends the torque matching residual and unknown
    '''
    ones_row    = segment.state.ones_row 
    motor       = propulsor.motor
    if motor != None:  
        segment.state.unknowns[propulsor.tag + '_motor_current']              = motor.design_current * ones_row(1) 
        segment.state.residuals.network[propulsor.tag +'_rotor_motor_torque'] = 0. * ones_row(1) 
    return 