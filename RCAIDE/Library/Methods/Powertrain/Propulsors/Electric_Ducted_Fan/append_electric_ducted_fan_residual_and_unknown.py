# RCAIDE/Library/Methods/Powertrain/Propulsors/Electric_Rotor_Propulsor/append_electric_ducted_fan_residual_and_unknowns.py 
# 
# Created:  Jun 2024, M. Clarke

import numpy as np
# ---------------------------------------------------------------------------------------------------------------------- 
#  append_electric_ducted_fan_residual_and_unknown
# ----------------------------------------------------------------------------------------------------------------------  
def append_electric_ducted_fan_residual_and_unknown(propulsor,segment):
    ''' 
    Appends the torque matching residual and unknown
    ''' 
    ones_row     = segment.state.ones_row 
    motor        = propulsor.motor 
    segment.state.unknowns[propulsor.tag + '_motor_current']                     = motor.design_current * ones_row(1) 
    segment.state.numerics.solver.upper_bounds[propulsor.tag + '_motor_current'] =   np.inf* ones_row(1) 
    segment.state.numerics.solver.lower_bounds[propulsor.tag + '_motor_current'] =  - np.inf* ones_row(1) 
    segment.state.number_of_unknowns  += 1
    segment.state.number_of_residuals += 1
    return 
