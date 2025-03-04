# RCAIDE/Library/Methods/Powertrain/Propulsors/Electric_Rotor_Propulsor/append_electric_ducted_fan_residual_and_unknowns.py 
# 
# Created:  Jun 2024, M. Clarke

# ---------------------------------------------------------------------------------------------------------------------- 
#  append_electric_ducted_fan_residual_and_unknown
# ----------------------------------------------------------------------------------------------------------------------  
def append_electric_ducted_fan_residual_and_unknown(propulsor,segment):
    ''' 
    Appends the torque matching residual and unknown
    ''' 
    ones_row    = segment.state.ones_row 
    ducted_fan   = propulsor.ducted_fan
    cp_init      = ducted_fan.cruise.design_power_coefficient
    segment.state.unknowns[ propulsor.tag  + '_ducted_fan_cp']               = cp_init * ones_row(1)  
    segment.state.residuals.network[ propulsor.tag  + '_ducted_fan_motor_torque'] = 0. * ones_row(1)    
    
    return 