# RCAIDE/Library/Methods/Powertrain/Propulsors/Electric_Rotor_Propulsor/append_electric_rotor_residual_and_unknown.py
# (c) Copyright 2023 Aerospace Research Community LLC
# 
# Created:  Jun 2024, M. Clarke  

# ----------------------------------------------------------------------------------------------------------------------
#  IMPORT
# ----------------------------------------------------------------------------------------------------------------------  
 # RCAIDE imports 
import RCAIDE 

# ---------------------------------------------------------------------------------------------------------------------- 
#  append_electric_rotor_residual_and_unknown
# ----------------------------------------------------------------------------------------------------------------------  
def append_electric_rotor_residual_and_unknown(propulsor,segment):
    ''' 
    appends the torque matching residual and unknown
    '''
    ones_row    = segment.state.ones_row
    rotor       = propulsor.rotor  
    motor       = propulsor.motor
    if type(rotor) == RCAIDE.Library.Components.Powertrain.Converters.Propeller:
        cp_init  = float(rotor.cruise.design_power_coefficient)
    elif (type(rotor) == RCAIDE.Library.Components.Powertrain.Converters.Lift_Rotor) or (type(rotor) == RCAIDE.Library.Components.Powertrain.Converters.Prop_Rotor):
        cp_init  = float(rotor.hover.design_power_coefficient)
    else:
        cp_init  = 0.5
         
    if (type(motor) == RCAIDE.Library.Components.Powertrain.Converters.PMSM_Motor):
        segment.state.unknowns[ propulsor.tag + '_current']                    = 50 * ones_row(1)  
    else:
        segment.state.unknowns[ propulsor.tag + '_rotor_cp']                    = cp_init * ones_row(1)  
    segment.state.residuals.network[propulsor.tag +'_rotor_motor_torque'] = 0. * ones_row(1)
    
    return 