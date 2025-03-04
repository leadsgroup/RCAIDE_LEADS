# RCAIDE/Library/Methods/Powertrain/Propulsors/Electric_Rotor_Propulsor/append_ice_residual_and_unknown.py
# (c) Copyright 2023 Aerospace Research Community LLC
# 
# Created:  Jun 2024, M. Clarke  

# ----------------------------------------------------------------------------------------------------------------------
#  IMPORT
# ----------------------------------------------------------------------------------------------------------------------  
from RCAIDE.Framework.Core import  Units

# ---------------------------------------------------------------------------------------------------------------------- 
#  append_ice_residual_and_unknown
# ----------------------------------------------------------------------------------------------------------------------  
def append_ice_residual_and_unknown(propulsor,segment):
    '''
    
    appends the torque matching residual and unknown
    '''
    
    ones_row    = segment.state.ones_row                   
    propeller  = propulsor.propeller 
    segment.state.unknowns[propulsor.tag  + '_propeller_rpm'] = ones_row(1) * float(propeller.cruise.design_angular_velocity) /Units.rpm   
    segment.state.residuals.network[ propulsor.tag + '_rotor_engine_torque'] = 0. * ones_row(1)
    
    return 