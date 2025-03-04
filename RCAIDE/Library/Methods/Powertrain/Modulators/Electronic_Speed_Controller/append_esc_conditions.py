# RCAIDE/Library/Methods/Powertrain/Modulators/Electronic_Speed_Controller/append_motor_conditions.py
# (c) Copyright 2023 Aerospace Research Community LLC
# 
# Created:  Jun 2024, M. Clarke  

from RCAIDE.Framework.Mission.Common     import   Conditions

# ---------------------------------------------------------------------------------------------------------------------- 
#  append_motor_conditions
# ----------------------------------------------------------------------------------------------------------------------    
def append_esc_conditions(esc,segment,propulsor_conditions): 
    ones_row    = segment.state.ones_row 
    propulsor_conditions[esc.tag]            = Conditions()
    propulsor_conditions[esc.tag].inputs     = Conditions()
    propulsor_conditions[esc.tag].outputs    = Conditions()
    propulsor_conditions[esc.tag].throttle   = 0. * ones_row(1)  
    
    return 
