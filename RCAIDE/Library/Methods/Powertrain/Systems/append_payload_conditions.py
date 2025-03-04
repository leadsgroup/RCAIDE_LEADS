# RCAIDE/Library/Methods/Powertrain/Systems/append_payload_conditions.py
# (c) Copyright 2023 Aerospace Research Community LLC
# 
# Created:  Jun 2024, M. Clarke  

from RCAIDE.Framework.Mission.Common     import   Conditions

# ---------------------------------------------------------------------------------------------------------------------- 
#  append_motor_conditions
# ----------------------------------------------------------------------------------------------------------------------    
def append_payload_conditions(payload,segment,bus):  
    ones_row    = segment.state.ones_row 
    segment.state.conditions.energy[bus.tag][payload.tag]       = Conditions()
    segment.state.conditions.energy[bus.tag][payload.tag].power = 0 * ones_row(1)  
    return 
