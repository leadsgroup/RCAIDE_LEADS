# RCAIDE/Library/Methods/Powertrain/Systems/append_avionics_conditions.py
# (c) Copyright 2023 Aerospace Research Community LLC
# 
# Created:  Jun 2024, M. Clarke  

from RCAIDE.Framework.Mission.Common     import   Conditions

# ---------------------------------------------------------------------------------------------------------------------- 
#  append_motor_conditions
# ----------------------------------------------------------------------------------------------------------------------    
def append_avionics_conditions(avionics,segment,bus):  
    ones_row    = segment.state.ones_row
    segment.state.conditions.energy[bus.tag][avionics.tag]            = Conditions()
    segment.state.conditions.energy[bus.tag][avionics.tag].power      = 0 * ones_row(1) 
    
    return 
